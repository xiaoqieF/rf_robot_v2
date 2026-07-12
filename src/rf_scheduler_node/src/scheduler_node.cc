#include "rf_scheduler_node/scheduler_node.hpp"

#include "elog/elog.h"
#include "rclcpp/rate.hpp"
#include "rf_util/robot_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <queue>
#include <string>
#include <tuple>
#include <utility>

namespace rf_scheduler
{

namespace
{

// The live /slam_map uses 0..100 occupancy probabilities. Treat any
// non-unknown cell below the occupied threshold as traversable for frontier
// detection, otherwise early maps stay "too occupied" and never seed.
constexpr int8_t kFreeThreshold = 65;
constexpr std::chrono::milliseconds kDefaultServiceWait{1000};
constexpr std::chrono::milliseconds kDefaultResponseWait{5000};
constexpr std::chrono::milliseconds kMapWarmupWait{4000};
constexpr std::chrono::milliseconds kFinalMapWait{2000};
constexpr std::chrono::milliseconds kFrontierRetrySleep{1000};
constexpr std::chrono::milliseconds kFrontierSettleSleep{1500};
constexpr std::chrono::seconds kInitialMappingTimeout{20};
constexpr std::chrono::seconds kNoFrontierStableDuration{8};
constexpr uint32_t kDefaultMaxFrontierFailures = 10;
constexpr uint32_t kDefaultMinFrontierClusterSize = 8;
constexpr double kReachedGoalSuppressRadius = 0.2;
constexpr double kFailedGoalSuppressRadius = 0.35;
constexpr std::size_t kMinimumSeedFreeCells = 50;
constexpr int8_t kGoalOccupiedThreshold = 80;
constexpr double kGoalMinStandoffMeters = 0.20;
constexpr double kGoalPreferredStandoffMeters = 0.40;
constexpr double kGoalSearchRadiusMeters = 1.20;
constexpr double kGoalClearanceRadiusMeters = 0.20;
constexpr double kGoalKnownRadiusMeters = 0.10;

enum class FutureWaitStatus
{
    READY,
    INTERRUPTED,
    SHUTDOWN,
};

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.z = std::sin(yaw * 0.5);
    quaternion.w = std::cos(yaw * 0.5);
    return quaternion;
}

bool isUnknownCell(int8_t value)
{
    return value < 0;
}

bool isFreeCell(int8_t value)
{
    return value >= 0 && value <= kFreeThreshold;
}

std::size_t countFreeCells(const nav_msgs::msg::OccupancyGrid& map)
{
    std::size_t free_cells = 0;
    for (const auto value : map.data) {
        if (isFreeCell(value)) {
            ++free_cells;
        }
    }
    return free_cells;
}

double squaredDistance(double ax, double ay, double bx, double by)
{
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy;
}

template <typename FutureT, typename InterruptFn>
FutureWaitStatus waitForFuture(
    FutureT& future,
    const InterruptFn& should_interrupt,
    const std::chrono::milliseconds poll_interval = std::chrono::milliseconds(100))
{
    while (rclcpp::ok() && future.wait_for(poll_interval) != std::future_status::ready) {
        if (should_interrupt()) {
            return FutureWaitStatus::INTERRUPTED;
        }
    }

    return rclcpp::ok() ? FutureWaitStatus::READY : FutureWaitStatus::SHUTDOWN;
}

template <typename FutureT, typename CancelFn>
void cancelFutureGoalIfAccepted(FutureT& future, const CancelFn& cancel_goal)
{
    using namespace std::chrono_literals;

    if (future.wait_for(500ms) != std::future_status::ready) {
        return;
    }

    auto goal_handle = future.get();
    if (goal_handle) {
        cancel_goal(goal_handle);
    }
}

} // namespace

#define SCHED_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_scheduler_node] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define SCHED_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_scheduler_node] " fmt_str, ##__VA_ARGS__); \
    } while(0)

void SchedulerNode::init()
{
    const auto slam_map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    goal_pose_sub_ = this->create_subscription<PoseStampedT>(
        "/goal_pose",
        10,
        [this](const PoseStampedT::SharedPtr msg) {
            if (exploration_active_.load()) {
                SCHED_WARN("Ignoring /goal_pose while an exploration job is running.");
                return;
            }

            {
                std::lock_guard<std::mutex> lock(goal_pose_mutex_);
                goal_pose_msg_ = msg;
                preempt_requested_ = true;
            }
            cancelActiveNavigationGoals();
        });

    slam_map_sub_ = this->create_subscription<OccupancyGridMsgT>(
        "/slam_map",
        slam_map_qos,
        [this](const OccupancyGridMsgT::SharedPtr msg) {
            if (!msg || msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) {
                return;
            }

            {
                std::lock_guard<std::mutex> lock(slam_map_mutex_);
                latest_slam_map_ = std::make_shared<OccupancyGridMsgT>(*msg);
                ++slam_map_update_count_;
            }
            slam_map_cv_.notify_all();
        });

    action_to_pose_client_ = rclcpp_action::create_client<ActionToPose>(
        this, "/compute_path_to_pose");
    follow_path_client_ = rclcpp_action::create_client<ActionFollowPath>(
        this, "/follow_path");

    build_map_client_ = this->create_client<ReqAckSrvT>("/build_map");
    save_map_client_ = this->create_client<ReqAckSrvT>("/save_map");
    localization_control_client_ = this->create_client<ReqAckSrvT>("/localization_control");
    local_map_control_client_ = this->create_client<ReqAckSrvT>("/local_map_control");
    global_map_control_client_ = this->create_client<ReqAckSrvT>("/global_map_control");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    explore_map_server_ = std::make_unique<rf_util::SimpleActionServer<ActionExploreMap>>(
        shared_from_this(),
        "/explore_map",
        [this]() {
            this->handleExploreMap();
        });

    loop_thread_ = std::make_unique<std::thread>(&SchedulerNode::loop, this);
    loop_thread_->detach();
}

void SchedulerNode::loop()
{
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        handleGoalPoseMsg();
        rate.sleep();
    }
}

void SchedulerNode::handleGoalPoseMsg()
{
    if (exploration_active_.load()) {
        return;
    }

    PoseStampedT::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        if (!goal_pose_msg_) {
            return;
        }
        msg = std::move(goal_pose_msg_);
        preempt_requested_ = false;
    }

    SCHED_INFO("Received goal pose: {}, {}", msg->pose.position.x, msg->pose.position.y);

    std::string failure_reason;
    const auto status = navigateToPose(
        *msg,
        [this]() {
            return this->hasPendingPreempt() || this->exploration_active_.load();
        },
        &failure_reason);

    switch (status) {
        case NavigateStatus::SUCCEEDED:
            SCHED_INFO("Goal pose navigation succeeded.");
            break;
        case NavigateStatus::INTERRUPTED:
            SCHED_WARN("Goal pose navigation interrupted.");
            break;
        default:
            SCHED_WARN("Goal pose navigation failed: {}", failure_reason);
            break;
    }
}

void SchedulerNode::handleExploreMap()
{
    auto result = std::make_shared<ActionExploreMap::Result>();
    if (!explore_map_server_->getCurrentGoal()) {
        result->success = false;
        result->error_code = ActionExploreMap::Result::INTERNAL_ERROR;
        result->error_msg = "explore map goal is unavailable";
        explore_map_server_->terminateCurrent(result);
        return;
    }

    exploration_active_ = true;
    explored_frontiers_count_ = 0;

    {
        std::lock_guard<std::mutex> lock(suppressed_frontiers_mutex_);
        suppressed_frontiers_.clear();
    }

    const auto start_time = this->now();

    auto finish_action = [this, &result, &start_time](
                             bool succeeded,
                             uint16_t error_code,
                             std::string message,
                             const bool save_map_before_cleanup) {
        if (save_map_before_cleanup) {
            std::string finish_reason;
            if (!finishExplorationAndSave(&finish_reason)) {
                succeeded = false;
                error_code = ActionExploreMap::Result::SAVE_MAP_FAILED;
                message = finish_reason;
            }
            cleanupExploration(false);
        } else {
            cleanupExploration(true);
        }

        exploration_active_ = false;

        result->success = succeeded;
        result->elapsed_time = this->now() - start_time;
        result->explored_frontiers = explored_frontiers_count_;
        result->error_code = error_code;
        result->error_msg = message;

        if (succeeded) {
            explore_map_server_->succeededCurrent(result);
        } else {
            explore_map_server_->terminateCurrent(result);
        }
    };

    std::string prepare_reason;
    if (!prepareExploration(&prepare_reason)) {
        const uint16_t error_code = prepare_reason.find("/build_map") != std::string::npos
            ? ActionExploreMap::Result::BUILD_MAP_FAILED
            : ActionExploreMap::Result::SERVICE_UNAVAILABLE;
        finish_action(false, error_code, prepare_reason, false);
        return;
    }

    publishExploreFeedback("exploring", 0, explored_frontiers_count_);

    uint32_t navigation_failures = 0;
    std::optional<rclcpp::Time> no_frontier_since;

    while (rclcpp::ok()) {
        if (shouldInterruptExploration()) {
            finish_action(false, ActionExploreMap::Result::CANCELED, "exploration canceled", false);
            return;
        }

        const auto exploration_context = collectExplorationContext();

        if (exploration_context.frontier_candidates.empty()) {
            const bool has_frontier_clusters = exploration_context.frontier_cluster_count > 0;
            publishExploreFeedback(
                !exploration_context.map_seeded
                    ? "waiting_for_map"
                    : has_frontier_clusters ? "searching_goal" : "searching_frontier",
                0,
                explored_frontiers_count_);

            if (!exploration_context.map_seeded) {
                no_frontier_since.reset();
                if ((this->now() - start_time) >= rclcpp::Duration(kInitialMappingTimeout)) {
                    finish_action(
                        false,
                        ActionExploreMap::Result::BUILD_MAP_FAILED,
                        "Not enough mapped free space was produced to seed frontier exploration.",
                        false);
                    return;
                }
                rclcpp::sleep_for(kFrontierRetrySleep);
                continue;
            }

            if (!no_frontier_since.has_value()) {
                no_frontier_since = this->now();
            }

            if ((this->now() - *no_frontier_since) >= rclcpp::Duration(kNoFrontierStableDuration)) {
                if (explored_frontiers_count_ > 0) {
                    if (has_frontier_clusters) {
                        SCHED_WARN(
                            "Frontier clusters remain, but none produced a navigable goal for {} seconds. Finishing exploration.",
                            kNoFrontierStableDuration.count());
                    }
                    finish_action(true, ActionExploreMap::Result::NONE, "", true);
                    return;
                }

                if (has_frontier_clusters) {
                    finish_action(
                        false,
                        ActionExploreMap::Result::NAVIGATION_FAILED,
                        "Detected frontier clusters, but none produced a navigable goal.",
                        false);
                    return;
                }
            }

            rclcpp::sleep_for(kFrontierRetrySleep);
            continue;
        }

        no_frontier_since.reset();
        publishExploreFeedback(
            "navigating",
            exploration_context.frontier_candidates.size(),
            explored_frontiers_count_);

        std::string navigation_reason;
        switch (navigateFrontierCandidates(
            exploration_context.frontier_candidates,
            navigation_failures,
            &navigation_reason)) {
            case FrontierNavigationResult::REACHED_FRONTIER:
                publishExploreFeedback("settling", 0, explored_frontiers_count_);
                rclcpp::sleep_for(kFrontierSettleSleep);
                continue;
            case FrontierNavigationResult::CANCELED:
                finish_action(false, ActionExploreMap::Result::CANCELED, "exploration canceled", false);
                return;
            case FrontierNavigationResult::FAILURE_LIMIT_REACHED:
                finish_action(false, ActionExploreMap::Result::NAVIGATION_FAILED, navigation_reason, false);
                return;
            case FrontierNavigationResult::RETRY_LATER:
                SCHED_WARN("All frontier candidates failed in this cycle: {}", navigation_reason);
                rclcpp::sleep_for(kFrontierRetrySleep);
                continue;
        }
    }

    finish_action(
        false,
        ActionExploreMap::Result::INTERNAL_ERROR,
        "exploration loop exited unexpectedly",
        false);
}

SchedulerNode::NavigateStatus SchedulerNode::navigateToPose(
    const PoseStampedT& goal_pose,
    const std::function<bool()>& should_interrupt,
    std::string* failure_reason)
{
    using namespace std::chrono_literals;

    std::lock_guard<std::mutex> navigation_lock(navigation_mutex_);

    auto set_failure_reason = [failure_reason](const std::string& reason) {
        if (failure_reason != nullptr) {
            *failure_reason = reason;
        }
    };

    if (!action_to_pose_client_->wait_for_action_server(500ms)) {
        set_failure_reason("global planner action server is not available");
        return NavigateStatus::SERVER_UNAVAILABLE;
    }

    ActionToPose::Goal plan_goal;
    plan_goal.goal = goal_pose;

    auto future_goal_handle = action_to_pose_client_->async_send_goal(plan_goal);
    switch (waitForFuture(future_goal_handle, should_interrupt)) {
        case FutureWaitStatus::READY:
            break;
        case FutureWaitStatus::INTERRUPTED:
            cancelFutureGoalIfAccepted(future_goal_handle, [this](const auto& goal_handle) {
                action_to_pose_client_->async_cancel_goal(goal_handle);
            });
            cancelActiveNavigationGoals();
            set_failure_reason("navigation interrupted while waiting for planner goal acceptance");
            return NavigateStatus::INTERRUPTED;
        case FutureWaitStatus::SHUTDOWN:
            set_failure_reason("ROS shutdown while waiting for planner goal acceptance");
            return NavigateStatus::INTERRUPTED;
    }

    auto plan_goal_handle = future_goal_handle.get();
    if (!plan_goal_handle) {
        set_failure_reason("global planner rejected goal");
        return NavigateStatus::REJECTED;
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_plan_goal_handle_ = plan_goal_handle;
    }

    if (should_interrupt()) {
        cancelActiveNavigationGoals();
        set_failure_reason("navigation interrupted right after planner goal acceptance");
        return NavigateStatus::INTERRUPTED;
    }

    auto future_plan_result = action_to_pose_client_->async_get_result(plan_goal_handle);
    switch (waitForFuture(future_plan_result, should_interrupt)) {
        case FutureWaitStatus::READY:
            break;
        case FutureWaitStatus::INTERRUPTED:
            cancelActiveNavigationGoals();
            set_failure_reason("navigation interrupted while waiting for planner result");
            return NavigateStatus::INTERRUPTED;
        case FutureWaitStatus::SHUTDOWN:
            {
                std::lock_guard<std::mutex> lock(goal_pose_mutex_);
                current_plan_goal_handle_.reset();
            }
            set_failure_reason("ROS shutdown while waiting for planner result");
            return NavigateStatus::INTERRUPTED;
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_plan_goal_handle_.reset();
    }

    auto plan_result = future_plan_result.get();
    if (plan_result.code != rclcpp_action::ResultCode::SUCCEEDED || !plan_result.result) {
        set_failure_reason("global planner failed to produce a valid result");
        return NavigateStatus::PLANNER_FAILED;
    }

    if (plan_result.result->path.poses.empty()) {
        set_failure_reason("global planner returned an empty path");
        return NavigateStatus::PLANNER_FAILED;
    }

    if (should_interrupt()) {
        set_failure_reason("navigation interrupted before local tracking started");
        return NavigateStatus::INTERRUPTED;
    }

    if (!follow_path_client_->wait_for_action_server(500ms)) {
        set_failure_reason("local planner action server is not available");
        return NavigateStatus::SERVER_UNAVAILABLE;
    }

    ActionFollowPath::Goal follow_goal;
    follow_goal.path = plan_result.result->path;

    auto future_follow_handle = follow_path_client_->async_send_goal(follow_goal);
    switch (waitForFuture(future_follow_handle, should_interrupt)) {
        case FutureWaitStatus::READY:
            break;
        case FutureWaitStatus::INTERRUPTED:
            cancelFutureGoalIfAccepted(future_follow_handle, [this](const auto& goal_handle) {
                follow_path_client_->async_cancel_goal(goal_handle);
            });
            cancelActiveNavigationGoals();
            set_failure_reason("navigation interrupted while waiting for local planner goal acceptance");
            return NavigateStatus::INTERRUPTED;
        case FutureWaitStatus::SHUTDOWN:
            set_failure_reason("ROS shutdown while waiting for local planner goal acceptance");
            return NavigateStatus::INTERRUPTED;
    }

    auto follow_goal_handle = future_follow_handle.get();
    if (!follow_goal_handle) {
        set_failure_reason("local planner rejected the tracking goal");
        return NavigateStatus::REJECTED;
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_follow_goal_handle_ = follow_goal_handle;
    }

    if (should_interrupt()) {
        cancelActiveNavigationGoals();
        set_failure_reason("navigation interrupted right after local planner goal acceptance");
        return NavigateStatus::INTERRUPTED;
    }

    auto future_follow_result = follow_path_client_->async_get_result(follow_goal_handle);
    switch (waitForFuture(future_follow_result, should_interrupt)) {
        case FutureWaitStatus::READY:
            break;
        case FutureWaitStatus::INTERRUPTED:
            cancelActiveNavigationGoals();
            set_failure_reason("navigation interrupted while waiting for local planner result");
            return NavigateStatus::INTERRUPTED;
        case FutureWaitStatus::SHUTDOWN:
            {
                std::lock_guard<std::mutex> lock(goal_pose_mutex_);
                current_follow_goal_handle_.reset();
            }
            set_failure_reason("ROS shutdown while waiting for local planner result");
            return NavigateStatus::INTERRUPTED;
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_follow_goal_handle_.reset();
    }

    auto follow_result = future_follow_result.get();
    switch (follow_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return NavigateStatus::SUCCEEDED;
        case rclcpp_action::ResultCode::CANCELED:
            set_failure_reason("path tracking was canceled");
            return NavigateStatus::INTERRUPTED;
        case rclcpp_action::ResultCode::ABORTED:
            if (follow_result.result) {
                set_failure_reason("path tracking aborted: " + follow_result.result->error_msg);
            } else {
                set_failure_reason("path tracking aborted without a result payload");
            }
            return NavigateStatus::TRACKING_FAILED;
        default:
            set_failure_reason("path tracking finished with an unexpected result code");
            return NavigateStatus::TRACKING_FAILED;
    }
}

SchedulerNode::ExplorationContext SchedulerNode::collectExplorationContext() const
{
    ExplorationContext context;

    auto latest_map = getLatestSlamMap();
    if (!latest_map) {
        return context;
    }

    PoseStampedT robot_pose;
    PoseStampedT* robot_pose_ptr = nullptr;
    if (getRobotPoseInMap(robot_pose)) {
        robot_pose_ptr = &robot_pose;
    }

    context.map_seeded = countFreeCells(*latest_map) >= kMinimumSeedFreeCells;
    context.frontier_candidates = computeFrontierCandidates(
        *latest_map,
        robot_pose_ptr,
        &context.frontier_cluster_count,
        &context.suppressed_cluster_count,
        &context.rejected_cluster_count);
    return context;
}

SchedulerNode::FrontierNavigationResult SchedulerNode::navigateFrontierCandidates(
    const std::vector<FrontierCandidate>& frontier_candidates,
    uint32_t& navigation_failures,
    std::string* failure_reason)
{
    std::string last_navigation_reason = "frontier navigation failed";

    for (const auto& frontier_candidate : frontier_candidates) {
        std::string navigation_reason;
        const auto navigation_status = navigateToPose(
            frontier_candidate.goal_pose,
            [this]() {
                return this->shouldInterruptExploration();
            },
            &navigation_reason);

        if (navigation_status == NavigateStatus::SUCCEEDED) {
            ++explored_frontiers_count_;
            navigation_failures = 0;
            suppressGoalPose(frontier_candidate.goal_pose, kReachedGoalSuppressRadius);
            if (failure_reason != nullptr) {
                failure_reason->clear();
            }
            return FrontierNavigationResult::REACHED_FRONTIER;
        }

        if (navigation_status == NavigateStatus::INTERRUPTED) {
            if (failure_reason != nullptr) {
                *failure_reason = "exploration canceled";
            }
            return FrontierNavigationResult::CANCELED;
        }

        suppressGoalPose(frontier_candidate.goal_pose, kFailedGoalSuppressRadius);
        ++navigation_failures;
        last_navigation_reason = navigation_reason;
        SCHED_WARN(
            "Frontier navigation failed for candidate ({:.2f}, {:.2f}): {}",
            frontier_candidate.goal_pose.pose.position.x,
            frontier_candidate.goal_pose.pose.position.y,
            navigation_reason);

        if (navigation_failures >= kDefaultMaxFrontierFailures) {
            if (failure_reason != nullptr) {
                *failure_reason = navigation_reason;
            }
            return FrontierNavigationResult::FAILURE_LIMIT_REACHED;
        }
    }

    if (failure_reason != nullptr) {
        *failure_reason = last_navigation_reason;
    }
    return FrontierNavigationResult::RETRY_LATER;
}

bool SchedulerNode::prepareExploration(std::string* reason)
{
    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        goal_pose_msg_.reset();
        preempt_requested_ = false;
    }

    cancelActiveNavigationGoals();

    if (!callReqAckService(
            localization_control_client_,
            ReqAckSrvT::Request::STOP,
            "/localization_control",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!callReqAckService(
            local_map_control_client_,
            ReqAckSrvT::Request::START,
            "/local_map_control",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!callReqAckService(
            global_map_control_client_,
            ReqAckSrvT::Request::START,
            "/global_map_control",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    std::size_t map_update_count = 0;
    {
        std::lock_guard<std::mutex> lock(slam_map_mutex_);
        map_update_count = slam_map_update_count_;
    }

    if (!callReqAckService(
            build_map_client_,
            ReqAckSrvT::Request::START,
            "/build_map",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!waitForMapUpdate(kMapWarmupWait, map_update_count + 1)) {
        if (reason != nullptr) {
            *reason = "Timed out waiting for /slam_map updates after starting /build_map.";
        }
        return false;
    }

    return true;
}

void SchedulerNode::cleanupExploration(const bool stop_build_map)
{
    cancelActiveNavigationGoals();

    std::string ignored_reason;
    if (stop_build_map) {
        if (!callReqAckService(
                build_map_client_,
                ReqAckSrvT::Request::STOP,
                "/build_map",
                &ignored_reason,
                kDefaultServiceWait,
                kDefaultResponseWait)) {
            SCHED_WARN("Failed to stop /build_map during cleanup: {}", ignored_reason);
        }
    }

    if (!callReqAckService(
            global_map_control_client_,
            ReqAckSrvT::Request::STOP,
            "/global_map_control",
            &ignored_reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        SCHED_WARN("Failed to stop /global_map_control during cleanup: {}", ignored_reason);
    }

    if (!callReqAckService(
            local_map_control_client_,
            ReqAckSrvT::Request::STOP,
            "/local_map_control",
            &ignored_reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        SCHED_WARN("Failed to stop /local_map_control during cleanup: {}", ignored_reason);
    }
}

bool SchedulerNode::finishExplorationAndSave(std::string* reason)
{
    std::size_t map_update_count = 0;
    {
        std::lock_guard<std::mutex> lock(slam_map_mutex_);
        map_update_count = slam_map_update_count_;
    }

    if (!callReqAckService(
            build_map_client_,
            ReqAckSrvT::Request::STOP,
            "/build_map",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!waitForMapUpdate(kFinalMapWait, map_update_count + 1)) {
        SCHED_WARN("Timed out waiting for a final /slam_map update after stopping /build_map.");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    if (!callReqAckService(
            save_map_client_,
            ReqAckSrvT::Request::START,
            "/save_map",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    return true;
}

bool SchedulerNode::callReqAckService(
    const rclcpp::Client<ReqAckSrvT>::SharedPtr& client,
    const uint8_t trigger,
    const std::string& service_name,
    std::string* reason,
    const std::chrono::milliseconds wait_for_service,
    const std::chrono::milliseconds wait_for_response)
{
    if (!client) {
        if (reason != nullptr) {
            *reason = "Client for " + service_name + " is not initialized.";
        }
        return false;
    }

    if (!client->wait_for_service(wait_for_service)) {
        if (reason != nullptr) {
            *reason = "Service " + service_name + " is not available.";
        }
        return false;
    }

    auto request = std::make_shared<ReqAckSrvT::Request>();
    request->trigger = trigger;

    auto future = client->async_send_request(request);
    if (future.wait_for(wait_for_response) != std::future_status::ready) {
        if (reason != nullptr) {
            *reason = "Timed out waiting for response from " + service_name + ".";
        }
        return false;
    }

    const auto response = future.get();
    if (!response || response->ack != ReqAckSrvT::Response::OK) {
        if (reason != nullptr) {
            if (response && !response->reason.empty()) {
                *reason = response->reason;
            } else {
                *reason = "Service " + service_name + " returned a failure acknowledgement.";
            }
        }
        return false;
    }

    return true;
}

bool SchedulerNode::waitForMapUpdate(std::chrono::milliseconds timeout, const std::size_t min_update_count)
{
    std::unique_lock<std::mutex> lock(slam_map_mutex_);
    return slam_map_cv_.wait_for(lock, timeout, [this, min_update_count]() {
        return latest_slam_map_ != nullptr && slam_map_update_count_ >= min_update_count;
    });
}

std::shared_ptr<OccupancyGridMsgT> SchedulerNode::getLatestSlamMap() const
{
    std::lock_guard<std::mutex> lock(slam_map_mutex_);
    return latest_slam_map_;
}

bool SchedulerNode::getRobotPoseInMap(PoseStampedT& pose) const
{
    if (!tf_buffer_) {
        return false;
    }
    return rf_util::getCurrentPose(pose, *tf_buffer_);
}

std::vector<SchedulerNode::FrontierCandidate> SchedulerNode::computeFrontierCandidates(
    const OccupancyGridMsgT& map,
    const PoseStampedT* robot_pose,
    std::size_t* frontier_cluster_count_out,
    std::size_t* suppressed_cluster_count_out,
    std::size_t* rejected_cluster_count_out) const
{
    std::vector<FrontierCandidate> candidates;
    std::size_t frontier_cluster_count = 0;
    std::size_t suppressed_cluster_count = 0;
    std::size_t clusters_without_goal_count = 0;

    const auto width = static_cast<int>(map.info.width);
    const auto height = static_cast<int>(map.info.height);
    if (width <= 0 || height <= 0 || map.data.size() != static_cast<std::size_t>(width * height)) {
        return candidates;
    }

    const uint32_t min_cluster_size = kDefaultMinFrontierClusterSize;
    const int search_radius_cells = std::max(
        1,
        static_cast<int>(std::ceil(kGoalSearchRadiusMeters / map.info.resolution)));
    const int min_standoff_cells = std::max(
        1,
        static_cast<int>(std::ceil(kGoalMinStandoffMeters / map.info.resolution)));
    const int clearance_radius_cells = std::max(
        1,
        static_cast<int>(std::ceil(kGoalClearanceRadiusMeters / map.info.resolution)));
    const int known_radius_cells = std::max(
        1,
        static_cast<int>(std::round(kGoalKnownRadiusMeters / map.info.resolution)));

    const auto index_of = [width](const int x, const int y) {
        return y * width + x;
    };

    const auto world_x_of = [&map](const int x) {
        return map.info.origin.position.x + (static_cast<double>(x) + 0.5) * map.info.resolution;
    };

    const auto world_y_of = [&map](const int y) {
        return map.info.origin.position.y + (static_cast<double>(y) + 0.5) * map.info.resolution;
    };

    const auto has_unknown_neighbor = [&](const int x, const int y) {
        static constexpr int kNeighborDx[4] = {1, -1, 0, 0};
        static constexpr int kNeighborDy[4] = {0, 0, 1, -1};
        for (int i = 0; i < 4; ++i) {
            const int nx = x + kNeighborDx[i];
            const int ny = y + kNeighborDy[i];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                continue;
            }
            if (isUnknownCell(map.data[static_cast<std::size_t>(index_of(nx, ny))])) {
                return true;
            }
        }
        return false;
    };

    const auto is_frontier_cell = [&](const int x, const int y) {
        return isFreeCell(map.data[static_cast<std::size_t>(index_of(x, y))]) && has_unknown_neighbor(x, y);
    };

    const auto has_goal_clearance = [&](const int x, const int y, const bool strict) {
        for (int dy = -clearance_radius_cells; dy <= clearance_radius_cells; ++dy) {
            for (int dx = -clearance_radius_cells; dx <= clearance_radius_cells; ++dx) {
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    return false;
                }

                if ((dx * dx + dy * dy) > clearance_radius_cells * clearance_radius_cells) {
                    continue;
                }

                const auto value = map.data[static_cast<std::size_t>(index_of(nx, ny))];
                const bool in_known_core = (dx * dx + dy * dy) <= known_radius_cells * known_radius_cells;

                if (strict) {
                    // Prefer goals with a ring of known-free space around them.
                    if (isUnknownCell(value)) {
                        if (in_known_core) {
                            return false;
                        }
                        continue;
                    }

                    if (!isFreeCell(value)) {
                        return false;
                    }
                    continue;
                }

                // Relaxed fallback: allow inflation-cost cells and unknown on the
                // outer rim, but still require a known-free core and reject hard
                // obstacles anywhere in the clearance disk.
                if (isUnknownCell(value)) {
                    if (in_known_core) {
                        return false;
                    }
                    continue;
                }

                if (value > kGoalOccupiedThreshold) {
                    return false;
                }
            }
        }
        return true;
    };

    const auto estimate_clearance_cells = [&](const int x, const int y, const bool strict) {
        int clearance = 0;
        while (clearance < search_radius_cells) {
            const int radius = clearance + 1;
            bool blocked = false;
            for (int dy = -radius; dy <= radius && !blocked; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dy)) != radius) {
                        continue;
                    }
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                        blocked = true;
                        break;
                    }

                    const auto value = map.data[static_cast<std::size_t>(index_of(nx, ny))];
                    if (strict ? !isFreeCell(value) : isUnknownCell(value) || value > kGoalOccupiedThreshold) {
                        blocked = true;
                        break;
                    }
                }
            }

            if (blocked) {
                break;
            }
            ++clearance;
        }
        return clearance;
    };

    std::vector<bool> visited(static_cast<std::size_t>(width * height), false);
    static constexpr int kClusterDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static constexpr int kClusterDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t seed_index = static_cast<std::size_t>(index_of(x, y));
            if (visited[seed_index] || !is_frontier_cell(x, y)) {
                continue;
            }

            std::vector<std::pair<int, int>> cluster_cells;
            cluster_cells.reserve(32);
            std::queue<std::pair<int, int>> bfs_queue;
            bfs_queue.emplace(x, y);
            visited[seed_index] = true;

            double sum_x = 0.0;
            double sum_y = 0.0;

            while (!bfs_queue.empty()) {
                const auto [cx, cy] = bfs_queue.front();
                bfs_queue.pop();

                cluster_cells.emplace_back(cx, cy);
                sum_x += static_cast<double>(cx);
                sum_y += static_cast<double>(cy);

                for (int i = 0; i < 8; ++i) {
                    const int nx = cx + kClusterDx[i];
                    const int ny = cy + kClusterDy[i];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                        continue;
                    }

                    const std::size_t neighbor_index = static_cast<std::size_t>(index_of(nx, ny));
                    if (visited[neighbor_index] || !is_frontier_cell(nx, ny)) {
                        continue;
                    }

                    visited[neighbor_index] = true;
                    bfs_queue.emplace(nx, ny);
                }
            }

            if (cluster_cells.size() < min_cluster_size) {
                continue;
            }
            ++frontier_cluster_count;

            const double centroid_x = sum_x / static_cast<double>(cluster_cells.size());
            const double centroid_y = sum_y / static_cast<double>(cluster_cells.size());
            const double frontier_centroid_wx = world_x_of(static_cast<int>(std::round(centroid_x)));
            const double frontier_centroid_wy = world_y_of(static_cast<int>(std::round(centroid_y)));

            if (isFrontierSuppressed(frontier_centroid_wx, frontier_centroid_wy, true)) {
                ++suppressed_cluster_count;
                continue;
            }

            std::vector<bool> cluster_frontier_mask(static_cast<std::size_t>(width * height), false);
            for (const auto& cell : cluster_cells) {
                cluster_frontier_mask[static_cast<std::size_t>(index_of(cell.first, cell.second))] = true;
            }

            std::vector<bool> search_visited(static_cast<std::size_t>(width * height), false);
            std::queue<std::tuple<int, int, int>> search_queue;
            for (const auto& cell : cluster_cells) {
                for (int i = 0; i < 8; ++i) {
                    const int nx = cell.first + kClusterDx[i];
                    const int ny = cell.second + kClusterDy[i];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                        continue;
                    }
                    const std::size_t neighbor_index = static_cast<std::size_t>(index_of(nx, ny));
                    if (cluster_frontier_mask[neighbor_index] || search_visited[neighbor_index]) {
                        continue;
                    }
                    if (!isFreeCell(map.data[neighbor_index])) {
                        continue;
                    }

                    search_visited[neighbor_index] = true;
                    search_queue.emplace(nx, ny, 1);
                }
            }

            std::optional<std::pair<int, int>> best_goal_cell;
            double best_score = -std::numeric_limits<double>::infinity();
            double best_robot_distance = 0.0;

            auto evaluate_goal_cells = [&](const bool strict) {
                auto search_queue_copy = search_queue;
                auto search_visited_copy = search_visited;

                while (!search_queue_copy.empty()) {
                    const auto [cx, cy, depth] = search_queue_copy.front();
                    search_queue_copy.pop();

                    const double wx = world_x_of(cx);
                    const double wy = world_y_of(cy);

                    double robot_distance = 0.0;
                    if (robot_pose != nullptr) {
                        robot_distance = std::hypot(
                            wx - robot_pose->pose.position.x,
                            wy - robot_pose->pose.position.y);
                    }

                    if (depth >= min_standoff_cells &&
                        !isFrontierSuppressed(wx, wy) &&
                        has_goal_clearance(cx, cy, strict)) {
                        const int clearance_cells = estimate_clearance_cells(cx, cy, strict);
                        const double frontier_gain_m =
                            static_cast<double>(cluster_cells.size()) * map.info.resolution;
                        const double clearance_m =
                            static_cast<double>(clearance_cells) * map.info.resolution;
                        const double goal_to_frontier_distance_m =
                            static_cast<double>(depth) * map.info.resolution;
                        const double centroid_distance_m = std::hypot(
                            static_cast<double>(cx) - centroid_x,
                            static_cast<double>(cy) - centroid_y) * map.info.resolution;
                        const double standoff_penalty_m =
                            std::abs(goal_to_frontier_distance_m - kGoalPreferredStandoffMeters);

                        double score =
                            frontier_gain_m * 3.0 +
                            clearance_m * (strict ? 2.0 : 1.0) +
                            goal_to_frontier_distance_m * 0.8 -
                            standoff_penalty_m * (strict ? 1.5 : 0.9) -
                            centroid_distance_m * 0.6;

                        if (robot_pose != nullptr) {
                            score -= robot_distance * 0.35;
                        }

                        if (score > best_score) {
                            best_score = score;
                            best_goal_cell = std::make_pair(cx, cy);
                            best_robot_distance = robot_distance;
                        }
                    }

                    if (depth >= search_radius_cells) {
                        continue;
                    }

                    for (int i = 0; i < 8; ++i) {
                        const int nx = cx + kClusterDx[i];
                        const int ny = cy + kClusterDy[i];
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }

                        const std::size_t neighbor_index = static_cast<std::size_t>(index_of(nx, ny));
                        if (search_visited_copy[neighbor_index] || cluster_frontier_mask[neighbor_index]) {
                            continue;
                        }
                        if (!isFreeCell(map.data[neighbor_index])) {
                            continue;
                        }

                        search_visited_copy[neighbor_index] = true;
                        search_queue_copy.emplace(nx, ny, depth + 1);
                    }
                }
            };

            evaluate_goal_cells(true);
            if (!best_goal_cell.has_value()) {
                evaluate_goal_cells(false);
            }

            if (!best_goal_cell.has_value()) {
                ++clusters_without_goal_count;
                continue;
            }

            FrontierCandidate candidate;
            candidate.cell_count = cluster_cells.size();
            candidate.distance = best_robot_distance;
            candidate.goal_pose.header.frame_id = map.header.frame_id.empty() ? "map" : map.header.frame_id;
            candidate.goal_pose.header.stamp = map.header.stamp;
            candidate.goal_pose.pose.position.x = world_x_of(best_goal_cell->first);
            candidate.goal_pose.pose.position.y = world_y_of(best_goal_cell->second);
            candidate.goal_pose.pose.position.z = 0.0;
            candidate.frontier_x = frontier_centroid_wx;
            candidate.frontier_y = frontier_centroid_wy;

            const double yaw = std::atan2(
                frontier_centroid_wy - candidate.goal_pose.pose.position.y,
                frontier_centroid_wx - candidate.goal_pose.pose.position.x);
            candidate.goal_pose.pose.orientation = quaternionFromYaw(yaw);
            candidate.score = best_score;
            if (robot_pose == nullptr) {
                candidate.distance = 0.0;
            }

            candidates.push_back(candidate);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const FrontierCandidate& lhs, const FrontierCandidate& rhs) {
        if (lhs.score == rhs.score) {
            return lhs.distance < rhs.distance;
        }
        return lhs.score > rhs.score;
    });

    if (candidates.empty() && frontier_cluster_count > 0) {
        SCHED_WARN(
            "Detected {} frontier clusters, but none produced a navigable goal (suppressed clusters: {}, rejected clusters: {}).",
            frontier_cluster_count,
            suppressed_cluster_count,
            clusters_without_goal_count);
    }

    if (frontier_cluster_count_out != nullptr) {
        *frontier_cluster_count_out = frontier_cluster_count;
    }
    if (suppressed_cluster_count_out != nullptr) {
        *suppressed_cluster_count_out = suppressed_cluster_count;
    }
    if (rejected_cluster_count_out != nullptr) {
        *rejected_cluster_count_out = clusters_without_goal_count;
    }

    return candidates;
}

void SchedulerNode::suppressFrontier(const FrontierCandidate& frontier_candidate, const double radius)
{
    std::lock_guard<std::mutex> lock(suppressed_frontiers_mutex_);
    suppressed_frontiers_.push_back(
        SuppressedFrontier{
            frontier_candidate.frontier_x,
            frontier_candidate.frontier_y,
            radius,
            true});
}

void SchedulerNode::suppressGoalPose(const PoseStampedT& goal_pose, const double radius)
{
    std::lock_guard<std::mutex> lock(suppressed_frontiers_mutex_);
    suppressed_frontiers_.push_back(
        SuppressedFrontier{
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            radius,
            false});
}

bool SchedulerNode::isFrontierSuppressed(const double x, const double y, const bool cluster_only) const
{
    std::lock_guard<std::mutex> lock(suppressed_frontiers_mutex_);
    return std::any_of(
        suppressed_frontiers_.begin(),
        suppressed_frontiers_.end(),
        [x, y, cluster_only](const SuppressedFrontier& suppressed_frontier) {
            if (cluster_only && !suppressed_frontier.cluster_level) {
                return false;
            }
            const double radius_squared = suppressed_frontier.radius * suppressed_frontier.radius;
            return squaredDistance(
                suppressed_frontier.x,
                suppressed_frontier.y,
                x,
                y) <= radius_squared;
        });
}

bool SchedulerNode::shouldInterruptExploration() const
{
    return !rclcpp::ok() || explore_map_server_->isCancelRequested() || explore_map_server_->isPreemptRequested();
}

void SchedulerNode::cancelActiveNavigationGoals()
{
    rclcpp_action::ClientGoalHandle<ActionToPose>::SharedPtr plan_goal_handle;
    rclcpp_action::ClientGoalHandle<ActionFollowPath>::SharedPtr follow_goal_handle;
    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        plan_goal_handle = current_plan_goal_handle_;
        follow_goal_handle = current_follow_goal_handle_;
        current_plan_goal_handle_.reset();
        current_follow_goal_handle_.reset();
    }

    if (follow_goal_handle) {
        follow_path_client_->async_cancel_goal(follow_goal_handle);
    }

    if (plan_goal_handle) {
        action_to_pose_client_->async_cancel_goal(plan_goal_handle);
    }
}

void SchedulerNode::publishExploreFeedback(
    const std::string& state,
    const std::size_t frontier_candidates,
    const uint32_t explored_frontiers)
{
    auto feedback = std::make_shared<ActionExploreMap::Feedback>();
    feedback->state = state;
    feedback->frontier_candidates = static_cast<uint32_t>(frontier_candidates);
    feedback->explored_frontiers = explored_frontiers;
    explore_map_server_->publishFeedback(feedback);
}

bool SchedulerNode::hasPendingPreempt() const
{
    std::lock_guard<std::mutex> lock(goal_pose_mutex_);
    return preempt_requested_ && goal_pose_msg_ != nullptr;
}

} // namespace rf_scheduler
