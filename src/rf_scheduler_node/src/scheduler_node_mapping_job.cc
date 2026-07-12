#include "scheduler_node_internal.hpp"

#include <chrono>
#include <optional>
#include <string>

namespace rf_scheduler::detail
{

MappingJob::MappingJob(SchedulerNode& node)
    : node_(node){}

void MappingJob::run()
{
    result_ = std::make_shared<ActionExploreMap::Result>();
    if (!node_.explore_map_server_->getCurrentGoal()) {
        result_->success = false;
        result_->error_code = ActionExploreMap::Result::INTERNAL_ERROR;
        result_->error_msg = "explore map goal is unavailable";
        node_.explore_map_server_->terminateCurrent(result_);
        return;
    }

    node_.exploration_active_ = true;
    node_.explored_frontiers_count_ = 0;
    resetSuppressedFrontiers();
    start_time_ = node_.now();

    std::string prepare_reason;
    if (!prepare(&prepare_reason)) {
        const uint16_t error_code = prepare_reason.find("/build_map") != std::string::npos
            ? ActionExploreMap::Result::BUILD_MAP_FAILED
            : ActionExploreMap::Result::SERVICE_UNAVAILABLE;
        finish(false, error_code, prepare_reason, false);
        return;
    }

    publishFeedback("exploring", 0);

    uint32_t navigation_failures = 0;
    std::optional<rclcpp::Time> no_frontier_since;

    while (rclcpp::ok()) {
        if (shouldInterrupt()) {
            finish(false, ActionExploreMap::Result::CANCELED, "exploration canceled", false);
            return;
        }

        const auto exploration_context = collectContext();
        if (exploration_context.frontier_candidates.empty()) {
            const bool has_frontier_clusters = exploration_context.frontier_cluster_count > 0;
            publishFeedback(
                !exploration_context.map_seeded
                    ? "waiting_for_map"
                    : has_frontier_clusters ? "searching_goal" : "searching_frontier",
                0);

            if (!exploration_context.map_seeded) {
                no_frontier_since.reset();
                if ((node_.now() - start_time_) >= rclcpp::Duration(kInitialMappingTimeout)) {
                    finish(
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
                no_frontier_since = node_.now();
            }

            if ((node_.now() - *no_frontier_since) >= rclcpp::Duration(kNoFrontierStableDuration)) {
                if (node_.explored_frontiers_count_ > 0) {
                    if (has_frontier_clusters) {
                        SCHED_WARN(
                            "Frontier clusters remain, but none produced a navigable goal for {} seconds. Finishing exploration.",
                            kNoFrontierStableDuration.count());
                    }
                    finish(true, ActionExploreMap::Result::NONE, "", true);
                    return;
                }

                if (has_frontier_clusters) {
                    finish(
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
        publishFeedback("navigating", exploration_context.frontier_candidates.size());

        std::string navigation_reason;
        switch (navigateFrontierCandidates(
            exploration_context.frontier_candidates,
            navigation_failures,
            &navigation_reason)) {
            case FrontierNavigationResult::REACHED_FRONTIER:
                continue;
            case FrontierNavigationResult::CANCELED:
                finish(false, ActionExploreMap::Result::CANCELED, "exploration canceled", false);
                return;
            case FrontierNavigationResult::FAILURE_LIMIT_REACHED:
                finish(false, ActionExploreMap::Result::NAVIGATION_FAILED, navigation_reason, false);
                return;
            case FrontierNavigationResult::RETRY_LATER:
                SCHED_WARN("All frontier candidates failed in this cycle: {}", navigation_reason);
                rclcpp::sleep_for(kFrontierRetrySleep);
                continue;
        }
    }

    finish(
        false,
        ActionExploreMap::Result::INTERNAL_ERROR,
        "exploration loop exited unexpectedly",
        false);
}

void MappingJob::finish(
    bool succeeded,
    uint16_t error_code,
    std::string message,
    const bool save_map_before_cleanup)
{
    if (save_map_before_cleanup) {
        std::string finish_reason;
        if (!finishAndSave(&finish_reason)) {
            succeeded = false;
            error_code = ActionExploreMap::Result::SAVE_MAP_FAILED;
            message = finish_reason;
        }
        cleanup(false);
    } else {
        cleanup(true);
    }

    node_.exploration_active_ = false;

    result_->success = succeeded;
    result_->elapsed_time = node_.now() - start_time_;
    result_->explored_frontiers = node_.explored_frontiers_count_;
    result_->error_code = error_code;
    result_->error_msg = std::move(message);

    if (succeeded) {
        node_.explore_map_server_->succeededCurrent(result_);
    } else {
        node_.explore_map_server_->terminateCurrent(result_);
    }
}

void MappingJob::resetSuppressedFrontiers()
{
    std::lock_guard<std::mutex> lock(node_.suppressed_frontiers_mutex_);
    node_.suppressed_frontiers_.clear();
}

void MappingJob::publishFeedback(const std::string& state, const std::size_t frontier_candidates) const
{
    auto feedback = std::make_shared<ActionExploreMap::Feedback>();
    feedback->state = state;
    feedback->frontier_candidates = static_cast<uint32_t>(frontier_candidates);
    feedback->explored_frontiers = node_.explored_frontiers_count_;
    node_.explore_map_server_->publishFeedback(feedback);
}

bool MappingJob::shouldInterrupt() const
{
    return !rclcpp::ok() || node_.explore_map_server_->isCancelRequested() || node_.explore_map_server_->isPreemptRequested();
}

bool MappingJob::prepare(std::string* reason)
{
    {
        std::lock_guard<std::mutex> lock(node_.goal_pose_mutex_);
        node_.goal_pose_msg_.reset();
        node_.preempt_requested_ = false;
    }

    node_.cancelActiveNavigationGoals();

    if (!node_.callReqAckService(
            node_.localization_control_client_,
            ReqAckSrvT::Request::STOP,
            "/localization_control",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!node_.callReqAckService(
            node_.local_map_control_client_,
            ReqAckSrvT::Request::START,
            "/local_map_control",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!node_.callReqAckService(
            node_.global_map_control_client_,
            ReqAckSrvT::Request::START,
            "/global_map_control",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    std::size_t map_update_count = 0;
    {
        std::lock_guard<std::mutex> lock(node_.slam_map_mutex_);
        map_update_count = node_.slam_map_update_count_;
    }

    if (!node_.callReqAckService(
            node_.build_map_client_,
            ReqAckSrvT::Request::START,
            "/build_map",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!node_.waitForMapUpdate(kMapWarmupWait, map_update_count + 1)) {
        if (reason != nullptr) {
            *reason = "Timed out waiting for /slam_map updates after starting /build_map.";
        }
        return false;
    }

    return true;
}

void MappingJob::cleanup(const bool stop_build_map)
{
    node_.cancelActiveNavigationGoals();

    std::string ignored_reason;
    if (stop_build_map) {
        if (!node_.callReqAckService(
                node_.build_map_client_,
                ReqAckSrvT::Request::STOP,
                "/build_map",
                &ignored_reason,
                kDefaultServiceWait,
                kDefaultResponseWait)) {
            SCHED_WARN("Failed to stop /build_map during cleanup: {}", ignored_reason);
        }
    }

    if (!node_.callReqAckService(
            node_.global_map_control_client_,
            ReqAckSrvT::Request::STOP,
            "/global_map_control",
            &ignored_reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        SCHED_WARN("Failed to stop /global_map_control during cleanup: {}", ignored_reason);
    }

    if (!node_.callReqAckService(
            node_.local_map_control_client_,
            ReqAckSrvT::Request::STOP,
            "/local_map_control",
            &ignored_reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        SCHED_WARN("Failed to stop /local_map_control during cleanup: {}", ignored_reason);
    }
}

bool MappingJob::finishAndSave(std::string* reason)
{
    std::size_t map_update_count = 0;
    {
        std::lock_guard<std::mutex> lock(node_.slam_map_mutex_);
        map_update_count = node_.slam_map_update_count_;
    }

    if (!node_.callReqAckService(
            node_.build_map_client_,
            ReqAckSrvT::Request::STOP,
            "/build_map",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    if (!node_.waitForMapUpdate(kFinalMapWait, map_update_count + 1)) {
        SCHED_WARN("Timed out waiting for a final /slam_map update after stopping /build_map.");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    if (!node_.callReqAckService(
            node_.save_map_client_,
            ReqAckSrvT::Request::START,
            "/save_map",
            reason,
            kDefaultServiceWait,
            kDefaultResponseWait)) {
        return false;
    }

    return true;
}

MappingJob::ExplorationContext MappingJob::collectContext() const
{
    ExplorationContext context;

    auto latest_map = node_.getLatestSlamMap();
    if (!latest_map) {
        return context;
    }

    PoseStampedT robot_pose;
    PoseStampedT* robot_pose_ptr = nullptr;
    if (node_.getRobotPoseInMap(robot_pose)) {
        robot_pose_ptr = &robot_pose;
    }

    context.map_seeded = countFreeCells(*latest_map) >= kMinimumSeedFreeCells;
    context.frontier_candidates = node_.computeFrontierCandidates(
        *latest_map,
        robot_pose_ptr,
        &context.frontier_cluster_count,
        &context.suppressed_cluster_count,
        &context.rejected_cluster_count);
    return context;
}

MappingJob::FrontierNavigationResult MappingJob::navigateFrontierCandidates(
    const std::vector<SchedulerNode::FrontierCandidate>& frontier_candidates,
    uint32_t& navigation_failures,
    std::string* failure_reason)
{
    std::string last_navigation_reason = "frontier navigation failed";

    for (const auto& frontier_candidate : frontier_candidates) {
        std::string navigation_reason;
        const auto navigation_status = node_.navigateToPose(
            frontier_candidate.goal_pose,
            [this]() {
                return this->shouldInterrupt();
            },
            &navigation_reason);

        if (navigation_status == SchedulerNode::NavigateStatus::SUCCEEDED) {
            ++node_.explored_frontiers_count_;
            navigation_failures = 0;
            node_.suppressFrontier(frontier_candidate, kReachedFrontierSuppressRadius);
            if (failure_reason != nullptr) {
                failure_reason->clear();
            }
            return FrontierNavigationResult::REACHED_FRONTIER;
        }

        if (navigation_status == SchedulerNode::NavigateStatus::INTERRUPTED) {
            if (failure_reason != nullptr) {
                *failure_reason = "exploration canceled";
            }
            return FrontierNavigationResult::CANCELED;
        }

        node_.suppressGoalPose(frontier_candidate.goal_pose, kFailedGoalSuppressRadius);
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

} // namespace rf_scheduler::detail
