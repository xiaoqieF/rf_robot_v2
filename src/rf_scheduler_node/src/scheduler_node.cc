#include "scheduler_node_internal.hpp"

#include "rclcpp/rate.hpp"
#include "rf_util/robot_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

#include <string>

namespace rf_scheduler
{

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

bool SchedulerNode::hasPendingPreempt() const
{
    std::lock_guard<std::mutex> lock(goal_pose_mutex_);
    return preempt_requested_ && goal_pose_msg_ != nullptr;
}

} // namespace rf_scheduler
