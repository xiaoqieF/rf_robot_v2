#include "rf_scheduler_node/scheduler_node.hpp"

#include "elog/elog.h"
#include "rclcpp/rate.hpp"

#include <chrono>
#include <future>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>

namespace rf_scheduler
{

#define SCHED_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_scheduler_node] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define SCHED_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_scheduler_node] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define SCHED_ERROR(fmt_str, ...) \
    do { \
        elog::error("[rf_scheduler_node] " fmt_str, ##__VA_ARGS__); \
    } while(0)

void SchedulerNode::init()
{
    goal_pose_sub_ = this->create_subscription<PoseStampedT>(
        "/goal_pose",
        10,
        [this](const PoseStampedT::SharedPtr msg) {
            rclcpp_action::ClientGoalHandle<ActionToPose>::SharedPtr plan_goal_handle;
            rclcpp_action::ClientGoalHandle<ActionFollowPath>::SharedPtr follow_goal_handle;
            {
                std::lock_guard<std::mutex> lock(goal_pose_mutex_);
                goal_pose_msg_ = msg;
                preempt_requested_ = true;
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
        });

    action_to_pose_client_ = rclcpp_action::create_client<ActionToPose>(
        this, "/compute_path_to_pose");

    follow_path_client_ = rclcpp_action::create_client<ActionFollowPath>(
        this, "/follow_path");

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
    using namespace std::chrono_literals;

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

    if (!action_to_pose_client_->wait_for_action_server(500ms)) {
        SCHED_WARN("Global planner action server not available");
        return;
    }

    ActionToPose::Goal plan_goal;
    plan_goal.goal.pose = msg->pose;
    plan_goal.goal.header = msg->header;

    auto future_goal_handle = action_to_pose_client_->async_send_goal(plan_goal);
    while (rclcpp::ok() && future_goal_handle.wait_for(100ms) != std::future_status::ready) {
        if (hasPendingPreempt()) {
            SCHED_INFO("Preempting while waiting for global planner goal acceptance");
            return;
        }
    }

    if (!rclcpp::ok()) {
        return;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
        SCHED_WARN("Global planner rejected goal");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_plan_goal_handle_ = goal_handle;
    }

    if (hasPendingPreempt()) {
        SCHED_INFO("Preempting global planning goal right after acceptance");
        action_to_pose_client_->async_cancel_goal(goal_handle);
        {
            std::lock_guard<std::mutex> lock(goal_pose_mutex_);
            current_plan_goal_handle_.reset();
        }
        return;
    }

    auto future_result = action_to_pose_client_->async_get_result(goal_handle);
    while (rclcpp::ok() && future_result.wait_for(100ms) != std::future_status::ready) {
        if (hasPendingPreempt()) {
            SCHED_INFO("Preempting while waiting for global planner result");
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_plan_goal_handle_.reset();
    }

    if (!rclcpp::ok()) {
        return;
    }

    auto plan_result = future_result.get();
    if (plan_result.code != rclcpp_action::ResultCode::SUCCEEDED || !plan_result.result) {
        SCHED_WARN("Global planner failed, result code: {}", static_cast<int>(plan_result.code));
        return;
    }

    if (plan_result.result->path.poses.empty()) {
        SCHED_WARN("Global planner returned an empty path");
        return;
    }

    SCHED_INFO("Global planner produced a path with {} poses", plan_result.result->path.poses.size());

    if (hasPendingPreempt()) {
        SCHED_INFO("Discarding planned path because a newer goal arrived");
        return;
    }

    if (!follow_path_client_->wait_for_action_server(500ms)) {
        SCHED_WARN("Local planner action server not available");
        return;
    }

    ActionFollowPath::Goal follow_goal;
    follow_goal.path = plan_result.result->path;

    auto future_follow_handle = follow_path_client_->async_send_goal(follow_goal);
    while (rclcpp::ok() && future_follow_handle.wait_for(100ms) != std::future_status::ready) {
        if (hasPendingPreempt()) {
            SCHED_INFO("Preempting while waiting for local planner goal acceptance");
            return;
        }
    }

    if (!rclcpp::ok()) {
        return;
    }

    auto follow_goal_handle = future_follow_handle.get();
    if (!follow_goal_handle) {
        SCHED_WARN("Local planner rejected path tracking goal");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_follow_goal_handle_ = follow_goal_handle;
    }

    if (hasPendingPreempt()) {
        SCHED_INFO("Preempting local tracking goal right after acceptance");
        follow_path_client_->async_cancel_goal(follow_goal_handle);
        {
            std::lock_guard<std::mutex> lock(goal_pose_mutex_);
            current_follow_goal_handle_.reset();
        }
        return;
    }

    auto future_follow_result = follow_path_client_->async_get_result(follow_goal_handle);
    while (rclcpp::ok() && future_follow_result.wait_for(100ms) != std::future_status::ready) {
        if (hasPendingPreempt()) {
            SCHED_INFO("Preempting while waiting for local planner result");
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        current_follow_goal_handle_.reset();
    }

    if (!rclcpp::ok()) {
        return;
    }

    auto follow_result = future_follow_result.get();
    switch (follow_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            SCHED_INFO("Path tracking succeeded");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            SCHED_WARN("Path tracking canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            if (follow_result.result) {
                SCHED_WARN("Path tracking aborted, error code: {}, reason: {}",
                    follow_result.result->error_code,
                    follow_result.result->error_msg);
            } else {
                SCHED_WARN("Path tracking aborted without result payload");
            }
            break;
        default:
            SCHED_WARN("Path tracking finished with unexpected result code: {}",
                static_cast<int>(follow_result.code));
            break;
    }
}

bool SchedulerNode::hasPendingPreempt() const
{
    std::lock_guard<std::mutex> lock(goal_pose_mutex_);
    return preempt_requested_ && goal_pose_msg_ != nullptr;
}

} // namespace rf_scheduler
