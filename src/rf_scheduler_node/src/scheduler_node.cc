#include "rf_scheduler_node/scheduler_node.hpp"
#include "rclcpp/rate.hpp"
#include "elog/elog.h"
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
            std::lock_guard<std::mutex> lock(goal_pose_mutex_);
            goal_pose_msg_ = msg;
        });

    action_to_pose_client_ = rclcpp_action::create_client<ActionToPose>(
        this, "/compute_path_to_pose");

    action_through_poses_client_ = rclcpp_action::create_client<ActionThroughPoses>(
        this, "/compute_path_through_poses");

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
    }

    SCHED_INFO("Received goal pose: {}, {}", msg->pose.position.x, msg->pose.position.y);

    if (!action_to_pose_client_->wait_for_action_server(10ms)) {
        SCHED_WARN("Action server not available");
        return;
    }

    ActionToPose::Goal goal;
    goal.goal.pose = msg->pose;
    goal.goal.header = msg->header;

    auto future_goal_handle = action_to_pose_client_->async_send_goal(goal);
    auto result = future_goal_handle.wait_for(100ms);

    if (result != std::future_status::ready) {
        SCHED_WARN("Goal accepted timeout");
        return;
    }

    auto future_result = action_to_pose_client_->async_get_result(future_goal_handle.get());
    result = future_result.wait_for(1s);

    if (result != std::future_status::ready) {
        SCHED_WARN("Goal result timeout");
        return;
    }

    auto result_msg = future_result.get();

    switch (result_msg.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            SCHED_INFO("Goal succeeded");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            SCHED_INFO("Goal canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            SCHED_WARN("Goal aborted, error code: {}", result_msg.result->error_code);
            break;
        default:
            break;
    }
}

} // namespace rf_scheduler