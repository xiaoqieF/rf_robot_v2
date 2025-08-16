#include "rf_scheduler_node/scheduler_node.hpp"
#include "rclcpp/rate.hpp"
#include "elog/elog.h"

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
    PoseStampedT::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        if (!goal_pose_msg_) {
            return;
        }
        msg = std::move(goal_pose_msg_);
    }

    SCHED_INFO("Received goal pose: {}, {}", msg->pose.position.x, msg->pose.position.y);

}

} // namespace rf_scheduler