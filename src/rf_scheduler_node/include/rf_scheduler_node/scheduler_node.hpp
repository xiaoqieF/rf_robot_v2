#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <thread>

namespace rf_scheduler
{

using PoseStampedT = geometry_msgs::msg::PoseStamped;

class SchedulerNode : public rclcpp::Node
{
public:
    SchedulerNode()
        : Node("rf_scheduler_node"){}

    void init();

private:
    void loop();
    void handleGoalPoseMsg();

private:
    rclcpp::Subscription<PoseStampedT>::SharedPtr goal_pose_sub_;

    std::mutex goal_pose_mutex_;
    PoseStampedT::SharedPtr goal_pose_msg_{nullptr};

    std::unique_ptr<std::thread> loop_thread_;
};

} // namespace rf_scheduler