#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rf_robot_msgs/action/compute_path_to_pose.hpp"
#include "rf_robot_msgs/action/compute_path_through_poses.hpp"

#include <memory>
#include <thread>

namespace rf_scheduler
{

using PoseStampedT = geometry_msgs::msg::PoseStamped;
using ActionToPose = rf_robot_msgs::action::ComputePathToPose;
using ActionThroughPoses = rf_robot_msgs::action::ComputePathThroughPoses;

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

    rclcpp_action::Client<ActionToPose>::SharedPtr action_to_pose_client_{nullptr};
    rclcpp_action::Client<ActionThroughPoses>::SharedPtr action_through_poses_client_{nullptr};

    std::unique_ptr<std::thread> loop_thread_;
};

} // namespace rf_scheduler