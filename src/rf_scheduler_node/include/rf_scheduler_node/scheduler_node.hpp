#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rf_robot_msgs/action/compute_path_to_pose.hpp"
#include "rf_robot_msgs/action/follow_path.hpp"

#include <memory>
#include <mutex>
#include <thread>

namespace rf_scheduler
{

using PoseStampedT = geometry_msgs::msg::PoseStamped;
using ActionToPose = rf_robot_msgs::action::ComputePathToPose;
using ActionFollowPath = rf_robot_msgs::action::FollowPath;

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
    bool hasPendingPreempt() const;

private:
    rclcpp::Subscription<PoseStampedT>::SharedPtr goal_pose_sub_;

    mutable std::mutex goal_pose_mutex_;
    PoseStampedT::SharedPtr goal_pose_msg_{nullptr};
    bool preempt_requested_{false};

    rclcpp_action::Client<ActionToPose>::SharedPtr action_to_pose_client_{nullptr};
    rclcpp_action::Client<ActionFollowPath>::SharedPtr follow_path_client_{nullptr};
    rclcpp_action::ClientGoalHandle<ActionToPose>::SharedPtr current_plan_goal_handle_{nullptr};
    rclcpp_action::ClientGoalHandle<ActionFollowPath>::SharedPtr current_follow_goal_handle_{nullptr};

    std::unique_ptr<std::thread> loop_thread_;
};

} // namespace rf_scheduler
