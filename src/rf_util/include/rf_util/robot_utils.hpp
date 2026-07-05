#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"

namespace rf_util
{

bool getCurrentPose(
    geometry_msgs::msg::PoseStamped& global_pose,
    tf2_ros::Buffer& tf_buffer,
    const std::string& global_frame = "map",
    const std::string& robot_frame = "base_link",
    const double timeout = 0.1,
    const rclcpp::Time stamp = rclcpp::Time());

bool transformPoseInTargetFrame(
    const geometry_msgs::msg::PoseStamped& pose_in,
    geometry_msgs::msg::PoseStamped& pose_out,
    tf2_ros::Buffer& tf_buffer,
    const std::string& target_frame,
    const double timeout = 0.1);

} // namespace rf_util