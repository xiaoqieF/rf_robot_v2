#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rf_robot_msgs/msg/costmap.hpp"
#include "rf_robot_msgs/action/compute_path_to_pose.hpp"
#include "rf_robot_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rf_util/simple_action_server.hpp"
#include "rf_global_planner/common.hpp"
#include "rf_global_planner/planner/global_planner.hpp"
#include <memory>

namespace rf_global_planner
{

using CostmapMsgT = rf_robot_msgs::msg::Costmap;
using ActionToPose = rf_robot_msgs::action::ComputePathToPose;
using ActionThroughPoses = rf_robot_msgs::action::ComputePathThroughPoses;

class GlobalPlannerNode : public rclcpp::Node
{
public:
    GlobalPlannerNode() : Node("global_planner_node") {}

    void init();

private:
    void publishPlan(const nav_msgs::msg::Path& path);
    void handleComputePathToPose();
    void handleComputePathThroughPoses();
    bool transformPoseToGlobalFrame(
        geometry_msgs::msg::PoseStamped& pose);
    nav_msgs::msg::Path getPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const std::string& planner_name);


private:
    rclcpp::Subscription<CostmapMsgT>::SharedPtr global_map_sub_;
    rf_util::SimpleActionServer<ActionToPose>::UniquePtr action_to_pose_server_;
    rf_util::SimpleActionServer<ActionThroughPoses>::UniquePtr action_through_poses_server_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    std::mutex global_costmap_mutex_;
    CostmapMsgT::SharedPtr global_costmap_msg_;

    std::map<std::string, std::unique_ptr<GlobalPlanner>> planners_;
};

} // namespace rf_global_planner