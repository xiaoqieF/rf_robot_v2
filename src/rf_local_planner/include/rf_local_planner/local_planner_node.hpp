#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rf_robot_msgs/action/follow_path.hpp"
#include "rf_robot_msgs/msg/costmap.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"
#include "rf_util/simple_action_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

namespace rf_local_planner
{

using CostmapMsgT = rf_robot_msgs::msg::Costmap;
using PathMsgT = nav_msgs::msg::Path;
using OdomMsgT = nav_msgs::msg::Odometry;
using TwistMsgT = geometry_msgs::msg::Twist;
using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;
using ActionFollowPath = rf_robot_msgs::action::FollowPath;

class LocalPlannerNode : public rclcpp::Node
{
public:
    LocalPlannerNode();

    void init();

private:
    struct Options
    {
        double control_frequency = 10.0;
        double sim_time = 1.5;
        double sim_granularity = 0.1;
        int vx_samples = 6;
        int vtheta_samples = 15;
        double max_vel_x = 0.35;
        double min_vel_x = 0.0;
        double max_vel_theta = 1.2;
        double min_vel_theta = -1.2;
        double acc_lim_x = 0.6;
        double acc_lim_theta = 1.8;
        double robot_radius = 0.18;
        double path_distance_bias = 2.0;
        double goal_distance_bias = 2.5;
        double obstacle_cost_bias = 3.0;
        double speed_bias = 0.4;
        double heading_bias = 0.8;
        double local_goal_distance = 1.2;
        double goal_tolerance_xy = 0.15;
        double goal_tolerance_yaw = 0.20;
        int max_no_control_cycles = 20;
        bool autostart = true;
    };

    struct Pose2D
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct TrajectorySample
    {
        double linear_vel = 0.0;
        double angular_vel = 0.0;
        double total_cost = std::numeric_limits<double>::infinity();
        double path_cost = 0.0;
        double goal_cost = 0.0;
        double obstacle_cost = 0.0;
        double speed_cost = 0.0;
        double heading_cost = 0.0;
        bool collision = true;
        std::vector<Pose2D> poses;
    };

private:
    void handleLocalCostmap(const CostmapMsgT::SharedPtr msg);
    void handleOdometry(const OdomMsgT::SharedPtr msg);
    void handleControlRequest(const ReqAckSrvT::Request::SharedPtr request,
                              const ReqAckSrvT::Response::SharedPtr response);
    void handleFollowPath();
    void publishZeroVelocity();
    void publishTrajectory(const TrajectorySample& sample, const std::string& frame_id);
    void publishRotateToGoal(double yaw_error);

    bool getRobotPose(Pose2D& pose);
    std::vector<geometry_msgs::msg::PoseStamped> prunePlan(
        const PathMsgT& plan, const Pose2D& robot_pose) const;
    geometry_msgs::msg::PoseStamped selectLocalGoal(
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const;
    TrajectorySample findBestTrajectory(
        const Pose2D& robot_pose,
        const OdomMsgT& odom,
        const CostmapMsgT& costmap,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment,
        const geometry_msgs::msg::PoseStamped& local_goal) const;
    TrajectorySample simulateTrajectory(
        const Pose2D& start_pose,
        double linear_vel,
        double angular_vel,
        const CostmapMsgT& costmap,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment,
        const geometry_msgs::msg::PoseStamped& local_goal) const;

    double computePathDistance(
        const Pose2D& pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const;
    bool isCollision(const Pose2D& pose, const CostmapMsgT& costmap, double* max_cost = nullptr) const;
    bool worldToMap(
        const CostmapMsgT& costmap, double wx, double wy, unsigned int& mx, unsigned int& my) const;
    double normalizeAngle(double angle) const;
    double computeGoalYaw(const geometry_msgs::msg::PoseStamped& pose) const;

private:
    Options options_;

    rclcpp::Subscription<CostmapMsgT>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<OdomMsgT>::SharedPtr odom_sub_;
    rclcpp::Publisher<TwistMsgT>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<PathMsgT>::SharedPtr local_traj_pub_;
    rclcpp::Service<ReqAckSrvT>::SharedPtr control_service_;
    rf_util::SimpleActionServer<ActionFollowPath>::UniquePtr follow_path_server_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    mutable std::mutex data_mutex_;
    CostmapMsgT::SharedPtr local_costmap_msg_;
    OdomMsgT::SharedPtr odom_msg_;
    bool active_ = true;
    bool zero_cmd_published_ = false;
};

} // namespace rf_local_planner
