#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rf_robot_msgs/action/follow_path.hpp"
#include "rf_robot_msgs/msg/costmap.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"
#include "rf_util/simple_action_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

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
        double sim_time = 1.0;                  // Time horizon for simulating trajectories, in seconds
        double sim_granularity = 0.05;           // Time step for simulating trajectories, in seconds
        int vx_samples = 8;
        int vtheta_samples = 12;
        double max_vel_x = 0.5;
        double min_vel_x = 0.0;
        double max_vel_theta = 3.0;
        double min_vel_theta = -3.0;
        double acc_lim_x = 2.0;
        double acc_lim_theta = 10.0;
        double path_distance_bias = 0.5;
        double goal_distance_bias = 1.0;
        double obstacle_cost_bias = 5.0;
        double progress_bias = 15.0;
        double speed_bias = 0.5;
        double heading_bias = 0.5;
        double local_goal_distance = 0.3;       // Choose a point on the global path that is this distance away from the robot as the local goal
        double goal_tolerance_xy = 0.15;
        double goal_tolerance_yaw = 0.10;
        int max_no_control_cycles = 20;
        double no_progress_timeout = 6.0;
        double min_progress_distance = 0.10;
        bool publish_debug_trajectories = true;
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
        double obstacle_mean_cost = 0.0;
        double obstacle_peak_cost = 0.0;
        double speed_cost = 0.0;
        double heading_cost = 0.0;
        double progress_reward = 0.0;
        bool collision = true;
        std::vector<Pose2D> poses;
    };

private:
    void handleLocalCostmap(const CostmapMsgT::SharedPtr msg);
    void handleOdometry(const OdomMsgT::SharedPtr msg);
    void handleFollowPath();
    void publishZeroVelocity();
    void publishTrajectory(const TrajectorySample& sample, const std::string& frame_id);
    void publishDebugTrajectories(
        const std::vector<TrajectorySample>& samples,
        const TrajectorySample* best,
        const std::string& frame_id);
    void clearDebugTrajectories();
    void publishRotateToGoal(double yaw_error);
    Options getOptions() const;
    rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
        const std::vector<rclcpp::Parameter>& parameters);

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
        const geometry_msgs::msg::PoseStamped& local_goal,
        std::vector<TrajectorySample>* debug_samples = nullptr) const;
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
    double computePathProgress(
        const Pose2D& pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const;
    bool isCollision(const Pose2D& pose, const CostmapMsgT& costmap, double* max_cost = nullptr) const;
    bool worldToMap(
        const CostmapMsgT& costmap, double wx, double wy, unsigned int& mx, unsigned int& my) const;
    double normalizeAngle(double angle) const;
    double computeGoalYaw(const geometry_msgs::msg::PoseStamped& pose) const;

private:
    mutable std::mutex options_mutex_;
    Options options_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    rclcpp::Subscription<CostmapMsgT>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<OdomMsgT>::SharedPtr odom_sub_;
    rclcpp::Publisher<TwistMsgT>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<PathMsgT>::SharedPtr local_traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_traj_pub_;
    rf_util::SimpleActionServer<ActionFollowPath>::UniquePtr follow_path_server_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    mutable std::mutex data_mutex_;
    CostmapMsgT::SharedPtr local_costmap_msg_;
    OdomMsgT::SharedPtr odom_msg_;
    bool zero_cmd_published_ = false;
    bool debug_markers_active_ = false;
};

} // namespace rf_local_planner
