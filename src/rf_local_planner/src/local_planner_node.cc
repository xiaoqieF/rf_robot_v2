#include "rf_local_planner/local_planner_node.hpp"

#include "rf_costmap/cost_values.hpp"
#include "rf_local_planner/common.hpp"
#include "rf_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace rf_local_planner
{

namespace
{

constexpr double kInscribedRecoverySpeedMetersPerSecond = 0.08;
constexpr double kInscribedRecoveryMaxDistanceMeters = 0.30;
constexpr double kInscribedRecoveryTimeoutSeconds = 5.0;
constexpr int kMaxInscribedRecoveryAttempts = 2;

} // namespace

LocalPlannerNode::LocalPlannerNode()
    : Node("local_planner_node")
{
    options_.control_frequency = this->declare_parameter<double>("control_frequency", options_.control_frequency);
    options_.sim_time = this->declare_parameter<double>("sim_time", options_.sim_time);
    options_.sim_granularity = this->declare_parameter<double>("sim_granularity", options_.sim_granularity);
    options_.vx_samples = this->declare_parameter<int>("vx_samples", options_.vx_samples);
    options_.vtheta_samples = this->declare_parameter<int>("vtheta_samples", options_.vtheta_samples);
    options_.max_vel_x = this->declare_parameter<double>("max_vel_x", options_.max_vel_x);
    options_.min_vel_x = this->declare_parameter<double>("min_vel_x", options_.min_vel_x);
    options_.max_vel_theta = this->declare_parameter<double>("max_vel_theta", options_.max_vel_theta);
    options_.min_vel_theta = this->declare_parameter<double>("min_vel_theta", options_.min_vel_theta);
    options_.acc_lim_x = this->declare_parameter<double>("acc_lim_x", options_.acc_lim_x);
    options_.acc_lim_theta = this->declare_parameter<double>("acc_lim_theta", options_.acc_lim_theta);
    options_.path_distance_bias = this->declare_parameter<double>("path_distance_bias", options_.path_distance_bias);
    options_.goal_distance_bias = this->declare_parameter<double>("goal_distance_bias", options_.goal_distance_bias);
    options_.obstacle_cost_bias = this->declare_parameter<double>("obstacle_cost_bias", options_.obstacle_cost_bias);
    options_.progress_bias = this->declare_parameter<double>("progress_bias", options_.progress_bias);
    options_.speed_bias = this->declare_parameter<double>("speed_bias", options_.speed_bias);
    options_.heading_bias = this->declare_parameter<double>("heading_bias", options_.heading_bias);
    options_.local_goal_distance = this->declare_parameter<double>("local_goal_distance", options_.local_goal_distance);
    options_.goal_tolerance_xy = this->declare_parameter<double>("goal_tolerance_xy", options_.goal_tolerance_xy);
    options_.goal_tolerance_yaw = this->declare_parameter<double>("goal_tolerance_yaw", options_.goal_tolerance_yaw);
    options_.enforce_goal_heading = this->declare_parameter<bool>(
        "enforce_goal_heading", options_.enforce_goal_heading);
    options_.max_no_control_cycles = this->declare_parameter<int>("max_no_control_cycles", options_.max_no_control_cycles);
    options_.no_progress_timeout = this->declare_parameter<double>("no_progress_timeout", options_.no_progress_timeout);
    options_.min_progress_distance = this->declare_parameter<double>("min_progress_distance", options_.min_progress_distance);
    options_.publish_debug_trajectories = this->declare_parameter<bool>(
        "publish_debug_trajectories", options_.publish_debug_trajectories);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LocalPlannerNode::handleParameterUpdate, this, std::placeholders::_1));
}

void LocalPlannerNode::init()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    local_costmap_sub_ = this->create_subscription<CostmapMsgT>(
        "/local_costmap_raw", 1, std::bind(&LocalPlannerNode::handleLocalCostmap, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<OdomMsgT>(
        "odom", 20, std::bind(&LocalPlannerNode::handleOdometry, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<TwistMsgT>("/cmd_vel", 10);
    local_traj_pub_ = this->create_publisher<PathMsgT>("/local_trajectory", 10);
    debug_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/local_trajectories_debug", 10);

    follow_path_server_ = std::make_unique<rf_util::SimpleActionServer<ActionFollowPath>>(
        shared_from_this(),
        "/follow_path",
        [this]() {
            this->handleFollowPath();
        }
    );

    L_PLANNER_INFO("Initialized simple DWA local planner action server.");
}

void LocalPlannerNode::handleLocalCostmap(const CostmapMsgT::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_costmap_msg_ = msg;
}

void LocalPlannerNode::handleOdometry(const OdomMsgT::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    odom_msg_ = msg;
}

void LocalPlannerNode::handleFollowPath()
{
    auto goal = follow_path_server_->getCurrentGoal();
    auto result = std::make_shared<ActionFollowPath::Result>();
    if (!goal || goal->path.poses.empty()) {
        result->error_code = ActionFollowPath::Result::INVALID_PATH;
        result->error_msg = "follow path goal is empty";
        follow_path_server_->terminateCurrent(result);
        publishZeroVelocity();
        return;
    }

    PathMsgT plan = goal->path;
    auto start_time = this->now();
    int no_control_cycles = 0;
    double best_goal_distance = std::numeric_limits<double>::infinity();
    auto last_progress_time = start_time;
    int inscribed_recovery_attempts = 0;
    bool inscribed_recovery_active = false;
    Pose2D inscribed_recovery_start_pose;
    double inscribed_recovery_target_distance = 0.0;
    rclcpp::Time inscribed_recovery_start_time;

    // Main control loop
    while (rclcpp::ok()) {
        const auto options = getOptions();
        rclcpp::Rate rate(std::max(1.0, options.control_frequency));

        // Check for cancellation or preemption
        if (follow_path_server_->isCancelRequested()) {
            result->tracking_time = this->now() - start_time;
            result->error_code = ActionFollowPath::Result::CANCELED;
            result->error_msg = "follow path goal canceled";
            follow_path_server_->terminateCurrent(result);
            publishZeroVelocity();
            return;
        }

        if (follow_path_server_->isPreemptRequested()) {
            goal = follow_path_server_->acceptPendingGoal();
            if (!goal || goal->path.poses.empty()) {
                result->tracking_time = this->now() - start_time;
                result->error_code = ActionFollowPath::Result::INVALID_PATH;
                result->error_msg = "preempted follow path goal is empty";
                follow_path_server_->terminateCurrent(result);
                publishZeroVelocity();
                return;
            }

            plan = goal->path;
            start_time = this->now();
            no_control_cycles = 0;
            best_goal_distance = std::numeric_limits<double>::infinity();
            last_progress_time = start_time;
            inscribed_recovery_attempts = 0;
            inscribed_recovery_active = false;
            L_PLANNER_INFO("Accepted preempted follow path goal with {} poses.", plan.poses.size());
        }

        CostmapMsgT::SharedPtr costmap_msg;
        OdomMsgT::SharedPtr odom_msg;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            costmap_msg = local_costmap_msg_;
            odom_msg = odom_msg_;
        }

        if (!costmap_msg || !odom_msg) {
            publishZeroVelocity();
            rate.sleep();
            continue;
        }

        if (!isOdomVelocityValid(*odom_msg, options)) {
            L_PLANNER_WARN(
                "Ignoring invalid odometry velocity: linear_x={}, angular_z={}; publishing stop.",
                odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z);
            publishZeroVelocity();
            rate.sleep();
            continue;
        }

        Pose2D robot_pose;
        if (!getRobotPose(robot_pose)) {
            publishZeroVelocity();
            rate.sleep();
            continue;
        }

        if (inscribed_recovery_active) {
            uint8_t cell_cost = 0;
            const auto cell_status = getCostmapCellStatus(robot_pose, *costmap_msg, &cell_cost);
            if (cell_status == CostmapCellStatus::TRAVERSABLE) {
                L_PLANNER_INFO(
                    "Inscribed-cost recovery completed after backing up {:.2f} m.",
                    std::hypot(
                        robot_pose.x - inscribed_recovery_start_pose.x,
                        robot_pose.y - inscribed_recovery_start_pose.y));
                inscribed_recovery_active = false;
                no_control_cycles = 0;
                last_progress_time = this->now();
            } else if (cell_status != CostmapCellStatus::INSCRIBED) {
                result->tracking_time = this->now() - start_time;
                result->error_code = ActionFollowPath::Result::NO_VALID_CONTROL;
                result->error_msg = "inscribed-cost recovery entered an unsafe local costmap cell";
                L_PLANNER_WARN(
                    "Stopping inscribed-cost recovery because the robot entered costmap status {} (cost {}).",
                    static_cast<int>(cell_status), static_cast<unsigned int>(cell_cost));
                follow_path_server_->terminateCurrent(result);
                publishZeroVelocity();
                return;
            } else {
                const double recovery_distance = std::hypot(
                    robot_pose.x - inscribed_recovery_start_pose.x,
                    robot_pose.y - inscribed_recovery_start_pose.y);
                const bool target_not_reached = recovery_distance < inscribed_recovery_target_distance;
                const bool timed_out = (this->now() - inscribed_recovery_start_time) >=
                    rclcpp::Duration::from_seconds(kInscribedRecoveryTimeoutSeconds);
                if (!target_not_reached || timed_out) {
                    result->tracking_time = this->now() - start_time;
                    result->error_code = ActionFollowPath::Result::REPLAN_REQUIRED;
                    result->error_msg = "inscribed-cost recovery could not reach a traversable local costmap cell";
                    L_PLANNER_WARN(
                        "Inscribed-cost recovery failed after backing up {:.2f} m for {:.2f} s (target {:.2f} m).",
                        recovery_distance,
                        (this->now() - inscribed_recovery_start_time).seconds(),
                        inscribed_recovery_target_distance);
                    follow_path_server_->terminateCurrent(result);
                    publishZeroVelocity();
                    return;
                }

                publishInscribedRecovery();
                rate.sleep();
                continue;
            }
        }

        // 1. Prune the plan to get the local segment, find the nearest point to the robot and remove all previous points
        auto plan_segment = prunePlan(plan, robot_pose);
        if (plan_segment.empty()) {
            result->tracking_time = this->now() - start_time;
            result->error_code = ActionFollowPath::Result::INVALID_PATH;
            result->error_msg = "no valid segment remains in the input path";
            follow_path_server_->terminateCurrent(result);
            publishZeroVelocity();
            return;
        }

        const auto& goal_pose = plan.poses.back();
        const double goal_distance = std::hypot(
            goal_pose.pose.position.x - robot_pose.x,
            goal_pose.pose.position.y - robot_pose.y);
        const double goal_yaw_error = normalizeAngle(computeGoalYaw(goal_pose) - robot_pose.yaw);

        auto feedback = std::make_shared<ActionFollowPath::Feedback>();
        feedback->distance_to_goal = static_cast<float>(goal_distance);
        follow_path_server_->publishFeedback(feedback);

        if ((best_goal_distance - goal_distance) >= options.min_progress_distance) {
            best_goal_distance = goal_distance;
            last_progress_time = this->now();
        } else if (!std::isfinite(best_goal_distance)) {
            best_goal_distance = goal_distance;
            last_progress_time = this->now();
        }

        if ((this->now() - last_progress_time) >= rclcpp::Duration::from_seconds(options.no_progress_timeout)) {
            result->tracking_time = this->now() - start_time;
            result->error_code = ActionFollowPath::Result::REPLAN_REQUIRED;
            result->error_msg = "path tracking stalled without measurable progress";
            follow_path_server_->terminateCurrent(result);
            publishZeroVelocity();
            return;
        }

        // 2. Check if the robot is within the goal tolerance, if so, stop and succeed
        if (goal_distance <= options.goal_tolerance_xy) {
            if (!options.enforce_goal_heading ||
                std::abs(goal_yaw_error) <= options.goal_tolerance_yaw) {
                publishZeroVelocity();
                result->tracking_time = this->now() - start_time;
                result->error_code = ActionFollowPath::Result::NONE;
                result->error_msg = "path tracked successfully";
                follow_path_server_->succeededCurrent(result);
                return;
            }

            publishRotateToGoal(goal_yaw_error);
            rate.sleep();
            continue;
        }

        // 3. Find the best trajectory
        auto local_goal = selectLocalGoal(plan_segment);
        std::vector<TrajectorySample> debug_samples;
        auto* debug_samples_ptr = options.publish_debug_trajectories ? &debug_samples : nullptr;
        auto best = findBestTrajectory(robot_pose, *odom_msg, *costmap_msg, plan_segment, local_goal, debug_samples_ptr);
        const auto debug_frame_id = plan.header.frame_id.empty() ? "map" : plan.header.frame_id;
        if (options.publish_debug_trajectories) {
            publishDebugTrajectories(debug_samples, best.collision ? nullptr : &best, debug_frame_id);
        } else if (debug_markers_active_) {
            clearDebugTrajectories();
        }
        if (best.collision || best.poses.empty()) {
            uint8_t current_cell_cost = 0;
            const auto current_cell_status = getCostmapCellStatus(
                robot_pose, *costmap_msg, &current_cell_cost);
            if (current_cell_status == CostmapCellStatus::INSCRIBED &&
                inscribed_recovery_attempts < kMaxInscribedRecoveryAttempts) {
                const auto recovery_distance = findReverseRecoveryDistance(robot_pose, *costmap_msg);
                if (recovery_distance.has_value()) {
                    ++inscribed_recovery_attempts;
                    inscribed_recovery_active = true;
                    inscribed_recovery_start_pose = robot_pose;
                    inscribed_recovery_target_distance = *recovery_distance;
                    inscribed_recovery_start_time = this->now();
                    no_control_cycles = 0;
                    last_progress_time = inscribed_recovery_start_time;
                    L_PLANNER_WARN(
                        "Starting inscribed-cost recovery attempt {}/{}: current cost {}, reverse target {:.2f} m.",
                        inscribed_recovery_attempts,
                        kMaxInscribedRecoveryAttempts,
                        static_cast<unsigned int>(current_cell_cost),
                        inscribed_recovery_target_distance);
                    publishInscribedRecovery();
                    rate.sleep();
                    continue;
                }

                inscribed_recovery_attempts = kMaxInscribedRecoveryAttempts;
                L_PLANNER_WARN(
                    "Cannot start inscribed-cost recovery: no traversable cell exists within {:.2f} m behind the robot.",
                    kInscribedRecoveryMaxDistanceMeters);
            }

            ++no_control_cycles;
            publishZeroVelocity();
            if (no_control_cycles >= std::max(1, options.max_no_control_cycles)) {
                result->tracking_time = this->now() - start_time;
                result->error_code = ActionFollowPath::Result::REPLAN_REQUIRED;
                result->error_msg = "failed to find a feasible local trajectory";
                follow_path_server_->terminateCurrent(result);
                return;
            }
            rate.sleep();
            continue;
        }

        no_control_cycles = 0;
        if (!std::isfinite(best.linear_vel) || !std::isfinite(best.angular_vel)) {
            L_PLANNER_WARN(
                "Refusing to publish non-finite velocity command: linear_x={}, angular_z={}.",
                best.linear_vel, best.angular_vel);
            publishZeroVelocity();
            rate.sleep();
            continue;
        }

        TwistMsgT cmd;
        cmd.linear.x = std::clamp(best.linear_vel, options.min_vel_x, options.max_vel_x);
        cmd.angular.z = std::clamp(best.angular_vel, options.min_vel_theta, options.max_vel_theta);
        cmd_vel_pub_->publish(cmd);
        zero_cmd_published_ = false;
        publishTrajectory(best, plan.header.frame_id.empty() ? "map" : plan.header.frame_id);

        rate.sleep();
    }

    result->tracking_time = this->now() - start_time;
    result->error_code = ActionFollowPath::Result::UNKNOWN;
    result->error_msg = "follow path loop exited unexpectedly";
    follow_path_server_->terminateCurrent(result);
    publishZeroVelocity();
}

void LocalPlannerNode::publishZeroVelocity()
{
    if (zero_cmd_published_) {
        return;
    }

    TwistMsgT cmd;
    cmd_vel_pub_->publish(cmd);
    zero_cmd_published_ = true;
}

void LocalPlannerNode::publishTrajectory(const TrajectorySample& sample, const std::string& frame_id)
{
    PathMsgT traj_msg;
    traj_msg.header.stamp = this->now();
    traj_msg.header.frame_id = frame_id.empty() ? "map" : frame_id;

    traj_msg.poses.reserve(sample.poses.size());
    for (const auto& pose2d : sample.poses) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = traj_msg.header;
        pose.pose.position.x = pose2d.x;
        pose.pose.position.y = pose2d.y;
        pose.pose.orientation.z = std::sin(pose2d.yaw * 0.5);
        pose.pose.orientation.w = std::cos(pose2d.yaw * 0.5);
        traj_msg.poses.push_back(pose);
    }

    local_traj_pub_->publish(traj_msg);
}

void LocalPlannerNode::publishDebugTrajectories(
    const std::vector<TrajectorySample>& samples,
    const TrajectorySample* best,
    const std::string& frame_id)
{
    if (!debug_traj_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    const auto stamp = this->now();
    int marker_id = 0;
    for (const auto& sample : samples) {
        if (sample.poses.empty()) {
            continue;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id.empty() ? "map" : frame_id;
        marker.ns = "dwa_samples";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
        marker.pose.orientation.w = 1.0;

        marker.color.r = sample.collision ? 1.0f : 0.1f;
        marker.color.g = sample.collision ? 0.2f : 0.9f;
        marker.color.b = 0.1f;
        marker.color.a = sample.collision ? 0.35f : 0.25f;

        marker.points.reserve(sample.poses.size());
        for (const auto& pose : sample.poses) {
            geometry_msgs::msg::Point point;
            point.x = pose.x;
            point.y = pose.y;
            marker.points.push_back(point);
        }

        marker_array.markers.push_back(marker);
    }

    if (best != nullptr && !best->poses.empty()) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id.empty() ? "map" : frame_id;
        marker.ns = "dwa_best";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.1f;
        marker.color.g = 0.6f;
        marker.color.b = 1.0f;
        marker.color.a = 0.95f;

        marker.points.reserve(best->poses.size());
        for (const auto& pose : best->poses) {
            geometry_msgs::msg::Point point;
            point.x = pose.x;
            point.y = pose.y;
            marker.points.push_back(point);
        }

        marker_array.markers.push_back(marker);
    }

    debug_traj_pub_->publish(marker_array);
    debug_markers_active_ = true;
}

void LocalPlannerNode::clearDebugTrajectories()
{
    if (!debug_traj_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    debug_traj_pub_->publish(marker_array);
    debug_markers_active_ = false;
}

void LocalPlannerNode::publishRotateToGoal(double yaw_error)
{
    const auto options = getOptions();
    TwistMsgT cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = std::clamp(yaw_error * 1.5, options.min_vel_theta, options.max_vel_theta);
    cmd_vel_pub_->publish(cmd);
    zero_cmd_published_ = false;
}

void LocalPlannerNode::publishInscribedRecovery()
{
    TwistMsgT cmd;
    cmd.linear.x = -kInscribedRecoverySpeedMetersPerSecond;
    cmd_vel_pub_->publish(cmd);
    zero_cmd_published_ = false;
}

bool LocalPlannerNode::isOdomVelocityValid(const OdomMsgT& odom, const Options& options) const
{
    const double control_dt = 1.0 / std::max(1.0, options.control_frequency);
    const double linear_velocity = odom.twist.twist.linear.x;
    const double angular_velocity = odom.twist.twist.angular.z;
    const double min_linear = options.min_vel_x - options.acc_lim_x * control_dt;
    const double max_linear = options.max_vel_x + options.acc_lim_x * control_dt;
    const double min_angular = options.min_vel_theta - options.acc_lim_theta * control_dt;
    const double max_angular = options.max_vel_theta + options.acc_lim_theta * control_dt;

    return std::isfinite(linear_velocity) && std::isfinite(angular_velocity) &&
        linear_velocity >= min_linear && linear_velocity <= max_linear &&
        angular_velocity >= min_angular && angular_velocity <= max_angular;
}

LocalPlannerNode::Options LocalPlannerNode::getOptions() const
{
    std::lock_guard<std::mutex> lock(options_mutex_);
    return options_;
}

rcl_interfaces::msg::SetParametersResult LocalPlannerNode::handleParameterUpdate(
    const std::vector<rclcpp::Parameter>& parameters)
{
    auto updated = getOptions();

    for (const auto& parameter : parameters) {
        const auto& name = parameter.get_name();

        if (name == "control_frequency") {
            updated.control_frequency = parameter.as_double();
        } else if (name == "sim_time") {
            updated.sim_time = parameter.as_double();
        } else if (name == "sim_granularity") {
            updated.sim_granularity = parameter.as_double();
        } else if (name == "vx_samples") {
            updated.vx_samples = parameter.as_int();
        } else if (name == "vtheta_samples") {
            updated.vtheta_samples = parameter.as_int();
        } else if (name == "max_vel_x") {
            updated.max_vel_x = parameter.as_double();
        } else if (name == "min_vel_x") {
            updated.min_vel_x = parameter.as_double();
        } else if (name == "max_vel_theta") {
            updated.max_vel_theta = parameter.as_double();
        } else if (name == "min_vel_theta") {
            updated.min_vel_theta = parameter.as_double();
        } else if (name == "acc_lim_x") {
            updated.acc_lim_x = parameter.as_double();
        } else if (name == "acc_lim_theta") {
            updated.acc_lim_theta = parameter.as_double();
        } else if (name == "path_distance_bias") {
            updated.path_distance_bias = parameter.as_double();
        } else if (name == "goal_distance_bias") {
            updated.goal_distance_bias = parameter.as_double();
        } else if (name == "obstacle_cost_bias") {
            updated.obstacle_cost_bias = parameter.as_double();
        } else if (name == "progress_bias") {
            updated.progress_bias = parameter.as_double();
        } else if (name == "speed_bias") {
            updated.speed_bias = parameter.as_double();
        } else if (name == "heading_bias") {
            updated.heading_bias = parameter.as_double();
        } else if (name == "local_goal_distance") {
            updated.local_goal_distance = parameter.as_double();
        } else if (name == "goal_tolerance_xy") {
            updated.goal_tolerance_xy = parameter.as_double();
        } else if (name == "goal_tolerance_yaw") {
            updated.goal_tolerance_yaw = parameter.as_double();
        } else if (name == "enforce_goal_heading") {
            updated.enforce_goal_heading = parameter.as_bool();
        } else if (name == "max_no_control_cycles") {
            updated.max_no_control_cycles = parameter.as_int();
        } else if (name == "no_progress_timeout") {
            updated.no_progress_timeout = parameter.as_double();
        } else if (name == "min_progress_distance") {
            updated.min_progress_distance = parameter.as_double();
        } else if (name == "publish_debug_trajectories") {
            updated.publish_debug_trajectories = parameter.as_bool();
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    if (updated.control_frequency <= 0.0) {
        result.reason = "control_frequency must be > 0";
        return result;
    }
    if (updated.sim_time <= 0.0) {
        result.reason = "sim_time must be > 0";
        return result;
    }
    if (updated.sim_granularity <= 0.0) {
        result.reason = "sim_granularity must be > 0";
        return result;
    }
    if (updated.vx_samples < 1) {
        result.reason = "vx_samples must be >= 1";
        return result;
    }
    if (updated.vtheta_samples < 1) {
        result.reason = "vtheta_samples must be >= 1";
        return result;
    }
    if (updated.max_vel_x < updated.min_vel_x) {
        result.reason = "max_vel_x must be >= min_vel_x";
        return result;
    }
    if (updated.max_vel_theta < updated.min_vel_theta) {
        result.reason = "max_vel_theta must be >= min_vel_theta";
        return result;
    }
    if (updated.acc_lim_x < 0.0) {
        result.reason = "acc_lim_x must be >= 0";
        return result;
    }
    if (updated.acc_lim_theta < 0.0) {
        result.reason = "acc_lim_theta must be >= 0";
        return result;
    }
    if (updated.path_distance_bias < 0.0 || updated.goal_distance_bias < 0.0 ||
        updated.obstacle_cost_bias < 0.0 || updated.speed_bias < 0.0 ||
        updated.heading_bias < 0.0 || updated.progress_bias < 0.0) {
        result.reason = "cost biases must be >= 0";
        return result;
    }
    if (updated.local_goal_distance < 0.0) {
        result.reason = "local_goal_distance must be >= 0";
        return result;
    }
    if (updated.goal_tolerance_xy < 0.0 || updated.goal_tolerance_yaw < 0.0) {
        result.reason = "goal tolerances must be >= 0";
        return result;
    }
    if (updated.max_no_control_cycles < 1) {
        result.reason = "max_no_control_cycles must be >= 1";
        return result;
    }
    if (updated.no_progress_timeout <= 0.0) {
        result.reason = "no_progress_timeout must be > 0";
        return result;
    }
    if (updated.min_progress_distance < 0.0) {
        result.reason = "min_progress_distance must be >= 0";
        return result;
    }

    {
        std::lock_guard<std::mutex> lock(options_mutex_);
        options_ = updated;
    }

    result.successful = true;
    return result;
}

bool LocalPlannerNode::getRobotPose(Pose2D& pose)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    if (!rf_util::getCurrentPose(pose_msg, *tf_buffer_)) {
        return false;
    }

    pose.x = pose_msg.pose.position.x;
    pose.y = pose_msg.pose.position.y;
    pose.yaw = tf2::getYaw(pose_msg.pose.orientation);
    return true;
}

std::vector<geometry_msgs::msg::PoseStamped> LocalPlannerNode::prunePlan(
    const PathMsgT& plan, const Pose2D& robot_pose) const
{
    if (plan.poses.empty()) {
        return {};
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < plan.poses.size(); ++i) {
        const auto& pose = plan.poses[i].pose.position;
        const double distance = std::hypot(pose.x - robot_pose.x, pose.y - robot_pose.y);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_index = i;
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> pruned;
    pruned.reserve(plan.poses.size() - nearest_index);
    for (std::size_t i = nearest_index; i < plan.poses.size(); ++i) {
        pruned.push_back(plan.poses[i]);
    }
    return pruned;
}

geometry_msgs::msg::PoseStamped LocalPlannerNode::selectLocalGoal(
    const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const
{
    const auto options = getOptions();
    if (plan_segment.empty()) {
        return geometry_msgs::msg::PoseStamped();
    }

    double accumulated_distance = 0.0;
    for (std::size_t i = 1; i < plan_segment.size(); ++i) {
        const auto& prev = plan_segment[i - 1].pose.position;
        const auto& cur = plan_segment[i].pose.position;
        accumulated_distance += std::hypot(cur.x - prev.x, cur.y - prev.y);
        if (accumulated_distance >= options.local_goal_distance) {
            return plan_segment[i];
        }
    }

    return plan_segment.back();
}

LocalPlannerNode::TrajectorySample LocalPlannerNode::findBestTrajectory(
    const Pose2D& robot_pose,
    const OdomMsgT& odom,
    const CostmapMsgT& costmap,
    const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment,
    const geometry_msgs::msg::PoseStamped& local_goal,
    std::vector<TrajectorySample>* debug_samples) const
{
    const auto options = getOptions();
    TrajectorySample best;

    const double control_dt = 1.0 / std::max(1.0, options.control_frequency);
    const double current_linear = odom.twist.twist.linear.x;
    const double current_angular = odom.twist.twist.angular.z;

    const double window_min_v = std::max(options.min_vel_x, current_linear - options.acc_lim_x * control_dt);
    const double window_max_v = std::min(options.max_vel_x, current_linear + options.acc_lim_x * control_dt);
    const double window_min_w = std::max(options.min_vel_theta, current_angular - options.acc_lim_theta * control_dt);
    const double window_max_w = std::min(options.max_vel_theta, current_angular + options.acc_lim_theta * control_dt);

    const int vx_samples = std::max(1, options.vx_samples);
    const int vtheta_samples = std::max(1, options.vtheta_samples);

    // For each sampled linear and angular velocity, simulate the trajectory and evaluate its cost
    for (int i = 0; i < vx_samples; ++i) {
        const double linear_vel = (vx_samples == 1)
            ? window_max_v
            : window_min_v + (window_max_v - window_min_v) * static_cast<double>(i) / static_cast<double>(vx_samples - 1);

        for (int j = 0; j < vtheta_samples; ++j) {
            const double angular_vel = (vtheta_samples == 1)
                ? 0.0
                : window_min_w + (window_max_w - window_min_w) * static_cast<double>(j) / static_cast<double>(vtheta_samples - 1);

            auto sample = simulateTrajectory(robot_pose, linear_vel, angular_vel, costmap, plan_segment, local_goal);
            if (debug_samples != nullptr) {
                debug_samples->push_back(sample);
            }
            if (sample.collision || sample.total_cost >= best.total_cost) {
                continue;
            }
            best = sample;
        }
    }

    auto stop_sample = simulateTrajectory(robot_pose, 0.0, 0.0, costmap, plan_segment, local_goal);
    if (debug_samples != nullptr) {
        debug_samples->push_back(stop_sample);
    }
    if (!stop_sample.collision && stop_sample.total_cost < best.total_cost) {
        best = stop_sample;
    }

    return best;
}

LocalPlannerNode::TrajectorySample LocalPlannerNode::simulateTrajectory(
    const Pose2D& start_pose,
    double linear_vel,
    double angular_vel,
    const CostmapMsgT& costmap,
    const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment,
    const geometry_msgs::msg::PoseStamped& local_goal) const
{
    const auto options = getOptions();
    TrajectorySample sample;
    sample.linear_vel = linear_vel;
    sample.angular_vel = angular_vel;
    sample.collision = false;

    Pose2D pose = start_pose;
    const double dt = std::max(0.02, options.sim_granularity);
    double max_obstacle_cost = 0.0;
    double obstacle_cost_sum = 0.0;
    std::size_t obstacle_cost_samples = 0;

    for (double t = 0.0; t < options.sim_time; t += dt) {
        pose.x += linear_vel * std::cos(pose.yaw) * dt;
        pose.y += linear_vel * std::sin(pose.yaw) * dt;
        pose.yaw = normalizeAngle(pose.yaw + angular_vel * dt);
        sample.poses.push_back(pose);

        double obstacle_cost = 0.0;
        if (isCollision(pose, costmap, &obstacle_cost)) {
            sample.collision = true;
            sample.total_cost = std::numeric_limits<double>::infinity();
            return sample;
        }
        max_obstacle_cost = std::max(max_obstacle_cost, obstacle_cost);
        obstacle_cost_sum += obstacle_cost;
        ++obstacle_cost_samples;
    }

    if (sample.poses.empty()) {
        sample.collision = true;
        sample.total_cost = std::numeric_limits<double>::infinity();
        return sample;
    }

    const auto& end_pose = sample.poses.back();
    const double local_goal_yaw = std::atan2(
        local_goal.pose.position.y - end_pose.y,
        local_goal.pose.position.x - end_pose.x);

    sample.path_cost = computePathDistance(end_pose, plan_segment);
    sample.goal_cost = std::hypot(
        local_goal.pose.position.x - end_pose.x,
        local_goal.pose.position.y - end_pose.y);
    sample.heading_cost = std::abs(normalizeAngle(local_goal_yaw - end_pose.yaw));
    sample.obstacle_peak_cost = max_obstacle_cost / static_cast<double>(rf_costmap::LETHAL_OBSTACLE);
    sample.obstacle_mean_cost = obstacle_cost_samples == 0
        ? 0.0
        : (obstacle_cost_sum / static_cast<double>(obstacle_cost_samples)) /
            static_cast<double>(rf_costmap::LETHAL_OBSTACLE);
    sample.obstacle_cost = sample.obstacle_mean_cost;
    sample.speed_cost = options.max_vel_x - linear_vel;
    sample.progress_reward = computePathProgress(end_pose, plan_segment);

    sample.total_cost =
        options.path_distance_bias * sample.path_cost +
        options.goal_distance_bias * sample.goal_cost +
        options.heading_bias * sample.heading_cost +
        options.obstacle_cost_bias * sample.obstacle_cost +
        options.speed_bias * sample.speed_cost -
        options.progress_bias * sample.progress_reward;

    return sample;
}

double LocalPlannerNode::computePathDistance(
    const Pose2D& pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const
{
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& path_pose : plan_segment) {
        const double distance = std::hypot(
            path_pose.pose.position.x - pose.x,
            path_pose.pose.position.y - pose.y);
        best_distance = std::min(best_distance, distance);
    }
    return std::isfinite(best_distance) ? best_distance : 0.0;
}

double LocalPlannerNode::computePathProgress(
    const Pose2D& pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const
{
    if (plan_segment.size() < 2) {
        return 0.0;
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < plan_segment.size(); ++i) {
        const auto& path_pose = plan_segment[i].pose.position;
        const double distance = std::hypot(path_pose.x - pose.x, path_pose.y - pose.y);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_index = i;
        }
    }

    double progress = 0.0;
    for (std::size_t i = 1; i <= nearest_index; ++i) {
        const auto& prev = plan_segment[i - 1].pose.position;
        const auto& cur = plan_segment[i].pose.position;
        progress += std::hypot(cur.x - prev.x, cur.y - prev.y);
    }

    return progress;
}

LocalPlannerNode::CostmapCellStatus LocalPlannerNode::getCostmapCellStatus(
    const Pose2D& pose, const CostmapMsgT& costmap, uint8_t* cost) const
{
    if (!std::isfinite(costmap.metadata.resolution) || costmap.metadata.resolution <= 0.0) {
        return CostmapCellStatus::OUT_OF_BOUNDS;
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!worldToMap(costmap, pose.x, pose.y, mx, my)) {
        return CostmapCellStatus::OUT_OF_BOUNDS;
    }

    const auto index = my * costmap.metadata.size_x + mx;
    if (index >= costmap.data.size()) {
        return CostmapCellStatus::OUT_OF_BOUNDS;
    }

    const uint8_t cell_cost = costmap.data[index];
    if (cost != nullptr) {
        *cost = cell_cost;
    }
    if (cell_cost == rf_costmap::NO_INFORMATION) {
        return CostmapCellStatus::UNKNOWN;
    }
    if (cell_cost >= rf_costmap::LETHAL_OBSTACLE) {
        return CostmapCellStatus::LETHAL;
    }
    if (cell_cost >= rf_costmap::INSCRIBED_INFLATED_OBSTACLE) {
        return CostmapCellStatus::INSCRIBED;
    }
    return CostmapCellStatus::TRAVERSABLE;
}

std::optional<double> LocalPlannerNode::findReverseRecoveryDistance(
    const Pose2D& pose, const CostmapMsgT& costmap) const
{
    const double resolution = costmap.metadata.resolution;
    if (!std::isfinite(resolution) || resolution <= 0.0) {
        return std::nullopt;
    }

    const double probe_step = std::clamp(resolution, 0.01, 0.05);
    for (double distance = probe_step;
         distance <= kInscribedRecoveryMaxDistanceMeters + std::numeric_limits<double>::epsilon();
         distance += probe_step) {
        Pose2D probe_pose = pose;
        probe_pose.x -= distance * std::cos(pose.yaw);
        probe_pose.y -= distance * std::sin(pose.yaw);

        const auto status = getCostmapCellStatus(probe_pose, costmap);
        if (status == CostmapCellStatus::TRAVERSABLE) {
            return distance;
        }
        if (status != CostmapCellStatus::INSCRIBED) {
            return std::nullopt;
        }
    }

    return std::nullopt;
}

bool LocalPlannerNode::isCollision(const Pose2D& pose, const CostmapMsgT& costmap, double* max_cost) const
{
    // The local costmap is already inflated using the robot footprint, so
    // checking the center cell is sufficient to determine if the robot is in collision.
    uint8_t cell_cost = 0;
    const auto status = getCostmapCellStatus(pose, costmap, &cell_cost);
    if (max_cost != nullptr) {
        *max_cost = static_cast<double>(cell_cost);
    }
    return status != CostmapCellStatus::TRAVERSABLE;
}

bool LocalPlannerNode::worldToMap(
    const CostmapMsgT& costmap, double wx, double wy, unsigned int& mx, unsigned int& my) const
{
    if (wx < costmap.metadata.origin.position.x || wy < costmap.metadata.origin.position.y) {
        return false;
    }

    mx = static_cast<unsigned int>((wx - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
    my = static_cast<unsigned int>((wy - costmap.metadata.origin.position.y) / costmap.metadata.resolution);
    return mx < costmap.metadata.size_x && my < costmap.metadata.size_y;
}

double LocalPlannerNode::normalizeAngle(double angle) const
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double LocalPlannerNode::computeGoalYaw(const geometry_msgs::msg::PoseStamped& pose) const
{
    return tf2::getYaw(pose.pose.orientation);
}

} // namespace rf_local_planner
