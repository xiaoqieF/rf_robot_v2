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
    options_.robot_radius = this->declare_parameter<double>("robot_radius", options_.robot_radius);
    options_.path_distance_bias = this->declare_parameter<double>("path_distance_bias", options_.path_distance_bias);
    options_.goal_distance_bias = this->declare_parameter<double>("goal_distance_bias", options_.goal_distance_bias);
    options_.obstacle_cost_bias = this->declare_parameter<double>("obstacle_cost_bias", options_.obstacle_cost_bias);
    options_.speed_bias = this->declare_parameter<double>("speed_bias", options_.speed_bias);
    options_.heading_bias = this->declare_parameter<double>("heading_bias", options_.heading_bias);
    options_.local_goal_distance = this->declare_parameter<double>("local_goal_distance", options_.local_goal_distance);
    options_.goal_tolerance_xy = this->declare_parameter<double>("goal_tolerance_xy", options_.goal_tolerance_xy);
    options_.goal_tolerance_yaw = this->declare_parameter<double>("goal_tolerance_yaw", options_.goal_tolerance_yaw);
    options_.max_no_control_cycles = this->declare_parameter<int>("max_no_control_cycles", options_.max_no_control_cycles);
    options_.autostart = this->declare_parameter<bool>("autostart", options_.autostart);

    active_ = options_.autostart;
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

    control_service_ = this->create_service<ReqAckSrvT>(
        "/local_planner_control",
        std::bind(&LocalPlannerNode::handleControlRequest, this, std::placeholders::_1, std::placeholders::_2));

    follow_path_server_ = std::make_unique<rf_util::SimpleActionServer<ActionFollowPath>>(
        shared_from_this(),
        "/follow_path",
        [this]() {
            this->handleFollowPath();
        }
    );

    L_PLANNER_INFO("Initialized simple DWA local planner action server. autostart: {}", active_);
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

void LocalPlannerNode::handleControlRequest(
    const ReqAckSrvT::Request::SharedPtr request,
    const ReqAckSrvT::Response::SharedPtr response)
{
    if (request == nullptr || response == nullptr) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        active_ = (request->trigger == ReqAckSrvT::Request::START);
    }

    if (!active_) {
        publishZeroVelocity();
    }

    response->ack = ReqAckSrvT::Response::OK;
    response->reason = active_ ? "local planner started" : "local planner stopped";
    L_PLANNER_INFO("Local planner {}.", active_ ? "started" : "stopped");
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
    rclcpp::Rate rate(std::max(1.0, options_.control_frequency));

    while (rclcpp::ok()) {
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
            L_PLANNER_INFO("Accepted preempted follow path goal with {} poses.", plan.poses.size());
        }

        bool active = true;
        CostmapMsgT::SharedPtr costmap_msg;
        OdomMsgT::SharedPtr odom_msg;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            active = active_;
            costmap_msg = local_costmap_msg_;
            odom_msg = odom_msg_;
        }

        if (!active) {
            result->tracking_time = this->now() - start_time;
            result->error_code = ActionFollowPath::Result::UNKNOWN;
            result->error_msg = "local planner stopped by control service";
            follow_path_server_->terminateCurrent(result);
            publishZeroVelocity();
            return;
        }

        if (!costmap_msg || !odom_msg) {
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

        if (goal_distance <= options_.goal_tolerance_xy) {
            if (std::abs(goal_yaw_error) <= options_.goal_tolerance_yaw) {
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

        auto local_goal = selectLocalGoal(plan_segment);
        auto best = findBestTrajectory(robot_pose, *odom_msg, *costmap_msg, plan_segment, local_goal);
        if (best.collision || best.poses.empty()) {
            ++no_control_cycles;
            publishZeroVelocity();
            if (no_control_cycles >= std::max(1, options_.max_no_control_cycles)) {
                result->tracking_time = this->now() - start_time;
                result->error_code = ActionFollowPath::Result::NO_VALID_CONTROL;
                result->error_msg = "failed to find a feasible local trajectory";
                follow_path_server_->terminateCurrent(result);
                return;
            }
            rate.sleep();
            continue;
        }

        no_control_cycles = 0;
        TwistMsgT cmd;
        cmd.linear.x = best.linear_vel;
        cmd.angular.z = best.angular_vel;
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

void LocalPlannerNode::publishRotateToGoal(double yaw_error)
{
    TwistMsgT cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = std::clamp(yaw_error * 1.5, options_.min_vel_theta, options_.max_vel_theta);
    cmd_vel_pub_->publish(cmd);
    zero_cmd_published_ = false;
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
    if (plan_segment.empty()) {
        return geometry_msgs::msg::PoseStamped();
    }

    double accumulated_distance = 0.0;
    for (std::size_t i = 1; i < plan_segment.size(); ++i) {
        const auto& prev = plan_segment[i - 1].pose.position;
        const auto& cur = plan_segment[i].pose.position;
        accumulated_distance += std::hypot(cur.x - prev.x, cur.y - prev.y);
        if (accumulated_distance >= options_.local_goal_distance) {
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
    const geometry_msgs::msg::PoseStamped& local_goal) const
{
    TrajectorySample best;

    const double control_dt = 1.0 / std::max(1.0, options_.control_frequency);
    const double current_linear = odom.twist.twist.linear.x;
    const double current_angular = odom.twist.twist.angular.z;

    const double window_min_v = std::max(options_.min_vel_x, current_linear - options_.acc_lim_x * control_dt);
    const double window_max_v = std::min(options_.max_vel_x, current_linear + options_.acc_lim_x * control_dt);
    const double window_min_w = std::max(options_.min_vel_theta, current_angular - options_.acc_lim_theta * control_dt);
    const double window_max_w = std::min(options_.max_vel_theta, current_angular + options_.acc_lim_theta * control_dt);

    const int vx_samples = std::max(1, options_.vx_samples);
    const int vtheta_samples = std::max(1, options_.vtheta_samples);

    for (int i = 0; i < vx_samples; ++i) {
        const double linear_vel = (vx_samples == 1)
            ? window_max_v
            : window_min_v + (window_max_v - window_min_v) * static_cast<double>(i) / static_cast<double>(vx_samples - 1);

        for (int j = 0; j < vtheta_samples; ++j) {
            const double angular_vel = (vtheta_samples == 1)
                ? 0.0
                : window_min_w + (window_max_w - window_min_w) * static_cast<double>(j) / static_cast<double>(vtheta_samples - 1);

            auto sample = simulateTrajectory(robot_pose, linear_vel, angular_vel, costmap, plan_segment, local_goal);
            if (sample.collision || sample.total_cost >= best.total_cost) {
                continue;
            }
            best = std::move(sample);
        }
    }

    auto stop_sample = simulateTrajectory(robot_pose, 0.0, 0.0, costmap, plan_segment, local_goal);
    if (!stop_sample.collision && stop_sample.total_cost < best.total_cost) {
        best = std::move(stop_sample);
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
    TrajectorySample sample;
    sample.linear_vel = linear_vel;
    sample.angular_vel = angular_vel;
    sample.collision = false;

    Pose2D pose = start_pose;
    const double dt = std::max(0.02, options_.sim_granularity);
    double max_obstacle_cost = 0.0;

    for (double t = 0.0; t < options_.sim_time; t += dt) {
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
    sample.obstacle_cost = max_obstacle_cost / static_cast<double>(rf_costmap::LETHAL_OBSTACLE);
    sample.speed_cost = options_.max_vel_x - linear_vel;

    sample.total_cost =
        options_.path_distance_bias * sample.path_cost +
        options_.goal_distance_bias * sample.goal_cost +
        options_.heading_bias * sample.heading_cost +
        options_.obstacle_cost_bias * sample.obstacle_cost +
        options_.speed_bias * sample.speed_cost;

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

bool LocalPlannerNode::isCollision(const Pose2D& pose, const CostmapMsgT& costmap, double* max_cost) const
{
    const int radius_cells = std::max(
        1,
        static_cast<int>(std::ceil(options_.robot_radius / costmap.metadata.resolution)));
    double local_max_cost = 0.0;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            const double offset_x = static_cast<double>(dx) * costmap.metadata.resolution;
            const double offset_y = static_cast<double>(dy) * costmap.metadata.resolution;
            if ((offset_x * offset_x + offset_y * offset_y) > (options_.robot_radius * options_.robot_radius)) {
                continue;
            }

            unsigned int mx = 0;
            unsigned int my = 0;
            if (!worldToMap(costmap, pose.x + offset_x, pose.y + offset_y, mx, my)) {
                return true;
            }

            const auto index = my * costmap.metadata.size_x + mx;
            if (index >= costmap.data.size()) {
                return true;
            }

            const uint8_t cell_cost = costmap.data[index];
            local_max_cost = std::max(local_max_cost, static_cast<double>(cell_cost));
            if (cell_cost == rf_costmap::NO_INFORMATION || cell_cost >= rf_costmap::INSCRIBED_INFLATED_OBSTACLE) {
                return true;
            }
        }
    }

    if (max_cost != nullptr) {
        *max_cost = local_max_cost;
    }
    return false;
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
