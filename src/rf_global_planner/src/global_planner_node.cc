#include "rf_global_planner/global_planner_node.hpp"
#include "rf_global_planner/planner/global_planner.hpp"
#include "rf_util/robot_utils.hpp"
#include "rf_global_planner/common.hpp"
#include "rf_global_planner/planner/default_planner.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>

namespace rf_global_planner
{

void GlobalPlannerNode::init()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf_buffer_->setCreateTimerInterface(timer_interface);

    auto default_planner = std::unique_ptr<DefaultPlanner>();
    default_planner->init(tf_buffer_);
    planners_.emplace("default_planner", std::move(default_planner));

    global_map_sub_ = this->create_subscription<CostmapMsgT>(
        "/global_costmap",
        1,
        [this](const CostmapMsgT::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(global_costmap_mutex_);
            global_costmap_msg_ = msg;
        }
    );

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/global_path",
        1
    );

    action_to_pose_server_ = std::make_unique<rf_util::SimpleActionServer<ActionToPose>>(
        shared_from_this(),
        "/compute_path_to_pose",
        [this]() {
            this->handleComputePathToPose();
        }
    );

    action_through_poses_server_ = std::make_unique<rf_util::SimpleActionServer<ActionThroughPoses>>(
        shared_from_this(),
        "/compute_path_through_poses",
        [this]() {
            this->handleComputePathThroughPoses();
        }
    );
}

void GlobalPlannerNode::handleComputePathToPose()
{
    auto start_time = this->now();

    auto goal = action_to_pose_server_->getCurrentGoal();
    auto result = std::make_shared<ActionToPose::Result>();

    if (action_to_pose_server_->isCancelRequested()) {
        G_PLANNER_INFO("Goal was canceled.");
        action_to_pose_server_->terminateAll();
        return;
    }

    if (action_to_pose_server_->isPreemptRequested()) {
        G_PLANNER_INFO("Goal was preempted.");
        goal = action_to_pose_server_->acceptPendingGoal();
        return;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (goal->use_start) {
        start_pose = goal->start;
    } else {
        if (!rf_util::getCurrentPose(start_pose, *tf_buffer_)) {
            G_PLANNER_ERROR("Failed to get current pose.");
            result->error_code = ActionToPose::Result::TF_ERROR;
            action_to_pose_server_->terminateCurrent();
            return;
        }
    }

    if (!transformPoseToGlobalFrame(start_pose) || !transformPoseToGlobalFrame(goal_pose)) {
        G_PLANNER_ERROR("Failed to transform poses to global frame.");
        result->error_code = ActionToPose::Result::TF_ERROR;
        action_to_pose_server_->terminateCurrent();
        return;
    }

    if (planners_.empty()) {
        throw std::runtime_error("No planners available. Please initialize planners before calling getPlan.");
    }

    std::string planner_id = goal->planner_id.empty() ? planners_.begin()->first : goal->planner_id;
    if (planners_.find(planner_id) == planners_.end()) {
        G_PLANNER_ERROR("Planner '{}' not found.", planner_id);
        result->error_code = ActionToPose::Result::INVALID_PLANNER;
        action_to_pose_server_->terminateCurrent();
        return;
    }
    auto [err_code,cur_path] = getPlan(start_pose, goal_pose, planner_id);

    if (err_code  == PlanErrorCode::PLANNER_NOT_FOUND) {
        G_PLANNER_ERROR("Planner '{}' not found.", planner_id);
        result->error_code = ActionThroughPoses::Result::INVALID_PLANNER;
        action_through_poses_server_->terminateCurrent();
        return;
    } else if (err_code == PlanErrorCode::GOAL_OUTSIDE_BOUNDS) {
        G_PLANNER_ERROR("Goal outside map.");
        result->error_code = ActionThroughPoses::Result::GOAL_OUTSIDE_MAP;
        action_through_poses_server_->terminateCurrent();
        return;
    } else if (err_code == PlanErrorCode::START_OUTSIDE_BOUNDS) {
        G_PLANNER_ERROR("Start outside map.");
        result->error_code = ActionThroughPoses::Result::START_OUTSIDE_MAP;
        action_through_poses_server_->terminateCurrent();
        return;
    } else if (err_code == PlanErrorCode::GOAL_OCCUPIED) {
        G_PLANNER_ERROR("goal occupied.");
        result->error_code = ActionThroughPoses::Result::GOAL_OCCUPIED;
        action_through_poses_server_->terminateCurrent();
        return;
    } else if (err_code != PlanErrorCode::OK) {
        G_PLANNER_ERROR("goal occupied.");
        result->error_code = ActionThroughPoses::Result::UNKNOWN;
        action_through_poses_server_->terminateCurrent();
        return;
    }

    if (result->path.poses.empty()) {
        G_PLANNER_ERROR("Failed to compute path from ({:.3f}, {:.3f}) to ({:.3f}, {:.3f}) using planner: {}",
            start_pose.pose.position.x, start_pose.pose.position.y,
            goal_pose.pose.position.x, goal_pose.pose.position.y,
            planner_id);
        result->error_code = ActionToPose::Result::NO_VALID_PATH;
        action_to_pose_server_->terminateCurrent();
        return;
    }

    publishPlan(result->path);
    auto dur = this->now() - start_time;
    result->planning_time = dur;
    G_PLANNER_INFO("Computed path from ({:.3f}, {:.3f}) to ({:.3f}, {:.3f}) using planner: {} in {:.3f} seconds",
        start_pose.pose.position.x, start_pose.pose.position.y,
        goal_pose.pose.position.x, goal_pose.pose.position.y,
        planner_id, dur.seconds());
    action_to_pose_server_->succeededCurrent(result);
}

void GlobalPlannerNode::handleComputePathThroughPoses()
{
    auto start_time = this->now();

    auto goal = action_through_poses_server_->getCurrentGoal();
    auto result = std::make_shared<ActionThroughPoses::Result>();

    nav_msgs::msg::Path concat_path;

    geometry_msgs::msg::PoseStamped cur_start, cur_goal;

    if (action_through_poses_server_->isCancelRequested()) {
        G_PLANNER_INFO("Goal was canceled.");
        action_through_poses_server_->terminateAll();
        return;
    }

    if (action_through_poses_server_->isPreemptRequested()) {
        G_PLANNER_INFO("Goal was preempted.");
        goal = action_through_poses_server_->acceptPendingGoal();
        return;
    }

    if (goal->goals.empty()) {
        G_PLANNER_ERROR("No poses provided in goal.");
        result->error_code = ActionThroughPoses::Result::NO_VIAPOINTS_GIVEN;
        action_through_poses_server_->terminateCurrent();
        return;
    }

    geometry_msgs::msg::PoseStamped start_pose;

    if (goal->use_start) {
        start_pose = goal->start;
    } else {
        if (!rf_util::getCurrentPose(start_pose, *tf_buffer_)) {
            G_PLANNER_ERROR("Failed to get current pose.");
            result->error_code = ActionToPose::Result::TF_ERROR;
            action_through_poses_server_->terminateCurrent();
            return;
        }
    }

    std::string planner_id = goal->planner_id.empty() ? planners_.begin()->first : goal->planner_id;
    if (planners_.find(planner_id) == planners_.end()) {
        G_PLANNER_ERROR("Planner '{}' not found.", planner_id);
        result->error_code = ActionToPose::Result::INVALID_PLANNER;
        action_through_poses_server_->terminateCurrent();
        return;
    }

    for (size_t i = 0; i < goal->goals.size(); ++ i) {
        if (i == 0) {
            cur_start = start_pose;
        } else {
            cur_start = concat_path.poses.back();
            cur_start.header = concat_path.header;
        }

        cur_goal = goal->goals[i];

        if (!transformPoseToGlobalFrame(cur_start) || !transformPoseToGlobalFrame(cur_goal)) {
            G_PLANNER_ERROR("Failed to transform poses to global frame.");
            result->error_code = ActionThroughPoses::Result::TF_ERROR;
            action_through_poses_server_->terminateCurrent();
            return;
        }

        auto [err_code,cur_path] = getPlan(cur_start, cur_goal, planner_id);

        if (err_code  == PlanErrorCode::PLANNER_NOT_FOUND) {
            G_PLANNER_ERROR("Planner '{}' not found.", planner_id);
            result->error_code = ActionThroughPoses::Result::INVALID_PLANNER;
            action_through_poses_server_->terminateCurrent();
            return;
        } else if (err_code == PlanErrorCode::GOAL_OUTSIDE_BOUNDS) {
            G_PLANNER_ERROR("Goal outside map.");
            result->error_code = ActionThroughPoses::Result::GOAL_OUTSIDE_MAP;
            action_through_poses_server_->terminateCurrent();
            return;
        } else if (err_code == PlanErrorCode::START_OUTSIDE_BOUNDS) {
            G_PLANNER_ERROR("Start outside map.");
            result->error_code = ActionThroughPoses::Result::START_OUTSIDE_MAP;
            action_through_poses_server_->terminateCurrent();
            return;
        } else if (err_code == PlanErrorCode::GOAL_OCCUPIED) {
            G_PLANNER_ERROR("goal occupied.");
            result->error_code = ActionThroughPoses::Result::GOAL_OCCUPIED;
            action_through_poses_server_->terminateCurrent();
            return;
        } else if (err_code != PlanErrorCode::OK) {
            G_PLANNER_ERROR("goal occupied.");
            result->error_code = ActionThroughPoses::Result::UNKNOWN;
            action_through_poses_server_->terminateCurrent();
            return;
        }

        if (cur_path.poses.empty()) {
            G_PLANNER_ERROR("Failed to compute path from ({:.3f}, {:.3f}) to ({:.3f}, {:.3f}) using planner: {}",
                cur_start.pose.position.x, cur_start.pose.position.y,
                cur_goal.pose.position.x, cur_goal.pose.position.y,
                planner_id);
            result->error_code = ActionThroughPoses::Result::NO_VALID_PATH;
            action_through_poses_server_->terminateCurrent();
            return;
        }

        concat_path.poses.insert(concat_path.poses.end(),
            cur_path.poses.begin(), cur_path.poses.end());
        concat_path.header = cur_path.header;
    }

    result->path = concat_path;
    publishPlan(result->path);

    auto dur = this->now() - start_time;
    result->planning_time = dur;
    G_PLANNER_INFO("Computed path through {} poses using planner: {} in {:.3f} seconds",
        goal->goals.size(), planner_id, dur.seconds());
    action_through_poses_server_->succeededCurrent(result);
}

bool GlobalPlannerNode::transformPoseToGlobalFrame(geometry_msgs::msg::PoseStamped& pose)
{
    if (pose.header.frame_id == "map") {
        return true; // Already in global frame
    }

    return rf_util::transformPoseInTargetFrame(pose, pose,
        *tf_buffer_, "map");
}

std::pair<PlanErrorCode, nav_msgs::msg::Path> GlobalPlannerNode::getPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const std::string& planner_name)
{
    G_PLANNER_INFO("Find path from ({:.3f}, {:.3f}) to ({:.3f}, {:.3f}) using planner: {}",
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y,
        planner_name);

    if (planners_.find(planner_name) == planners_.end()) {
        G_PLANNER_ERROR("Planner '{}' not found.", planner_name);
        return std::make_pair(PlanErrorCode::PLANNER_NOT_FOUND, nav_msgs::msg::Path());
    }

    CostmapMsgT::SharedPtr global_costmap;
    {
        std::lock_guard<std::mutex> lock(global_costmap_mutex_);
        global_costmap = global_costmap_msg_;
    }

    return planners_[planner_name]->getPlan(start, goal, global_costmap);
}

void GlobalPlannerNode::publishPlan(const nav_msgs::msg::Path& path)
{
    auto msg = std::make_unique<nav_msgs::msg::Path>(path);
    path_publisher_->publish(std::move(msg));
}

} // namespace rf_global_planner