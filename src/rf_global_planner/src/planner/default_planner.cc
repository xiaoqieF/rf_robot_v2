#include "rf_global_planner/planner/default_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rf_global_planner/common.hpp"
#include <limits>
#include <rf_util/execution_timer.hpp>
#include <algorithm>
#include <cstdint>
#include <queue>
#include <variant>

namespace rf_global_planner
{

static constexpr uint8_t NO_INFORMATION = 255;
static constexpr uint8_t LETHAL_OBSTACLE = 254;
static constexpr uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr uint8_t FREE_SPACE = 0;
static constexpr uint8_t COST_NEUTRAL = 50;

void DefaultPlanner::init(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    tf_buffer_ = tf_buffer;
}

std::pair<PlanErrorCode, nav_msgs::msg::Path> DefaultPlanner::getPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    rf_robot_msgs::msg::Costmap::SharedPtr costmap)
{
    if (costmap == nullptr) {
        return std::make_pair(PlanErrorCode::UNKNOWN, nav_msgs::msg::Path());
    }
    costmap_ = costmap;

    preprocessCostmap();

    int mx_start, my_start, mx_goal, my_goal;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
        return std::make_pair(PlanErrorCode::START_OUTSIDE_BOUNDS, nav_msgs::msg::Path());
    }
    if (!worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
        return std::make_pair(PlanErrorCode::GOAL_OUTSIDE_BOUNDS, nav_msgs::msg::Path());
    }

    if (tolerance == 0 && costmap_->data[getIndex(mx_goal, my_goal)] == LETHAL_OBSTACLE) {
        return std::make_pair(PlanErrorCode::GOAL_OCCUPIED, nav_msgs::msg::Path());
    }

    unsigned int map_size = costmap_->metadata.size_x * costmap_->metadata.size_y;
    g_scores_.assign(map_size, std::numeric_limits<double>::infinity());
    vistited_.assign(map_size, false);
    parents_x_.assign(map_size, -1);
    parents_y_.assign(map_size, -1);
    open_list_ = std::priority_queue<Cell, std::vector<Cell>, CellCompare>();
    found_ = false;

    searchPlan(mx_start, my_start, mx_goal, my_goal);

    if (found_) {
        return {PlanErrorCode::OK, backtracePath(mx_start, my_start, mx_goal, my_goal)};
    }

    if (tolerance == 0) {
        return std::make_pair(PlanErrorCode::NO_PATH_FOUND, nav_msgs::msg::Path());
    }

    // Try find path with tolerance
    double min_cost = std::numeric_limits<double>::infinity();
    int best_x = -1, best_y = -1;

    int tolerance_grid = tolerance / costmap_->metadata.resolution;

    for (int dx = -tolerance_grid; dx <= tolerance_grid; dx++) {
        for (int dy = -tolerance_grid; dy <= tolerance_grid; dy++) {
            int x = mx_goal + dx;
            int y = my_goal + dy;

            if (x < 0 || y < 0 || x >= static_cast<int>(costmap_->metadata.size_x) ||
                y >= static_cast<int>(costmap_->metadata.size_y)) {
                continue;
            }

            int idx = getIndex(x, y);
            if (vistited_[idx] && g_scores_[idx] < min_cost &&
                costmap_->data[idx] != LETHAL_OBSTACLE) {
                min_cost = g_scores_[idx];
                best_x = x;
                best_y = y;
            }
        }
    }

    if (best_x == -1 || best_y == -1) {
        return std::make_pair(PlanErrorCode::NO_PATH_FOUND, nav_msgs::msg::Path());
    }

    G_PLANNER_INFO("Find alternate goal: ({}, {})", best_x, best_y);
    return {PlanErrorCode::OK, backtracePath(mx_start, my_start, best_x, best_y)};
}

void DefaultPlanner::searchPlan(
    int mx_start, int my_start, int mx_goal, int my_goal)
{
    Cell start = {mx_start, my_start, 0, 0, mx_start, my_start};
    g_scores_[getIndex(mx_start, my_start)] = 0;
    open_list_.push(start);

    static constexpr int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static constexpr int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    static constexpr double cost[8] = {1, 1.4, 1, 1.4, 1, 1.4, 1, 1.4};

    found_ = false;

    rf_util::ExecutionTimer ticker;
    ticker.tick();

    while (!open_list_.empty()) {
        Cell current = open_list_.top();
        open_list_.pop();

        int cur_idx = getIndex(current.x, current.y);
        if (vistited_[cur_idx]) {
            continue;
        }
        vistited_[cur_idx] = true;

        if (current.x == mx_goal && current.y == my_goal) {
            found_ = true;
            break;
        }

        for (size_t i = 0; i < sizeof(dx) / sizeof(dx[0]); i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];
            if (nx < 0 || ny < 0 ||
                nx >= static_cast<int>(costmap_->metadata.size_x) ||
                ny >= static_cast<int>(costmap_->metadata.size_y)) {
                continue;
            }

            int n_idx = getIndex(nx, ny);
            if (vistited_[n_idx]) {
                continue;
            }

            if (costmap_->data[n_idx] == LETHAL_OBSTACLE) {
                continue;
            }

            double tentative_g = g_scores_[cur_idx] + cost[i] + costmap_->data[n_idx] / 255.0;

            if (tentative_g < g_scores_[n_idx]) {
                g_scores_[n_idx] = tentative_g;
                parents_x_[n_idx] = current.x;
                parents_y_[n_idx] = current.y;
                double h = use_astar ? heuristic(nx, ny, mx_goal, my_goal) : 0;
                open_list_.push({nx, ny, tentative_g, h, current.x, current.y});
            }
        }
    }

    ticker.toc();
    G_PLANNER_INFO("Search plan spend: {} ms, found: {}",
        std::chrono::duration_cast<std::chrono::milliseconds>(ticker.elapsed()).count(), found_);
}

nav_msgs::msg::Path DefaultPlanner::backtracePath(
    int mx_start, int my_start, int mx_goal, int my_goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    int cx = mx_goal;
    int cy = my_goal;

    while (mx_start != cx || my_start != cy) {
        geometry_msgs::msg::PoseStamped pose;
        mapToWorld(cx,cy, pose.pose.position.x, pose.pose.position.y);
        pose.header.frame_id = "map";
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
        int p_idx = getIndex(cx, cy);
        int px = parents_x_[p_idx];
        int py = parents_y_[p_idx];

        if (px == -1 || py == -1) {
            break;
        }
        cx = px;
        cy = py;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    mapToWorld(mx_start, my_start, start_pose.pose.position.x, start_pose.pose.position.y);
    start_pose.header.frame_id = "map";
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = 0.0;
    start_pose.pose.orientation.w = 1.0;

    path.poses.push_back(start_pose);
    G_PLANNER_INFO("Backtrace Path success, length: {}", path.poses.size());

    std::reverse(path.poses.begin(), path.poses.end());
    return path;
}

void DefaultPlanner::preprocessCostmap()
{
    int width = costmap_->metadata.size_x;
    int height = costmap_->metadata.size_y;

    // Convert INSCRIBED_INFLATED_OBSTACLE to LETHAL_OBSTACLE
    // Convert FREE_SPACE to COST_NEUTRAL
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            if (costmap_->data[getIndex(i, j)] >= INSCRIBED_INFLATED_OBSTACLE) {
                costmap_->data[getIndex(i, j)] = LETHAL_OBSTACLE;
            } else {
                int v = COST_NEUTRAL + 0.8 * costmap_->data[getIndex(i, j)];
                if (v >= LETHAL_OBSTACLE) {
                    v = LETHAL_OBSTACLE - 1;
                }
                costmap_->data[getIndex(i, j)] = v;
            }
        }
    }
}

} // namespace rf_global_planner