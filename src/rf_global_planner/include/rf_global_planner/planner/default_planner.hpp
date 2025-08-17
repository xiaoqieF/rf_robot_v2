#pragma once

#include "rf_global_planner/planner/global_planner.hpp"
#include <cmath>
#include <queue>
#include <vector>

namespace rf_global_planner
{

struct Cell
{
    int x, y;
    double g;
    double h;
    int parent_x, parent_y;

    double f() const { return g + h; }
};

struct CellCompare
{
    bool operator()(const Cell& lhs, const Cell& rhs) const
    {
        return lhs.f() > rhs.f();
    }
};

class DefaultPlanner : public GlobalPlanner
{
public:
    DefaultPlanner() = default;
    ~DefaultPlanner() override = default;

    void init(std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;

    std::pair<PlanErrorCode, nav_msgs::msg::Path> getPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        rf_robot_msgs::msg::Costmap::SharedPtr costmap) override;


private:
    void searchPlan(int mx_start, int my_start, int mx_goal, int my_goal);
    nav_msgs::msg::Path backtracePath(int mx_start, int my_start, int mx_goal, int my_goal);

    bool worldToMap(double wx, double wy, int& mx, int& my)
    {
        mx = static_cast<int>((wx - costmap_->metadata.origin.position.x)
            / costmap_->metadata.resolution);
        my = static_cast<int>((wy - costmap_->metadata.origin.position.y)
            / costmap_->metadata.resolution);
        return (mx >= 0 && my >= 0 && mx < static_cast<int>(costmap_->metadata.size_x)
            && my < static_cast<int>(costmap_->metadata.size_y));
    }

    void mapToWorld(int mx, int my, double &wx, double &wy)
    {
        wx = costmap_->metadata.origin.position.x + (mx + 0.5) *
            costmap_->metadata.resolution;
        wy = costmap_->metadata.origin.position.y + (my + 0.5) *
            costmap_->metadata.resolution;
    }

    double heuristic(int x1, int y1, int x2, int y2)
    {
        return std::hypot(x2 - x1, y2 - y1);
    }

    int getIndex(int x, int y) { return y * costmap_->metadata.size_x + x; }

    void preprocessCostmap();

private:
    static constexpr bool use_astar{true};
    static constexpr double tolerance{0.0};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rf_robot_msgs::msg::Costmap::SharedPtr costmap_;

    bool found_{false};
    std::vector<bool> vistited_;
    std::vector<double> g_scores_;
    std::vector<int> parents_x_;
    std::vector<int> parents_y_;
    std::priority_queue<Cell, std::vector<Cell>, CellCompare> open_list_;
};

} // namespace rf_global_planner