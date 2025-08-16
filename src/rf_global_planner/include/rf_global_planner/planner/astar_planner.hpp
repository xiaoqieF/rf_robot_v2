#pragma once

#include "rf_global_planner/planner/global_planner.hpp"

namespace rf_global_planner
{

class AstarPlanner : public GlobalPlanner
{
public:
    AstarPlanner() = default;
    ~AstarPlanner() override = default;

    void init(std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;

    nav_msgs::msg::Path getPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        rf_robot_msgs::msg::Costmap::SharedPtr costmap) override;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace rf_global_planner