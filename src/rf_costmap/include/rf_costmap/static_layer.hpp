#pragma once

#include "rf_costmap/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rf_costmap
{

class StaticLayer : public CostmapLayer
{
public:
    StaticLayer() : CostmapLayer() {}
    void onInitialize() override;
    void reset() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(Costmap2D& master_grid,
                     unsigned int min_i, unsigned int min_j,
                     unsigned int max_i, unsigned int max_j) override;
    void matchSize() override;

private:
    std::string map_topic_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

} // namespace rf_costmap