#pragma once

#include <atomic>
#include <cstdint>

#include "rf_costmap/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rf_costmap
{

class StaticLayer : public CostmapLayer
{
public:
    StaticLayer(const std::string& map_topic) : CostmapLayer(), map_topic_(map_topic) {}
    void onInitialize() override;
    void reset() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(Costmap2D& master_grid,
                     unsigned int min_i, unsigned int min_j,
                     unsigned int max_i, unsigned int max_j) override;
    void matchSize() override;

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void processMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);
    uint8_t interpretCostValue(uint8_t cost) const;

private:
    static constexpr uint8_t UNKNOWN_COST_VALUE = 255;
    static constexpr uint8_t LETHAL_THRESHOLD = 100;

    std::string map_topic_{"map"};
    std::atomic_bool map_received_{false};
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_buffer_{nullptr};
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_{nullptr};
};

} // namespace rf_costmap