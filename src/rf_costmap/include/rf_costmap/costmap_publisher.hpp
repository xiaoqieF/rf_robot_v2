#pragma once

#include "rf_robot_msgs/msg/costmap.hpp"
#include "rf_costmap/costmap_2d.hpp"
#include <rclcpp/node.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace rf_costmap
{

class CostmapPublisher
{
public:
    CostmapPublisher(rclcpp::Node::SharedPtr node,
        const std::string& topic_name, Costmap2D* costmap);

    void publishCostmap();

private:
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> prepareGrid();
    std::unique_ptr<rf_robot_msgs::msg::Costmap> prepareCostmap();

private:
    std::unique_ptr<int8_t[]> cost_trans_table_{nullptr};
    rclcpp::Node::SharedPtr node_;
    std::string topic_name_;
    Costmap2D* costmap_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Publisher<rf_robot_msgs::msg::Costmap>::SharedPtr raw_publisher_;
};

} // namespace rf_costmap