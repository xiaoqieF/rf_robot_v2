#include "rf_costmap/costmap_publisher.hpp"
#include "rf_costmap/costmap_2d.hpp"
#include <cstdint>

namespace rf_costmap
{

CostmapPublisher::CostmapPublisher(rclcpp::Node::SharedPtr node,
    const std::string& topic_name, Costmap2D* costmap)
    : node_(node), topic_name_(topic_name), costmap_(costmap)
{
    publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name_, rclcpp::QoS(1));
    raw_publisher_ = node_->create_publisher<rf_robot_msgs::msg::Costmap>(topic_name_ + "_raw", rclcpp::QoS(1));
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid> CostmapPublisher::prepareGrid()
{
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid_msg->header.frame_id = "map"; // Set the frame_id as needed
    grid_msg->header.stamp = node_->now();

    grid_msg->info.resolution = costmap_->getResolution();
    grid_msg->info.width = costmap_->getSizeX();
    grid_msg->info.height = costmap_->getSizeY();
    grid_msg->info.origin.position.x = costmap_->getOriginX();
    grid_msg->info.origin.position.y = costmap_->getOriginY();
    grid_msg->info.origin.position.z = 0.0;
    grid_msg->info.origin.orientation.w = 1.0;
    grid_msg->data.resize(costmap_->getSizeX() * costmap_->getSizeY());

    for (unsigned int i = 0; i < costmap_->getSizeX(); ++i) {
        for (unsigned int j = 0; j < costmap_->getSizeY(); ++j) {
            grid_msg->data[costmap_->getIndex(i, j)] = costmap_->getCost(i, j);
        }
    }

    return grid_msg;
}

std::unique_ptr<rf_robot_msgs::msg::Costmap> CostmapPublisher::prepareCostmap()
{
    auto costmap_msg = std::make_unique<rf_robot_msgs::msg::Costmap>();
    costmap_msg->header.frame_id = "map"; // Set the frame_id as needed
    costmap_msg->header.stamp = node_->now(); // Set the current time

    costmap_msg->metadata.layer = "master"; // Set the layer name as needed
    costmap_msg->metadata.resolution = costmap_->getResolution();
    costmap_msg->metadata.size_x = costmap_->getSizeX();
    costmap_msg->metadata.size_y = costmap_->getSizeY();
    costmap_msg->metadata.origin.position.x = costmap_->getOriginX();
    costmap_msg->metadata.origin.position.y = costmap_->getOriginY();
    costmap_msg->metadata.origin.position.z = 0.0;
    costmap_msg->metadata.origin.orientation.w = 1.0;

    costmap_msg->data.resize(costmap_->getSizeX() * costmap_->getSizeY());

    uint8_t* costmap_data = costmap_->getCostmapData();
    memcpy(costmap_msg->data.data(), costmap_data,
           costmap_->getSizeX() * costmap_->getSizeY() * sizeof(uint8_t));

    return costmap_msg;
}

void CostmapPublisher::publishCostmap()
{
    auto grid_map = prepareGrid();
    publisher_->publish(std::move(grid_map));
    auto raw_msg = prepareCostmap();
    raw_publisher_->publish(std::move(raw_msg));
}

} // namespace rf_costmap