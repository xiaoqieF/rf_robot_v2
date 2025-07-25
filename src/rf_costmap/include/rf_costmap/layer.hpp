#pragma once

#include <string>
#include <tf2_ros/buffer.h>

#include "rclcpp/rclcpp.hpp"

#include "rf_costmap/costmap_2d.hpp"

namespace rf_costmap
{
class MasterCostmap;

class Layer
{
public:
    Layer();
    virtual ~Layer() = default;
    virtual void initialize(const std::string& name,
        rclcpp::Node::SharedPtr node,
        MasterCostmap* master_costmap,
        tf2_ros::Buffer* tf_buffer);
    virtual void reset() = 0;
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y, double* max_x, double* max_y) = 0;
    virtual void updateCosts(Costmap2D& master_grid,
                             unsigned int min_i, unsigned int min_j,
                             unsigned int max_i, unsigned int max_j) = 0;
    virtual void matchSize() = 0;
    std::string getName() const { return name_; }

protected:
    virtual void onInitialize() = 0;

protected:
    std::string name_;
    rclcpp::Node::SharedPtr node_;
    MasterCostmap* master_costmap_;
    tf2_ros::Buffer* tf_buffer_;
};

} // namespace rf_costmap