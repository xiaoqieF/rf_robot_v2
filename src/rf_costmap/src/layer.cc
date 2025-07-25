#include "rf_costmap/layer.hpp"

namespace rf_costmap
{

Layer::Layer()
    : node_(nullptr),
      master_costmap_(nullptr)
{
}

void Layer::initialize(const std::string& name,
                       rclcpp::Node::SharedPtr node,
                       MasterCostmap* master_costmap,
                       tf2_ros::Buffer* tf_buffer)
{
    name_ = name;
    node_ = node;
    master_costmap_ = master_costmap;
    tf_buffer_ = tf_buffer;
    onInitialize();
}



} // namespace rf_costmap