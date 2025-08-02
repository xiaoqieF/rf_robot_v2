#pragma once

#include "rf_costmap/costmap_interface.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"

namespace rf_local_map
{

using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;

class LocalMapNode : public rclcpp::Node
{
public:
    LocalMapNode() : Node("local_map_node") {};
    void init();

private:
    std::unique_ptr<rf_costmap::CostmapInterface> costmap_interface_;
    rclcpp::Service<ReqAckSrvT>::SharedPtr local_map_srv_;
};

} // namespace rf_local_map