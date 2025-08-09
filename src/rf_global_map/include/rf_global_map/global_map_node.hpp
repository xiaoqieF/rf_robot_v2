#pragma once

#include "rf_costmap/costmap_interface.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/client.hpp>

namespace rf_global_map
{

using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;

class GlobalMapNode : public rclcpp::Node
{
public:
    GlobalMapNode() : Node("global_map_node") {}
    void init();

private:
    void startMapUpdate();
    void stopMapUpdate();

private:
    std::unique_ptr<rf_costmap::CostmapInterface> costmap_interface_;
    rclcpp::Service<ReqAckSrvT>::SharedPtr global_map_srv_;

    rclcpp::Client<ReqAckSrvT>::SharedPtr static_map_client_;
};

} // namespace rf_global_map