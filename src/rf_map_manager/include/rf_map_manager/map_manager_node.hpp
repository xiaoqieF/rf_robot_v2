#pragma once

#include "rf_map_manager/common.hpp"
#include "rf_map_manager/map_manager.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"

namespace rf_map_manager
{

using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;

class MapManagerNode : public rclcpp::Node
{
public:
    MapManagerNode()
    : Node("map_manager_node")
    {
    }

    void init();

private:
    std::unique_ptr<MapManager> manager_;
    rclcpp::Service<ReqAckSrvT>::SharedPtr pub_map_service_;
};

} // namespace rf_map_manager