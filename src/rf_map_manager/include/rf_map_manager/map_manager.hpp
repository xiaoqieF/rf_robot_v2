#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rf_robot_msgs/srv/get_map.hpp"
#include "rf_robot_msgs/srv/dump_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rf_map_manager
{

using OccupancyGridMsgT = nav_msgs::msg::OccupancyGrid;
using GetMapSrvT = rf_robot_msgs::srv::GetMap;
using DumpMapSrvT = rf_robot_msgs::srv::DumpMap;

class MapManager
{
public:
    MapManager(rclcpp::Node::SharedPtr node, const std::string& map_dir);
    ~MapManager() = default;

    void publishMap();
    bool saveCachedMap(std::string* reason = nullptr);
    void handleGetMapService(const std::shared_ptr<GetMapSrvT::Request> request,
                             std::shared_ptr<GetMapSrvT::Response> response);
    void handleDumpMapService(const std::shared_ptr<DumpMapSrvT::Request> request,
                             std::shared_ptr<DumpMapSrvT::Response> response);

private:
    void handleMapUpdate(const OccupancyGridMsgT::SharedPtr msg);
    void reloadMap();
    bool dumpMap(const OccupancyGridMsgT& map);
    bool ensureMapDir() const;

private:
    rclcpp::Node::SharedPtr node_;
    std::string map_dir_;
    OccupancyGridMsgT::SharedPtr cached_map_;
    std::mutex map_mutex_;

    rclcpp::Publisher<OccupancyGridMsgT>::SharedPtr map_publisher_;
    rclcpp::Subscription<OccupancyGridMsgT>::SharedPtr map_subscription_;
    rclcpp::Service<GetMapSrvT>::SharedPtr get_map_service_;
    rclcpp::Service<DumpMapSrvT>::SharedPtr dump_map_service_;
};

} // namespace rf_map_manager
