#pragma once

#include <memory>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <thread>

#include "rf_costmap/costmap_config.hpp"
#include "rf_costmap/master_costmap.hpp"
#include "rf_costmap/costmap_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"

namespace rf_costmap
{

class CostmapInterface
{
public:
    CostmapInterface(rclcpp::Node::SharedPtr node, const CostmapConfig& config);
    ~CostmapInterface() = default;

    void init();
    void start();
    void stop();

private:
    void mapUpdateLoop();
    void updateMap();

private:
    CostmapConfig config_;

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<MasterCostmap> master_costmap_;
    std::unique_ptr<CostmapPublisher> costmap_publisher_;
    std::chrono::steady_clock::time_point last_publish_time_{};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::thread map_update_thread_;
    std::atomic_bool running_{false};
};

} // namespace rf_costmap
