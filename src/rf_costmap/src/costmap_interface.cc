#include "elog/elog.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "rf_costmap/costmap_interface.hpp"
#include "rf_costmap/static_layer.hpp"
#include "rf_util/execution_timer.hpp"
#include "rf_util/robot_utils.hpp"

namespace rf_costmap
{

CostmapInterface::CostmapInterface(rclcpp::Node::SharedPtr node, const CostmapConfig& config)
    : config_(config), node_(node)
{
    master_costmap_ = std::make_unique<MasterCostmap>(config_.rolling_window);
}

void CostmapInterface::init()
{
    elog::info("Initializing Costmap Interface map_name: {}", config_.map_name);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    costmap_publisher_ = std::make_unique<CostmapPublisher>(node_, config_.map_name, master_costmap_->getCostmap());

    master_costmap_->resizeMap(config_.width, config_.height,
        config_.resolution, 0.0, 0.0);

    for (const auto& layer_name : config_.layer_names) {
        elog::info("Adding layer: {}", layer_name);
        if (layer_name == "static_layer") {
            auto layer = std::make_shared<StaticLayer>("map");
            layer->initialize(layer_name, node_, master_costmap_.get());
            master_costmap_->addLayer(layer);
        } else {
            elog::warn("Layer {} is not recognized or not implemented.", layer_name);
        }
    }
}

void CostmapInterface::stop()
{
    elog::info("Stopping Costmap Interface map_name: {}", config_.map_name);
    running_ = false;
    if (map_update_thread_.joinable()) {
        map_update_thread_.join();
    }
}

void CostmapInterface::start()
{
    elog::info("Starting Costmap Interface map_name: {}", config_.map_name);
    map_update_thread_ = std::thread(&CostmapInterface::mapUpdateLoop, this);
}

void CostmapInterface::mapUpdateLoop()
{
    rclcpp::Rate rate(config_.update_rate);
    running_ = true;
    auto pub_duration = std::chrono::duration<double>(1.0 / config_.publish_rate);

    while (rclcpp::ok() && running_) {
        rf_util::ExecutionTimer timer;

        timer.tick();
        updateMap();
        timer.toc();
        elog::info("Costmap update took {} ms",
            std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed()).count());

        // Pub costmap
        auto now = std::chrono::steady_clock::now();
        if (now - last_publish_time_ >= pub_duration) {
            costmap_publisher_->publishCostmap();
            last_publish_time_ = now;
        }
    }
}

void CostmapInterface::updateMap()
{
    geometry_msgs::msg::PoseStamped robot_pose;
    if (rf_util::getCurrentPose(robot_pose, *tf_buffer_)) {
        const double robot_x = robot_pose.pose.position.x;
        const double robot_y = robot_pose.pose.position.y;
        const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        master_costmap_->updateMap(robot_x, robot_y, robot_yaw);
    }
}

} // namespace rf_costmap