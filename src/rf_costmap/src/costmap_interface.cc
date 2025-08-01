#include "rf_costmap/cost_values.hpp"
#include "rf_costmap/costmap_interface.hpp"
#include "rf_costmap/static_layer.hpp"
#include "rf_costmap/obstacle_layer.hpp"
#include "rf_costmap/inflation_layer.hpp"
#include "rf_util/execution_timer.hpp"
#include "rf_util/robot_utils.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pthread.h>
#include <tf2_ros/create_timer_ros.h>

namespace rf_costmap
{

CostmapInterface::CostmapInterface(rclcpp::Node::SharedPtr node, const CostmapConfig& config)
    : config_(config), node_(node)
{
    master_costmap_ = std::make_unique<MasterCostmap>(config_.rolling_window);
}

void CostmapInterface::init()
{
    COSTMAP_INFO("Initializing Costmap Interface map_name: {}", config_.map_name);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node_->get_node_base_interface(),
        node_->get_node_timers_interface()
    );
    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    costmap_publisher_ = std::make_unique<CostmapPublisher>(node_, config_.map_name, master_costmap_->getCostmap());

    master_costmap_->resizeMap(static_cast<unsigned int>(config_.width / config_.resolution),
        static_cast<unsigned int>(config_.height / config_.resolution),
        config_.resolution, 0.0, 0.0);

    for (const auto& layer_name : config_.layer_names) {
        COSTMAP_INFO("Adding layer: {}", layer_name);
        if (layer_name == "static_layer") {
            auto layer = std::make_shared<StaticLayer>();
            layer->initialize(layer_name, node_, master_costmap_.get(), tf_buffer_.get());
            master_costmap_->addLayer(layer);
        }else if (layer_name == "obstacle_layer") {
            auto layer = std::make_shared<ObstacleLayer>();
            layer->initialize(layer_name, node_, master_costmap_.get(), tf_buffer_.get());
            master_costmap_->addLayer(layer);
        } else if (layer_name == "inflation_layer") {
            auto layer = std::make_shared<InflationLayer>();
            layer->initialize(layer_name, node_, master_costmap_.get(), tf_buffer_.get());
            master_costmap_->addLayer(layer);
        } else {
            COSTMAP_WARN("Layer {} is not recognized or not implemented.", layer_name);
        }
    }
}

void CostmapInterface::stop()
{
    COSTMAP_INFO("Stopping Costmap Interface map_name: {}", config_.map_name);
    running_ = false;
    if (map_update_thread_.joinable()) {
        map_update_thread_.join();
    }
}

void CostmapInterface::start()
{
    COSTMAP_INFO("Starting Costmap Interface map_name: {}", config_.map_name);
    map_update_thread_ = std::thread(&CostmapInterface::mapUpdateLoop, this);
    pthread_setname_np(map_update_thread_.native_handle(),
        (config_.map_name + "_update_thread").c_str());
}

void CostmapInterface::mapUpdateLoop()
{
    rclcpp::Rate rate(config_.update_rate);
    running_ = true;
    auto pub_duration_ticks = std::max(1u, config_.update_rate / config_.publish_rate);
    uint64_t tick = 0;

    while (rclcpp::ok() && running_) {
        rf_util::ExecutionTimer timer;

        // Update map
        timer.tick();
        updateMap();
        timer.toc();
        COSTMAP_INFO("Costmap update took {} ms",
            std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed()).count());

        // Pub costmap
        auto now = std::chrono::steady_clock::now();
        if (tick % pub_duration_ticks == 0) {
            costmap_publisher_->publishCostmap();
            last_publish_time_ = now;
            COSTMAP_INFO("Costmap published at {}", rclcpp::Clock().now().seconds());
        }

        rate.sleep();
        ++ tick;
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