#include "rf_util/robot_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "elog/elog.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/timer.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rf_costmap/costmap_interface.hpp"

class TestCostmapNode: public rclcpp::Node
{
public:
    TestCostmapNode()
    : Node("test_costmap_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                // Example usage of getCurrentPose
                geometry_msgs::msg::PoseStamped global_pose;
                if (rf_util::getCurrentPose(global_pose, *tf_buffer_)) {
                    elog::info("Robot current pose: x: {}, y: {}, z: {}",
                        global_pose.pose.position.x,
                        global_pose.pose.position.y,
                        global_pose.pose.position.z);
                } else {
                    elog::error("Failed to retrieve current pose.");
                }
            });
    }

    void init()
    {
        rf_costmap::CostmapConfig config;
        config.map_name = "global_costmap";
        config.resolution = 0.05; // 5 cm resolution
        config.width = 5; // 10 meters wide
        config.height = 5; // 10 meters tall
        config.rolling_window = false;
        config.update_rate = 10; // 10 Hz
        config.publish_rate = 5; // 5 Hz
        config.layer_names = {"static_layer", "obstacle_layer"};

        costmap_interface_ = std::make_unique<rf_costmap::CostmapInterface>(shared_from_this(), config);
        costmap_interface_->init();
        costmap_interface_->start();
    }


private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<rf_costmap::CostmapInterface> costmap_interface_;
};

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<TestCostmapNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}