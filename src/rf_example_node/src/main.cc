#include "rf_util/robot_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "elog/elog.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/timer.hpp>

class RfExampleNode : public rclcpp::Node
{
public:
    RfExampleNode()
    : Node("rf_example_node")
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


private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<RfExampleNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}