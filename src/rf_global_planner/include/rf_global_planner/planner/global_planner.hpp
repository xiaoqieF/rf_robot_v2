#pragma once

#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "rf_robot_msgs/msg/costmap.hpp"
#include <memory>

namespace rf_global_planner
{

class GlobalPlanner
{
public:
    virtual ~GlobalPlanner() = default;

    virtual void init(std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;

    virtual nav_msgs::msg::Path getPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        rf_robot_msgs::msg::Costmap::SharedPtr costmap
    ) = 0;
};

}