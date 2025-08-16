#pragma once

#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "rf_robot_msgs/msg/costmap.hpp"
#include <memory>

namespace rf_global_planner
{

enum class PlanErrorCode
{
    OK = 0,
    PLANNER_NOT_FOUND = 1,
    START_OUTSIDE_BOUNDS = 2,
    GOAL_OUTSIDE_BOUNDS = 3,
    GOAL_OCCUPIED = 4,
    NO_PATH_FOUND = 5,
    UNKNOWN = 6
};

class GlobalPlanner
{
public:
    virtual ~GlobalPlanner() = default;

    virtual void init(std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;

    virtual std::pair<PlanErrorCode, nav_msgs::msg::Path> getPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        rf_robot_msgs::msg::Costmap::SharedPtr costmap
    ) = 0;
};

}