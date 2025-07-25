#pragma once

#include "rf_costmap/observation.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>

#include <list>
#include <mutex>
#include <vector>

namespace rf_costmap
{

class ObservationBuffer
{
public:
    ObservationBuffer(rclcpp::Node::SharedPtr node, tf2_ros::Buffer* tf_buffer);
    ~ObservationBuffer() = default;

    void bufferCloud(const sensor_msgs::msg::PointCloud2& cloud);
    std::vector<Observation> getObservations();
    void resetLastUpdated() {
        std::lock_guard<std::mutex> lock(mutex_);
        last_updated_ = clock_->now();
    }
    bool isCurrent() const;

private:
    void purgeStaleObservations();

private:
    static constexpr double raytrace_min_range = 0.0;
    static constexpr double raytrace_max_range = 10.0;
    static constexpr double obstacle_min_range = 0.0;
    static constexpr double obstacle_max_range = 10.0;
    static constexpr double min_obstacle_height = 0.0;
    static constexpr double max_obstacle_height = 2.0;
    static constexpr double observation_keep_time = 0.0;   // seconds, 0.0 means keep only one observation
    static constexpr double expected_update_timeout = 0.2;

    mutable std::mutex mutex_;
    rclcpp::Clock::SharedPtr clock_;
    tf2_ros::Buffer* tf_buffer_;
    tf2::Duration tf_tolerance_;
    rclcpp::Time last_updated_;
    std::list<Observation> observation_list_;
};

} // namespace rf_costmap