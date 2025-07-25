#include "rf_costmap/observation_buffer.hpp"
#include "rf_costmap/cost_values.hpp"
#include "rf_costmap/observation.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <mutex>
#include <string>

namespace rf_costmap
{

ObservationBuffer::ObservationBuffer(rclcpp::Node::SharedPtr node, tf2_ros::Buffer* tf_buffer)
    : clock_(node->get_clock()), tf_buffer_(tf_buffer), tf_tolerance_(tf2::durationFromSec(0.1))
{
    last_updated_ = clock_->now();
}

std::vector<Observation> ObservationBuffer::getObservations()
{
    std::lock_guard<std::mutex> lock(mutex_);
    purgeStaleObservations();
    return std::vector<Observation>(observation_list_.begin(), observation_list_.end());
}

void ObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PointStamped global_origin;

    Observation new_observation;

    std::string origin_frame = cloud.header.frame_id;

    try {
        geometry_msgs::msg::PointStamped local_origin;
        local_origin.header.stamp = cloud.header.stamp;
        local_origin.header.frame_id = origin_frame;
        local_origin.point.x = 0.0;
        local_origin.point.y = 0.0;
        local_origin.point.z = 0.0;

        tf_buffer_->transform(local_origin, global_origin, "map");
        new_observation.origin = global_origin.point;

        new_observation.raytrace_min_range = raytrace_min_range;
        new_observation.raytrace_max_range = raytrace_max_range;
        new_observation.obstacle_min_range = obstacle_min_range;
        new_observation.obstacle_max_range = obstacle_max_range;

        sensor_msgs::msg::PointCloud2 global_cloud;
        tf_buffer_->transform(cloud, global_cloud, "map", tf_tolerance_);

        new_observation.cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
        new_observation.cloud->header.frame_id = global_cloud.header.frame_id;
        new_observation.cloud->header.stamp = global_cloud.header.stamp;

        new_observation.cloud->height = global_cloud.height;
        new_observation.cloud->width = global_cloud.width;
        new_observation.cloud->is_dense = global_cloud.is_dense;
        new_observation.cloud->is_bigendian = global_cloud.is_bigendian;
        new_observation.cloud->fields = global_cloud.fields;
        new_observation.cloud->point_step = global_cloud.point_step;
        new_observation.cloud->row_step = global_cloud.row_step;

        // Remove points that are too low or too high
        unsigned int cloud_size = global_cloud.width * global_cloud.height;
        sensor_msgs::PointCloud2Modifier modifier(*new_observation.cloud);
        modifier.resize(cloud_size);
        unsigned int point_count = 0;

        sensor_msgs::PointCloud2Iterator<float> iter_z(global_cloud, "z");
        auto iter_global = global_cloud.data.begin();
        auto iter_end = global_cloud.data.end();
        auto iter_valid = global_cloud.data.begin();
        for (; iter_global != iter_end; ++ iter_z, iter_global += global_cloud.point_step) {
            if ((*iter_z) <= max_obstacle_height && (*iter_z) >= min_obstacle_height) {
                std::copy(iter_global, iter_global + global_cloud.point_step, iter_valid);
                iter_valid += global_cloud.point_step;
                ++ point_count;
            }
        }

        modifier.resize(point_count);

        observation_list_.push_front(std::move(new_observation));
    } catch (tf2::TransformException& ex) {
        COSTMAP_WARN("Failed to transform from {} to map, what: {}", origin_frame, ex.what());
        return;
    }

    last_updated_ = clock_->now();

    purgeStaleObservations();
}

void ObservationBuffer::purgeStaleObservations()
{
    if (!observation_list_.empty()) {
        auto it = observation_list_.begin();
        if (observation_keep_time < 1e-5) {
            // keep only one observation when observation_keep_time is 0
            observation_list_.erase(++ it, observation_list_.end());
        } else {
            for (it = observation_list_.begin(); it != observation_list_.end(); ++ it) {
                if ((clock_->now() - it->cloud->header.stamp).seconds() > observation_keep_time) {
                    observation_list_.erase(++ it, observation_list_.end());
                    return;
                }
            }
        }
    }
}

bool ObservationBuffer::isCurrent() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (expected_update_timeout < 1e-5) {
        return true;
    }
    bool current = (clock_->now() - last_updated_).seconds() < expected_update_timeout;

    if (!current) {
        COSTMAP_WARN("ObservationBuffer is not current, last updated {} seconds ago", (clock_->now() - last_updated_).seconds());
    }
    return current;
}

} // namespace rf_costmap
