#pragma once

#include "rf_map_builder/tf_bridge.hpp"
#include "rf_robot_msgs/msg/landmark_list.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/transform/rigid_transform.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cartographer/sensor/odometry_data.h>
#include <memory>

namespace rf_map_builder
{

class SensorBridge
{
public:
    explicit SensorBridge(const std::string& tracking_frame,
        double lookup_transform_timeout_sec,
        tf2_ros::Buffer* tf_buffer,
        cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder)
        : tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
          trajectory_builder_(trajectory_builder) {}

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    std::unique_ptr<cartographer::sensor::OdometryData> toOdometryData(
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    std::unique_ptr<cartographer::sensor::ImuData> toImuData(
        const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

    void handleOdometryMessage(const std::string& sensor_id,
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void handleLandmarkMessage(const std::string& sensor_id,
        const rf_robot_msgs::msg::LandmarkList::ConstSharedPtr& msg);
    void handleImuMessage(const std::string& sensor_id,
        const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void handleLaserScanMessage(const std::string& sensor_id,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void handlePointCloud2Message(const std::string& sensor_id,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    const TfBridge& tfBridge() const { return tf_bridge_; }

private:
    const TfBridge tf_bridge_;
    cartographer::mapping::TrajectoryBuilderInterface* const trajectory_builder_;
    std::optional<cartographer::transform::Rigid3d> ecef_to_local_frame_;
};

} // namespace rf_map_builder