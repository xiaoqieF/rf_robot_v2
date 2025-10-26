#pragma once

#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cartographer/transform/transform.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "rf_robot_msgs/msg/landmark_list.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rf_map_builder
{

cartographer::transform::Rigid3d toRigid3d(const geometry_msgs::msg::TransformStamped& transform);
cartographer::transform::Rigid3d toRigid3d(const geometry_msgs::msg::Pose& pose);
Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& vector3);
Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion& quaternion);
cartographer::sensor::LandmarkData toLandmarkData(const rf_robot_msgs::msg::LandmarkList& landmark_list);

sensor_msgs::msg::PointCloud2 toPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const cartographer::sensor::TimedPointCloud& point_cloud);

std::tuple<cartographer::sensor::PointCloudWithIntensities, cartographer::common::Time>
    toPointCloudWithIntensities(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
std::tuple<cartographer::sensor::PointCloudWithIntensities, cartographer::common::Time>
    toPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

geometry_msgs::msg::Transform toGeometryMsgTransform(const cartographer::transform::Rigid3d& rigid3d);
geometry_msgs::msg::Pose toGeometryMsgPose(const cartographer::transform::Rigid3d& rigid3d);
geometry_msgs::msg::Point toGeometryMsgPoint(const Eigen::Vector3d& vector3d);
rclcpp::Time toRos(cartographer::common::Time time);
cartographer::common::Time fromRos(const rclcpp::Time& time);

} // namespace rf_map_builder