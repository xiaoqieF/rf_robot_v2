#include "rf_map_builder/msg_conversion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <Eigen/src/Geometry/AngleAxis.h>
#include <cartographer/sensor/landmark_data.h>
#include <cartographer/sensor/point_cloud.h>
#include <cartographer/sensor/rangefinder_point.h>
#include <cartographer/transform/rigid_transform.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <tuple>

namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT {
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  float unused_padding[2];
};

}  // namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, time, time))

namespace rf_map_builder
{

using cartographer::transform::Rigid3d;
using cartographer::sensor::LandmarkData;
using cartographer::sensor::PointCloudWithIntensities;

// The ros::sensor_msgs::msg::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

namespace
{
    bool pointCloud2HasField(const sensor_msgs::msg::PointCloud2& msg, const std::string& field_name) {
        for (const auto& field : msg.fields) {
            if (field.name == field_name) {
                return true;
            }
        }
        return false;
    }
}

Rigid3d toRigid3d(const geometry_msgs::msg::TransformStamped& transform) {
    return Rigid3d(toEigen(transform.transform.translation),
                      toEigen(transform.transform.rotation));
}

Rigid3d toRigid3d(const geometry_msgs::msg::Pose& pose) {
    return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                   toEigen(pose.orientation));
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& vector3) {
    return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion& quaternion) {
    return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                              quaternion.z);
}

LandmarkData toLandmarkData(const rf_robot_msgs::msg::LandmarkList& landmark_list)
{
    LandmarkData landmark_data;
    landmark_data.time = fromRos(landmark_list.header.stamp);
    for (const auto& entry : landmark_list.landmarks) {
        landmark_data.landmark_observations.push_back(
            {entry.id, toRigid3d(entry.tracking_from_landmark_transform),
                entry.translation_weight, entry.rotation_weight});
    }
    return landmark_data;
}

// Converts cartographer::common::Time to rclcpp::Time, 100ns resolution.
rclcpp::Time toRos(cartographer::common::Time time)
{
    int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
    int64_t ns_since_unix_epoch =
        (uts_timestamp - cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
            10000000ll) * 100ll;
    rclcpp::Time ros_time(ns_since_unix_epoch, rcl_clock_type_t::RCL_ROS_TIME);
    return ros_time;
}

cartographer::common::Time fromRos(const rclcpp::Time& time)
{
    // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
    // exactly 719162 days before the Unix epoch.
    return ::cartographer::common::FromUniversal(
        cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll +
        (time.nanoseconds() + 50) / 100);  // + 50 to get the rounding correct.
}

sensor_msgs::msg::PointCloud2 preparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = toRos(::cartographer::common::FromUniversal(timestamp));
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = num_points;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step = 16 * msg.width;
    msg.is_dense = true;
    msg.data.resize(16 * num_points);
    return msg;
}

sensor_msgs::msg::PointCloud2 toPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud)
{
    auto msg = preparePointCloud2Message(timestamp, frame_id, point_cloud.size());
    size_t offset = 0;
    float * const data = reinterpret_cast<float*>(&msg.data[0]);
    for (const auto& point : point_cloud) {
        data[offset++] = point.position.x();
        data[offset++] = point.position.y();
        data[offset++] = point.position.z();
        data[offset++] = kPointCloudComponentFourMagic;
    }
    return msg;
}

std::tuple<PointCloudWithIntensities, cartographer::common::Time>
    toPointCloudWithIntensities(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    PointCloudWithIntensities point_cloud;
    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++ i) {
        auto echo = msg->ranges[i];
        if (echo >= msg->range_min && echo <= msg->range_max) {
            const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
            const cartographer::sensor::TimedRangefinderPoint point {
                rotation * Eigen::Vector3f(echo, 0.f, 0.f),
                msg->time_increment * i
            };
            point_cloud.points.push_back(point);
            if (msg->intensities.size() > 0) {
                point_cloud.intensities.push_back(msg->intensities[i]);
            } else {
                point_cloud.intensities.push_back(0.f);
            }
        }
        angle += msg->angle_increment;
    }
    // 在 carto 中, 点云的时间戳是最后一个点的时间，其中所有点的时间是相对于最后一个点的时间偏移的
    // 因此所有点的时间都是非正值
    auto timestamp = fromRos(msg->header.stamp);
    if (!point_cloud.points.empty()) {
        const double duration = point_cloud.points.back().time;
        timestamp += cartographer::common::FromSeconds(duration);
        for (auto& point : point_cloud.points) {
            point.time -= duration;
        }
    }

    return std::make_tuple(point_cloud, timestamp);
}

std::tuple<cartographer::sensor::PointCloudWithIntensities, cartographer::common::Time>
    toPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    PointCloudWithIntensities point_cloud;
    // We check for intensity field here to avoid run-time warnings if we pass in
    // a PointCloud2 without intensity.
    if (pointCloud2HasField(*msg, "intensity")) {
        if (pointCloud2HasField(*msg, "time")) {
            pcl::PointCloud<PointXYZIT> pcl_point_cloud;
            pcl::fromROSMsg(*msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
                point_cloud.intensities.push_back(point.intensity);
            }
        } else {
            pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
            pcl::fromROSMsg(*msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
                point_cloud.intensities.push_back(point.intensity);
            }
        }
    } else {
        // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
        if (pointCloud2HasField(*msg, "time")) {
            pcl::PointCloud<PointXYZT> pcl_point_cloud;
            pcl::fromROSMsg(*msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
                point_cloud.intensities.push_back(1.0f);
            }
        } else {
            pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
            pcl::fromROSMsg(*msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
                point_cloud.intensities.push_back(1.0f);
            }
        }
    }
    auto timestamp = fromRos(msg->header.stamp);
    if (!point_cloud.points.empty()) {
        // 点云时间戳是最后一个点的时间
        const double duration = point_cloud.points.back().time;
        timestamp += cartographer::common::FromSeconds(duration);
        for (auto& point : point_cloud.points) {
            point.time -= duration;
        }
    }
    return std::make_tuple(point_cloud, timestamp);
}

geometry_msgs::msg::Transform toGeometryMsgTransform(const ::cartographer::transform::Rigid3d& rigid3d)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = rigid3d.translation().x();
    transform.translation.y = rigid3d.translation().y();
    transform.translation.z = rigid3d.translation().z();
    transform.rotation.w = rigid3d.rotation().w();
    transform.rotation.x = rigid3d.rotation().x();
    transform.rotation.y = rigid3d.rotation().y();
    transform.rotation.z = rigid3d.rotation().z();
    return transform;
}

geometry_msgs::msg::Pose toGeometryMsgPose(const ::cartographer::transform::Rigid3d& rigid3d)
{
    geometry_msgs::msg::Pose pose;
    pose.position = toGeometryMsgPoint(rigid3d.translation());
    pose.orientation.w = rigid3d.rotation().w();
    pose.orientation.x = rigid3d.rotation().x();
    pose.orientation.y = rigid3d.rotation().y();
    pose.orientation.z = rigid3d.rotation().z();
    return pose;
}

geometry_msgs::msg::Point toGeometryMsgPoint(const Eigen::Vector3d& vector3d)
{
    geometry_msgs::msg::Point point;
    point.x = vector3d.x();
    point.y = vector3d.y();
    point.z = vector3d.z();
    return point;
}

} // namespace rf_map_builder