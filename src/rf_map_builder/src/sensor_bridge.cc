#include "rf_map_builder/sensor_bridge.hpp"
#include "rf_map_builder/msg_conversion.hpp"
#include <cartographer/common/time.h>
#include <cartographer/sensor/imu_data.h>
#include <cartographer/sensor/odometry_data.h>
#include <cartographer/sensor/timed_point_cloud_data.h>
#include <memory>
#include <tf2/convert.hpp>

namespace rf_map_builder
{

using namespace cartographer;

std::unique_ptr<cartographer::sensor::OdometryData> SensorBridge::toOdometryData(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    const common::Time time = fromRos(msg->header.stamp);
    // 返回的是 sensor 系到 tracking 系(通常是 base_link) 的变换
    // 也就是 sensor系 在 tracking 系下的位姿 T^track_sensor (sensor_to_tracking)
    // P_track = T^track_sensor * P_sensor
    const auto sensor_to_tracking = tf_bridge_.lookupToTracking(time, msg->child_frame_id);
    if (sensor_to_tracking == nullptr) {
        return nullptr;
    }
    // 对于 odom msg，pose 是在 odom 系下的位姿 T^odom_sensor
    // 因此需要得到 odom 系下 tracking 的位姿 T^odom_track = T^odom_sensor * (T^track_sensor)^-1
    return std::make_unique<cartographer::sensor::OdometryData>(
        cartographer::sensor::OdometryData{time, toRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

std::unique_ptr<cartographer::sensor::ImuData> SensorBridge::toImuData(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    if (msg->linear_acceleration_covariance[0] == -1 ||
        msg->angular_velocity_covariance[0] == -1) {
        MAP_BUILDER_WARN("Ignoring IMU message with no covariance.");
        return nullptr;
    }

    const auto time = fromRos(msg->header.stamp);
    auto sensor_to_tracking =
        tf_bridge_.lookupToTracking(time, msg->header.frame_id);
    if (sensor_to_tracking == nullptr) {
        return nullptr;
    }

    // Imu安装必须处在 tracking 系中心位置
    if (sensor_to_tracking->translation().norm() > 1e-5) {
        MAP_BUILDER_WARN(
            "Ignoring IMU message with non-zero translation between tracking frame "
            "and sensor frame, translation: x: {:.3f}, y: {:.3f}, z: {:.3f}",
            sensor_to_tracking->translation().x(),
            sensor_to_tracking->translation().y(),
            sensor_to_tracking->translation().z());
        return nullptr;
    }

    return std::make_unique<sensor::ImuData>(
        cartographer::sensor::ImuData{time, sensor_to_tracking->rotation() * toEigen(msg->linear_acceleration),
        sensor_to_tracking->rotation() * toEigen(msg->angular_velocity)});
}

void SensorBridge::handleOdometryMessage(const std::string& sensor_id,
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    auto odom_data = toOdometryData(msg);
    if (odom_data == nullptr) {
        return;
    }
    trajectory_builder_->AddSensorData(sensor_id, *odom_data);
}

void SensorBridge::handleLandmarkMessage(const std::string& sensor_id,
    const rf_robot_msgs::msg::LandmarkList::ConstSharedPtr& msg)
{
    auto landmark_data = toLandmarkData(*msg);

    // 将 landmark 在 sensor 系下的位姿转换到 tracking 系(通常是 base_link)下
    auto landmark_to_tracking =
        tf_bridge_.lookupToTracking(landmark_data.time, msg->header.frame_id);
    if (landmark_to_tracking != nullptr) {
        for (auto& observation : landmark_data.landmark_observations) {
            observation.landmark_to_tracking_transform = *landmark_to_tracking *
                observation.landmark_to_tracking_transform;
        }
    }
    trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

void SensorBridge::handleImuMessage(const std::string& sensor_id,
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    auto img_data = toImuData(msg);
    if (img_data == nullptr) {
        return;
    }
    trajectory_builder_->AddSensorData(sensor_id, *img_data);
}

void SensorBridge::handleLaserScanMessage(const std::string& sensor_id,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    auto [point_cloud, timestamp] = toPointCloudWithIntensities(msg);

    if (point_cloud.points.empty()) {
        return;
    }
    auto sensor_to_tracking =
        tf_bridge_.lookupToTracking(timestamp, msg->header.frame_id);
    if (sensor_to_tracking != nullptr) {
        trajectory_builder_->AddSensorData(sensor_id,
            sensor::TimedPointCloudData{
                timestamp,
                sensor_to_tracking->translation().cast<float>(),
                sensor::TransformTimedPointCloud(point_cloud.points, sensor_to_tracking->cast<float>()),
                point_cloud.intensities
            });
    }
}

void SensorBridge::handlePointCloud2Message(const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    auto [point_cloud, timestamp] = toPointCloudWithIntensities(msg);

    if (point_cloud.points.empty()) {
        return;
    }
    auto sensor_to_tracking =
        tf_bridge_.lookupToTracking(timestamp, msg->header.frame_id);
    if (sensor_to_tracking != nullptr) {
        trajectory_builder_->AddSensorData(sensor_id,
            sensor::TimedPointCloudData{
                timestamp,
                sensor_to_tracking->translation().cast<float>(),
                sensor::TransformTimedPointCloud(point_cloud.points, sensor_to_tracking->cast<float>()),
                point_cloud.intensities
            });
    }

}
} // namespace rf_map_builder