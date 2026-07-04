#include "rf_map_builder/map_builder_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rf_map_builder/config_options.hpp"
#include "cartographer/mapping/map_builder.h"
#include "rf_map_builder/msg_conversion.hpp"
#include "rf_map_builder/sensor_bridge.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cartographer/sensor/point_cloud.h>
#include <cartographer/sensor/rangefinder_point.h>
#include <cartographer/transform/transform.h>
#include <rclcpp/subscription_base.hpp>
#include <tf2/time.hpp>

namespace rf_map_builder
{

namespace {
template <typename MsgT>
rclcpp::SubscriptionBase::SharedPtr createSubscription(
    rclcpp::Node* node, const std::string& topic,
    std::function<void(const std::string&, const typename MsgT::ConstSharedPtr)> callback) {
    return node->create_subscription<MsgT>(
        topic, rclcpp::SensorDataQoS(),
        [topic, callback] (const typename MsgT::ConstSharedPtr msg) {
            callback(topic, msg);
        });
}

} // namespace

MapBuilderNode::MapBuilderNode()
    : Node("map_builder_node")
{
    MAP_BUILDER_INFO("Initializing MapBuilderNode...");

    std::string cfg_file = "default_map_builder_cfg.lua";
    std::vector<std::string> cfg_search_paths;
    cfg_search_paths.emplace_back(
        ament_index_cpp::get_package_share_directory("rf_map_builder") + "/config");

    if (const char* home = std::getenv("HOME"); home != nullptr) {
        cfg_search_paths.emplace_back(std::string(home) + "/.rf_robot/config");
    }

    std::tie(node_options_, trajectory_options_) = loadOptions(cfg_search_paths, cfg_file);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10), this);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    auto map_builder = cartographer::mapping::CreateMapBuilder(node_options_.map_builder_options);

    map_builder_bridge_ = std::make_unique<MapBuilderBridge>(
        node_options_,
        std::move(map_builder),
        tf_buffer_.get());

    sensor_samplers_ = std::make_unique<TrajectorySensorSamplers>(
        trajectory_options_.rangefinder_sampling_ratio,
        trajectory_options_.odometry_sampling_ratio,
        trajectory_options_.fixed_frame_pose_sampling_ratio,
        trajectory_options_.imu_sampling_ratio,
        trajectory_options_.landmarks_sampling_ratio);
    pose_extrapolator_ = std::make_unique<cartographer::mapping::PoseExtrapolator>(
        cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec), kGravityTimeConstant
    );

    // Publishers
    submap_list_publisher_ = this->create_publisher<SubmapListMsgT>(kSubmapListTopic, 10);
    tracked_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kTrackedPoseTopic, 10);
    scan_matched_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(kScanMatchedPointCloudTopic, 10);

    submap_list_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(node_options_.submap_publish_period_sec * 1000)),
        [this]() {
            publishSubmapList();
        });

    local_trajectory_data_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(node_options_.pose_publish_period_sec * 1000)),
        [this]() {
            publishLocalTrajectoryData();
        });

    // Sensor subscriptions
    for (const auto& topic : computeRepeatedTopicNames(kLaserScanTopic, trajectory_options_.num_laser_scans)) {
        sensor_subscriptions_.push_back(createSubscription<sensor_msgs::msg::LaserScan>(
            this, topic,
            [this](const std::string& topic, const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                handleLaserScanMessage(topic, msg);
            }));
    }

    for (const auto& topic : computeRepeatedTopicNames(kPointCloud2Topic, trajectory_options_.num_point_clouds)) {
        sensor_subscriptions_.push_back(createSubscription<sensor_msgs::msg::PointCloud2>(
            this, topic,
            [this](const std::string& topic, const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                handlePointCloud2Message(topic, msg);
            }));
    }

    if (trajectory_options_.use_odometry) {
        sensor_subscriptions_.push_back(createSubscription<nav_msgs::msg::Odometry>(
            this, kOdometryTopic,
            [this](const std::string& topic, const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                handleOdometryMessage(topic, msg);
            }));
    }

    if (trajectory_options_.use_landmarks) {
        sensor_subscriptions_.push_back(createSubscription<rf_robot_msgs::msg::LandmarkList>(
            this, kLandmarkTopic,
            [this](const std::string& topic, const rf_robot_msgs::msg::LandmarkList::ConstSharedPtr msg) {
                handleLandmarkMessage(topic, msg);
            }));
    }

    if (node_options_.map_builder_options.use_trajectory_builder_2d() &&
        trajectory_options_.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data()) {
        sensor_subscriptions_.push_back(createSubscription<sensor_msgs::msg::Imu>(
            this, kImuTopic,
            [this](const std::string& topic, const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                handleImuMessage(topic, msg);
            }));
    }
}

void MapBuilderNode::init()
{
    build_map_srv_ = this->create_service<ReqAckSrvT>(
        "/build_map",
        [this](const std::shared_ptr<ReqAckSrvT::Request> request,
               std::shared_ptr<ReqAckSrvT::Response> response) {
            (void)request;
            std::lock_guard<std::mutex> lock(mutex_);
            if (addTrajectory(trajectory_options_) != -1) {
                response->ack = true;
                MAP_BUILDER_INFO("Started trajectory with ID: {}", current_trajectory_id_.load());
            } else {
                response->ack = false;
                MAP_BUILDER_WARN("Failed to start a new trajectory.");
            }
        });

}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    MapBuilderNode::computeExpectedSensorIds(const TrajectoryOptions& options) const
{
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;

    std::set<SensorId> expected_topics;
    for (const auto& topic : computeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const auto& topic : computeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    if (node_options_.map_builder_options.use_trajectory_builder_2d() &&
        options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data()) {
        expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
    }
    if (options.use_odometry) {
        expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
    }
    if (options.use_landmarks) {
        expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    }

    return expected_topics;
}

int MapBuilderNode::addTrajectory(const TrajectoryOptions& trajectory_options)
{
    if (current_trajectory_id_ != -1) {
        MAP_BUILDER_WARN("Current trajectory already exists with ID: {}. Cannot add another trajectory.",
            current_trajectory_id_.load());
        return current_trajectory_id_;
    }
    const auto expected_sensor_ids = computeExpectedSensorIds(trajectory_options);
    current_trajectory_id_ = map_builder_bridge_->addTrajectory(expected_sensor_ids, trajectory_options);
    return current_trajectory_id_;
}

void MapBuilderNode::handleLaserScanMessage(const std::string& sensor_id,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    if (current_trajectory_id_ == -1) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sensor_samplers_->rangefinder_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensorBridge(current_trajectory_id_)->handleLaserScanMessage(sensor_id, msg);
}

void MapBuilderNode::handlePointCloud2Message(const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    if (current_trajectory_id_ == -1) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sensor_samplers_->rangefinder_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensorBridge(current_trajectory_id_)->handlePointCloud2Message(sensor_id, msg);
}

void MapBuilderNode::handleImuMessage(const std::string& sensor_id,
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    if (current_trajectory_id_ == -1) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sensor_samplers_->imu_sampler.Pulse()) {
        return;
    }
    auto imu_data = map_builder_bridge_->sensorBridge(current_trajectory_id_)->toImuData(msg);
    if (imu_data) {
        pose_extrapolator_->AddImuData(*imu_data);
    }
    map_builder_bridge_->sensorBridge(current_trajectory_id_)->handleImuMessage(sensor_id, msg);
}

void MapBuilderNode::handleOdometryMessage(const std::string& sensor_id,
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    if (current_trajectory_id_ == -1) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sensor_samplers_->odometry_sampler.Pulse()) {
        return;
    }
    auto odom_data = map_builder_bridge_->sensorBridge(current_trajectory_id_)->toOdometryData(msg);
    if (odom_data == nullptr) {
        return;
    }
    pose_extrapolator_->AddOdometryData(*odom_data);
    map_builder_bridge_->sensorBridge(current_trajectory_id_)->handleOdometryMessage(sensor_id, msg);
}

void MapBuilderNode::handleLandmarkMessage(const std::string& sensor_id,
    const rf_robot_msgs::msg::LandmarkList::ConstSharedPtr& msg)
{
    if (current_trajectory_id_ == -1) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sensor_samplers_->landmarks_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensorBridge(current_trajectory_id_)->handleLandmarkMessage(sensor_id, msg);
}

void MapBuilderNode::publishSubmapList()
{
    if (current_trajectory_id_ == -1) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto submap_list_msg = map_builder_bridge_->getSubmapList(this->now());
    submap_list_publisher_->publish(submap_list_msg);
}

void MapBuilderNode::publishLocalTrajectoryData()
{
    if (current_trajectory_id_ == -1) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto local_trajectory_data_msg = map_builder_bridge_->getLocalTrajectoryData();
    if (local_trajectory_data_msg.find(current_trajectory_id_) == local_trajectory_data_msg.end()) {
        return;
    }
    const auto& local_trajectory_data = local_trajectory_data_msg.at(current_trajectory_id_);

    if (local_trajectory_data.local_slam_data->time != pose_extrapolator_->GetLastPoseTime()) {
        if (scan_matched_points_publisher_->get_subscription_count() > 0) {
            cartographer::sensor::TimedPointCloud point_cloud;
            point_cloud.reserve(local_trajectory_data.local_slam_data->range_data_in_local.returns.size());
            for (const auto& point : local_trajectory_data.local_slam_data->range_data_in_local.returns) {
                point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(point, 0.f));
            }
            scan_matched_points_publisher_->publish(toPointCloud2Message(
                cartographer::common::ToUniversal(local_trajectory_data.local_slam_data->time),
                node_options_.map_frame,
                cartographer::sensor::TransformTimedPointCloud(
                    point_cloud, local_trajectory_data.local_to_map.cast<float>())));
        }

        pose_extrapolator_->AddPose(
            local_trajectory_data.local_slam_data->time,
            local_trajectory_data.local_slam_data->local_pose);
    }

    auto now = std::max(fromRos(this->now()), pose_extrapolator_->GetLastPoseTime());
    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = toRos(now);

    const auto tracking_to_local_3d = pose_extrapolator_->ExtrapolatePose(now);

    auto tracking_to_local = [&] {
        if (local_trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
            return cartographer::transform::Embed3D(
                cartographer::transform::Project2D(tracking_to_local_3d));
        }
        return tracking_to_local_3d;
    }();

    const auto tracking_to_map = local_trajectory_data.local_to_map * tracking_to_local;

    // publish to tf (map -> odom)
    stamped_transform.header.frame_id = node_options_.map_frame;
    stamped_transform.child_frame_id = local_trajectory_data.trajectory_options.odom_frame;

    auto odom_to_tracking = map_builder_bridge_->sensorBridge(current_trajectory_id_)->tfBridge().lookupToTracking(
        now, local_trajectory_data.trajectory_options.odom_frame);

    if (odom_to_tracking != nullptr) {
        stamped_transform.transform = toGeometryMsgTransform(tracking_to_map * (*odom_to_tracking));
        tf_broadcaster_->sendTransform(stamped_transform);
    } else {
        MAP_BUILDER_WARN("Cannot publish transform from {} to {} at time {}",
            node_options_.map_frame,
            local_trajectory_data.trajectory_options.odom_frame,
            toRos(now).seconds());
    }

    // publish tracked pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = node_options_.map_frame;
    pose_msg.header.stamp = stamped_transform.header.stamp;
    pose_msg.pose = toGeometryMsgPose(tracking_to_map);
    tracked_pose_publisher_->publish(pose_msg);
}

} // namespace rf_map_builder
