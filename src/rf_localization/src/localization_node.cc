#include "rf_localization/localization_node.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "rf_localization/common.hpp"
#include "rf_localization/icp_localizer.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace rf_localization
{

LocalizationNode::LocalizationNode()
    : Node("localization_node")
{
    options_.localization_method = this->declare_parameter<std::string>("localization_method", "icp");
    options_.map_frame = this->declare_parameter<std::string>("map_frame", "map");
    options_.odom_frame = this->declare_parameter<std::string>("odom_frame", "odom");
    options_.base_frame = this->declare_parameter<std::string>("base_frame", "base_link");
    options_.tf_publish_period_sec = this->declare_parameter<double>("tf_publish_period_sec", 0.05);
    options_.tf_lookup_timeout_sec = this->declare_parameter<double>("tf_lookup_timeout_sec", 0.1);
    options_.initial_pose_x = this->declare_parameter<double>("initial_pose.x", 0.0);
    options_.initial_pose_y = this->declare_parameter<double>("initial_pose.y", 0.0);
    options_.initial_pose_yaw = this->declare_parameter<double>("initial_pose.yaw", 0.0);
    options_.max_map_to_odom_translation_jump = this->declare_parameter<double>(
        "max_map_to_odom_translation_jump", options_.max_map_to_odom_translation_jump);
    options_.max_map_to_odom_yaw_jump = this->declare_parameter<double>(
        "max_map_to_odom_yaw_jump", options_.max_map_to_odom_yaw_jump);
    options_.map_to_odom_smoothing_alpha = this->declare_parameter<double>(
        "map_to_odom_smoothing_alpha", options_.map_to_odom_smoothing_alpha);
    options_.reject_large_map_to_odom_jump = this->declare_parameter<bool>(
        "reject_large_map_to_odom_jump", options_.reject_large_map_to_odom_jump);
    options_.publish_aligned_scan = this->declare_parameter<bool>("publish_aligned_scan", true);
    options_.autostart = this->declare_parameter<bool>("autostart", true);

    active_ = options_.autostart;
    initial_map_pose_ = makePlanarTransform(
        options_.initial_pose_x,
        options_.initial_pose_y,
        options_.initial_pose_yaw);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0), this);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    if (!initializeLocalizationMethod()) {
        throw std::invalid_argument("Unsupported localization method: " + options_.localization_method);
    }
}

LocalizationNode::~LocalizationNode()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_worker_ = true;
    }
    scan_cv_.notify_all();

    if (localization_worker_.joinable()) {
        localization_worker_.join();
    }
}

void LocalizationNode::init()
{
    LOCALIZATION_INFO("Initializing localization node with method: {}", options_.localization_method);

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&LocalizationNode::handleMap, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 50,
        std::bind(&LocalizationNode::handleOdometry, this, std::placeholders::_1));

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&LocalizationNode::handleScan, this, std::placeholders::_1));

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("localization_pose", 10);
    aligned_scan_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "localized_scan", 10);

    control_service_ = this->create_service<ReqAckSrvT>(
        "/localization_control",
        std::bind(
            &LocalizationNode::handleControlRequest,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    tf_publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(options_.tf_publish_period_sec),
        [this]() {
            publishMapToOdomTransform(this->now());
        });

    localization_worker_ = std::thread(&LocalizationNode::localizationWorkerLoop, this);
}

void LocalizationNode::handleMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
{
    if (msg == nullptr) {
        return;
    }

    if (!localization_method_->setMap(*msg)) {
        LOCALIZATION_WARN("Received map, but failed to build ICP target cloud.");
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    map_received_ = true;
    LOCALIZATION_INFO("Localization map updated from topic: /map");
}

void LocalizationNode::handleOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    if (msg == nullptr) {
        return;
    }

    auto odom_to_base = computeOdomToBase(*msg);
    if (!odom_to_base.has_value()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_to_base_ = normalizePlanarTransform(*odom_to_base);
    if (!map_to_odom_initialized_) {
        map_to_odom_ = initial_map_pose_ * latest_odom_to_base_->inverse();
        map_to_odom_initialized_ = true;
        LOCALIZATION_INFO(
            "Initialized map->odom from initial pose [{:.3f}, {:.3f}, {:.3f}].",
            options_.initial_pose_x,
            options_.initial_pose_y,
            options_.initial_pose_yaw);
    }
}

void LocalizationNode::handleScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    if (msg == nullptr) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_scan_msg_ = msg;
    }
    scan_cv_.notify_one();
}

void LocalizationNode::localizationWorkerLoop()
{
    while (true) {
        sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            scan_cv_.wait(lock, [this]() {
                return stop_worker_ || pending_scan_msg_ != nullptr;
            });

            if (stop_worker_) {
                return;
            }

            scan_msg = pending_scan_msg_;
            pending_scan_msg_.reset();
        }

        processScan(scan_msg);
    }
}

void LocalizationNode::processScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    if (msg == nullptr) {
        return;
    }

    Eigen::Matrix4f odom_to_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f map_to_odom = Eigen::Matrix4f::Identity();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || !map_received_ || !map_to_odom_initialized_ || !latest_odom_to_base_.has_value()) {
            return;
        }
        odom_to_base = *latest_odom_to_base_;
        map_to_odom = map_to_odom_;
    }

    auto scan_cloud = laserScanToPointCloud(msg);
    if (!scan_cloud.has_value() || *scan_cloud == nullptr || (*scan_cloud)->empty()) {
        return;
    }

    LocalizationInput input;
    input.stamp = msg->header.stamp;
    input.scan_in_base = *scan_cloud;
    input.initial_guess = normalizePlanarTransform(map_to_odom * odom_to_base);

    auto result = localization_method_->localize(input);
    if (!result.has_value()) {
        return;
    }

    const Eigen::Matrix4f localized_map_to_base = normalizePlanarTransform(result->pose_in_map);
    const Eigen::Matrix4f candidate_map_to_odom =
        normalizePlanarTransform(localized_map_to_base * odom_to_base.inverse());
    Eigen::Matrix4f stabilized_map_to_odom = candidate_map_to_odom;
    Eigen::Matrix4f stabilized_map_to_base = localized_map_to_base;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double dx = static_cast<double>(candidate_map_to_odom(0, 3) - map_to_odom_(0, 3));
        const double dy = static_cast<double>(candidate_map_to_odom(1, 3) - map_to_odom_(1, 3));
        const double translation_jump = std::hypot(dx, dy);
        const double yaw_jump = std::abs(normalizeAngle(
            planarTransformYaw(candidate_map_to_odom) - planarTransformYaw(map_to_odom_)));

        if (options_.reject_large_map_to_odom_jump &&
            (translation_jump > options_.max_map_to_odom_translation_jump ||
             yaw_jump > options_.max_map_to_odom_yaw_jump)) {
            LOCALIZATION_WARN(
                "Rejected map->odom correction jump: translation={:.3f} m yaw={:.3f} rad fitness={:.4f}",
                translation_jump,
                yaw_jump,
                result->fitness_score);
            stabilized_map_to_odom = map_to_odom_;
        } else {
            stabilized_map_to_odom = blendPlanarTransform(
                map_to_odom_,
                candidate_map_to_odom,
                options_.map_to_odom_smoothing_alpha);
            map_to_odom_ = stabilized_map_to_odom;
        }

        stabilized_map_to_base = normalizePlanarTransform(stabilized_map_to_odom * odom_to_base);
    }

    publishLocalizedPose(stabilized_map_to_base, msg->header.stamp);
    publishMapToOdomTransform(msg->header.stamp);

    if (options_.publish_aligned_scan) {
        publishAlignedScan(*result->aligned_scan_in_map, msg->header.stamp);
    }
}

void LocalizationNode::handleControlRequest(
    const ReqAckSrvT::Request::SharedPtr request,
    const ReqAckSrvT::Response::SharedPtr response)
{
    if (request == nullptr || response == nullptr) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = (request->trigger == ReqAckSrvT::Request::START);
    }

    response->ack = ReqAckSrvT::Response::OK;
    LOCALIZATION_INFO("Localization {}.",
        active_ ? "started" : "stopped");
}

void LocalizationNode::publishMapToOdomTransform(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = options_.map_frame;
    transform_stamped.child_frame_id = options_.odom_frame;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!map_to_odom_initialized_) {
            return;
        }
        transform_stamped.transform = matrixToTransform(map_to_odom_);
    }

    tf_broadcaster_->sendTransform(transform_stamped);
}

void LocalizationNode::publishLocalizedPose(const Eigen::Matrix4f& map_to_base, const rclcpp::Time& stamp)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = options_.map_frame;
    pose_msg.pose = matrixToPose(map_to_base);
    pose_publisher_->publish(pose_msg);
}

void LocalizationNode::publishAlignedScan(const PointCloudT& aligned_scan, const rclcpp::Time& stamp)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(aligned_scan, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = options_.map_frame;
    aligned_scan_publisher_->publish(cloud_msg);
}

bool LocalizationNode::initializeLocalizationMethod()
{
    if (options_.localization_method == "icp") {
        IcpLocalizer::Options icp_options;
        icp_options.map_occupied_threshold =
            this->declare_parameter<int>("icp.map_occupied_threshold", 65);
        icp_options.min_scan_points =
            this->declare_parameter<int>("icp.min_scan_points", 20);
        icp_options.map_voxel_leaf_size = static_cast<float>(
            this->declare_parameter<double>("icp.map_voxel_leaf_size", 0.05));
        icp_options.scan_voxel_leaf_size = static_cast<float>(
            this->declare_parameter<double>("icp.scan_voxel_leaf_size", 0.05));
        icp_options.max_correspondence_distance = static_cast<float>(
            this->declare_parameter<double>("icp.max_correspondence_distance", 0.5));
        icp_options.transformation_epsilon = static_cast<float>(
            this->declare_parameter<double>("icp.transformation_epsilon", 1e-4));
        icp_options.fitness_epsilon = static_cast<float>(
            this->declare_parameter<double>("icp.fitness_epsilon", 1e-3));
        icp_options.fitness_score_threshold = static_cast<float>(
            this->declare_parameter<double>("icp.fitness_score_threshold", 0.3));
        icp_options.max_iterations =
            this->declare_parameter<int>("icp.max_iterations", 50);

        localization_method_ = std::make_unique<IcpLocalizer>(icp_options);
        return true;
    }

    return false;
}

std::optional<PointCloudT::Ptr> LocalizationNode::laserScanToPointCloud(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    try {
        laser_projector_.transformLaserScanToPointCloud(
            options_.base_frame,
            *msg,
            cloud_msg,
            *tf_buffer_);
    } catch (const tf2::TransformException& ex) {
        if (msg->header.frame_id != options_.base_frame) {
            LOCALIZATION_WARN(
                "Failed to transform laser scan from {} to {}: {}",
                msg->header.frame_id,
                options_.base_frame,
                ex.what());
            return std::nullopt;
        }

        try {
            laser_projector_.projectLaser(*msg, cloud_msg);
        } catch (const std::runtime_error& runtime_ex) {
            LOCALIZATION_WARN("Failed to project laser scan: {}", runtime_ex.what());
            return std::nullopt;
        }
    } catch (const std::runtime_error& ex) {
        LOCALIZATION_WARN("Failed to project laser scan: {}", ex.what());
        return std::nullopt;
    }

    auto cloud = std::make_shared<PointCloudT>();
    pcl::fromROSMsg(cloud_msg, *cloud);
    return cloud;
}

std::optional<Eigen::Matrix4f> LocalizationNode::computeOdomToBase(
    const nav_msgs::msg::Odometry& msg) const
{
    Eigen::Matrix4f odom_to_child = normalizePlanarTransform(makePlanarTransform(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        tf2::getYaw(msg.pose.pose.orientation)));

    if (msg.child_frame_id.empty() || msg.child_frame_id == options_.base_frame) {
        return odom_to_child;
    }

    try {
        const auto base_to_child = tf_buffer_->lookupTransform(
            options_.base_frame,
            msg.child_frame_id,
            msg.header.stamp,
            tf2::durationFromSec(options_.tf_lookup_timeout_sec));

        tf2::Transform base_to_child_tf;
        tf2::fromMsg(base_to_child.transform, base_to_child_tf);
        tf2::Matrix3x3 rotation(base_to_child_tf.getRotation());
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        rotation.getRPY(roll, pitch, yaw);

        const Eigen::Matrix4f base_to_child_matrix = makePlanarTransform(
            base_to_child.transform.translation.x,
            base_to_child.transform.translation.y,
            yaw);
        return normalizePlanarTransform(odom_to_child * base_to_child_matrix.inverse());
    } catch (const tf2::TransformException& ex) {
        LOCALIZATION_WARN(
            "Failed to resolve odom child frame {} to base frame {}: {}",
            msg.child_frame_id,
            options_.base_frame,
            ex.what());
    }

    return std::nullopt;
}

Eigen::Matrix4f LocalizationNode::makePlanarTransform(double x, double y, double yaw) const
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = static_cast<float>(std::cos(yaw));
    transform(0, 1) = static_cast<float>(-std::sin(yaw));
    transform(1, 0) = static_cast<float>(std::sin(yaw));
    transform(1, 1) = static_cast<float>(std::cos(yaw));
    transform(0, 3) = static_cast<float>(x);
    transform(1, 3) = static_cast<float>(y);
    return transform;
}

Eigen::Matrix4f LocalizationNode::normalizePlanarTransform(const Eigen::Matrix4f& transform) const
{
    const float yaw = static_cast<float>(planarTransformYaw(transform));
    return makePlanarTransform(transform(0, 3), transform(1, 3), yaw);
}

Eigen::Matrix4f LocalizationNode::blendPlanarTransform(
    const Eigen::Matrix4f& current,
    const Eigen::Matrix4f& target,
    double alpha) const
{
    const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
    const double blended_x = (1.0 - clamped_alpha) * static_cast<double>(current(0, 3)) +
        clamped_alpha * static_cast<double>(target(0, 3));
    const double blended_y = (1.0 - clamped_alpha) * static_cast<double>(current(1, 3)) +
        clamped_alpha * static_cast<double>(target(1, 3));
    const double yaw_delta = normalizeAngle(planarTransformYaw(target) - planarTransformYaw(current));
    const double blended_yaw = planarTransformYaw(current) + clamped_alpha * yaw_delta;
    return makePlanarTransform(blended_x, blended_y, blended_yaw);
}

double LocalizationNode::planarTransformYaw(const Eigen::Matrix4f& transform) const
{
    return std::atan2(static_cast<double>(transform(1, 0)), static_cast<double>(transform(0, 0)));
}

double LocalizationNode::normalizeAngle(double angle) const
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Pose LocalizationNode::matrixToPose(const Eigen::Matrix4f& transform) const
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform(0, 3);
    pose.position.y = transform(1, 3);
    pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, planarTransformYaw(transform));
    pose.orientation = tf2::toMsg(quaternion);
    return pose;
}

geometry_msgs::msg::Transform LocalizationNode::matrixToTransform(const Eigen::Matrix4f& transform) const
{
    geometry_msgs::msg::Transform tf_msg;
    tf_msg.translation.x = transform(0, 3);
    tf_msg.translation.y = transform(1, 3);
    tf_msg.translation.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, planarTransformYaw(transform));
    tf_msg.rotation = tf2::toMsg(quaternion);
    return tf_msg;
}

} // namespace rf_localization
