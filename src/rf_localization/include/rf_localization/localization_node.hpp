#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rf_localization/common.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rf_localization/localization_method.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace rf_localization
{

using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();
    ~LocalizationNode() override;

    void init();

private:
    struct NodeOptions
    {
        std::string localization_method = "icp";
        std::string map_frame = "map";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        double tf_publish_period_sec = 0.05;
        double tf_lookup_timeout_sec = 0.1;
        double initial_pose_x = 0.0;
        double initial_pose_y = 0.0;
        double initial_pose_yaw = 0.0;
        double max_map_to_odom_translation_jump = 0.08;
        double max_map_to_odom_yaw_jump = 0.08;
        double map_to_odom_smoothing_alpha = 0.2;
        bool reject_large_map_to_odom_jump = true;
        bool publish_aligned_scan = true;
        bool autostart = true;
    };

private:
    void handleMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);
    void handleOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void handleScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void handleControlRequest(const ReqAckSrvT::Request::SharedPtr request,
                              const ReqAckSrvT::Response::SharedPtr response);
    void localizationWorkerLoop();
    void processScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void publishMapToOdomTransform(const rclcpp::Time& stamp);
    void publishLocalizedPose(const Eigen::Matrix4f& map_to_base, const rclcpp::Time& stamp);
    void publishAlignedScan(const PointCloudT& aligned_scan, const rclcpp::Time& stamp);
    bool initializeLocalizationMethod();
    std::optional<PointCloudT::Ptr> laserScanToPointCloud(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    std::optional<Eigen::Matrix4f> computeOdomToBase(const nav_msgs::msg::Odometry& msg) const;
    Eigen::Matrix4f makePlanarTransform(double x, double y, double yaw) const;
    Eigen::Matrix4f normalizePlanarTransform(const Eigen::Matrix4f& transform) const;
    Eigen::Matrix4f blendPlanarTransform(
        const Eigen::Matrix4f& current,
        const Eigen::Matrix4f& target,
        double alpha) const;
    double planarTransformYaw(const Eigen::Matrix4f& transform) const;
    double normalizeAngle(double angle) const;
    geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f& transform) const;
    geometry_msgs::msg::Transform matrixToTransform(const Eigen::Matrix4f& transform) const;

private:
    NodeOptions options_;
    std::unique_ptr<LocalizationMethod> localization_method_;

    laser_geometry::LaserProjection laser_projector_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_publisher_;
    rclcpp::Service<ReqAckSrvT>::SharedPtr control_service_;
    rclcpp::TimerBase::SharedPtr tf_publish_timer_;

    mutable std::mutex mutex_;
    std::condition_variable scan_cv_;
    std::thread localization_worker_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr pending_scan_msg_;
    Eigen::Matrix4f initial_map_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f map_to_odom_ = Eigen::Matrix4f::Identity();
    std::optional<Eigen::Matrix4f> latest_odom_to_base_;
    bool map_received_ = false;
    bool map_to_odom_initialized_ = false;
    bool active_ = true;
    bool stop_worker_ = false;
};

} // namespace rf_localization
