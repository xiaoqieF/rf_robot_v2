#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rf_map_builder/config_options.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rf_map_builder/map_builder_bridge.hpp"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rf_robot_msgs/msg/landmark_list.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"
#include <cartographer/mapping/trajectory_builder_interface.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace rf_map_builder
{

using SubmapListMsgT = rf_robot_msgs::msg::SubmapList;
using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;

class MapBuilderNode : public rclcpp::Node
{
public:
    explicit MapBuilderNode();

    void init();

private:
    int addTrajectory(const TrajectoryOptions& trajectory_options);
    void publishSubmapList();
    void publishLocalTrajectoryData();
    void handleLaserScanMessage(const std::string& sensor_id,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void handlePointCloud2Message(const std::string& sensor_id,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void handleImuMessage(const std::string& sensor_id,
        const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void handleOdometryMessage(const std::string& sensor_id,
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void handleLandmarkMessage(const std::string& sensor_id,
        const rf_robot_msgs::msg::LandmarkList::ConstSharedPtr& msg);

    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
        computeExpectedSensorIds(const TrajectoryOptions& options) const;

private:
    NodeOptions node_options_;
    TrajectoryOptions trajectory_options_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<MapBuilderBridge> map_builder_bridge_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex mutex_;
    rclcpp::Publisher<SubmapListMsgT>::SharedPtr submap_list_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_points_publisher_;
    rclcpp::TimerBase::SharedPtr submap_list_timer_;
    rclcpp::TimerBase::SharedPtr local_trajectory_data_timer_;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> sensor_subscriptions_;

    struct TrajectorySensorSamplers
    {
        TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                                 const double odometry_sampling_ratio,
                                 const double fixed_frame_pose_sampling_ratio,
                                 const double imu_sampling_ratio,
                                 const double landmarks_sampling_ratio)
            : rangefinder_sampler(rangefinder_sampling_ratio),
              odometry_sampler(odometry_sampling_ratio),
              fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
              imu_sampler(imu_sampling_ratio),
              landmarks_sampler(landmarks_sampling_ratio) {};

        cartographer::common::FixedRatioSampler rangefinder_sampler;
        cartographer::common::FixedRatioSampler odometry_sampler;
        cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
        cartographer::common::FixedRatioSampler imu_sampler;
        cartographer::common::FixedRatioSampler landmarks_sampler;
    };

    std::unique_ptr<TrajectorySensorSamplers> sensor_samplers_;
    std::unique_ptr<cartographer::mapping::PoseExtrapolator> pose_extrapolator_;
    std::atomic_int current_trajectory_id_{-1};

    rclcpp::Service<ReqAckSrvT>::SharedPtr build_map_srv_;
};

} // namespace rf_map_builder