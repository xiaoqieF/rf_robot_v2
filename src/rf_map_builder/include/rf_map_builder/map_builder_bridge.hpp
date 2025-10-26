#pragma once

#include "rf_map_builder/config_options.hpp"

#include <memory>
#include <mutex>
#include <unordered_map>

#include <tf2_ros/buffer.h>
#include "rf_map_builder/sensor_bridge.hpp"
#include "rf_robot_msgs/msg/submap_list.hpp"
#include "cartographer/mapping/map_builder_interface.h"
#include <cartographer/transform/rigid_transform.h>
#include <cartographer/mapping/pose_graph_interface.h>
#include <cartographer/mapping/trajectory_builder_interface.h>

namespace rf_map_builder
{

class SensorBridge;

class MapBuilderBridge
{
public:
    struct LocalTrajectoryData
    {
        struct LocalSlamData
        {
            cartographer::common::Time time;
            cartographer::transform::Rigid3d local_pose;
            cartographer::sensor::RangeData range_data_in_local;
        };

        std::shared_ptr<const LocalSlamData> local_slam_data;
        cartographer::transform::Rigid3d local_to_map;
        TrajectoryOptions trajectory_options;
    };

    MapBuilderBridge(const NodeOptions& node_options,
        std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
        tf2_ros::Buffer* tf_buffer);

    MapBuilderBridge(const MapBuilderBridge&) = delete;
    MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

    int addTrajectory(
        const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
        const TrajectoryOptions& trajectory_options);

    void FinishTrajectory(int trajectory_id);
    void runFinalOptimization();
    std::map<int, cartographer::mapping::PoseGraphInterface::TrajectoryState> getTrajectoryStates();
    std::unordered_map<int, LocalTrajectoryData> getLocalTrajectoryData();
    rf_robot_msgs::msg::SubmapList getSubmapList(rclcpp::Time node_time);

    SensorBridge* sensorBridge(int trajectory_id);

private:
    void onLocalSlamResult(const int trajectory_id,
        const cartographer::common::Time time,
        const cartographer::transform::Rigid3d local_pose,
        cartographer::sensor::RangeData range_data_in_local);

private:
    std::mutex mutex_;
    const NodeOptions node_options_;

    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
    tf2_ros::Buffer* tf_buffer_;

    // keyed with trajectory id
    std::unordered_map<int, std::shared_ptr<const LocalTrajectoryData::LocalSlamData>> local_slam_data_;
    std::unordered_map<int, TrajectoryOptions> trajectory_options_;
    std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
};

}  // namespace rf_map_builder