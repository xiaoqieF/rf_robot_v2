#include <memory>

#include "rf_map_builder/map_builder_bridge.hpp"
#include "rf_map_builder/msg_conversion.hpp"
#include "rf_map_builder/sensor_bridge.hpp"
#include "rf_robot_msgs/msg/submap_entry.hpp"
#include "rf_robot_msgs/msg/submap_list.hpp"
#include <cartographer/transform/rigid_transform.h>
#include <elog/elog.h>
#include <mutex>

namespace rf_map_builder
{
using namespace cartographer;

MapBuilderBridge::MapBuilderBridge(const NodeOptions& node_options,
    std::unique_ptr<mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* tf_buffer)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer) {}

int MapBuilderBridge::addTrajectory(
    const std::set<mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
    const TrajectoryOptions& trajectory_options)
{
    const int traj_id = map_builder_->AddTrajectoryBuilder(
        expected_sensor_ids, trajectory_options.trajectory_builder_options,
        [this] (int trajectory_id, const common::Time time,
            const transform::Rigid3d local_pose, sensor::RangeData range_data,
            const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult>) {
            onLocalSlamResult(trajectory_id, time, local_pose, range_data);
        });

    MAP_BUILDER_INFO("Added trajectory with ID: {}.", traj_id);

    sensor_bridges_[traj_id] = std::make_unique<SensorBridge>(
        trajectory_options.tracking_frame,
        node_options_.lookup_transform_timeout_sec,
        tf_buffer_,
        map_builder_->GetTrajectoryBuilder(traj_id)
    );

    trajectory_options_[traj_id] = trajectory_options;

    return traj_id;
}

void MapBuilderBridge::FinishTrajectory(int trajectory_id)
{
    MAP_BUILDER_INFO("Finishing trajectory with ID: {}.", trajectory_id);

    if (getTrajectoryStates().count(trajectory_id) == 0) {
        MAP_BUILDER_ERROR("Trajectory with ID: {} does not exist.", trajectory_id);
        return;
    }

    map_builder_->FinishTrajectory(trajectory_id);
    sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::runFinalOptimization()
{
    MAP_BUILDER_INFO("Running final optimization...");
    map_builder_->pose_graph()->RunFinalOptimization();
}

std::map<int, mapping::PoseGraphInterface::TrajectoryState> MapBuilderBridge::getTrajectoryStates()
{
    auto traj_states = map_builder_->pose_graph()->GetTrajectoryStates(); // Add active trajectories that are not yet in the pose graph, but are e.g.  // waiting for input sensor data and thus already have a sensor bridge.  // Add active trajectories that are not yet in the pose graph, but are e.g.  // waiting for input sensor data and thus already have a sensor bridge.  // Add active trajKctories that are not yet in the pose graph, but are e.g.  traj_states.emplace(sensor_bridge.first, mapping::PoseGraphInterface::TrajectoryState::ACTIVE); } return traj_states; } std::unordered_map<int, LocalTrajectoryData> getLocalTrajectoryData(); rf_robot_msgs::msg::SubmapList getSubmapList(rclcpp::Time node_time); SensorBridge* sensorBridge(int trajectory_id);

    // Add active trajectories that are not yet in the pose graph, but are e.g.
    // waiting for input sensor data and thus already have a sensor bridge.
    for (const auto& sensor_bridge : sensor_bridges_) {
        traj_states.emplace(sensor_bridge.first, mapping::PoseGraphInterface::TrajectoryState::ACTIVE);
    }
    return traj_states;
}


std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData> MapBuilderBridge::getLocalTrajectoryData()
{
    std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
    for (auto& [trajectory_id, sensor_bridge] : sensor_bridges_) {
        std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (local_slam_data_.count(trajectory_id) == 0) {
                continue;
            }

            local_slam_data = local_slam_data_.at(trajectory_id);
        }

        local_trajectory_data[trajectory_id] = {
            local_slam_data,
            map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
            trajectory_options_[trajectory_id]
        };
    }

    return local_trajectory_data;
}

rf_robot_msgs::msg::SubmapList MapBuilderBridge::getSubmapList(rclcpp::Time node_time)
{
    rf_robot_msgs::msg::SubmapList submap_list;
    submap_list.header.stamp = node_time;
    submap_list.header.frame_id = node_options_.map_frame;
    auto all_submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
    for (const auto& submap_id_pose : all_submap_poses) {
        rf_robot_msgs::msg::SubmapEntry submap_entry;
        submap_entry.is_frozen = map_builder_->pose_graph()->IsTrajectoryFrozen(submap_id_pose.id.trajectory_id);
        submap_entry.submap_index = submap_id_pose.id.submap_index;
        submap_entry.submap_version = submap_id_pose.data.version;
        submap_entry.pose = toGeometryMsgPose(submap_id_pose.data.pose);
        submap_list.submap.push_back(submap_entry);
    }

    return submap_list;
}

SensorBridge* MapBuilderBridge::sensorBridge(int trajectory_id)
{
    return sensor_bridges_.at(trajectory_id).get();
}

void MapBuilderBridge::onLocalSlamResult(const int trajectory_id,
    const cartographer::common::Time time,
    const cartographer::transform::Rigid3d local_pose,
    cartographer::sensor::RangeData range_data_in_local)
{
    auto local_slam_data = std::make_shared<LocalTrajectoryData::LocalSlamData>(
        LocalTrajectoryData::LocalSlamData{time, local_pose, std::move(range_data_in_local)}
    );

    std::lock_guard<std::mutex> lock(mutex_);
    local_slam_data_[trajectory_id] = local_slam_data;
}

} // namespace rf_map_builder