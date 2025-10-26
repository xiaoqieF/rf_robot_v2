#pragma once

#include "elog/elog.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include <cartographer/mapping/proto/trajectory_builder_options.pb.h>

namespace rf_map_builder
{

struct NodeOptions
{
    cartographer::mapping::proto::MapBuilderOptions map_builder_options;
    std::string map_frame;
    double lookup_transform_timeout_sec;
    double submap_publish_period_sec;
    double pose_publish_period_sec;
    double trajectory_publish_period_sec;
};

struct TrajectoryOptions
{
    cartographer::mapping::proto::TrajectoryBuilderOptions trajectory_builder_options;
    std::string tracking_frame;
    std::string odom_frame;
    bool use_odometry;
    bool use_nav_sat;
    bool use_landmarks;
    bool publish_frame_projected_to_2d;
    int num_laser_scans;
    int num_point_clouds;
    double rangefinder_sampling_ratio;
    double odometry_sampling_ratio;
    double fixed_frame_pose_sampling_ratio;
    double imu_sampling_ratio;
    double landmarks_sampling_ratio;
};

#define MAP_BUILDER_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_map_builder] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define MAP_BUILDER_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_map_builder] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define MAP_BUILDER_ERROR(fmt_str, ...) \
    do { \
        elog::error("[rf_map_builder] " fmt_str, ##__VA_ARGS__); \
    } while(0)

inline constexpr char kLaserScanTopic[] = "scan";
inline constexpr char kMultiEchoLaserScanTopic[] = "echoes";
inline constexpr char kPointCloud2Topic[] = "points2";
inline constexpr char kImuTopic[] = "imu";
inline constexpr char kOdometryTopic[] = "odom";
inline constexpr char kNavSatFixTopic[] = "fix";
inline constexpr char kLandmarkTopic[] = "landmark";
inline constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
inline constexpr char kOccupancyGridTopic[] = "map";
inline constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
inline constexpr char kSubmapListTopic[] = "submap_list";
inline constexpr char kTrackedPoseTopic[] = "tracked_pose";
inline constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
inline constexpr double kExtrapolationEstimationTimeSec = 0.001;
inline constexpr double kGravityTimeConstant = 10.0;

std::vector<std::string> computeRepeatedTopicNames(
    const std::string& base_topic, const int count);

std::tuple<NodeOptions, TrajectoryOptions> loadOptions(
    const std::string& config_file_directory, const std::string& config_file_name);
NodeOptions createNodeOptions(cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);
TrajectoryOptions createTrajectoryBuilderOptions(cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

} // namespace rf_map_builder