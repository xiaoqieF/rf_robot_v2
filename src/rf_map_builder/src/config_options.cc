#include "rf_map_builder/config_options.hpp"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include <string>

namespace rf_map_builder
{

std::vector<std::string> computeRepeatedTopicNames(
    const std::string& topic, const int num_topics)
{
    if (num_topics == 1) {
        return {topic};
    }
    std::vector<std::string> topics;
    topics.reserve(num_topics);
    for (int i = 0; i < num_topics; ++i) {
        topics.emplace_back(topic + "_" + std::to_string(i + 1));
    }
    return topics;
}

std::tuple<NodeOptions, TrajectoryOptions> loadOptions(
    const std::string& config_file_directory, const std::string& config_file_name)
{
    auto file_resolver = std::make_unique<cartographer::common::ConfigurationFileResolver>(
        std::vector<std::string>{config_file_directory});
    const std::string code = file_resolver->GetFileContentOrDie(config_file_name);
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));

    return std::make_tuple(
        createNodeOptions(&lua_parameter_dictionary),
        createTrajectoryBuilderOptions(&lua_parameter_dictionary));
}

NodeOptions createNodeOptions(cartographer::common::LuaParameterDictionary* lua_parameter_dictionary)
{
    NodeOptions node_options;

    node_options.map_builder_options =
        cartographer::mapping::CreateMapBuilderOptions(
            lua_parameter_dictionary->GetDictionary("map_builder").get());

    node_options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    node_options.lookup_transform_timeout_sec = lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
    node_options.submap_publish_period_sec = lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
    node_options.pose_publish_period_sec = lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
    node_options.trajectory_publish_period_sec = lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");

    return node_options;
}

TrajectoryOptions createTrajectoryBuilderOptions(cartographer::common::LuaParameterDictionary* lua_parameter_dictionary)
{
    TrajectoryOptions trajectory_options;

    trajectory_options.trajectory_builder_options =
        cartographer::mapping::CreateTrajectoryBuilderOptions(
            lua_parameter_dictionary->GetDictionary("trajectory_builder").get());

    trajectory_options.tracking_frame = lua_parameter_dictionary->GetString("tracking_frame");
    trajectory_options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
    trajectory_options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
    trajectory_options.use_nav_sat = lua_parameter_dictionary->GetBool("use_nav_sat");
    trajectory_options.use_landmarks = lua_parameter_dictionary->GetBool("use_landmarks");
    trajectory_options.publish_frame_projected_to_2d = lua_parameter_dictionary->GetBool("publish_frame_projected_to_2d");
    trajectory_options.num_laser_scans = lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
    trajectory_options.num_point_clouds = lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
    trajectory_options.rangefinder_sampling_ratio = lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
    trajectory_options.odometry_sampling_ratio = lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
    trajectory_options.fixed_frame_pose_sampling_ratio = lua_parameter_dictionary->GetDouble("fixed_frame_pose_sampling_ratio");
    trajectory_options.imu_sampling_ratio = lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
    trajectory_options.landmarks_sampling_ratio = lua_parameter_dictionary->GetDouble("landmarks_sampling_ratio");

    return trajectory_options;
}


} // namespace rf_map_builder