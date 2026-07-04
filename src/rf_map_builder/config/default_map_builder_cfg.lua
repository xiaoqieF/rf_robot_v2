include "map_builder.lua"
include "trajectory_builder.lua"

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.collate_by_trajectory = false

TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = true
TRAJECTORY_BUILDER.trajectory_builder_2d.min_range = 0.12
TRAJECTORY_BUILDER.trajectory_builder_2d.max_range = 10.0
TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 90
TRAJECTORY_BUILDER.trajectory_builder_2d.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER.trajectory_builder_2d.pose_extrapolator.use_imu_based = false

return {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.2,
  trajectory_builder = TRAJECTORY_BUILDER,
  tracking_frame = "base_link",
  odom_frame = "odom",
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = true,
  num_laser_scans = 1,
  num_point_clouds = 0,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}
