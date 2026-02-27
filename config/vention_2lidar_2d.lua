include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "vention_base_link",      -- you already have TF to lidars
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,                -- because you already have odom->base_link (from vSLAM right now)
  publish_frame_projected_to_2d = true,

  use_odometry = true,                       -- uses your /odom tf
  use_nav_sat = false,
  use_landmarks = false,

  num_point_clouds = 0,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.05,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  imu_sampling_ratio = 0.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false   -- set true only if you have a real IMU frame
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0       -- set to your RPLidar model max
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0

-- Tuning that’s usually sane for a small mobile base
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Local SLAM
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- Scan matching
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- Pose graph
POSE_GRAPH.optimize_every_n_nodes = 90


print("DEBUG options counts:",
  "num_point_clouds=", options.num_point_clouds,
  "num_laser_scans=", options.num_laser_scans,
  "num_multi_echo_laser_scans=", options.num_multi_echo_laser_scans)

return options