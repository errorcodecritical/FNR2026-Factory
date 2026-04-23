-- mapping.lua
-- Cartographer 2D SLAM configuration for a mecanum robot with a 2D LIDAR.
-- Used for the slam_mapping profile (building new maps).

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frame IDs — must match your URDF and EKF output
  map_frame = "map",
  tracking_frame = "imu_link",          -- frame with IMU, closest to robot centre
  published_frame = "base_link",
  odom_frame = "odom",

  -- Use odometry from EKF — gives Cartographer a good prior
  provide_odom_frame = false,           -- EKF already provides odom→base_link
  publish_frame_projected_to_2d = true,
  use_odometry = true,                  -- fuse /odometry/filtered
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,                  -- one 2D LIDAR
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,       -- 200 Hz pose output
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- ── 2D map builder ────────────────────────────────────────────────────────
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- ── 2D trajectory builder ─────────────────────────────────────────────────
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Motion filter — only process scan when robot has moved enough
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- Scan matcher — increase weight on real-time scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres solver for local SLAM
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e3
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0

-- Submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- ── Pose graph (global SLAM / loop closure) ───────────────────────────────
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3

return options