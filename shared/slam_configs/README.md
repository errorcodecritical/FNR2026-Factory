# SLAM Setup Configuration for FNR26 Robot
#
# This file contains best-practice configurations for slam_toolbox
# on the FNR26 Mecanum-wheel robot with RPLIDAR scanner.
#
# Generated: 2026-04-20
# ROS Distribution: Jazzy
# Hardware: 
#   - Platform: Mecanum 4WD drive
#   - Main Sensor: RPLIDAR A1 (12m range, 360° scan)
#   - Odometry: RF2O Laser Odometry (vision-based) + Sensors
#   - IMU: MinIMU-9 v5 (fused via EKF)
#   - Odometry Input: EKF-fused output (combines RF2O + wheel sensors + IMU)
#
# Key Configurations:
# 1. online_async.yaml - For active mapping (building new maps)
# 2. localization.yaml - For using pre-built maps for localization
#
# The robot uses a multi-layer sensor fusion approach:
# - Raw Sensors: LIDAR scan + IMU + Wheel encoders
# - Processing: RF2O → EKF filter → SLAM  
# - Output: Accurate 2D map and precise localization
#
# Node Dependencies:
# - slam_mapping (slam_toolbox) depends on:
#   - rplidar (hardware driver)
#   - rf2o_odometry (laser-based odometry)
#   - imu_filter (IMU data processing)
#   - robot_localization (EKF fusion)
#   - publisher (robot_state_publisher)
#
# Launch Configuration:
# The slam_mapping service in docker-compose.yml launches:
#   ros2 launch slam_toolbox online_async_launch.py \
#     slam_params_file:=/root/nav2_ws/config/mapper_params_online_async.yaml
#
# Configuration Management:
# - Copy desired config to /config/mapper_params_online_async.yaml
# - or use /shared/slam_configs/* configs
# - Modify parameters based on robot's specific environment
#
# Performance Tips:
# - map_update_interval: Lower = faster map updates but higher CPU
# - resolution: Lower = higher detail but larger map files & CPU usage
# - correlation_search_space_dimension: Smaller = faster but less accurate
# - loop_search_maximum_distance: Higher = catches wider loops but slower
#
# Common Issues & Solutions:
#
# 1. Map updates are slow:
#    - Increase map_update_interval (e.g., 10.0)
#    - Decrease resolution (e.g., 0.1)
#    - Reduce correlation_search_space_dimension (e.g., 0.3)
#
# 2. Loop closure not working:
#    - Increase loop_search_maximum_distance
#    - Decrease loop_match_minimum_response_coarse
#    - Increase loop_match_minimum_chain_size
#
# 3. Transforms not updating:
#    - Check robot_state_publisher is running
#    - Verify TF frames are correctly named
#    - Check CYCLONEDDS_URI environment variable
#
# 4. Noisy odometry:
#    - Check RF2O parameters in /shared/rf2o_params.yaml
#    - Verify lidar is mounted horizontally
#    - Check IMU calibration
#
# Usage:
# 
# Start full SLAM system:
#   docker compose up slam_mapping
#
# View in RViz:
#   docker compose up rviz2
#
# Check system health:
#   docker compose logs slam_mapping
#
# Save map after mapping:
#   ros2 run nav2_map_server map_saver_cli -f ~/my_map
#
# Load map for localization:
#   1. Place map files in /shared/maps/
#   2. Update map_file_name in localization.yaml
#   3. Use localization.yaml instead of online_async.yaml
