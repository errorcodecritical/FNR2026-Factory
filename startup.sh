#!/bin/bash
set -e

cd "$(dirname "$0")"

# Startup script for the common runtime stack.
# For staged workflows, use compose profiles directly:
#   --profile record_session   (headless rosbag recording via controller)
#   --profile offline_mapping  (build map from recorded bag)
#   --profile slam_localization --profile waypoint_capture
#   --profile nav2_navigation

echo "Starting up ROS2 Jazzy containers for FNR26 Factory..."

docker compose up -d publisher electromagnet_gpio rplidar mecanum_driver minimu9_publisher imu_filter slam_localization robot_localization teleop