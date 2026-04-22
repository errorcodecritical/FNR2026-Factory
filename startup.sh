#!/bin/bash
set -e

# Startup script to initialize docker containers for each package, from the list:

# base                mecanum_driver      rplidar
# electromagnet_gpio  minimu9_publisher   rviz2
# factory_autonomy    nav2_navigator      slam_localization
# imu_filter          publisher           slam_mapping
# map_saver           robot_localization  teleop

echo "Starting up ROS2 Jazzy containers for FNR26 Factory..."

docker compose up -d publisher electromagnet_gpio rplidar mecanum_driver minimu9_publisher imu_filter slam_localization robot_localization teleop