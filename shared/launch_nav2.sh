#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

echo "Waiting for dependencies..."
sleep 7

echo "Launching custom Nav2 stack (expects SLAM Toolbox localization running)..."
ros2 launch nav2_localization nav2_localization.launch.py \
  map:=/shared/maps/fnr26_map/fnr26_map.yaml \
  params_file:=/shared/nav2_params.yaml \
  use_sim_time:=false