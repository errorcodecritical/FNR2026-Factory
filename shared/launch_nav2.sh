#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

echo "Waiting for dependencies..."
sleep 7

echo "Launching Nav2 bringup..."
ros2 launch nav2_bringup bringup_launch.py \
  map:=/shared/maps/fnr26_map/fnr26_map.yaml \
  params_file:=/shared/nav2_params.yaml \
  use_sim_time:=false