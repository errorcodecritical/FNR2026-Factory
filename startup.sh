#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

SCRIPT_NAME="$(basename "$0")"

BASE_SERVICES=(
	publisher
	electromagnet_gpio
	rplidar
	mecanum_driver
	minimu9_publisher
	imu_filter
	robot_localization
	teleop
)

COMPETITION_BASE_SERVICES=(
	publisher
	electromagnet_gpio
	rplidar
	mecanum_driver
	minimu9_publisher
	imu_filter
	robot_localization
)

print_help() {
	cat <<EOF
Usage: ./$SCRIPT_NAME <stage> [--attach|--detach]

Stages:
	base                Start base runtime stack (drivers + teleop + localization)
	record              Start base stack + session recording control
	offline_map         Build map from latest recording (one-shot)
	slam_map            Start online SLAM mapping
	localize_waypoints  Start localization + waypoint capture
	nav                 Start Nav2 navigation
	competition         Start headless full competition stack (no teleop)
	goto                Send a test NavigateToPose goal by waypoint name
	list_waypoints      List saved waypoint names and poses
	autonomy            Start factory autonomy node
	waypoint_markers    Start waypoint marker publisher only
	replay_viz          Replay latest bag with sim-time localization + RViz + markers
	rviz                Start RViz2 container
	stop                Stop all compose services

Examples:
	./$SCRIPT_NAME base
	./$SCRIPT_NAME record
	./$SCRIPT_NAME offline_map
	./$SCRIPT_NAME offline_map --resolution 0.03
	./$SCRIPT_NAME competition
	./$SCRIPT_NAME list_waypoints
	./$SCRIPT_NAME goto wp_001
	./$SCRIPT_NAME replay_viz
	./$SCRIPT_NAME localize_waypoints
	./$SCRIPT_NAME localize_waypoints --detach
EOF
}

run_base() {
	docker compose up -d "${BASE_SERVICES[@]}"
}

run_competition_base() {
	docker compose up -d "${COMPETITION_BASE_SERVICES[@]}"
}

resolve_latest_recording() {
	if [[ -L shared/recordings/latest || -d shared/recordings/latest ]]; then
		local resolved
		resolved=$(readlink -f shared/recordings/latest 2>/dev/null || true)
		if [[ -n "$resolved" && -d "$resolved" ]]; then
			echo "$resolved"
			return 0
		fi
	fi

	local newest
	newest=$(ls -1dt shared/recordings/session_* 2>/dev/null | head -n 1 || true)
	if [[ -n "$newest" && -d "$newest" ]]; then
		echo "$(readlink -f "$newest")"
		return 0
	fi

	return 1
}

stage="${1:-}"
mode="${2:-}"

case "$stage" in
	base)
		run_base
		;;

	record)
		run_base
		docker compose --profile record_session up -d session_record
		;;

	offline_map)
		resolution=""
		if [[ -z "$mode" ]]; then
			:
		elif [[ "$mode" == --resolution=* ]]; then
			resolution="${mode#--resolution=}"
			if [[ -z "$resolution" ]]; then
				echo "Missing value for --resolution" >&2
				exit 1
			fi
		elif [[ "$mode" == "--resolution" ]]; then
			resolution="${3:-}"
			if [[ -z "$resolution" ]]; then
				echo "Missing value for --resolution" >&2
				exit 1
			fi
		elif [[ "$mode" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
			resolution="$mode"
		else
			echo "Invalid offline_map option: $mode" >&2
			echo "Use: ./$SCRIPT_NAME offline_map [--resolution <meters>]" >&2
			exit 1
		fi

		if [[ -n "$resolution" ]]; then
			echo "Running offline map with MAP_RESOLUTION=$resolution m/px"
			docker compose --profile offline_mapping run --rm -e MAP_RESOLUTION="$resolution" offline_map_builder
		else
			echo "Running offline map with default MAP_RESOLUTION=0.05 m/px"
			docker compose --profile offline_mapping run --rm offline_map_builder
		fi
		;;

	slam_map)
		run_base
		docker compose --profile slam_mapping up -d slam_mapping
		;;

	localize_waypoints)
		run_base
		case "$mode" in
			""|--attach)
				docker compose --profile slam_localization --profile waypoint_capture up slam_localization waypoint_capture
				;;
			--detach)
				docker compose --profile slam_localization --profile waypoint_capture up -d slam_localization waypoint_capture
				;;
			*)
				echo "Invalid mode for localize_waypoints: $mode" >&2
				echo "Use --attach or --detach" >&2
				exit 1
				;;
		esac
		;;

	nav)
		run_base
		docker compose --profile slam_localization --profile nav2_navigation up -d slam_localization nav2
		;;

	competition)
		run_competition_base
		docker compose --profile slam_localization --profile waypoint_capture --profile nav2_navigation up -d \
			slam_localization waypoint_capture nav2 factory_autonomy
		;;

	goto)
		waypoint_name="$mode"
		if [[ -z "$waypoint_name" ]]; then
			echo "Usage: ./$SCRIPT_NAME goto <waypoint_name>" >&2
			exit 1
		fi

		echo "Sending NavigateToPose goal for waypoint: $waypoint_name"
		docker compose run --rm --entrypoint bash -e WAYPOINT_NAME="$waypoint_name" factory_autonomy -lc '
			source /opt/ros/jazzy/setup.bash
			cd /docker_ws
			colcon build --packages-select factory_autonomy --symlink-install
			source /docker_ws/install/setup.bash
			python3 - <<"PY"
import math
import os
import sys
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

WAYPOINT_FILE = "/shared/factory_waypoints.yaml"
waypoint_name = os.environ["WAYPOINT_NAME"].strip()

with open(WAYPOINT_FILE, "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

waypoints = data.get("waypoints", data)
if not isinstance(waypoints, dict) or waypoint_name not in waypoints:
    available = sorted(waypoints.keys()) if isinstance(waypoints, dict) else []
    print(f"Waypoint not found: {waypoint_name}", file=sys.stderr)
    print(f"Available waypoints: {available}", file=sys.stderr)
    raise SystemExit(2)

wp = waypoints[waypoint_name]
x = float(wp["x"])
y = float(wp["y"])
yaw = float(wp.get("yaw", 0.0))
frame_id = str(wp.get("frame_id", "map"))

rclpy.init()
node = Node("waypoint_goal_tester")
client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

if not client.wait_for_server(timeout_sec=8.0):
    print("NavigateToPose action server not available on /navigate_to_pose", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(3)

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = frame_id
goal.pose.pose.position.x = x
goal.pose.pose.position.y = y
goal.pose.pose.position.z = 0.0
goal.pose.pose.orientation.x = 0.0
goal.pose.pose.orientation.y = 0.0
goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

print(f"Sending goal {waypoint_name} in frame {frame_id}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
send_future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future, timeout_sec=8.0)
goal_handle = send_future.result()

if goal_handle is None or not goal_handle.accepted:
    print("Goal rejected by Nav2", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(4)

result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
result = result_future.result()
status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN

if status == GoalStatus.STATUS_SUCCEEDED:
    print(f"Goal succeeded: {waypoint_name}")
    exit_code = 0
else:
    print(f"Goal finished with status {status}", file=sys.stderr)
    exit_code = 5

node.destroy_node()
rclpy.shutdown()
raise SystemExit(exit_code)
PY
		'
		;;

	list_waypoints)
		docker compose run --rm --entrypoint bash factory_autonomy -lc '
			source /opt/ros/jazzy/setup.bash
			python3 - <<"PY"
import yaml

path = "/shared/factory_waypoints.yaml"
with open(path, "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

waypoints = data.get("waypoints", data)
if not isinstance(waypoints, dict) or not waypoints:
    print(f"No waypoints found in {path}")
    raise SystemExit(0)

print(f"Waypoints in {path}:")
for name in sorted(waypoints):
    value = waypoints[name] or {}
    x = float(value.get("x", 0.0))
    y = float(value.get("y", 0.0))
    yaw = float(value.get("yaw", 0.0))
    frame = value.get("frame_id", "map")
    print(f"- {name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, frame={frame}")
PY
		'
		;;

	autonomy)
		docker compose up -d factory_autonomy
		;;

	waypoint_markers)
		docker compose --profile visualization up -d waypoint_markers
		;;

	replay_viz)
		echo "Preparing replay environment (stopping potentially conflicting live services)..."
		docker compose stop \
			publisher electromagnet_gpio rplidar mecanum_driver minimu9_publisher imu_filter robot_localization teleop \
			slam_localization waypoint_capture nav2 factory_autonomy rviz2 waypoint_markers \
			2>/dev/null || true

		bag_host_path="$(resolve_latest_recording || true)"
		if [[ -z "$bag_host_path" ]]; then
			echo "No recording found in shared/recordings (expected latest or session_*)" >&2
			exit 1
		fi

		bag_name="$(basename "$bag_host_path")"
		bag_container_path="/shared/recordings/$bag_name"
		echo "Replaying bag: $bag_container_path"

		loc_cmd="source /opt/ros/jazzy/setup.bash; if [ -d /docker_ws/install ]; then source /docker_ws/install/setup.bash; fi; ros2 launch slam_toolbox localization_launch.py slam_params_file:=/shared/slam_configs/localization.yaml use_sim_time:=true"
		rviz_cmd="source /opt/ros/jazzy/setup.bash; ros2 run rviz2 rviz2 -d /shared/view.rviz --ros-args -p use_sim_time:=true"
		markers_cmd="source /opt/ros/jazzy/setup.bash; cd /docker_ws; colcon build --packages-select factory_autonomy --symlink-install; source /docker_ws/install/setup.bash; ros2 run factory_autonomy waypoint_markers --ros-args -p waypoints_file:=/shared/factory_waypoints.yaml"
		play_cmd="source /opt/ros/jazzy/setup.bash; ros2 bag play $bag_container_path --clock --topics /scan /odometry/filtered /tf /tf_static"

		cleanup_replay() {
			jobs -p | xargs -r kill 2>/dev/null || true
			wait || true
		}

		trap cleanup_replay INT TERM EXIT

		echo "Starting sim-time localization..."
		docker compose --profile slam_localization run --rm --entrypoint bash slam_localization -lc "$loc_cmd" &
		sleep 3
		echo "Starting waypoint markers..."
		docker compose --profile visualization run --rm --entrypoint bash waypoint_markers -lc "$markers_cmd" &
		sleep 1
		echo "Starting RViz..."
		docker compose --profile visualization run --rm --entrypoint bash rviz2 -lc "$rviz_cmd" &
		sleep 2
		echo "Playing rosbag with /clock..."
		docker compose --profile offline_mapping run --rm --entrypoint bash offline_map_builder -lc "$play_cmd"

		echo "Replay completed. Stopping replay processes..."
		cleanup_replay
		trap - INT TERM EXIT
		;;

	rviz)
		docker compose --profile visualization up -d rviz2 waypoint_markers
		;;

	stop)
		docker compose down
		;;

	-h|--help|help|"")
		print_help
		;;

	*)
		echo "Unknown stage: $stage" >&2
		print_help
		exit 1
		;;
esac

echo "Stage completed: ${stage:-help}"