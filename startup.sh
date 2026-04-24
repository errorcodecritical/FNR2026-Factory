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

print_help() {
	cat <<EOF
Usage: ./$SCRIPT_NAME <stage>

Stages:
	base                Start base runtime stack (drivers + teleop + localization)
	record              Start base stack + session recording control
	offline_map         Build map from latest recording (one-shot)
	slam_map            Start online SLAM mapping
	localize_waypoints  Start localization + waypoint capture
	nav                 Start Nav2 navigation
	autonomy            Start factory autonomy node
	rviz                Start RViz2 container
	stop                Stop all compose services

Examples:
	./$SCRIPT_NAME base
	./$SCRIPT_NAME record
	./$SCRIPT_NAME offline_map
EOF
}

run_base() {
	docker compose up -d "${BASE_SERVICES[@]}"
}

stage="${1:-}"

case "$stage" in
	base)
		run_base
		;;

	record)
		run_base
		docker compose --profile record_session up -d session_record
		;;

	offline_map)
		docker compose --profile offline_mapping up offline_map_builder
		;;

	slam_map)
		run_base
		docker compose --profile slam_mapping up -d slam_mapping
		;;

	localize_waypoints)
		run_base
		docker compose --profile slam_localization --profile waypoint_capture up -d slam_localization waypoint_capture
		;;

	nav)
		run_base
		docker compose --profile slam_localization --profile nav2_navigation up -d slam_localization nav2
		;;

	autonomy)
		docker compose up -d factory_autonomy
		;;

	rviz)
		docker compose --profile visualization up -d rviz2
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