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
Usage: ./$SCRIPT_NAME <stage> [--attach|--detach]

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
	./$SCRIPT_NAME offline_map --resolution 0.03
	./$SCRIPT_NAME localize_waypoints
	./$SCRIPT_NAME localize_waypoints --detach
EOF
}

run_base() {
	docker compose up -d "${BASE_SERVICES[@]}"
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