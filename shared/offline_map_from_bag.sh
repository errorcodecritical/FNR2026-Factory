#!/usr/bin/env bash
set -euo pipefail

BAG_PATH=${1:-/shared/recordings/latest}
MAP_NAME=${2:-fnr26_map}
MAP_DIR="/shared/maps/${MAP_NAME}"
MAP_FILE="${MAP_DIR}/${MAP_NAME}"
SLAM_PARAMS=${SLAM_PARAMS:-/shared/slam_configs/online_async.yaml}

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "Bag path does not exist: ${BAG_PATH}" >&2
  exit 1
fi

mkdir -p "${MAP_DIR}"

echo "Starting slam_toolbox (use_sim_time=true)..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="${SLAM_PARAMS}" \
  use_sim_time:=true &
SLAM_PID=$!

cleanup() {
  if kill -0 "${SLAM_PID}" >/dev/null 2>&1; then
    kill "${SLAM_PID}" || true
    wait "${SLAM_PID}" || true
  fi
}
trap cleanup EXIT

sleep 3

echo "Playing bag: ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock

sleep 2

echo "Saving occupancy map to ${MAP_FILE}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "${MAP_FILE}" --ros-args -p save_map_timeout:=20.0

echo "Serializing slam toolbox posegraph to ${MAP_FILE}.posegraph"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${MAP_FILE}'}"

echo "Offline mapping complete: ${MAP_DIR}"
