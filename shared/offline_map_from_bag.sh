#!/usr/bin/env bash
set -euo pipefail

BAG_PATH=${1:-/shared/recordings/latest}
MAP_NAME=${2:-fnr26_map}
MAP_DIR="/shared/maps/${MAP_NAME}"
MAP_FILE="${MAP_DIR}/${MAP_NAME}"
SLAM_PARAMS=${SLAM_PARAMS:-/shared/slam_configs/online_async.yaml}

resolve_bag_path() {
  local requested_path="$1"

  if [[ -e "${requested_path}" ]]; then
    echo "${requested_path}"
    return
  fi

  if [[ "${requested_path}" == "/shared/recordings/latest" ]]; then
    local newest_session
    newest_session=$(ls -1dt /shared/recordings/session_* 2>/dev/null | head -n 1 || true)
    if [[ -n "${newest_session}" ]]; then
      echo "${newest_session}"
      return
    fi
  fi

  echo ""
}

BAG_PATH_RESOLVED=$(resolve_bag_path "${BAG_PATH}")
if [[ -z "${BAG_PATH_RESOLVED}" ]]; then
  echo "Bag path does not exist: ${BAG_PATH}" >&2
  echo "No session_* recording found under /shared/recordings" >&2
  exit 1
fi

if [[ "${BAG_PATH_RESOLVED}" != "${BAG_PATH}" ]]; then
  echo "Bag path '${BAG_PATH}' not found, using latest session: ${BAG_PATH_RESOLVED}"
fi

BAG_PATH="${BAG_PATH_RESOLVED}"

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
