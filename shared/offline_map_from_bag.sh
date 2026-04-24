#!/usr/bin/env bash
set -euo pipefail

BAG_PATH=${1:-/shared/recordings/latest}
MAP_NAME=${2:-fnr26_map}
MAP_RESOLUTION=${3:-${MAP_RESOLUTION:-0.05}}
MAP_DIR="/shared/maps/${MAP_NAME}"
MAP_FILE="${MAP_DIR}/${MAP_NAME}"
SLAM_PARAMS=${SLAM_PARAMS:-/shared/slam_configs/online_async.yaml}
SLAM_PARAMS_RUNTIME="${SLAM_PARAMS}"
TMP_SLAM_PARAMS=""

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

if ! [[ "${MAP_RESOLUTION}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Invalid MAP_RESOLUTION '${MAP_RESOLUTION}' (expected positive number in meters, e.g. 0.05)" >&2
  exit 1
fi

echo "Using map resolution: ${MAP_RESOLUTION} m/px"

mkdir -p "${MAP_DIR}"

TMP_SLAM_PARAMS=$(mktemp /tmp/slam_params_resolution_XXXX.yaml)
python3 - "${SLAM_PARAMS}" "${TMP_SLAM_PARAMS}" "${MAP_RESOLUTION}" <<'PY'
import sys
import yaml

source_path, out_path, resolution = sys.argv[1], sys.argv[2], float(sys.argv[3])

with open(source_path, 'r', encoding='utf-8') as stream:
    data = yaml.safe_load(stream) or {}

slam = data.setdefault('slam_toolbox', {})
params = slam.setdefault('ros__parameters', {})
params['resolution'] = resolution

with open(out_path, 'w', encoding='utf-8') as stream:
    yaml.safe_dump(data, stream, sort_keys=False)
PY
SLAM_PARAMS_RUNTIME="${TMP_SLAM_PARAMS}"

echo "Starting slam_toolbox (use_sim_time=true)..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="${SLAM_PARAMS_RUNTIME}" \
  use_sim_time:=true &
SLAM_PID=$!

cleanup() {
  if kill -0 "${SLAM_PID}" >/dev/null 2>&1; then
    kill "${SLAM_PID}" || true
    wait "${SLAM_PID}" || true
  fi
  if [[ -n "${TMP_SLAM_PARAMS}" && -f "${TMP_SLAM_PARAMS}" ]]; then
    rm -f "${TMP_SLAM_PARAMS}"
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
