#!/bin/bash
MAP_NAME=${1:-fnr26_map}
MAP_DIR="/shared/maps/$MAP_NAME"
MAP_FILE="$MAP_DIR/$MAP_NAME"

mkdir -p "$MAP_DIR"

echo "Saving occupancy grid..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_FILE"

echo "Serializing posegraph..."
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "filename: '$MAP_FILE'"

if [ -f "${MAP_FILE}.yaml" ] && [ -f "${MAP_FILE}.posegraph" ]; then
  echo "Map saved successfully!"
  ls -lh "${MAP_DIR}/"
else
  echo "Failed — files present:"
  ls -lh "${MAP_DIR}/"
  exit 1
fi