# SLAM Toolbox Navigation Workflow

This workspace now uses `slam_toolbox` end-to-end (no AMCL, no Cartographer).

## Startup Script (Stage Runner)

Use the stage runner script instead of long `docker compose` commands:

```bash
./startup.sh --help
```

Main stages:
- `./startup.sh base`
- `./startup.sh record`
- `./startup.sh offline_map`
- `./startup.sh slam_map`
- `./startup.sh localize_waypoints`
- `./startup.sh nav`
- `./startup.sh autonomy`
- `./startup.sh rviz`
- `./startup.sh stop`

## Xbox Keybinds

- `A` (button 0): start recording sensor session
- `B` (button 1): stop recording sensor session
- `X` (button 2): save waypoint during localization

Keybind params are in:
- `/shared/session_control_record.yaml`
- `/shared/session_control_waypoint.yaml`

## 1) Headless Recording Stage (teleop + sensors only)

Bring up base drivers + teleop + recorder control:

```bash
./startup.sh record
```

Drive with the Xbox controller:
- Press `A` to begin rosbag recording
- Press `B` to stop recording

Bags are stored in `/shared/recordings/session_YYYYMMDD_HHMMSS`.
`/shared/recordings/latest` is updated automatically.

## 2) Offline Mapping from Recorded Data

Run offline map generation from the latest bag:

```bash
./startup.sh offline_map
```

Outputs:
- `/shared/maps/fnr26_map/fnr26_map.yaml`
- `/shared/maps/fnr26_map/fnr26_map.pgm`
- `/shared/maps/fnr26_map/fnr26_map.posegraph`

## 3) Localization Stage + Waypoint Capture

Start localization and waypoint capture:

```bash
./startup.sh localize_waypoints
```

While localizing:
- Press `X` to append a waypoint to `/shared/factory_waypoints.yaml`
- Waypoints are named `wp_001`, `wp_002`, ...

You can rename waypoint keys later in the YAML file.

## 4) Navigation + Factory Autonomy

Start navigation:

```bash
./startup.sh nav
```

Start autonomous task execution over Wi-Fi competition protocol:

```bash
./startup.sh autonomy
```

The autonomy node:
- reads `/shared/factory_waypoints.yaml`
- sends `navigate_to_pose` goals
- toggles `eletro` for pickup/drop-off actions

## 5) Useful Notes

- RViz fixed frame should be `map`.
- If joystick indices differ, update the two `session_control_*.yaml` files.
- Offline mapping script path: `/shared/offline_map_from_bag.sh`.

## 6) Bag-Based Localization Replay + RViz Container

Use this when validating localization against a recorded dataset:

1. Start localization (sim time) with the replay config:

```bash
docker compose --profile slam_localization run --rm \
	-v "$PWD/datasets:/datasets" \
	--entrypoint bash slam_localization -lc \
	'source /opt/ros/jazzy/setup.bash && ros2 launch slam_toolbox localization_launch.py \
	slam_params_file:=/shared/slam_configs/localization_bag_replay.yaml use_sim_time:=true'
```

2. Start RViz in container with sim time enabled:

```bash
docker compose --profile visualization run --rm --entrypoint bash rviz2 -lc \
	'source /opt/ros/jazzy/setup.bash && ros2 run rviz2 rviz2 -d /shared/view.rviz --ros-args -p use_sim_time:=true'
```

3. Replay only relevant topics from bag:

```bash
docker compose --profile offline_mapping run --rm \
	-v "$PWD/datasets:/datasets" \
	--entrypoint bash offline_map_builder -lc \
	'source /opt/ros/jazzy/setup.bash && ros2 bag play /datasets/rosbag2_2026_04_24-01_24_09 \
	--clock -r 0.7 --topics /scan /odometry/filtered /tf /tf_static'
```
