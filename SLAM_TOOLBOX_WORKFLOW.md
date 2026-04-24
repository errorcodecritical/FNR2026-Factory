# SLAM Toolbox Navigation Workflow

This workspace now uses `slam_toolbox` end-to-end (no AMCL, no Cartographer).

## Xbox Keybinds

- `START` (button 7): start recording sensor session
- `BACK` (button 6): stop recording sensor session
- `X` (button 2): save waypoint during localization

Keybind params are in:
- `/shared/session_control_record.yaml`
- `/shared/session_control_waypoint.yaml`

## 1) Headless Recording Stage (teleop + sensors only)

Bring up base drivers + teleop + recorder control:

```bash
docker compose up publisher rplidar minimu9_publisher mecanum_driver imu_filter robot_localization teleop
```

In another terminal, run recording control:

```bash
docker compose --profile record_session up session_record
```

Drive with the Xbox controller:
- Press `START` to begin rosbag recording
- Press `BACK` to stop recording

Bags are stored in `/shared/recordings/session_YYYYMMDD_HHMMSS`.
`/shared/recordings/latest` is updated automatically.

## 2) Offline Mapping from Recorded Data

Run offline map generation from the latest bag:

```bash
docker compose --profile offline_mapping up offline_map_builder
```

Outputs:
- `/shared/maps/fnr26_map/fnr26_map.yaml`
- `/shared/maps/fnr26_map/fnr26_map.pgm`
- `/shared/maps/fnr26_map/fnr26_map.posegraph`

## 3) Localization Stage + Waypoint Capture

Start localization and waypoint capture:

```bash
docker compose --profile slam_localization --profile waypoint_capture up slam_localization waypoint_capture
```

While localizing:
- Press `X` to append a waypoint to `/shared/factory_waypoints.yaml`
- Waypoints are named `wp_001`, `wp_002`, ...

You can rename waypoint keys later in the YAML file.

## 4) Navigation + Factory Autonomy

Start navigation:

```bash
docker compose --profile nav2_navigation up nav2
```

Start autonomous task execution over Wi-Fi competition protocol:

```bash
docker compose up factory_autonomy
```

The autonomy node:
- reads `/shared/factory_waypoints.yaml`
- sends `navigate_to_pose` goals
- toggles `eletro` for pickup/drop-off actions

## 5) Useful Notes

- RViz fixed frame should be `map`.
- If joystick indices differ, update the two `session_control_*.yaml` files.
- Offline mapping script path: `/shared/offline_map_from_bag.sh`.
