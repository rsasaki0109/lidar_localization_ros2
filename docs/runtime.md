# Runtime Details

Behavior that matters once the localizer runs, moved out of the README.

## Parameter precedence

With `lidar_localization.launch.py` and `mid360_legged_localization.launch.py`, the
parameter YAML (`localization_param_dir:=...`) is the source of truth. Frame, IMU,
deskew, map and initial-pose launch arguments override it only when you pass them.
The static TF publishers use the same resolved `base_frame_id` as the node.

## Deskew

Continuous-time deskew is on by default. Until point timing and motion data are
ready, it leaves scans unchanged. To turn it off, launch with
`use_continuous_time_deskew:=false`.

Livox FLOAT64 `timestamp` fields can hold nanoseconds. `point_timestamp_unit: auto`
(the default) picks one unit for the whole cloud, including points near the scan
start. Set `seconds` or `nanoseconds` explicitly for ambiguous data. Floating
`t`/`offset_time` fields are always seconds and integer time fields are always
nanoseconds. See [IMU and deskew](imu_estimation.md).

## Quickstart initialization options

- `--no-auto-initialize` disables saved-pose restoration and global initialization.
- Global search scores candidates at the map's ground under them plus 1.0 m. If the
  sensor sits much higher or lower, pass its height above the ground with
  `--sensor-height`, or one fixed map-frame z with `--global-seed-z`.
- To build the occupancy grid yourself:
  `ros2 run lidar_localization_ros2 generate_occupancy_map_from_pcd --pcd map.pcd --output-dir maps`,
  then pass it with `--occupancy-map maps/map.yaml`.
- If no safe candidate is found, quickstart asks for **2D Pose Estimate** in RViz. It
  never falls back to the map origin.

Details: [quickstart and automatic initialization](quickstart.md).

## `/path` history

`/path` keeps the latest 2000 poses. `path_max_poses: 0` keeps the whole history
(memory and publish cost grow with runtime); negative values fall back to 2000.
Diagnostics report `path_max_poses` and `path_pose_count`.

## Lifecycle startup

The standalone, Nav2 localization and MID-360 launch files activate the localizer
through the `GetState`/`ChangeState` services, without relying on transition events.
The helper exits once the node is active and does not watch or restart it afterward.
A callback failure, an unexpected state, or a 60 s wall-clock deadline is reported as
an error with a nonzero exit. A node that is already active is left as is.

To use the helper directly:

```bash
ros2 run lidar_localization_ros2 start_lifecycle_node.py <node_name> --timeout <seconds>
```

`<node_name>` may be relative to the helper's ROS namespace.
