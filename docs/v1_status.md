# v1 Status

This document defines what `v1.0.0` means for this repository.

## Included

- ROS 2 LiDAR localization node with lifecycle support
- Nav2 integration launch files for localization-only and full navigation bringup
- helper scripts for occupancy-map generation, replay restamping, odom fallback, and goal dispatch
- rosbag benchmark tooling and trajectory comparison helpers
- mapless-public dataset scaffolding for split mapping/localization workflows
- Boreas raw-sequence to rosbag2 conversion for the first public `LiDAR + IMU + GT` starter path
- a recommended Nav2 preset: `param/nav2_ndt_urban.yaml`
- a runtime-facing reinitialization-request signal on `/reinitialization_requested`
- an optional external supervisor that republishes `/initialpose` when `/reinitialization_requested` goes true

## Validated scope

The following paths were validated in this workspace before the `v1.0.0` packaging pass:

- `colcon build --packages-up-to lidar_localization_ros2 --symlink-install`
- localizer launch and lifecycle activation
- Nav2 wrapper launch argument resolution
- demo `navigate_to_pose` smoke using the built-in odom/localization demo path
- replay `navigate_to_pose` smoke using the real localizer path and helper orchestration
- shutdown behavior after the destructor-path crash fix
- public regression suite entry point: `scripts/run_public_regression_suite.sh`
- reinitialization supervisor regression entry point: `ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_regression.sh`
- experiment suite entry point: `ros2 run lidar_localization_ros2 run_experiment_suite.py`
- mapless-public dataset scaffolding:
  - `ros2 run lidar_localization_ros2 scaffold_mapless_public_dataset_bundle.py --spec /absolute/path/to/spec.yaml`
  - generated `run_extract_reference.sh` path for bundles that need GT reference / initial-pose extraction
- experiment-first comparison entry point: `ros2 run lidar_localization_ros2 run_imu_guard_experiments.py`

## Recommended entry points

- localization only:
  - `ros2 launch lidar_localization_ros2 nav2_lidar_localization.launch.py`
- full Nav2 wrapper:
  - `ros2 launch lidar_localization_ros2 nav2_navigation.launch.py map_yaml:=/absolute/path/to/map.yaml`
- self-contained smoke:
  - `ros2 run lidar_localization_ros2 run_nav2_demo_smoke --map-yaml /absolute/path/to/map.yaml --initial-pose-x 0.0 --initial-pose-y 0.0 --goal-x 1.0 --goal-y 0.0`
- replay smoke:
  - `ros2 run lidar_localization_ros2 run_nav2_replay_smoke --map-yaml /absolute/path/to/map.yaml --pcd-map-path /absolute/path/to/map.pcd --bag-path /absolute/path/to/bag`
- reinitialization supervisor regression:
  - `ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_regression.sh`
- public regression:
  - `scripts/run_public_regression_suite.sh`
  - Istanbul `60 s` default-on no-IMU safety run
  - HDL `60 s` IMU safety / throughput compare
- release regression:
  - `ros2 run lidar_localization_ros2 run_release_regression_suite.sh`
  - combines the public suite and the long Nav2 reinitialization-supervisor regression
- experiment comparison:
  - `ros2 run lidar_localization_ros2 run_experiment_suite.py`
  - current problems: IMU correction guard, borderline measurement gate, recovery action selection, reinitialization trigger
  - generated docs: `docs/interfaces.md`, `docs/experiments.md`, `docs/decisions.md`
  - problem-specific runners remain available for narrower iteration
  - `ros2 run lidar_localization_ros2 run_imu_guard_experiments.py`
  - `ros2 run lidar_localization_ros2 run_borderline_gate_experiments.py`
  - `ros2 run lidar_localization_ros2 run_recovery_action_experiments.py`
  - `ros2 run lidar_localization_ros2 run_reinit_trigger_experiments.py`

## Recommended preset

Use `param/nav2_ndt_urban.yaml` unless you are intentionally running one of the experimental presets.

Current rationale:

- it is the most conservative branch among the tested Nav2-facing presets
- more aggressive recovery branches improved some intermediate diagnostics but regressed long-horizon translation in later replay tests
- when enabling the external reinitialization supervisor, keep `reinitialization_supervisor_use_latest_pose:=false`
  and start with `reinitialization_supervisor_publish_count:=1`
- the current long replay regression boundary for that policy is `run_nav2_reinit_supervisor_regression.sh`
- the current one-command release boundary is `run_release_regression_suite.sh`

## Known limits

- long-horizon urban replay remains the main unresolved robustness gap
- experimental recovery presets are kept for research, not as defaults
- occupancy-map generation from PCD is route-crop oriented and is not intended as a universal mapper
- smoke success does not replace real-robot validation with real odom, real TF, and moving goals

## Non-goals for v1

- claiming universal recovery from prolonged reject streaks
- claiming production-grade performance for every dense urban replay segment
- replacing a full SLAM or mapping stack
