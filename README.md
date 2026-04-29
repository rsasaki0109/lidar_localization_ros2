# lidar_localization_ros2
A ROS2 package of 3D LIDAR-based Localization.

## Quick Start

From this repository in the local workspace:

```bash
source scripts/setup_local_env.sh
cd ../build_ws
colcon build --symlink-install --packages-up-to lidar_localization_ros2
cd ../repo
source scripts/setup_local_env.sh
```

Then choose the path that matches what you want to do:

| Goal | Start here |
|---|---|
| Build the package in this workspace | [Local Build](docs/local_build.md) |
| Launch the LiDAR localizer for Nav2 | [Nav2 launch](#nav2-launch) |
| Run a self-contained Nav2 smoke path | [Recommended entry points](docs/v1_status.md#recommended-entry-points) |
| Run public replay/regression checks | [Benchmarking](#benchmarking) |
| Evaluate a rosbag against reference poses | [Benchmarking guide](docs/benchmarking.md) |
| Develop or compare recovery behavior | [Experiment-First Development](#experiment-first-development) |
| Check what `v1.0.0` does and does not claim | [v1 status](docs/v1_status.md) |

For Nav2 use, provide a pointcloud map, matching 2D `map_yaml` when launching the full Nav2 stack,
an odom source publishing `odom -> base_link`, and an initial pose on `/initialpose`.

## Status

The repo is now packaged as `v1.0.0`.

- recommended Nav2 preset: `param/nav2_ndt_urban.yaml`
- verified flows: `nav2_lidar_localization.launch.py`, `nav2_navigation.launch.py`, `run_nav2_demo_smoke`, `run_nav2_replay_smoke`
- current boundary: long-horizon urban replay beyond the validated smoke path is still an active robustness problem, not a solved one

See [CHANGELOG.md](CHANGELOG.md) for the release summary and [docs/v1_status.md](docs/v1_status.md) for the concrete validated scope and known limits.

<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)

## Requirements

- [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2.git)
- [small_gicp](https://github.com/koide3/small_gicp) (optional, for `SMALL_GICP` and `SMALL_VGICP`)

If `small_gicp` is installed as a CMake package at build time, `SMALL_GICP` and `SMALL_VGICP`
are enabled automatically.

## local build

For the no-sudo local prefix workflow used in this workspace, see [docs/local_build.md](docs/local_build.md).
After the first successful build, load the environment with:

```bash
source scripts/setup_local_env.sh
```

## Experiment-First Development

Runtime code lives in `src/` and `include/`. Competing ideas live in `experiments/` and are assumed to be discardable.

For the current experiment problems, this repo keeps multiple comparable implementations behind minimal interfaces and regenerates the comparison docs from one suite entry point:

```bash
ros2 run lidar_localization_ros2 run_experiment_suite.py
```

Problem-specific runners are also available:

- `ros2 run lidar_localization_ros2 run_imu_guard_experiments.py`
- `ros2 run lidar_localization_ros2 run_borderline_gate_experiments.py`
- `ros2 run lidar_localization_ros2 run_recovery_action_experiments.py`
- `ros2 run lidar_localization_ros2 run_reinit_trigger_experiments.py`

The suite updates:

- [docs/interfaces.md](docs/interfaces.md)
- [docs/experiments.md](docs/experiments.md)
- [docs/decisions.md](docs/decisions.md)

The rule is simple: new behavior should be introduced as multiple comparable variants first, then only the winning behavior is promoted into runtime core.

For a one-command regression pass across both the short public checks and the heavier Nav2 long-replay recovery check:

```bash
ros2 run lidar_localization_ros2 run_release_regression_suite.sh
```

This aggregates:

- `run_public_regression_suite.sh`
- `run_nav2_reinit_supervisor_regression.sh`

and writes a combined summary under `artifacts/public/release_regression_suite/`.

## IO
- input  
/cloud  (sensor_msgs/PointCloud2)  
/map  (sensor_msgs/PointCloud2)  
/initialpose (geometry_msgs/PoseStamed)(when `set_initial_pose` is false)  
/odom (nav_msgs/Odometry)(optional)   
/imu  (sensor_msgs/Imu)(optional)  

- output  
/pcl_pose (geometry_msgs/PoseStamped)  
/path (nav_msgs/Path)  
/alignment_status (diagnostic_msgs/DiagnosticArray)  
/reinitialization_requested (std_msgs/Bool)  
/initial_map (sensor_msgs/PointCloud2)(when `use_pcd_map` is true)  

## params

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"NDT_OMP"|"NDT" or "GICP" or "NDT_OMP" or "GICP_OMP" or "SMALL_GICP" or "SMALL_VGICP"|
|score_threshold|double|2.0|maximum accepted registration fitness score|
|ndt_resolution|double|2.0|resolution size of voxels[m]|
|ndt_step_size|double|0.1|step_size maximum step length[m]|
|ndt_num_threads|int|4|threads using NDT_OMP(if `0` is set, maximum alloawble threads are used.)|
|gicp_corr_randomness|int|20|number of neighbors for GICP covariance estimation|
|gicp_max_correspondence_distance|double|2.0|max correspondence distance for GICP-family backends[m]|
|vgicp_voxel_resolution|double|1.0|voxel resolution for `SMALL_VGICP`[m]|
|transform_epsilon|double|0.01|transform epsilon to stop iteration in registration|
|voxel_leaf_size|double|0.2|down sample size of input cloud[m]|
|scan_max_range|double|100.0|max range of input cloud[m]|
|scan_min_range|double|1.0|min range of input cloud[m]|
|scan_periad|double|0.1|scan period of input cloud[sec]|
|use_pcd_map|bool|false|whether pcd_map is used or not|
|map_path|string|"/map/map.pcd"|pcd_map or ply_map file path|
|set_initial_pose|bool|false|whether or not to set the default value in the param file|
|initial_pose_x|double|0.0|x-coordinate of the initial pose value[m]|
|initial_pose_y|double|0.0|y-coordinate of the initial pose value[m]|
|initial_pose_z|double|0.0|z-coordinate of the initial pose value[m]|
|initial_pose_qx|double|0.0|Quaternion x of the initial pose value|
|initial_pose_qy|double|0.0|Quaternion y of the initial pose value|
|initial_pose_qz|double|0.0|Quaternion z of the initial pose value|
|initial_pose_qw|double|1.0|Quaternion w of the initial pose value|
|use_odom|bool|false|whether odom is used or not for initial attitude in point cloud registration|
|use_twist_prediction|bool|false|use a `geometry_msgs/msg/TwistWithCovarianceStamped` topic to advance the registration initial guess between accepted pose updates|
|twist_prediction_use_angular_velocity|bool|true|apply angular velocity as well as linear velocity during twist-based prediction|
|max_twist_prediction_dt|double|0.5|maximum time span integrated in one twist-based prediction step[sec]|
|use_imu|bool|false|whether 9-axis imu is used or not for point cloud distortion correction|
|use_imu_preintegration|bool|true|enable IMU preintegration for seed prediction; if no new IMU samples have arrived, the node falls back to the twist/previous-delta seed path, and if the IMU smoother diverges it disables IMU preintegration for the remainder of the run|
|imu_prediction_correction_guard_translation_m|double|2.0|disable IMU preintegration for the remainder of the run when an IMU-predicted scan needs a larger translation correction than this threshold[m]|
|imu_prediction_correction_guard_yaw_deg|double|4.0|disable IMU preintegration for the remainder of the run when an IMU-predicted scan needs a larger yaw correction than this threshold[deg]|
|enable_scan_voxel_filter|bool|true|apply the built-in `VoxelGrid` downsampling pass to each incoming scan before range filtering|
|enable_debug|bool|false|whether debug is done or not|
|predict_pose_from_previous_delta|bool|true|use the previous accepted pose delta as the next registration initial guess|
|enable_local_map_crop|bool|false|crop the target map around the current seed before registration; useful for large city-scale maps that overflow full-map NDT targets|
|local_map_radius|double|150.0|radius of the cropped local map around the current seed[m]|
|local_map_min_points|int|100|minimum number of cropped map points required before attempting registration|
|reject_above_score_threshold|bool|true|skip pose updates when `fitness_score` exceeds `score_threshold`|
|enable_borderline_seed_rejection_gate|bool|false|reject borderline `fitness_score` updates when the seed has already drifted from the last accepted pose|
|borderline_seed_gate_score_threshold|double|5.25|lower bound of the borderline fitness band used by `enable_borderline_seed_rejection_gate`|
|borderline_seed_gate_min_seed_translation_m|double|1.0|minimum seed drift before the borderline gate becomes active|
|enable_rejected_seed_update|bool|false|do not accept or publish a rejected alignment, but optionally reuse its converged pose as the next prediction seed|
|rejected_seed_update_min_rejections|int|0|minimum consecutive rejects required before `enable_rejected_seed_update` can apply|
|rejected_seed_update_max_fitness|double|10.0|maximum rejected `fitness_score` allowed for seed-only reuse|
|rejected_seed_update_max_correction_translation_m|double|2.0|maximum correction translation allowed for rejected seed reuse[m]|
|rejected_seed_update_max_correction_yaw_deg|double|2.0|maximum correction yaw allowed for rejected seed reuse[deg]|
|enable_recovery_retry_from_last_pose|bool|false|when a scan is rejected, optionally retry registration once from the last accepted pose before giving up|
|recovery_retry_from_last_pose_min_rejections|int|1|minimum consecutive rejects required before `enable_recovery_retry_from_last_pose` can apply|
|recovery_retry_from_last_pose_max_accepted_gap_sec|double|1.0|maximum open-loop gap allowed before `enable_recovery_retry_from_last_pose` may retry from the last accepted pose[sec]|
|enable_reinitialization_request_output|bool|true|publish a latched `/reinitialization_requested` flag and include reinit trigger fields in `/alignment_status`|
|reinitialization_trigger_threshold|double|0.95|score threshold for escalating from local recovery to a reinitialization request|
|reinitialization_trigger_gap_scale_sec|double|30.0|accepted-gap scale used by the reinitialization trigger score[sec]|
|reinitialization_trigger_seed_translation_scale_m|double|100.0|seed-drift scale used by the reinitialization trigger score[m]|
|reinitialization_trigger_reject_streak_scale|double|200.0|reject-streak scale used by the reinitialization trigger score|
|reinitialization_trigger_fitness_explosion_threshold|double|1000.0|fitness threshold treated as a catastrophic divergence for reinitialization scoring|
|enable_timer_publishing|bool|false|if true, publish tf and pose on a set timer frequency|
|pose_publish_frequency|double|10.0|publishing frequency if enable_timer_publishing is true|

## Nav2 launch

For Nav2 integration, use:

```bash
ros2 launch lidar_localization_ros2 nav2_lidar_localization.launch.py
```

This launch defaults to `param/nav2_ndt_urban.yaml`, a twist-aided NDT_OMP preset tuned on the
Autoware Istanbul urban localization dataset. The current default is
`thr6 + borderline_seed_gate`.
It expects:

- an odom source publishing `odom -> base_link`
- a twist topic on the configured `twist_topic`
- an initial pose on `/initialpose`

`use_imu_preintegration` is now default-on in the generic and recommended presets. The repo has
guard rails for it: if IMU messages are present, the localizer can use gyro + accelerometer
preintegration for the next registration seed; if no new IMU samples have arrived it falls back
to the older twist / previous-delta seed path, and if the IMU smoother diverges it disables IMU
preintegration for the remainder of the run instead of poisoning the localization state. The
early correction guard is also parameterized through
`imu_prediction_correction_guard_translation_m` and
`imu_prediction_correction_guard_yaw_deg`.

The default Nav2 preset also enables `enable_local_map_crop:=true`, which avoids the
full-map NDT overflow/crash path on large city-scale pointcloud maps by rebuilding the NDT target
around the current seed each scan.

### Map Format Notes

- Runtime `map_path` supports `.ply` and `.pcd`, but the current benchmarked path in this repo
  should prefer `.ply`.
- For generated benchmark maps, use binary little-endian float32 PLY (`property float x/y/z`).
- Avoid Open3D's default double-precision PLY output for runtime benchmarking.
- Generated `.pcd` maps are fine for inspection tools, but they are not the recommended
  benchmark/runtime path in this repo today.

The Nav2 presets also set `enable_scan_voxel_filter:=false` because the Autoware Istanbul
`/localization/util/downsample/pointcloud` topic is already downsampled upstream.

If you want the wrapper to react to `/reinitialization_requested`, enable the optional external
supervisor:

```bash
ros2 launch lidar_localization_ros2 nav2_navigation.launch.py \
  enable_reinitialization_supervisor:=true \
  set_initial_pose:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_z:=0.0 \
  initial_pose_qx:=0.0 initial_pose_qy:=0.0 initial_pose_qz:=0.0 initial_pose_qw:=1.0
```

That launches `republish_initialpose_on_reinit.py`, which listens to the latched
`/reinitialization_requested` flag and republishes `/initialpose` in a short burst. By default it
reuses the configured `initial_pose_*`; if you want it to republish the latest localization pose
instead, set `reinitialization_supervisor_use_latest_pose:=true`. The main tuning args are:

- `enable_reinitialization_supervisor`
- `reinitialization_supervisor_use_latest_pose`
- `reinitialization_supervisor_publish_count`
- `reinitialization_supervisor_publish_interval_sec`
- `reinitialization_supervisor_cooldown_sec`

This node is intentionally outside the core localizer. The localizer only emits the request
signal; the wrapper decides whether and how to act on it.

Current recommendation: keep `reinitialization_supervisor_use_latest_pose:=false` and
`reinitialization_supervisor_publish_count:=1`. In long Istanbul replay, republishing the
configured initial pose recovered to `ok` more often than republishing the latest localization
pose, while both were clearly better than leaving the request unhandled. A follow-up `120 s`
configured-pose burst sweep also favored `publish_count=1` over `3`, while `5` aborted before the
supervisor could help.

To re-run the long replay policy check as a pass/fail regression:

```bash
ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_regression.sh
```

That compares `no_supervisor` against `configured_initial_pose_count1` on the same `120 s` replay
and writes `comparison.json`, `summary.md`, and `regression_result.json` under
`artifacts/public/nav2_reinit_supervisor_regression_120/`.
The pass/fail gate is intentionally conservative about goal success and stuck-request reduction,
not a single observed `ok` recovery row, because that specific post-trigger signal is noisier
across reruns than the large drop in `reinitialization_requested_rows`.

If your robot already publishes the sensor static transforms, disable the built-in zero-offset publishers with
`publish_lidar_tf:=false publish_imu_tf:=false`. You can also override
`global_frame_id`, `odom_frame_id`, `base_frame_id`, `lidar_frame_id`, and `imu_frame_id`
from the launch file instead of editing the YAML preset.

If you want the generic parameters instead, override `localization_param_dir:=.../param/localization.yaml`.

For replay experiments on long reject streaks, there are also experimental presets:

- `param/nav2_ndt_urban_rejected_seed_update_r1.yaml`
- `param/nav2_ndt_urban_recovery_retry_from_last_pose.yaml`
- `param/nav2_ndt_urban_recovery_retry_from_last_pose_r3.yaml`

Keep these experimental. `rejected_seed_update_r1` improved some reject-streak diagnostics but lost on
the Istanbul `60 s` repeat benchmark. `recovery_retry_from_last_pose(min_rejections=1)` recovered too
aggressively and reduced benchmark throughput. The capped `seed15` variant helped on the `120 s` repeat,
but a seeded `180 s` compare later showed a large translation regression, so it remains experimental and
the default has been reverted to `borderline_seed_gate` only. Relevant artifacts:

- `artifacts/public/istanbul_60s_recovery_retry_from_last_pose_r3_repeat_compare/`
- `artifacts/public/istanbul_120s_recovery_retry_from_last_pose_r3_gap1_repeat_compare/`
- `artifacts/public/istanbul_120s_recovery_retry_from_last_pose_r3_gap1_seed15_repeat_compare_rerun2/`
- `artifacts/public/istanbul_180s_recovery_retry_from_last_pose_r3_gap1_seed15_compare_seeded/`

For a full Nav2 wrapper launch, use:

```bash
ros2 launch lidar_localization_ros2 nav2_navigation.launch.py \
  map_yaml:=/absolute/path/to/occupancy_map.yaml
```

This launch:

- starts `nav2_lidar_localization.launch.py`
- launches `nav2_map_server` for the occupancy grid map
- includes `nav2_bringup/navigation_launch.py` when Nav2 packages are installed
- rewrites the Nav2 params at launch time so the costmaps use `pointcloud_topic` and the stack uses `odom_topic`
- enables localizer timer publishing by default so `map -> odom` and pose keep updating for Nav2

Additional requirements for the full Nav2 stack:

- `nav2_bringup`, `nav2_map_server`, and `nav2_lifecycle_manager` installed
- an occupancy-grid `map_yaml` for planning; the pointcloud map used by `lidar_localization_ros2` is not enough for Nav2 global planning
- a pointcloud topic for costmap obstacle layers; the sample params in `param/nav2_humble_pointcloud.yaml` default to `PointCloud2`
- an odom source publishing `odom -> base_link` and `/odom`; for smoke tests without one, you can use `publish_identity_odom:=true` to start a zero-motion fallback publisher, or `publish_twist_odom:=true` to integrate a `TwistWithCovarianceStamped` topic into `/odom`

If you only have a 3D pointcloud map, the wrapper can generate a cropped occupancy map on demand and
override the localizer map path in the same command:

```bash
ros2 launch lidar_localization_ros2 nav2_navigation.launch.py \
  pcd_map_path:=/absolute/path/to/map.pcd \
  reference_csv:=/absolute/path/to/reference.csv \
  generate_map_from_pcd:=true \
  generated_map_output_dir:=/tmp/nav2_map \
  set_initial_pose:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_z:=0.0 \
  initial_pose_qx:=0.0 initial_pose_qy:=0.0 initial_pose_qz:=0.0 initial_pose_qw:=1.0
```

In that mode, `nav2_navigation.launch.py`:

- rewrites the localizer params so `map_path` points at `pcd_map_path`
- generates `map_yaml` from `pcd_map_path + reference_csv` when `map_yaml` is omitted
- optionally injects the initial pose into the localizer params before launch
- rewrites the sample Nav2 params so `global_frame_id`, `odom_frame_id`, `base_frame_id`, and optional `robot_radius` stay aligned with the localizer launch

For a pure smoke or demo setup with no real odometry source, add:

```bash
publish_identity_odom:=true
```

This starts `publish_identity_odom.py`, which publishes zero-motion `/odom` and `odom -> base_link`.
Use it only when a real odom source is unavailable.

If you have a twist topic but no odom topic, add:

```bash
publish_twist_odom:=true
```

This starts `publish_odom_from_twist.py`, which integrates the configured `twist_topic` into
`/odom` and `odom -> base_link`. It is useful for bag replay and minimal integrations where
`TwistWithCovarianceStamped` is available before a real odom source is wired in.

For a self-contained moving Nav2 smoke test with no real LiDAR localizer and no real odom source, use:

```bash
ros2 launch lidar_localization_ros2 nav2_navigation.launch.py \
  map_yaml:=/absolute/path/to/occupancy_map.yaml \
  use_odom_localization_demo:=true \
  publish_cmd_vel_odom:=true \
  set_initial_pose:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_z:=0.0 \
  initial_pose_qx:=0.0 initial_pose_qy:=0.0 initial_pose_qz:=0.0 initial_pose_qw:=1.0
```

In that mode:

- `publish_pose_from_odom.py` publishes `/localization/pose_with_covariance` and `map -> odom`
- `publish_cmd_vel_odom.py` integrates `/cmd_vel` into `/odom` and `odom -> base_link`
- Nav2 can execute a non-zero `navigate_to_pose` goal in a self-contained demo path

Use this only for smoke/demo validation. It bypasses the real LiDAR localizer.

To replay the same smoke test with one command, use:

```bash
ros2 run lidar_localization_ros2 run_nav2_demo_smoke \
  --map-yaml /absolute/path/to/occupancy_map.yaml \
  --initial-pose-x 0.0 --initial-pose-y 0.0 \
  --goal-x 1.0 --goal-y 0.0
```

The helper launches `nav2_navigation.launch.py` with `use_odom_localization_demo:=true` and
checks that `/navigate_to_pose` finishes with `SUCCEEDED` using an internal `rclpy` action client,
so it does not depend on `ros2 action ...` CLI daemon behavior.

For a real localizer + odom setup that is already running, you can send a relative validation goal
without hand-calculating map coordinates:

```bash
ros2 run lidar_localization_ros2 send_nav2_goal.py \
  --use-current-pose \
  --wait-for-pose-motion-m 1.0 \
  --forward-m 1.0
```

This reads `/localization/pose_with_covariance`, computes a goal `1.0 m` forward in the current robot frame,
and sends it to `/navigate_to_pose`. `--wait-for-pose-motion-m` avoids dispatching the goal before the
localizer has actually moved away from its initial pose.

For replayed real-localizer validation, also wait for `odom <- base_link` and the Nav2 lifecycle nodes
to become active before sending the relative goal:

```bash
ros2 run lidar_localization_ros2 send_nav2_goal.py \
  --use-current-pose \
  --wait-for-pose-motion-m 0.2 \
  --wait-for-transform-target-frame odom \
  --wait-for-transform-source-frame base_link \
  --wait-for-lifecycle-node controller_server \
  --wait-for-lifecycle-node planner_server \
  --wait-for-lifecycle-node behavior_server \
  --wait-for-lifecycle-node bt_navigator \
  --post-ready-delay-sec 1.0 \
  --forward-m 1.0
```

Without those waits, `navigate_to_pose` can be visible but still reject the goal during replay bringup
while the local costmap and lifecycle transitions are still catching up.

To replay that production-style path with one command, use:

```bash
ros2 run lidar_localization_ros2 run_nav2_replay_smoke \
  --map-yaml /absolute/path/to/occupancy_map.yaml \
  --pcd-map-path /absolute/path/to/map.pcd \
  --bag-path /absolute/path/to/localization_rosbag \
  --initial-pose-x 0.0 --initial-pose-y 0.0 \
  --initial-pose-z 0.0 \
  --initial-pose-qx 0.0 --initial-pose-qy 0.0 \
  --initial-pose-qz 0.0 --initial-pose-qw 1.0
```

The helper runs the restamp relay, launches `nav2_navigation.launch.py` with
`publish_twist_odom:=true`, starts `ros2 bag play --clock`, and sends a relative validation goal with
the pose/TF/lifecycle waits above. It is intended for the real localizer path, unlike
`run_nav2_demo_smoke`.

If you also want the replay helper to arm the optional reinitialization supervisor, add:

```bash
  --enable-reinitialization-supervisor \
  --reinitialization-supervisor-publish-count 3 \
  --reinitialization-supervisor-publish-interval-sec 0.2 \
  --reinitialization-supervisor-cooldown-sec 15.0
```

Add `--reinitialization-supervisor-use-latest-pose` if you want the supervisor to republish the latest
`/localization/pose_with_covariance` instead of the configured `initial_pose_*`.

For rosbag replay with `publish_twist_odom:=true`, also enable simulated time in the launch and publish `/clock`
from `ros2 bag play`:

```bash
ros2 launch lidar_localization_ros2 nav2_navigation.launch.py \
  map_yaml:=/absolute/path/to/occupancy_map.yaml \
  pcd_map_path:=/absolute/path/to/map.pcd \
  cloud_topic:=/localization/replay/downsample/pointcloud \
  pointcloud_topic:=/localization/replay/downsample/pointcloud \
  twist_topic:=/localization/replay/twist_with_covariance \
  publish_twist_odom:=true \
  use_sim_time:=true

ros2 run lidar_localization_ros2 relay_localization_inputs_with_current_stamp.py \
  --ros-args -p use_sim_time:=true

ros2 bag play /absolute/path/to/localization_rosbag \
  --clock 20.0 \
  --rate 1 \
  --topics /tf_static /localization/util/downsample/pointcloud /localization/twist_estimator/twist_with_covariance
```

`publish_odom_from_twist.py` integrates twist using the message header timestamps but stamps the published
odom/TF with the current ROS clock so replay bags with mismatched topic header stamps can still satisfy
Nav2 and `map -> odom` lookups.
`relay_localization_inputs_with_current_stamp.py` does the same for replayed pointcloud/twist inputs so
the real localizer, Nav2 costmaps, and `/clock` all operate on a consistent time base during rosbag replay.

To generate a coarse occupancy map from a 3D PCD map, use:

```bash
ros2 run lidar_localization_ros2 generate_occupancy_map_from_pcd.py \
  --pcd /absolute/path/to/map.pcd \
  --reference-csv /absolute/path/to/reference.csv \
  --route-padding-m 20 \
  --resolution 0.25 \
  --output-dir /tmp/nav2_map
```

For large city-scale PCD maps, use `--reference-csv` or explicit `--x-min/--x-max/--y-min/--y-max`
to crop around the route of interest before rasterizing.

## demo

demo data(ROS1) by Tier IV（The link has changed and is now broken.)  
https://data.tier4.jp/rosbag_details/?id=212  
To use ros1 rosbag , use [rosbags](https://pypi.org/project/rosbags/).  
The Velodyne VLP-16 was used in this data.

Before running, put `bin_tc-2017-10-15-ndmap.pcd` into your `map` directory and  
edit the `map_path` parameter of `localization.yaml` in the `param` directory accordingly.
```
rviz2 -d src/lidar_localization_ros2/rviz/localization.rviz
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
ros2 bag play tc_2017-10-15-15-34-02_free_download/
```

<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)

## roadmap

See [docs/competitive_roadmap.md](docs/competitive_roadmap.md) for a competitor analysis and a prioritized roadmap against Autoware, hdl_localization, and small_gicp/GLIM-class systems.

## benchmarking

See [docs/benchmarking.md](docs/benchmarking.md) for the built-in rosbag benchmark harness and trajectory evaluation flow.

For a reproducible real dataset, fetch the official `hdl_localization` sample with:

```bash
source scripts/setup_local_env.sh
scripts/fetch_official_hdl_localization_sample.sh
```

This prepares `hdl_400.bag`, `map.pcd`, and a filtered `rosbag2` copy under `data/official/hdl_localization`.

For any public or externally shared benchmark result, use this official `hdl_localization` sample.
Do not publish results based on private local field logs or graph-derived synthetic bags as if they were open datasets.

To rerun the current public validation boundary in one command, use:

```bash
source scripts/setup_local_env.sh
scripts/run_public_regression_suite.sh
```

This emits the final gate result to:

- `artifacts/public/public_regression_suite/summary.json`
- `artifacts/public/public_regression_suite/summary.md`

The suite currently checks:

- Autoware Istanbul `60 s` default-on no-IMU safety
- HDL `60 s` IMU safety / throughput regression for the guarded default-on path, repeated twice by default

Interpretation:

- Istanbul has no IMU acceleration stream, so it is only used to confirm that the default-on IMU-capable preset does not regress a no-IMU public dataset
- HDL is the public IMU smoke path; because it has no strong public GT and single-run throughput is noisy, the suite uses broad safety / throughput bounds there

For private or NC datasets, keep the raw bag paths outside the repository and use the manifest wrapper instead:

```bash
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest /absolute/path/to/private_dataset_run_manifest.yaml
```

Example templates live in:

- `param/benchmark/private_dataset_run_manifest.example.yaml`
- `param/benchmark/private_dataset_sweep_manifest.example.yaml`

The manifest path can stay outside the repo; the wrapper copies it into the benchmark output directory
and writes the generated dataset-specific localization YAML there as well.
For repo-owned references such as `base_param_yaml` or `config_json`, use `repo://...` in the manifest.
If you keep a real private manifest inside the repo, use an ignored name like
`param/benchmark/private_dataset_run_manifest.local.yaml`.

For a public dataset that has `LiDAR + IMU + GT` but no packaged pointcloud map, use the
mapless-public workflow instead:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 scaffold_mapless_public_dataset_bundle.py \
  --spec /absolute/path/to/mapless_public_dataset_bundle.yaml
```

Start from:

- `param/benchmark/mapless_public_dataset_bundle.example.yaml`
- `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml` for the Boreas starter path

That scaffolds:

- `run_mapping.sh`
- `run_extract_reference.sh` when the spec includes `reference:`
- `localization_run_manifest.yaml`
- `run_localization_benchmark.sh`
- `run_generate_nav2_map.sh`

The intent is to keep `mapping run` and `localization run` separate. See
[docs/mapless_public_dataset_workflow.md](docs/mapless_public_dataset_workflow.md).
When the dataset has GT in-bag, use the `reference:` section so the bundle also generates a
reference CSV and an initial-pose YAML before the localization benchmark.
For Boreas specifically, first convert raw sequences with
`ros2 run lidar_localization_ros2 convert_boreas_sequence_to_rosbag2.py`, then scaffold from the
Boreas example spec. See
[docs/boreas_mapless_public_dataset_workflow.md](docs/boreas_mapless_public_dataset_workflow.md).

For a stronger public `map-based lidar localization` benchmark, fetch the official Autoware Istanbul localization dataset with:

```bash
source scripts/setup_local_env.sh
scripts/fetch_official_autoware_istanbul_dataset.sh
```

This prepares `pointcloud_map.pcd` and a `localization_rosbag` directory under `data/official/autoware_istanbul`.

Use `ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 ...` to extract
the GNSS reference trajectory directly from the bag instead of recording it through replay.
When testing twist-aided prediction on this dataset, launch with
`twist_topic:=/localization/twist_estimator/twist_with_covariance`.
The sweep helper also supports this via
`--twist-topic /localization/twist_estimator/twist_with_covariance`.
