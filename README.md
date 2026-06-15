<div align="center">

<h1>lidar_localization_ros2</h1>

<p><strong>Map-based 3D LiDAR localization for ROS 2, Nav2, and repeatable rosbag evaluation.</strong></p>

<p>
  <a href="https://github.com/rsasaki0109/lidar_localization_ros2/actions/workflows/main.yml">
    <img alt="build" src="https://github.com/rsasaki0109/lidar_localization_ros2/actions/workflows/main.yml/badge.svg">
  </a>
  <img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS%202-Jazzy-2563eb">
  <img alt="ROS 2 Humble compatible" src="https://img.shields.io/badge/ROS%202-Humble-compatible-1f5b99">
  <img alt="Release v1.0.0" src="https://img.shields.io/badge/release-v1.0.0-2f855a">
  <img alt="Backend NDT OMP" src="https://img.shields.io/badge/default-NDT__OMP-4a5568">
  <img alt="License BSD 2 Clause" src="https://img.shields.io/badge/license-BSD--2--Clause-6b46c1">
</p>

<img src="./images/path.png" alt="LiDAR localization path over pointcloud map" width="720">

<sub>Green: localized path. Red: pointcloud map. Grid: 50 m x 50 m cells.</sub>

</div>

## What This Is

`lidar_localization_ros2` is a ROS 2 package for 3D pointcloud map localization. It provides a
runtime localizer, Nav2 launch wrappers, benchmark tools, and experiment runners for recovery and
relocalization work.

Current default:

```text
param/nav2_ndt_urban.yaml
```

Current boundary: short public replay, smoke, and controlled regression paths are validated.
Long-horizon robustness on stronger public `LiDAR + IMU + GT` data and real robot deployment still
need dataset or hardware-specific validation.

v1.1 relocalization stops at a validated dry-run `/initialpose` command artifact.
It does not claim automatic runtime recovery or production-grade global relocalization.
See [docs/v1_1_relocalization.md](docs/v1_1_relocalization.md).

## 30-Minute Public Demo

Reproduce map-based localization on the official **Autoware Istanbul** public dataset:

```bash
cd /path/to/lidarloc_ws/repo
source scripts/setup_local_env.sh
scripts/run_public_demo.sh
```

What it does:

- downloads the Istanbul map + bag when missing
- builds `lidar_localization_ros2` when the overlay is missing
- replays `60 s` of urban localization
- writes `demo_report.md`, `demo_report.json`, and `trajectory_xy.png`

Typical first-run time: about **15–30 minutes** (network download dominates).
Re-runs with `--resume` are much faster.

Latest local smoke on Autoware Istanbul `60 s` (dataset already present, 2026-06-10,
new preset with `local_map_crop` + `recovery_retry_from_last_pose`):

- public demo best run: translation RMSE `0.95 m`, rotation `2.13 deg`, matched `220`
- public regression gate: translation RMSE `1.00 m`, rotation `2.33 deg`, matched `104` — **pass**
- outlier runs still happen on identical seed/map (e.g. `3.31 m`, `19` matched); see variance notes
- wall time: about `70 s` per demo run

Istanbul replay is run-to-run variable; see [docs/benchmarking.md](docs/benchmarking.md)
for the variance notes and prefer `scripts/run_public_regression_suite.sh` for release gates.

Example report output directory:

```text
lidarloc_ws/artifacts/public/demo/latest/
  demo_report.md
  demo_report.json
  trajectory_xy.png
```

## Global Localization Demo (experimental)

On-demand map-wide relocalization with no initial pose, on the official HDL
sample bag: the `~/query` service of `scripts/global_localization_node.py`
runs a BBS_2D branch-and-bound search over the occupancy grid, the top
candidate is published as `/initialpose`, and NDT localization tracks the rest
of the run.

<img src="./images/global_localization_demo.gif" alt="Kidnapped start, BBS_2D global localization candidates, NDT tracking resumes" width="480">

The service node is opt-in and never part of the default launch. An optional
supervisor (`scripts/reinitialization_supervisor_node.py`) can close the loop
automatically — on the `/reinitialization_requested` signal it queries the service
and re-seeds `/initialpose` behind explicit safety guards. See
[docs/global_localization.md](docs/global_localization.md) for how to run the
service and the supervisor, and
[docs/global_localization_roadmap.md](docs/global_localization_roadmap.md) for
status and limits (query latency, validated windows).

## Quick Start

Expected workspace layout:

```text
lidarloc_ws/
  build_ws/
  local_prefix/
  repo/
```

Build:

```bash
cd /path/to/lidarloc_ws/repo
source scripts/setup_local_env.sh
cd ../build_ws
colcon build --symlink-install --packages-up-to lidar_localization_ros2
cd ../repo
source scripts/setup_local_env.sh
```

For the full no-sudo local-prefix workflow, see [docs/local_build.md](docs/local_build.md).

## Choose A Workflow

| Goal | Command |
|---|---|
| LiDAR localization only | `ros2 launch lidar_localization_ros2 nav2_lidar_localization.launch.py` |
| Full Nav2 wrapper | `ros2 launch lidar_localization_ros2 nav2_navigation.launch.py map_yaml:=/absolute/path/to/map.yaml` |
| Jetson + Livox MID-360 | `ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py map_path:=/absolute/path/to/map.pcd cloud_topic:=/livox/points imu_topic:=/livox/imu` |
| Self-contained Nav2 smoke | `ros2 run lidar_localization_ros2 run_nav2_demo_smoke --map-yaml /absolute/path/to/map.yaml --initial-pose-x 0.0 --initial-pose-y 0.0 --goal-x 1.0 --goal-y 0.0` |
| Real-localizer replay smoke | `ros2 run lidar_localization_ros2 run_nav2_replay_smoke --map-yaml /absolute/path/to/map.yaml --pcd-map-path /absolute/path/to/map.pcd --bag-path /absolute/path/to/bag` |
| **30-minute public demo** | `source scripts/setup_local_env.sh && scripts/run_public_demo.sh` |
| Public regression | `source scripts/setup_local_env.sh && scripts/run_public_regression_suite.sh` |

## Frames And TF

Default frames: `map` (global), `odom`, `base_link`.

- `/initialpose` must use `header.frame_id = map`
- default TF mode publishes `map -> base_link`
- Nav2-style mode uses `enable_map_odom_tf: true` and requires an external `odom -> base_link` publisher
- `use_odom: true` uses the `/odom` topic for pose prediction; it is not the same as odom TF

See [docs/frame_contract.md](docs/frame_contract.md) for the full contract for
issues [#58](https://github.com/rsasaki0109/lidar_localization_ros2/issues/58) and
[#27](https://github.com/rsasaki0109/lidar_localization_ros2/issues/27).

If the node starts but pose does not update, see [docs/troubleshooting.md](docs/troubleshooting.md).

## Nav2 Requirements

For full Nav2 use, provide:

- a 3D pointcloud map for the localizer
- a 2D occupancy `map_yaml` for Nav2 planning
- an odom source publishing `odom -> base_link`
- an initial pose on `/initialpose` in the `map` frame

If you only have a 3D PCD map, generate a cropped occupancy map:

```bash
ros2 run lidar_localization_ros2 generate_occupancy_map_from_pcd.py \
  --pcd /absolute/path/to/map.pcd \
  --reference-csv /absolute/path/to/reference.csv \
  --route-padding-m 20 \
  --resolution 0.25 \
  --output-dir /tmp/nav2_map
```

## Topics

Main inputs:

- `/cloud` or configured `cloud_topic`: `sensor_msgs/msg/PointCloud2`
- `/map` or `map_path`: pointcloud map
- `/initialpose`: `geometry_msgs/msg/PoseWithCovarianceStamped`
- `/odom`: `nav_msgs/msg/Odometry`, optional but recommended for Nav2
- `/imu`: `sensor_msgs/msg/Imu`, optional

Main outputs:

- `/pcl_pose`: `geometry_msgs/msg/PoseStamped`
- `/path`: `nav_msgs/msg/Path`
- `/alignment_status`: `diagnostic_msgs/msg/DiagnosticArray`
- `/reinitialization_requested`: `std_msgs/msg/Bool`
- `/initial_map`: `sensor_msgs/msg/PointCloud2` when `use_pcd_map=true`

## Map Notes

- Runtime map paths support `.ply` and `.pcd`.
- Public benchmark runs should prefer binary little-endian float32 `.ply` maps.
- Generated `.pcd` maps are useful for inspection, but are not the preferred benchmark path.

## Latest Public Validation

Regenerate the dashboard from local demo / regression artifacts:

```bash
source scripts/setup_local_env.sh
scripts/run_public_validation_dashboard.sh
```

Outputs:

```text
lidarloc_ws/artifacts/public/dashboard/
  index.md
  index.html
  dashboard.json
```

| Suite | Dataset | What it checks |
|---|---|---|
| Public demo | Autoware Istanbul `60 s` | Star-friendly replay + trajectory report |
| Public regression | Istanbul `60 s` | No-IMU safety gate on public urban replay |
| Public regression | HDL `hdl_400` `60 s` | IMU safety / throughput smoke |
| Release regression | Nav2 reinit supervisor `150 s` | Controlled recovery wrapper behavior |

Historical snapshots and interpretation notes live in
[docs/public_validation_log.md](docs/public_validation_log.md).

## Validation

Fast build check:

```bash
source scripts/setup_local_env.sh
cd ../build_ws
colcon build --symlink-install --packages-up-to lidar_localization_ros2
```

Experiment comparison:

```bash
ros2 run lidar_localization_ros2 run_experiment_suite.py
```

Release-style regression:

```bash
ros2 run lidar_localization_ros2 run_release_regression_suite.sh
```

This validation path is public replay regression, not Jetson + MID-360 hardware validation.

## Branch Workflow

Keep branch usage simple:

- `main`: stable documentation and release-ready defaults
- `feat/<topic>`: active feature or benchmark work
- `fix/<topic>`: focused bug fixes
- `review/pr-<number>`: temporary PR review branches
- `backup/<name>`: local safety snapshots only

Safe cleanup flow:

```bash
git switch main
git pull --ff-only
git branch --merged main
git branch -d <merged-branch>
git branch --no-merged main
git log --oneline main..<branch>
```

Only delete unmerged branches after checking their diff:

```bash
git diff main...<branch>
git branch -D <branch>
```

## Read More

| Need | Read |
|---|---|
| Current validated scope | [docs/v1_status.md](docs/v1_status.md) |
| Local build setup | [docs/local_build.md](docs/local_build.md) |
| Nav2 and benchmark commands | [docs/benchmarking.md](docs/benchmarking.md) |
| MID-360 legged robot bringup | [docs/mid360_legged_jetson.md](docs/mid360_legged_jetson.md) |
| Public validation history | [docs/public_validation_log.md](docs/public_validation_log.md) |
| v1.1 relocalization work | [docs/v1_1_relocalization.md](docs/v1_1_relocalization.md) |
| Experiment interfaces and decisions | [docs/interfaces.md](docs/interfaces.md), [docs/experiments.md](docs/experiments.md), [docs/decisions.md](docs/decisions.md) |
| Roadmap | [docs/competitive_roadmap.md](docs/competitive_roadmap.md) |
| Development plan | [docs/development_plan.md](docs/development_plan.md) |
| Global localization (run it) | [docs/global_localization.md](docs/global_localization.md) |
| Global localization phases | [docs/global_localization_roadmap.md](docs/global_localization_roadmap.md) |
| Reliability / open issues | [docs/reliability_roadmap.md](docs/reliability_roadmap.md) |
| Bringup troubleshooting | [docs/troubleshooting.md](docs/troubleshooting.md) |
| Map alignment / pose offset | [docs/map_alignment.md](docs/map_alignment.md) |
| Pose covariance semantics | [docs/pose_covariance.md](docs/pose_covariance.md) |
| IMU estimation (preintegration) | [docs/imu_estimation.md](docs/imu_estimation.md) |
| Frame / TF contract | [docs/frame_contract.md](docs/frame_contract.md) |

## ROS 2 Support

| Distro | Status | CI |
|---|---|---|
| Jazzy | primary target for new users (2026) | `jazzy_build` in [`.github/workflows/main.yml`](.github/workflows/main.yml) |
| Humble | supported for existing deployments | `humble_build` in [`.github/workflows/main.yml`](.github/workflows/main.yml) |

Local development in this workspace still uses the Humble-based `scripts/setup_local_env.sh`
overlay workflow documented in [docs/local_build.md](docs/local_build.md).

## Dependencies

- ROS 2 Jazzy or Humble
- [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2.git) (`humble` branch; used for both distro CI builds today)
- [small_gicp](https://github.com/koide3/small_gicp), optional for `SMALL_GICP` and `SMALL_VGICP`

See [CHANGELOG.md](CHANGELOG.md) for release notes and distro support notes.
