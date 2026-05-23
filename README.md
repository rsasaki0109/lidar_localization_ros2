<div align="center">

<h1>lidar_localization_ros2</h1>

<p><strong>Map-based 3D LiDAR localization for ROS 2, Nav2, and repeatable rosbag evaluation.</strong></p>

<p>
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-1f5b99">
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
| Public regression | `source scripts/setup_local_env.sh && scripts/run_public_regression_suite.sh` |

## Nav2 Requirements

For full Nav2 use, provide:

- a 3D pointcloud map for the localizer
- a 2D occupancy `map_yaml` for Nav2 planning
- an odom source publishing `odom -> base_link`
- an initial pose on `/initialpose`

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

Latest recorded public validation snapshot:

- [docs/public_validation_log.md](docs/public_validation_log.md)
- `2026-05-22`, commit `2a5f11f`, release regression `overall_pass=true`
- Istanbul `60 s` no-IMU safety check: `1.176 m` translation RMSE, `0.393 deg` rotation RMSE
- HDL `60 s` IMU safety check, two-repeat median: pose rows `558.5 -> 553.5`
- Nav2 reinitialization supervisor `150 s`: requested rows `944 -> 7`

This snapshot is public replay validation, not Jetson + MID-360 hardware validation.

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

## Dependencies

- ROS 2 Humble
- [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2.git)
- [small_gicp](https://github.com/koide3/small_gicp), optional for `SMALL_GICP` and `SMALL_VGICP`

See [CHANGELOG.md](CHANGELOG.md) for release notes.
