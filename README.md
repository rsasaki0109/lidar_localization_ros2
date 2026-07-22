<div align="center">

<h1>lidar_localization_ros2</h1>

<p><strong>Map-based 3D LiDAR localization for ROS 2 and Nav2.</strong></p>

<p>
  <a href="https://github.com/rsasaki0109/lidar_localization_ros2/actions/workflows/main.yml">
    <img alt="build" src="https://github.com/rsasaki0109/lidar_localization_ros2/actions/workflows/main.yml/badge.svg">
  </a>
  <img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS%202-Jazzy-2563eb">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-compatible-1f5b99">
  <img alt="License BSD 2 Clause" src="https://img.shields.io/badge/license-BSD--2--Clause-6b46c1">
</p>

<img src="./images/path.png" alt="LiDAR localization path over pointcloud map" width="720">

<p><a href="./docs/koide_gif_gallery.md">Explore the complete Koide indoor/outdoor GIF gallery →</a></p>

</div>

## Features

- NDT/GICP localization against `.pcd` and `.ply` maps
- standalone, Nav2, and Livox MID-360 launch configurations
- odometry/IMU prediction, scan deskew, diagnostics, and guarded recovery
- rosbag demo and regression tools

ROS 2 Jazzy with NDT_OMP is the recommended starting point. Continuous-time deskew is
enabled by default and safely leaves scans unchanged until point timing and motion data
are ready. Guarded global initialization is enabled automatically when quickstart is
given a matching occupancy map. See [v1 status](docs/v1_status.md) for validated scope
and limitations.

## Install

```bash
mkdir -p ~/lidarloc_ws/src
cd ~/lidarloc_ws/src
git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
cd lidar_localization_ros2
scripts/bootstrap_colcon_workspace.sh --build
source ~/lidarloc_ws/install/setup.bash
```

For manual builds and no-sudo setup, see [local build](docs/local_build.md).

## Quick Start

Start localization and RViz with one command:

```bash
ros2 run lidar_localization_ros2 quickstart.py \
  --profile standalone \
  --map /absolute/path/to/map.pcd
```

Quickstart detects unambiguous sensor topics, generates a reusable configuration,
restores only a pose saved against the same map, and verifies tracking. Add the matching
occupancy map to use the default guarded global initialization with 3D NDT scoring:

```bash
ros2 run lidar_localization_ros2 quickstart.py \
  --profile mid360 \
  --map /absolute/path/to/map.pcd \
  --occupancy-map /absolute/path/to/map.yaml
```

If no safe candidate is available, it asks for **2D Pose Estimate** in RViz; it never
guesses the origin. See [quickstart and automatic initialization](docs/quickstart.md).
Use `--no-auto-initialize` to disable saved-pose restoration and global initialization,
or launch with `use_continuous_time_deskew:=false` to disable deskew.

Common launches:

```bash
# Standalone localization
ros2 launch lidar_localization_ros2 nav2_lidar_localization.launch.py

# Nav2
ros2 launch lidar_localization_ros2 nav2_navigation.launch.py \
  map_yaml:=/absolute/path/to/map.yaml

# Livox MID-360
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd \
  cloud_topic:=/livox/points imu_topic:=/livox/imu
```

Check topics, TF, pose output, and diagnostics with:

```bash
ros2 run lidar_localization_ros2 check_lidar_localization_bringup.py \
  --profile standalone
```

## Runtime Contract

The default frames are `map`, `odom`, and `base_link`.

- `/initialpose` is expressed in `map`.
- Standalone mode publishes `map -> base_link`.
- Nav2 mode publishes `map -> odom` and requires an external `odom -> base_link`.
- `use_odom: true` consumes `/odom`; it does not publish odometry TF.
- Static LiDAR and IMU transforms must have only one publisher.

Main inputs are `/cloud`, `/initialpose`, `/odom`, and `/imu`. Main outputs are
`/pcl_pose`, `/path`, `/alignment_status`, and `/reinitialization_requested`. All
topic names are configurable. See [frame contract](docs/frame_contract.md) and
[troubleshooting](docs/troubleshooting.md) for details.

Nav2 additionally requires a 2D occupancy map and an `odom -> base_link` source.

## Public Demo

The Autoware Istanbul demo downloads its public assets, builds when needed, replays
60 seconds, and writes a trajectory report:

```bash
source scripts/setup_local_env.sh
scripts/run_public_demo.sh
```

First-time setup may take 15–30 minutes. For datasets, metrics, and regression
commands, see [benchmarking](docs/benchmarking.md).

## Documentation

- [Validated scope](docs/v1_status.md)
- [Frames](docs/frame_contract.md) and [troubleshooting](docs/troubleshooting.md)
- [Benchmarking](docs/benchmarking.md)
- [MID-360 bringup](docs/mid360_legged_jetson.md)
- [IMU estimation](docs/imu_estimation.md) and [pose covariance](docs/pose_covariance.md)
- [Global localization](docs/global_localization.md)
- [Quickstart and automatic initialization](docs/quickstart.md)
- [Koide demo gallery](docs/koide_gif_gallery.md)
- [Release notes](CHANGELOG.md)

## Support

ROS 2 Jazzy is the primary target; Humble remains supported for existing deployments.
[ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2) is required and
[small_gicp](https://github.com/koide3/small_gicp) is optional.
