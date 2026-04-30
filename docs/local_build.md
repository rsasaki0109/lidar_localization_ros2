# Local Build

This workspace has been set up to build without `sudo` by keeping third-party dependencies in a local prefix and building ROS packages in a sibling overlay workspace.

## Workspace Layout

These instructions assume the following layout:

```text
lidarloc_ws/
  build_ws/
  local_prefix/
  repo/
```

- `repo/` is this repository
- `local_prefix/` contains locally installed third-party libraries such as `PCL 1.12` and `small_gicp`
- `build_ws/` is a ROS 2 overlay workspace that builds `pcl_msgs`, `pcl_conversions`, `ndt_omp_ros2`, and `lidar_localization_ros2`

## Load the Environment

From `repo/`, run:

```bash
source scripts/setup_local_env.sh
```

The script:

- sources `/opt/ros/humble/setup.bash`
- adds `../local_prefix` to `CMAKE_PREFIX_PATH`, `LD_LIBRARY_PATH`, and related include and pkg-config paths
- sources `../build_ws/install/setup.bash` when the overlay has already been built

Override paths if needed:

```bash
export LIDAR_LOCALIZATION_WS_ROOT=/path/to/lidarloc_ws
export LIDAR_LOCALIZATION_LOCAL_PREFIX=/path/to/custom_prefix
export LIDAR_LOCALIZATION_OVERLAY=/path/to/custom_overlay/install/setup.bash
source scripts/setup_local_env.sh
```

## Build

After sourcing the environment, build from `../build_ws`:

```bash
colcon build --symlink-install --packages-up-to lidar_localization_ros2
```

## Current Local Dependency Set

The current no-sudo setup in this workspace uses:

- `small_gicp` installed into `../local_prefix`
- `PCL 1.12` built from source and installed into `../local_prefix`
- Ubuntu `.deb` contents extracted into `../local_prefix` for `flann`, `lz4`, and `qhull`

## Notes

- The `ndt_omp_ros2` checkout in `../build_ws/src/ndt_omp_ros2` is patched locally so the library is exported cleanly and the visualization sample app is disabled by default.
- After the build finishes, `source scripts/setup_local_env.sh` is enough to make `ros2 run lidar_localization_ros2 ...` and `ros2 run lidar_localization_ros2 benchmark_runner ...` resolve correctly in a new shell.
