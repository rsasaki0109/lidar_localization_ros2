# Jetson MID-360 Legged Robot Bringup

This path targets Jetson-class compute, a Livox MID-360 pointcloud, and
quadruped or biped robots. It is a localization stack boundary: it does not
own walking control, motor state estimation, global planning, or the MID-360
driver process.

Current validation status:

- package build, policy/unit tests, and public replay regression have passed
  after the MID-360 bringup merge
- latest public replay snapshot:
  [public_validation_log.md](public_validation_log.md#2026-05-22-main-after-mid-360-bringup-merge)
- Jetson + MID-360 hardware validation has not been run in this workspace

## Runtime Contract

Required inputs:

- `PointCloud2` from the MID-360, default launch remap: `/livox/points`
- optional MID-360 IMU, default launch remap: `/livox/imu`
- optional body twist as `TwistWithCovarianceStamped`, default launch remap: `/twist`
- an odometry source publishing `odom -> base_link` when `enable_map_odom_tf:=true`
- an initial pose on `/initialpose`, unless `set_initial_pose:=true`
- a prebuilt `.pcd` or `.ply` pointcloud map

Published outputs:

- `/localization/pose_with_covariance`
- `/path`
- `/alignment_status`
- `/reinitialization_requested`
- `map -> odom` TF by default
- `/initial_map`

## Launch

Use the MID-360 legged preset:

```bash
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd \
  cloud_topic:=/livox/points \
  imu_topic:=/livox/imu \
  lidar_frame_id:=livox_frame \
  lidar_tf_x:=0.0 lidar_tf_y:=0.0 lidar_tf_z:=0.0 \
  lidar_tf_roll:=0.0 lidar_tf_pitch:=0.0 lidar_tf_yaw:=0.0
```

If your Livox driver publishes a different `PointCloud2` topic, override
`cloud_topic`. If the driver publishes a Livox custom message instead of
`sensor_msgs/msg/PointCloud2`, configure the driver or add a bridge so this
localizer receives `PointCloud2`.

For the first field run, either publish `/initialpose` from RViz/Nav2 or pass a
known starting pose:

```bash
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd \
  set_initial_pose:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_z:=0.0 \
  initial_pose_qx:=0.0 initial_pose_qy:=0.0 initial_pose_qz:=0.0 initial_pose_qw:=1.0
```

## Frames

Recommended TF tree:

```text
map -> odom -> base_link -> livox_frame
```

The robot state estimator should publish `odom -> base_link`. This package
publishes `map -> odom`. The launch file can publish a static
`base_link -> livox_frame`, but the default zero transform is only a placeholder;
replace it with measured MID-360 extrinsics.

The preset enables `imu_preintegration_use_base_frame_transform`. That means IMU
gyro and acceleration samples are rotated into `base_frame_id` before
preintegration when the IMU message frame differs from `base_link`. If the IMU
message uses a separate frame, publish it:

```bash
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
  map_path:=/absolute/path/to/map.pcd \
  publish_imu_tf:=true \
  imu_frame_id:=livox_imu_frame \
  imu_tf_x:=0.0 imu_tf_y:=0.0 imu_tf_z:=0.0 \
  imu_tf_roll:=0.0 imu_tf_pitch:=0.0 imu_tf_yaw:=0.0
```

If you are not confident in the IMU frame or extrinsics yet, start with:

```bash
use_imu_preintegration:=false
```

## Jetson Defaults

`param/mid360_legged.yaml` starts conservative:

- `NDT_OMP` with `ndt_num_threads: 4`
- scan voxel leaf size `0.25 m`
- range clamp `0.8 m` to `60 m`
- local map crop radius `80 m`
- timer publishing at `20 Hz` for stable `map -> odom`
- twist prediction enabled, but angular twist prediction disabled
- IMU preintegration enabled with correction guards

For a smaller Jetson or thermal-limited run, first reduce CPU cost:

```bash
ndt_num_threads:=2
```

Then increase `voxel_leaf_size` or reduce `scan_max_range` in a copied YAML if
CPU is still saturated. For Jetson Orin-class devices with cooling, `4` to `6`
threads is usually the first range to test before widening map crop or scan
range.

## Bringup Checks

Before starting localization:

```bash
ros2 topic hz /livox/points
ros2 topic echo /livox/imu --once
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo odom base_link
```

After launch:

```bash
ros2 topic echo /alignment_status --once
ros2 topic echo /localization/pose_with_covariance --once
ros2 run tf2_ros tf2_echo map odom
```

Or run the bringup doctor:

```bash
ros2 run lidar_localization_ros2 check_mid360_legged_bringup.py \
  --duration-sec 5 \
  --cloud-topic /livox/points \
  --imu-topic /livox/imu \
  --base-frame base_link \
  --lidar-frame livox_frame
```

For a stricter check after localization is active and the robot estimator is
publishing odometry:

```bash
ros2 run lidar_localization_ros2 check_mid360_legged_bringup.py \
  --require-imu \
  --require-map-odom-tf \
  --require-localization-output
```

Watch `/alignment_status` first. If fitness is rejected repeatedly, check the
initial pose, map frame, pointcloud frame, and whether the MID-360 scan is being
converted to `PointCloud2` with `x/y/z` fields.

## Scope Boundary

In scope:

- MID-360 pointcloud localization against an existing 3D map
- `map -> odom` publication for a legged robot stack
- IMU/twist-aided registration seeds
- Jetson-oriented launch and parameters
- diagnostics for rejected alignments and reinitialization requests

Out of scope:

- walking control
- motor or contact state estimation
- Nav2 global/local planning
- live SLAM map building as part of this launch
- cloud services or UI
