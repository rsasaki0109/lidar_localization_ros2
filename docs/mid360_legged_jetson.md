# Jetson MID-360 Legged Robot Bringup

This path targets Jetson-class compute, a Livox MID-360 pointcloud, and
quadruped or biped robots. It is a localization stack boundary: it does not
own walking control, motor state estimation, global planning, or the MID-360
driver process.

Current validation status:

- package build, policy/unit tests, and public replay regression have passed
  after the MID-360 bringup merge
- latest public replay snapshot:
  [public_validation_log.md](dev/public_validation_log.md#2026-05-22-main-after-mid-360-bringup-merge)
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

Generate a small parameter file first. The tool prints the matching launch
command and bringup doctor command, including the IMU frame checks used by the
MID-360 preset:

```bash
ros2 run lidar_localization_ros2 create_lidar_localization_config.py \
  --profile mid360 \
  --map-path /absolute/path/to/map.pcd \
  --lidar-tf 0 0 0 0 0 0 \
  --imu-tf 0 0 0 0 0 0 \
  --output /tmp/mid360_lidar_localization.yaml
```

For the experimental continuous-time deskew path, add
`--enable-continuous-time-deskew`. The generated doctor command will require
IMU data, `base_link -> livox_imu_frame`, and a per-point cloud timing field.

Or launch the MID-360 legged preset directly:

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
                       \-> livox_imu_frame
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
- the full map as the NDT target (built once at map load); the local map crop
  (`80 m`, re-cropped every `20 m`) is off, because each re-crop of a dense map
  stalls the output for about 2 s
- timer publishing at `20 Hz` for stable `map -> odom`
- twist prediction enabled, but angular twist prediction disabled
- IMU preintegration enabled with correction guards

For a smaller Jetson or thermal-limited run, first reduce CPU cost:

```bash
ndt_num_threads:=2
```

Then increase `voxel_leaf_size` or reduce `scan_max_range` in a copied YAML if
CPU is still saturated. For Jetson Orin-class devices with cooling, `4` to `6`
threads is usually the first range to test before widening scan
range. With odometry, `param/mid360_legged_low_resource.yaml` does this and more
for about half the CPU ([below](#low-resource-preset)).

## Low-Resource Preset

`param/mid360_legged_low_resource.yaml` is for a Jetson Orin Nano/NX that also runs
perception, on a Unitree Go2 or G1-class robot: `mid360_legged.yaml` with

| parameter | `mid360_legged.yaml` | low resource | why |
| --- | --- | --- | --- |
| `voxel_leaf_size` | `0.25` | `0.5` | about 400 instead of 1600 scan points: the largest single saving |
| `imu_smoother_window_size` | `50` (built in) | `20` | the IMU smoother re-optimizes its whole window every scan |
| `ndt_num_threads` | `4` | `2` | no more threads than the cores it gets |
| `ndt_max_iterations` | `30` | `15` | caps the slow scans (p95 of the alignment time) |
| `use_odom_tf_prediction`, `publish_bridge_pose_when_lost` | off | on | seeds every scan from odometry; required, see below |
| `enable_timer_publishing` | `20 Hz` | off | `map -> odom` per aligned scan; the odometry carries the pose in between |
| `imu_accel_scale` | `1.0` | `9.80665` | `livox_ros_driver2` publishes the MID-360 acceleration in g |

**It needs odometry**: `odom -> base_link` (or the LiDAR frame) from the robot's state
estimator or a LiDAR-inertial odometry such as RKO-LIO
(`ros2 launch lidarslam rko_lio_odometry.launch.py`, as in the
[Go2 quickstart](go2_mid360_quickstart.md)). Without odometry the 0.5 m scan locked
onto the wrong stair level of EIL_Stairs (1.5 m up, 12% of the scans rejected), and
even the smaller cuts (two threads, 15 iterations, a 20-scan window at the 0.25 m
scan) lost the robot by 13 m. Without odometry, keep `mid360_legged.yaml`.

```bash
ros2 launch lidarslam rko_lio_odometry.launch.py      # or the robot's odometry
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
  localization_param_dir:=$(ros2 pkg prefix lidar_localization_ros2)/share/lidar_localization_ros2/param/mid360_legged_low_resource.yaml \
  map_path:=/absolute/path/to/map.pcd \
  cloud_topic:=/livox/lidar imu_topic:=/livox/imu
```

For RKO-LIO on a MID-360, set its `imu_acceleration_unit` to `g` (or `auto`): read
as m/s^2, gravity has magnitude 1 and the odometry starts with an 8.8 m/s^2
accelerometer bias.

### Measured on a Go2 replay

JEPLO EIL_Stairs (Unitree Go2, upside-down MID-360, motion-capture ground truth;
137 s with stairs, turns up to 2 rad/s, walking up to 1.0 m/s and 9% standing
still), replayed at 1x on 2026-11-18. The localizer was pinned to two cores of a
4-vCPU x86 host (Xeon, 2.3 GHz) and RKO-LIO ran next to it; the map has 580k points.
CPU is in cores (CPU seconds per second of data):

| | `mid360_legged.yaml` | low resource |
| --- | --- | --- |
| RMSE / largest error | 0.077 / 0.82 m | 0.067 / 0.26 m |
| scans aligned | 1359 of 1370 | 1363 of 1370 |
| CPU, mean / p95 | 0.27 / 0.36 cores | 0.14 / 0.19 cores |
| CPU per scan | 27 ms | 14 ms |
| memory: resident / anonymous | 132 / 78 MB | 121 / 67 MB |
| pose latency, median / p95 | 24 / 34 ms | 16 / 21 ms |
| RKO-LIO alongside | 0.07 cores, 40 MB (13 MB anonymous) | the same |

The resident memory includes about 53 MB of shared-library pages; the anonymous
part is what grows with the map. In earlier runs of the same comparison
`mid360_legged.yaml` scored 0.066 m at 0.24 cores.

Under tighter budgets (CFS CPU quota on the two cores, then a memory cgroup):

| budget | `mid360_legged.yaml` | low resource |
| --- | --- | --- |
| 0.25 cores | 87-91% of scans aligned, latency p95 252-261 ms, at most 562-765 ms | every scan, latency p95 21 ms |
| 0.15 cores | - | 98-99% of scans, latency p95 113-129 ms, at most 212-233 ms |
| 0.10 cores | - | 78% of scans, latency median 136 ms; RMSE still 0.068 m |
| 0.10 cores at 5 Hz | - | every second scan as intended, latency p95 116 ms |
| memory cgroup | passes at 256 MB | passes at 256, 128 and 96 MB; 0.067-0.068 m |

When the CPU runs short the node drops scans (`cloud_queue_depth: 1`) instead of
building up lag, and the odometry carries the pose, so the error did not grow: no
10 Hz run with odometry on EIL_Stairs came more than 0.82 m off.

**Five Hertz** (`min_scan_interval_sec: 0.19`) costs about 30% less again (0.10
cores), but the 5 Hz run of the preset slipped 1.3 m for about a second (RMSE
0.127 m), and with a 50-scan window it came 0.52 m off. Use it only when 10 Hz does
not fit.

**Not verified**: a Jetson, a live MID-360 and ROS 2 Humble. The Orin's Cortex-A78AE
cores are slower than this host's for this code (Eigen and PCL on NEON instead of
AVX-512), so read these CPU figures as a lower bound: plan on two to three times as
much of an Orin core until it is measured there. Walking was at most 1.0 m/s (the
target is 1.5 m/s). EIL_Box, the other short indoor bag, is not a usable check yet:
both presets lost 1-2 m there with a near-perfect fitness, and why was not found.
The full measurements are in
[benchmark.md](benchmark.md#low-resource-budget-go2-mid-360).

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
ros2 run lidar_localization_ros2 check_lidar_localization_bringup.py \
  --profile mid360 \
  --duration-sec 5 \
  --cloud-topic /livox/points \
  --imu-topic /livox/imu \
  --base-frame base_link \
  --lidar-frame livox_frame \
  --imu-frame livox_imu_frame \
  --require-imu \
  --require-imu-base-tf
```

If you are preparing continuous-time deskew work by hand, add
`--require-cloud-time-field`. The check fails unless the converted `PointCloud2`
keeps a per-point timing field such as `time`, `timestamp`, or `offset_time`.

For a stricter check after localization is active and the robot estimator is
publishing odometry:

```bash
ros2 run lidar_localization_ros2 check_lidar_localization_bringup.py \
  --profile mid360 \
  --require-imu \
  --require-imu-base-tf \
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
