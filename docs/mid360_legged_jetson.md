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
- IMU preintegration enabled with correction guards, `imu_accel_scale: 9.80665`
  because `livox_ros_driver2` publishes the acceleration in g (set `1.0` for a
  driver or bag in m/s^2: at the wrong value the preintegration is wrong by a
  factor 9.8, see [benchmark.md](benchmark.md#imu-acceleration-units))

For a smaller Jetson or thermal-limited run, see
[Running on small computers](troubleshooting.md#running-on-small-computers) for the
knobs and where each holds, then first reduce CPU cost:

```bash
ndt_num_threads:=2
```

Then increase `voxel_leaf_size` or reduce `scan_max_range` in a copied YAML if
CPU is still saturated. For Jetson Orin-class devices with cooling, `4` to `6`
threads is usually the first range to test before widening scan
range. With odometry, `param/mid360_legged_low_resource.yaml` does this and more
for about a quarter less CPU ([below](#low-resource-preset)).

## Low-Resource Preset

`param/mid360_legged_low_resource.yaml` is for a Jetson Orin Nano/NX that also runs
perception, on a Unitree Go2 or G1-class robot: `mid360_legged.yaml` with

| parameter | `mid360_legged.yaml` | low resource | why |
| --- | --- | --- | --- |
| `voxel_leaf_size` | `0.25` | `0.5` | about 550 instead of 1550 scan points on the Go2 bag: the largest single saving |
| `ndt_num_threads` | `4` | `2` | no more threads than the cores it gets |
| `ndt_max_iterations` | `30` | `15` | caps the slow scans |
| `use_odom_tf_prediction`, `publish_bridge_pose_when_lost` | off | on | seeds every scan from odometry; required, see below |
| `enable_timer_publishing` | `20 Hz` | off | `map -> odom` per aligned scan; the odometry carries the pose in between |
| `imu_accel_scale` | `1.0` | `9.80665` | `livox_ros_driver2` publishes the MID-360 acceleration in g |

**It needs odometry**: `odom -> base_link` (or the LiDAR frame) from the robot's state
estimator or a LiDAR-inertial odometry such as RKO-LIO
(`ros2 launch lidarslam rko_lio_odometry.launch.py`, as in the
[Go2 quickstart](go2_mid360_quickstart.md)). Without odometry the 0.5 m scan locked
onto the wrong stair level of EIL_Stairs (1.5 m up, 12% of the scans rejected), and
even smaller cuts (two threads, 15 iterations, a 20-scan IMU window at the 0.25 m
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

### Measured on replays

Replayed at 1x on 2026-11-18 (after the memory and CPU changes of that day), the
localizer pinned to two cores of a 4-vCPU x86 host
(Xeon, 2.3 GHz), RKO-LIO beside it on the other two. CPU is in cores (CPU seconds per
second of data); memory is resident (with about 53 MB of shared-library pages) and
anonymous.

**Unitree Go2, JEPLO EIL_Stairs**: upside-down MID-360, motion-capture ground truth,
137 s with stairs, turns up to 2 rad/s, walking up to 1.0 m/s and 9% standing still;
a 580k-point map.

| | `mid360_legged.yaml` | low resource |
| --- | --- | --- |
| RMSE / largest error | 0.066 / 0.25 m | 0.067 / 0.26 m |
| scans aligned | 1364 of 1370 | 1370 of 1370 |
| CPU, mean / p95 | 0.21 / 0.28 cores | 0.145 / 0.18 cores |
| CPU per scan | 21 ms | 14.5 ms |
| memory: resident / anonymous | 100 / 46 MB | 98 / 44 MB |
| pose latency, median / p95 | 18 / 25 ms | 15.5 / 20 ms |
| RKO-LIO alongside | 0.07 cores, 40 MB (13 MB anonymous) | the same |

**Handheld MID-360 outdoors, Koide outdoor_hard_02b**: 298 s, partly off the map; a
2.0M-point map.

| | `mid360_legged.yaml` | low resource |
| --- | --- | --- |
| RMSE (on the map) | 0.238 m (0.193 m) | 0.227-0.235 m (0.201-0.203 m) |
| CPU, mean / p95 | 0.59 / 0.96 cores | 0.40-0.41 / 0.70-0.71 cores |
| memory: resident / anonymous | 280 / 225 MB | 272-275 / 218-221 MB |
| pose latency, median / p95 | 37 / 137 ms | 26-27 / 90-93 ms |
| RKO-LIO alongside | 0.31 cores, 170 MB | the same |

Under tighter budgets on EIL_Stairs (a CFS CPU quota on the two cores, or a memory
cgroup):

| budget | `mid360_legged.yaml` | low resource |
| --- | --- | --- |
| 0.25 cores | 87-91% of scans aligned, latency p95 252-261 ms, at most 562-765 ms | every scan, latency p95 97 ms |
| 0.15 cores | - | 77% of scans, latency median 112 ms; RMSE still 0.069 m |
| 256 MB | passes | passes |
| 224 MB, Koide's 2.0M-point map | - | passes (0.235 m); killed at 256 MB before |

When the CPU runs short the node drops scans (`cloud_queue_depth: 1`) instead of
building up lag, and the odometry carries the pose, so the error did not grow.

**Memory is set by the map, not by the preset.** The map points take 32 bytes each
and the NDT voxel grid about 30 bytes per map point at `ndt_resolution: 1.0`; the
fitness score's kd-tree, once about 50 bytes per point (FLANN), takes 7. Resident
memory comes to about 100 MB with the 580k-point map and 275 MB with the 2.0M-point
one, which runs in a 224 MB memory cgroup (it was killed at 256 MB before the
change; 192 MB is too little). Voxelizing the map saves memory but costs accuracy: on
EIL_Stairs a 0.1 m map took 35 MB less and raised the error by 23%.

**Five Hertz** (`min_scan_interval_sec: 0.19`) costs about 35% less (0.14 against
0.21 cores on EIL_Stairs), but came up to 0.52 m off there (1.3 m with a 20-scan IMU
window). Use it only when 10 Hz does not fit.

**A shorter IMU smoother window** (`imu_smoother_window_size`, default 50) is no
longer worth it: the window is now solved by blocks, about four times faster, and 20
scans save about 0.01 cores (0.134 against 0.145 on EIL_Stairs, the same on Koide).
It used to "lose the pose" on the Koide bag, but the cause was an IMU retry seed after
a rejection streak, accepted 10-35 m off, which scans seeded from odometry no longer
get; since then 20 scans held there too. Keep 50.

**Not verified**: a Jetson, a live MID-360 and ROS 2 Humble. The Orin's Cortex-A78AE
cores are slower than this host's for this code (Eigen and PCL on NEON instead of
AVX-512), so read these CPU figures as a lower bound: plan on two to three times as
much of an Orin core until it is measured there. Walking was at most 1.0 m/s (the
target is 1.5 m/s). EIL_Box, the other short indoor bag, is not a usable check yet:
with odometry both presets came 1-3 m off with a near-perfect fitness, and without
it `mid360_legged.yaml` accepted 5% of the scans, so the fixture (the EIL_Mix map
against this run, or its start pose) is suspect; it was not investigated further.
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
