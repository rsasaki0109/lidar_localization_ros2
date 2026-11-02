# Parameter files

This directory contains runtime and dataset presets. Files stay at their existing
paths for launch-file and downstream compatibility.

## Start here

| File | Use |
| --- | --- |
| `localization.yaml` | Generic `lidar_localization.launch.py` starting point |
| `nav2_ndt_urban.yaml` | Recommended conservative Nav2 localizer preset |
| `nav2_humble_pointcloud.yaml` | Companion Nav2 stack configuration |
| `mid360_legged.yaml` | Jetson + Livox MID-360 legged-robot preset |
| `nav2_replay_safe.yaml` | Single-thread NDT replay/debug fallback |

The localizer and Nav2 configurations are separate. A normal Nav2 launch uses
both `nav2_ndt_urban.yaml` and `nav2_humble_pointcloud.yaml`.

## Dataset and validation presets

| File | Scope |
| --- | --- |
| `boreas_ndt_velodyne.yaml` | Boreas Velodyne 128 |
| `hdl_imu_preint.yaml` | Official hdl_localization sample with IMU preintegration |
| `istanbul_gtsam_smoother.yaml` | Autoware Istanbul GTSAM smoother experiment |
| `istanbul_small_gicp.yaml` | Autoware Istanbul optional SMALL_GICP comparison |
| `koide_indoor_ndt.yaml` | Koide indoor NDT_OMP baseline |
| `koide_indoor_smallgicp.yaml` | Koide indoor optional SMALL_GICP comparison |
| `public_istanbul_60s_benchmark.yaml` | Autoware Istanbul public 60 s validation template |

Dataset presets are not universal defaults. Replace placeholder map paths and
verify frames, topics, initial pose, sensor units, and timing before use.

## Experiment presets

One-off experiment inputs are not kept here. Pass the overrides to a replay instead
(`--set KEY=VALUE` in `tools/benchmark/boreas_replay.py` and `istanbul_replay.py`),
and promote a winner into a preset only after repeat validation.

The six `nav2_ndt_urban_*` derivatives and the external odometry configurations in
`odometry/` (FAST-LIO, KISS-ICP, GLIM) were removed unused; `git checkout 7af1287`
has them. The recovery retry that won those Istanbul comparisons (3 rejections,
gap <= 1 s, seed <= 15 m) is in `public_istanbul_60s_benchmark.yaml`.

## Editing conventions

Keep existing filenames and parameter names backward compatible. For localizer
presets, group parameters in this order when practical:

1. registration backend and convergence;
2. scan filtering and timing;
3. map and initial pose;
4. odometry, twist, and IMU prediction;
5. acceptance and recovery guards;
6. frames, TF, publishing, and visualization.

Every top-level preset starts with `Preset` and `Status` comments. Explicit
values should describe an intentional override; inherited node defaults need
not be copied into every file. Avoid YAML anchors and merge keys because ROS 2
parameter parser behavior differs across distributions.
