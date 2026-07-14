# Parameter files

This directory contains runtime presets and reproducible benchmark inputs. Files
stay at their existing paths for launch-file and downstream compatibility.

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
| `koide_indoor_ndt.yaml` | Koide indoor NDT_OMP baseline |
| `koide_indoor_smallgicp.yaml` | Koide indoor optional SMALL_GICP comparison |
| `public_istanbul_60s_benchmark.yaml` | Autoware Istanbul public 60 s validation template |

Dataset presets are not universal defaults. Replace placeholder map paths and
verify frames, topics, initial pose, sensor units, and timing before use.

## Experimental Nav2 derivatives

These files preserve named experiment inputs and are not recommended defaults:

- `nav2_ndt_urban_borderline_only.yaml`
- `nav2_ndt_urban_recovery_retry_from_last_pose.yaml`
- `nav2_ndt_urban_recovery_retry_from_last_pose_r3.yaml`
- `nav2_ndt_urban_recovery_retry_from_last_pose_r3_gap1_seed15.yaml`
- `nav2_ndt_urban_rejected_seed_update.yaml`
- `nav2_ndt_urban_rejected_seed_update_r1.yaml`

New one-off comparisons belong under [`benchmark/`](benchmark/) instead of the
top level. Promote a winner only after repeat validation.

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

## Benchmark inputs

[`benchmark/`](benchmark/) contains immutable comparison inputs, run manifests,
and result-comparison specifications. See its README before adding a file.

