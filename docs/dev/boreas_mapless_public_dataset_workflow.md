# Boreas Starter Workflow

## Goal

Use Boreas as the first public `LiDAR + IMU + GT` dataset for the mapless-public workflow.

The intended split remains:

1. convert one raw Boreas sequence into a rosbag2 `mapping run`
2. convert a different raw Boreas sequence into a rosbag2 `localization run`
3. build a map from the mapping run
4. benchmark localization on the localization run

## Why Boreas

Boreas is a strong first candidate because it provides:

- 128-beam lidar
- Applanix IMU
- post-processed GT poses
- repeated urban driving routes

This repository does not consume the raw Boreas folder layout directly. Convert the desired sequence first.

## Convert a Raw Sequence

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 convert_boreas_sequence_to_rosbag2.py \
  --sequence-dir /absolute/path/to/boreas-2020-11-26-13-58 \
  --bag-dir /tmp/boreas_mapping_rosbag2 \
  --force
```

The converter writes:

- `/velodyne_points` from `lidar/*.bin`
- `/imu/data` from `applanix/imu.csv`
- `/ground_truth/pose_with_covariance` from `applanix/lidar_poses.csv`
- `/vehicle/twist` from `applanix/lidar_poses.csv`

Optional outputs:

```bash
ros2 run lidar_localization_ros2 convert_boreas_sequence_to_rosbag2.py \
  --sequence-dir /absolute/path/to/boreas-2020-11-26-13-58 \
  --bag-dir /tmp/boreas_localization_rosbag2 \
  --output-reference-csv /tmp/boreas_reference.csv \
  --output-initial-pose-yaml /tmp/boreas_initial_pose.yaml \
  --force
```

## Scaffold the Bundle

Start from:

- `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml`

Fill in the converted bag paths and scaffold:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 scaffold_mapless_public_dataset_bundle.py \
  --spec /absolute/path/to/boreas_mapless_public_dataset_bundle.yaml
```

The generated bundle can then be run in this order:

1. `run_mapping.sh`
2. `run_extract_reference.sh`
3. `run_localization_benchmark.sh`
4. optional `run_generate_nav2_map.sh`

## Notes

- Prefer `applanix/lidar_poses.csv` over `gps_post_process.csv` for the GT topic used by localization evaluation.
- The converter rotates Boreas ENU linear velocity into the lidar body frame before writing `/vehicle/twist`.
- The pointcloud writer preserves `ring` and a relative `time` field so future IMU undistortion experiments are not blocked on a format change.

## Current Status

As of 2026-05-24, the checked Boreas localizer-only path is diagnostic, not a benchmark claim.
Smoke runs on `boreas-2021-09-02-11-42` with the local GT-aligned map completed, but did not produce
a usable tracking result:

- `use_twist_prediction: true`, IMU off, `30 s`: `translation_rmse_m=40.495`, `rotation_rmse_deg=8.641`, `ok_rows=38/148`
- twist prediction off, IMU off, `30 s`: `translation_rmse_m=43.167`, `rotation_rmse_deg=6.811`, `ok_rows=85/85`, but the pose stayed near the initial pose
- IMU preintegration on, twist prediction off, `30 s`: `translation_rmse_m=77.036`, `rotation_rmse_deg=10.340`, `ok_rows=3/77`

The map bounds and sampled nearest-map distances overlap the localization reference path, so this is
not simply a wrong-route or missing-map failure. Treat Boreas as blocked until the prediction,
frame/extrinsic, and map-split assumptions are fixed and revalidated.
