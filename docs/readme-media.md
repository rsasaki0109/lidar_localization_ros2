# README media

## Handheld outdoor localization (`images/readme/localization_koide_outdoor_hard_02b.gif`)

**Data:** [Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133) (Koide et al., CC BY 4.0).
- Sequence `outdoor_hard_02b` on the provided `map_outdoor_hard.ply`.
- Handheld Livox MID-360. About 389 m of ground-truth path in 274 s.
- Parts of the run leave the map, and in some stretches the scans fit the map poorly. For example, around 39–47 s the NDT fitness stays above the default `score_threshold` of 6 even at the correct pose.

### Why odometry fusion

On this sequence, map-only NDT localization (`nav2_ndt_urban.yaml`, IMU preintegration seed) **loses track**:
- Replays fail at about 27 s or 39 s, and the outcome varies between runs. Replaying at 0.2× does not help, so CPU speed is not the cause.
- Two things combine:
  - Some LiDAR messages contain two to four merged frames (0.2–0.4 s of points). For those scans deskew is skipped (`scan_time_range_too_large`).
  - Once rejections last longer than the 1 s IMU window, deskew and the IMU seed are disabled for every scan (`deskew_imu_integration_window_too_large`). The undistorted scans keep failing, and the prediction drifts until a wrong correction is accepted.

The robust configuration lets an external LiDAR-inertial odometry carry the pose through these stretches. Localization only corrects `map -> odom`:

```bash
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=<params> base_frame_id:=livox_frame \
  enable_map_odom_tf:=true use_odom_tf_prediction:=true publish_bridge_pose_when_lost:=true \
  use_imu_preintegration:=false cloud_topic:=/livox/points imu_topic:=/livox/imu
```

The odometry source must publish `odom -> livox_frame`, for example RKO-LIO from [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2).

### How the GIF run was produced

The goal was a deterministic evaluation on a shared, loaded workstation, so no real-time performance is claimed.

1. **Odometry:** RKO-LIO was run offline over every scan, with the outdoor settings from `lidar_slam_ros2/tools/readme_media/rko_lio_koide_outdoor.yaml`. The dataset publishes IMU acceleration in g, so it was first rescaled with `lidar_slam_ros2/tools/readme_media/scale_imu_bag.py`. Odometry APE (Umeyama SE(3)) is 0.63 m RMSE.
2. **TF bag:** `tools/readme_media/add_odom_tf.py` writes that trajectory into a copy of the bag as `odom -> livox_frame` TF. The TF is delivered 0.3 s ahead of its stamp, so it is available when each scan arrives (odometry without latency).
3. **Localization:**
   - Built from #142 (`4bdb9ca`, merged as `2947d85`). The later #143 and #144 do not change localization behaviour for explicit launch arguments.
   - `param/nav2_ndt_urban.yaml` with `enable_scan_voxel_filter: true`, `voxel_leaf_size: 0.5`, `base_frame_id: livox_frame` and `map_path` changed.
   - The launch arguments shown above. The bag was played at 0.5×.
   - The initial pose is the dataset ground truth at 4 s (the sensor is still), published on `/initialpose` after odometry TF is available.
4. **Render:**
   - `lidar_slam_ros2/tools/readme_media/render_lidar_demo.py --mode localization --overview`. Each scan is drawn at the published pose, together with the estimated path and the dashed ground-truth path.
   - The GIF was encoded at 560 px, 10 fps, with a 48-colour palette.

**Result** (map frame, no alignment, against the dataset ground truth):

| Run | Poses | RMSE | Median | p95 | Max | Longest output gap |
| --- | --- | --- | --- | --- | --- | --- |
| GIF run | 1346 over 274 s | 0.40 m | 0.24 m | 0.64 m | 2.89 m | 1.2 s |
| Repeat | 1192 over 272 s | 0.44 m | 0.25 m | 0.74 m | 2.90 m | 1.5 s |

Live real-time replays on the same loaded workstation were not reliable. The online RKO-LIO node fell behind, dropped scans, and re-anchored on the resulting gaps. A real-time claim needs a quiet machine and is outside this note.
