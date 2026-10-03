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

1. **Odometry:** RKO-LIO was run offline over every scan with the handheld MID-360 profile, [`lidar_slam_ros2/lidarslam/param/rko_lio_mid360_handheld_outdoor.yaml`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/param/rko_lio_mid360_handheld_outdoor.yaml), with RKO-LIO at `e5f731f`. The dataset publishes IMU acceleration in g, so it was first rescaled with `lidar_slam_ros2/tools/readme_media/scale_imu_bag.py`. Odometry APE (Umeyama SE(3)) is 0.34 m RMSE.
2. **TF bag:** `tools/readme_media/add_odom_tf.py` writes that trajectory into a copy of the bag as `odom -> livox_frame` TF. The TF is delivered 0.3 s ahead of its stamp, so it is available when each scan arrives (odometry without latency).
3. **Localization:**
   - Built from `main` at `e4d2b69`.
   - `param/nav2_ndt_urban.yaml` (including `local_map_update_distance: 10.0`) with `enable_scan_voxel_filter: true`, `voxel_leaf_size: 0.5`, `base_frame_id: livox_frame` and `map_path` changed.
   - The launch arguments shown above. The bag was played at 0.5×.
   - The initial pose is the dataset ground truth at 4 s (the sensor is still), published on `/initialpose` after odometry TF is available.
4. **Render:**
   - `lidar_slam_ros2/tools/readme_media/render_lidar_demo.py --mode localization --overview`. Each scan is drawn at the published pose, together with the estimated path and the dashed ground-truth path.
   - The GIF was encoded at 560 px, 10 fps, with a 48-colour palette.

**Result** (map frame, no alignment, against the dataset ground truth):

| Run | Poses | RMSE | Median | p95 | Max | Longest output gap |
| --- | --- | --- | --- | --- | --- | --- |
| GIF run | 1946 over 273 s | 0.26 m | 0.088 m | 0.67 m | 1.44 m | 1.2 s |
| Repeat | 1946 over 272 s | 0.26 m | 0.088 m | 0.66 m | 1.44 m | 1.2 s |

The earlier GIF, with odometry from the previous outdoor RKO-LIO settings (0.63 m APE) and the build at #142, gave 0.40 m RMSE (median 0.24 m, max 2.89 m, 1346 poses).

### Live real-time replays

**Setup:** online RKO-LIO node and localization together, bag at 1.0×, initial pose as above.

**Heavily loaded workstation** (load average 10–20 from unrelated jobs): live runs were not reliable. The online RKO-LIO node fell behind, dropped scans, and re-anchored on the resulting gaps.

**Less loaded workstation** (load average 2.5–6.6). The online RKO-LIO node includes the scan-gap fix (rko_lio#16). Localization was built from `main` at `768b557`; that build also contained an unmerged guard, which was disabled and has no effect.

| `local_map_update_distance` | Runs | Poses per run | RMSE | Median | p95 | Max |
| --- | --- | --- | --- | --- | --- | --- |
| `10` (`nav2_ndt_urban.yaml`) | 4 | 1793–1797 over ~277 s | 0.33–0.36 m | 0.087–0.091 m | 0.80–0.92 m | 1.39–2.14 m |
| `0` | 3 | 308–321 over ~268 s | 0.31–0.98 m | 0.10–0.11 m | 0.70–2.20 m | 1.36–2.35 m |

**Reading:**
- Inside the map, real-time tracking held in all runs. Reusing the local-map target raises the output from about 1.1 to 6.5 poses/s and makes the result repeatable.
- The long section outside the map (about 160–260 s) is carried by odometry alone. Every run with `10` re-acquired afterwards (error 0.14–0.15 m over the last 15 s). With `0`, one of the three runs ended about 1 m off.
- Before rko_lio#16, the online node lost the motion of the 1.1–1.5 s scan gaps it produces under load. With `10`, 5 of 6 such runs then failed (#153).
- This is not a guarantee for long map-free stretches.
