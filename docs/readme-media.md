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

### Reproduce the run

From the [dataset record](https://zenodo.org/records/10122133), take the `outdoor_hard_02b`
sequence, `map_outdoor_hard.ply`, and `gt.zip` (`traj_lidar_outdoor_hard_02.txt` is the
02a/02b ground truth in TUM format). The odometry step needs a built
[lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2) workspace.

```bash
# 1. Odometry, from the lidar_slam_ros2 checkout (IMU acceleration is published in g)
python3 tools/readme_media/scale_imu_bag.py <dataset>/sequences/outdoor_hard_02b outdoor_hard_02b_scaled
bash scripts/run_rko_lio_graph_benchmark.sh --gt-blind --skip-map-save \
  --bag outdoor_hard_02b_scaled --lidar-topic /livox/points --imu-topic /livox/imu \
  --base-frame livox_frame --publish-static-tf false \
  --rko-param lidarslam/param/rko_lio_mid360_handheld_outdoor.yaml \
  --lidarslam-param lidarslam/param/lidarslam.yaml \
  --output-dir rko_02b --run-name rko_02b
# odometry: rko_02b/rko_02b_0/rko_02b_tum_0.txt

# 2. TF bag and parameters, from this repository
python3 tools/readme_media/add_odom_tf.py outdoor_hard_02b_scaled \
  rko_02b/rko_02b_0/rko_02b_tum_0.txt outdoor_hard_02b_odomtf
sed "s|MAP_OUTDOOR_HARD_PLY|$PWD/map_outdoor_hard.ply|" \
  tools/readme_media/koide_outdoor_hard_02b.yaml > koide_02b.yaml

# 3. Localization, with the launch arguments shown above
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=$PWD/koide_02b.yaml use_sim_time:=true \
  base_frame_id:=livox_frame lidar_frame_id:=livox_frame publish_lidar_tf:=false \
  enable_map_odom_tf:=true use_odom_tf_prediction:=true publish_bridge_pose_when_lost:=true \
  use_imu_preintegration:=false cloud_topic:=/livox/points imu_topic:=/livox/imu

# 4. In other terminals: record, play at 0.5x, and send the ground-truth pose at 4 s
ros2 bag record -o rec /pcl_pose /alignment_status
ros2 bag play outdoor_hard_02b_odomtf --clock --rate 0.5 --topics /livox/points /livox/imu /tf
# about 8 s into the playback, run the command this prints:
python3 tools/readme_media/initial_pose_from_tum.py traj_lidar_outdoor_hard_02.txt outdoor_hard_02b_odomtf

# 5. Evaluate
python3 tools/readme_media/evaluate_pose_against_tum.py rec traj_lidar_outdoor_hard_02.txt
```

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

### Reduced-radius online replay profile

See [Online Koide replay profile](koide-online-replay.md) for the measured 75 m
local-map radius, RELIABLE point QoS, queue-depth 10 configuration and 1x replay
commands. The comparison includes two runs per radius with live RKO odometry.

## Global localization on a Go2

`images/readme/global_localization_jeplo_long_stairs.gif`: the quickstart starts with no
initial pose on a map built from another recording, searches the whole map, and tracks
from its first pose on.

**Data:** [JEPLO dataset](https://huggingface.co/datasets/ASIG-Lab/JEPLO-Dataset) (Yuan et al.).
- The dataset card states the GPL-3.0 licence. The GIF is rendered from the
  dataset, so it is a derived work and is shared under GPL-3.0 as well. The data,
  the scripts and the commands below are its source.
- **Map: run `Long_Stairs`** (2026-08-25 16:33 in the bag clock, 249 s): a Unitree Go2
  with a Livox MID-360 mounted upside down walks out of a 10 m x 4 m motion-capture
  room, down a long staircase into a hall (about 5 m lower), and back.
  `bags/Long_Stairs/Long_Stairs_0.db3` (sha256 `54e88bb271ce40ab71a3c316ab91eacc7cbd877751a88f8103e895b21db51f20`,
  the dataset's LFS id).
- **Scans: run `Outdoor2`** (18:06 the same day, 639 s), its first 165 s: the same
  route out of the room and along the corridor (0-80 s), a wait at the top of the
  staircase (82-130 s), down the staircase (135-145 s) into the hall. After that it
  leaves the area of the map (hall, then outdoors), so the bag is cut there. Only the
  first 1.5 GB of `bags/Outdoor2/Outdoor2_0.db3` (LFS id
  `359f01c3f53af970cad69bb15ade0e44e2b4af1e47a0415209097c6907cbb45c`, 3.0 GB) were
  downloaded and read page by page (300 s of data), so the file's sha256 was not
  checked; `gt/Outdoor2.txt` matches the dataset's git blob id. The same reader gave
  clouds identical to `jeplo_to_pc2_bag.py` (headers, points, order) on the whole
  EIL_Mask2 bag.
- Ground truth (motion capture, LiDAR frame) covers only the room: 0-61 s of the
  Outdoor2 cut. Both runs share the motion-capture frame: Outdoor2's odometry, fitted to
  its own ground truth, runs within 0.01-0.17 m of the Long_Stairs path in the room
  and along the corridor.
- **What differs between the two runs:** the trajectory, the timing (Outdoor2 stops
  for 48 s at the top of the stairs), and whoever was around. The room itself was not
  rearranged: 99-100% of each map's room points (floor to 2.5 m) lie within 0.1-0.2 m of
  the other run's map. So this pair tests a different recording, not a changed room.
  Pairs with moved props, an occluded sensor and the symmetric room are in
  [benchmark.md](benchmark.md#cross-session-go2-jeplo), failures included.

### How the GIF run was produced

1. **Bags:** `tools/benchmark/jeplo_to_pc2_bag.py` (10 Hz `PointCloud2`, two sweeps per
   cloud); Outdoor2 with `--duration 165`.
2. **Map from Long_Stairs only:** RKO-LIO `offline_node`
   ([rsasaki0109/rko_lio](https://github.com/rsasaki0109/rko_lio) at `7e91814`) with
   `tools/benchmark/suites/rko_lio_jeplo.yaml`, then
   `tools/benchmark/build_map_from_odometry.py` with its defaults (every second
   cloud, 0.8-40 m, 0.05 m voxels, 1.19 million points), moved into the ground-truth
   frame with `tools/benchmark/align_map_to_ground_truth.py` (trajectory residual 0.040 m).
   This is the map of the earlier same-recording GIF; nothing of Outdoor2 went into it.
3. **Localization:** built from `main` at `6303352` (the localizer is unchanged here).
   - `tools/benchmark/run_benchmark.py` with the `Long_Stairs_map__Outdoor2` case of
     `tools/benchmark/suites/go2_jeplo_cross_session.yaml`, two repeats. It starts the
     online RKO-LIO odometry (lidar_slam_ros2 `rko_lio_odometry.launch.py` at `d13d9e7`),
     plays the bag at 1×, starts the quickstart 3 s later with no pose, and records
     `/pcl_pose`, the startup states and the quickstart log.
   - Quickstart options: the [Large maps](go2_mid360_quickstart.md#large-maps) settings,
     `--occupancy-inflate-radius-m 0.2 --global-z-min 0.0 --global-max-scan-points 1024
     --global-max-candidates 16 --refine-global-candidates`. The scan band was sparse, so
     both queries ran the sparse-band search and returned 32 candidates.
   - One 4-core machine, ROS 2 Jazzy.
4. **Render:** `tools/readme_media/render_global_localization.py`, 720 × 405 px, 12 fps,
   128-colour palette (ffmpeg `palettegen` / `paletteuse` with Bayer dithering),
   201 frames, 16.8 s, 0.9 MB.

**What the GIF shows, and what is drawn for display only:**
- Every candidate of the two queries up to the accepted answer, at its position and
  yaw from the `query answered:` log line of `global_localization_node`. The circle size
  follows the score; #1 is gold. The white × is the ground truth at the query's scan.
  The scores and NDT fitness values printed are the logged ones.
- The live scan in the inset is the query's scan in the sensor frame, levelled with the
  odometry roll and pitch that the query used.
- The move of the scan from the inset onto the map is a screen-space interpolation
  over 14 frames. Its start (the scan in the sensor frame) and end (the first published
  pose) are real.
- During tracking, the scan nearest to each frame time is drawn at `/pcl_pose`,
  interpolated linearly between two published poses. The cyan path is every published
  pose; the white dashed path is the ground truth after the first pose (the room only).
- The view is fitted to the candidates and the path with a 3 m margin; the map and the
  search extend beyond it. Map points above 2.5 m (the ceiling) are not drawn, and 80%
  of the map points are.
- The clock is time since the start of the bag. The queries took 2.6 s of wall time
  each; the GIF does not show that wait.

**Result** (map frame, against the motion-capture ground truth in the room;
`run_benchmark.py` scores; the last column compares with Outdoor2's own offline
RKO-LIO odometry, fitted to its ground truth, which is a reference and not ground truth):

| Run | First pose | First-pose error | Candidates | Top candidate vs truth | Wrong pose | Median / p95 / max xy (room) | Corridor and stairs vs odometry: median / max |
| --- | --- | --- | --- | --- | --- | --- | --- |
| GIF run | 13.3 s (search started at 6.9 s) | 0.02 m (0.01 m for the first pose itself) | 2 queries × 32 | 0.01 m, 0.02 m; ranked #1 in both | none | 0.02 / 0.04 / 0.07 m | 0.02 / 0.19 m |
| Repeat | 13.3 s | 0.02 m | 2 queries × 32 | 0.02 m, 0.02 m; ranked #1 in both | none | 0.02 / 0.04 / 0.07 m | 0.02 / 0.19 m |

In both queries of the GIF run, NDT told the true place apart clearly: fitness 0.120
and 0.033 against 0.753 and 0.658 for the second-best candidate. Neither run asked
for a manual pose. The reverse pair (a map from the Outdoor2 cut, Long_Stairs scans)
also localized in both runs, after 12.3-12.6 s, 0.02 m off, and tracked down the stairs
and back with 0.02 m median and 0.10-0.14 m maximum error in the room.

### Reproduce the GIF

From the [dataset](https://huggingface.co/datasets/ASIG-Lab/JEPLO-Dataset), take
`bags/Long_Stairs`, `bags/Outdoor2` (3.0 GB; only its first 165 s are used),
`gt/Long_Stairs.txt` and `gt/Outdoor2.txt`. `JEPLO_ROOT` holds `gt/`,
`bags_pc2_merged/` and `maps_loc/`, as the Go2 suites expect. The odometry needs
[rko_lio](https://github.com/rsasaki0109/rko_lio), and the replay a
[lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2) checkout for its
odometry launch file.

```bash
# 1. Bags, odometry and the map (from Long_Stairs only)
python3 tools/benchmark/jeplo_to_pc2_bag.py <raw>/bags/Long_Stairs $JEPLO_ROOT/bags_pc2_merged/Long_Stairs
python3 tools/benchmark/jeplo_to_pc2_bag.py <raw>/bags/Outdoor2 $JEPLO_ROOT/bags_pc2_merged/Outdoor2_0-165s --duration 165
ros2 run rko_lio offline_node --ros-args --params-file long_stairs_rko.yaml
# long_stairs_rko.yaml: rko_lio_jeplo.yaml plus lidar_topic, imu_topic, base_frame: livox_frame,
# bag_path, dump_results: true, results_dir, run_name: Long_Stairs
python3 tools/benchmark/build_map_from_odometry.py --bag $JEPLO_ROOT/bags_pc2_merged/Long_Stairs \
  --trajectory <results_dir>/Long_Stairs_0/Long_Stairs_tum_0.txt --out map_odom.pcd
python3 tools/benchmark/align_map_to_ground_truth.py --map map_odom.pcd \
  --trajectory <results_dir>/Long_Stairs_0/Long_Stairs_tum_0.txt \
  --ground-truth $JEPLO_ROOT/gt/Long_Stairs.txt --out $JEPLO_ROOT/maps_loc/Long_Stairs/map.pcd

# 2. Replay the Outdoor2 scans with no initial pose (1x)
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/go2_jeplo_cross_session.yaml \
  --case Long_Stairs_map__Outdoor2 --repeats 1 --out /tmp/go2_gif

# 3. Render (needs Pillow, numpy and ffmpeg; source ROS 2 for rosbag2_py)
python3 tools/readme_media/render_global_localization.py \
  --map $JEPLO_ROOT/maps_loc/Long_Stairs/map.pcd \
  --bag $JEPLO_ROOT/bags_pc2_merged/Outdoor2_0-165s \
  --run /tmp/go2_gif/Long_Stairs_map__Outdoor2/run1 --gt $JEPLO_ROOT/gt/Outdoor2.txt \
  --caption "JEPLO (GPL-3.0) · Unitree Go2, upside-down MID-360 · map: run Long_Stairs · scans: run Outdoor2" \
  --out images/readme/global_localization_jeplo_long_stairs.gif
```

If the lidar_slam_ros2 package is not installed, set the suite's `odometry_launch` to
the path of its `lidarslam/launch/rko_lio_odometry.launch.py`, as was done here. A
different run gives different candidates (the search follows the scans the localizer
happens to query), so the GIF differs in detail.

**Not verified:** the GIF run and one repeat only; one machine (4 cores, Jazzy). The
dataset has a single Long_Stairs recording; Outdoor2 is the only other run on that
route. The room was not rearranged between the two runs, so the GIF does not show a
changed environment, and outside the room there is no ground truth. The Outdoor2 bag
was converted from its first 1.5 GB, not from the checked whole file.
