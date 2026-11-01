# Gyro deskew for the online Koide check

Question: why does NDT need 10-31 iterations per scan 100-132 s into Koide
`outdoor_hard_02b`, where most clouds the online check misses lie
([koide_online_coverage](../koide_online_coverage/README.md#open))?

## Cause: motion distortion while the sensor turns fast

The handheld MID-360 turns at 83 deg/s (median) there, 215 deg/s at p90, against 6.5 deg/s
elsewhere (GT orientation between clouds). A cloud spans 0.1 s, so a cloud taken at
150 deg/s holds points from orientations 15 deg apart. The online profile does not deskew:
`use_continuous_time_deskew` waits for IMU preintegration, which the profile disables
because RKO-LIO supplies the odometry (`continuous_time_deskew_status:
continuous_time_deskew_imu_preintegration_disabled` on every cloud).

The iterations follow the correction from the seed (Spearman +0.88 over one run), not
the point count (+0.13) or the speed (+0.15). The seed is not the problem, though. Aligned
from the GT pose itself, a raw cloud taken at 150 deg/s or more still needed 14
iterations and ended 0.29 m off: the distorted cloud's best fit is not the true pose.

## Offline check

`extract_scans.py` writes each cloud three ways, all in the sensor frame at the header
stamp (the sweep start; `t` is the point time in ns from it, and the GT poses sit at the
header stamps):

- `raw`: as recorded;
- `desk`: each point moved with the GT pose interpolated at its time;
- `gyro`: each point rotated with the `/livox/imu` gyro (200 Hz, `livox_frame`)
  integrated from the sweep start to its time. No translation.

`batch_align.cpp` aligns them with the profile's NDT settings and preprocessing:

- 75 m crop, rebuilt every 10 m
- 0.5 m voxels, 1-60 m range
- resolution 1.0, step 0.1, epsilon 0.01, at most 35 iterations, 4 threads
- `libndt_omp` built as Release

The seed is the GT pose moved 0.15 m and turned 0.8 deg (the median correction during
the turns). The clouds are every cloud from 96 to 134 s, binned by the GT rotation
rate, and every fourth from 40 to 80 s. Errors are against the GT pose at the header
stamp. Median (p90):

| GT rotation rate | Cloud | Clouds | Iterations | Translation error m | Yaw error deg | Alignment ms |
| --- | --- | --- | --- | --- | --- | --- |
| calm (40-80 s) | raw | 66 | 5 (7) | 0.066 (0.112) | 0.32 (1.70) | 54 |
| | gyro | 66 | 5 (6) | 0.068 (0.110) | **0.05** (0.14) | 52 |
| | desk | 66 | 4 (5) | 0.118 (0.170) | 0.10 (0.33) | 53 |
| < 60 deg/s | raw | 131 | 5 (7) | 0.068 (0.135) | 0.40 (1.50) | 49 |
| | gyro | 131 | 4 (5) | **0.058** (0.113) | **0.04** (0.13) | 43 |
| | desk | 131 | 4 (5) | 0.101 (0.190) | 0.18 (0.60) | 39 |
| 60-150 deg/s | raw | 87 | 9 (16) | 0.142 (0.324) | 0.58 (2.06) | 66 |
| | gyro | 87 | **5** (6) | **0.073** (0.166) | **0.08** (0.28) | 39 |
| | desk | 87 | 4 (9) | 0.158 (0.242) | 0.33 (0.88) | 38 |
| >= 150 deg/s | raw | 61 | 14 (27) | 0.296 (0.535) | 0.91 (2.23) | 109 |
| | gyro | 61 | **4** (5) | **0.075** (0.145) | **0.11** (0.28) | 38 |
| | desk | 61 | 4 (6) | 0.156 (0.268) | 0.32 (0.87) | 37 |

- **Rotation-only gyro deskew removes the surge.** At 150 deg/s or more it needs 4
  iterations instead of 14 and ends 0.08 m and 0.1 deg off instead of 0.30 m and 0.9 deg,
  in 38 ms instead of 109 ms (39 instead of 66 ms at 60-150 deg/s).
- **The gyro beats the GT trajectory here.** GT has one pose per cloud (10 Hz), and
  interpolating between them misses the swings within a cloud. The gyro samples them at
  200 Hz. Dropping the translation costs little: at 1.6 m/s a sweep moves 0.16 m,
  while a 7 deg turn moves a point at 20 m by 2.4 m.
- **The calm clouds gain too.** Gyro deskew changes their iterations and translation
  error little, and cuts their yaw error from 0.32 to 0.05 deg.

`results.json` holds these numbers (the `off` seed; `gt` is the unperturbed seed).

Measured 2026-11-01 on ROS 2 Jazzy, 4 vCPUs. This is an offline check of one public
sequence, not the online replay.

## Online check

The same build alternated `run_koide_public_bag.py` runs between two profiles. One is
the profile on `main` (60 m range, background map rebuild). The other adds
`continuous_time_deskew_mode: imu_pose_history`, which with this change runs on the gyro
alone. Same setup and RKO-LIO binary as in
[koide_online_coverage](../koide_online_coverage/README.md). Median (range) of three
runs each:

| Profile | Failed | Coverage % | Missed clouds | RMSE m | Alignment p95 ms | Median iterations |
| --- | --- | --- | --- | --- | --- | --- |
| `main` | 0 | 99.0 (99.0-99.3) | 23 (15-23) | 0.234 (0.233-0.240) | 150 (139-156) | 4 |
| + gyro deskew | 0 | **99.7** (99.5-100.0) | **6** (1-12) | **0.227** (0.227-0.231) | **114** (112-127) | 3 |

- **Every gate metric improves**, and the three runs of each profile do not overlap in
  coverage, RMSE or alignment p95. The seed still comes from the RKO `odom` TF, as
  without the deskew, and the published pose stays the registration result.
- **The deskew applied to 69% of the clouds.**
  - 16% span more than `scan_period` x `scan_time_range_max_duration_ratio` (0.2 s),
    most of them only just: median 0.201 s, up to 0.7 s. They hold two sweeps.
  - 13-15% arrived before the IMU sample covering the sweep end
    (`continuous_time_deskew_pose_history_ends_too_early`). In the bag's order that
    sample is at most 4.3 ms late (median 0.4 ms), less than one IMU period.
- These `main` runs scored higher than the runs of
  [koide_online_coverage](../koide_online_coverage/README.md) (97.2-99.2%) on the same
  profile, in another hour of the same container. Compare profiles within one session.

`online_results.json` holds the per-run numbers (`summarize_runs.py`).

## Rerun

```bash
# FIRST_NS: stamp of the first cloud from 4 s after the bag start (run_koide_public_bag.py)
python3 experiments/koide_gyro_deskew/extract_scans.py koide-data/outdoor_hard_02b_si \
  koide-data/gt/traj_lidar_outdoor_hard_02.txt "$FIRST_NS" /tmp/koide_scans
g++ -O2 -std=c++17 -fopenmp experiments/koide_gyro_deskew/batch_align.cpp -o /tmp/batch_align \
  -I"$WS/install/ndt_omp_ros2/include" -I/usr/include/pcl-1.14 -I/usr/include/eigen3 \
  "$WS/install/ndt_omp_ros2/lib/libndt_omp.a" -lpcl_io -lpcl_common -lpcl_kdtree \
  -lpcl_search -lpcl_filters -lpcl_registration -lpcl_octree
/tmp/batch_align koide-data/map_outdoor_hard.ply /tmp/koide_scans > results.csv
```
