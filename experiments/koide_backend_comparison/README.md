# Koide online check: NDT_OMP and SMALL_GICP

Question: does SMALL_GICP hold up on Koide `outdoor_hard_02b` now that NDT_OMP runs
optimized? On Istanbul both backends track alike
([istanbul_ndt_tracking](../istanbul_ndt_tracking/README.md#unoptimized-ndt_omp)), and the
backend decision gate asks for a second public dataset
([competitive_roadmap](../../docs/dev/competitive_roadmap.md#decision-gates)).

## Setup

- **Check.** `run_koide_public_bag.py`, full sequence at 1x. It passes at RMSE ≤ 0.35 m,
  diagnostic coverage ≥ 97% and GT-matched poses ≥ 95%.
- **Platform.** ROS 2 Jazzy, 4 vCPUs, shared with RKO-LIO, as in
  [koide_online_coverage](../koide_online_coverage/README.md).
- **Code.** `main` at `16e315c`, one build. small_gicp 2.1.0 from
  `ros-jazzy-small-gicp-vendor`.
- **Odometry.** Online RKO-LIO 0.3.2, the same binary in every run.
- **Variants.**
  - `NDT_OMP`: the online profile as on `main`.
  - `SMALL_GICP`: the same profile with `registration_method: SMALL_GICP` and nothing else
    changed. The target is the crop voxel-filtered at 0.5 m, 4 threads.
  - `SMALL_GICP_2_threads`: also `ndt_num_threads: 2`, which sets the small_gicp
    thread count.
- **Order.** Runs alternated between variants. Load was 0.0-2.4 at the start of each run.

The fitness score both backends are gated on is `pcl::Registration::getFitnessScore()`,
so `score_threshold: 6.0` means the same for both.

## Result

Median (range):

| Variant | Runs | Failed | Coverage % | RMSE m | RMSE outside the stretch m | Max error in the stretch m (horizontal, z) |
| --- | --- | --- | --- | --- | --- | --- |
| **NDT_OMP** | 4 | **0** | **99.8** (99.6-100.0) | **0.230** (0.227-0.232) | **0.177** (0.176-0.178) | **0.80** (0.78-0.83) (0.24, -0.77) |
| SMALL_GICP | 3 | 3 | 91.8 (91.2-92.8) | 0.401 (0.378-0.411) | 0.192 (0.191-0.193) | 1.41 (1.37-1.42) (0.18, -1.40) |
| SMALL_GICP, 2 threads | 2 | 2 | 88.9 (87.3-90.6) | 0.362 (0.324-0.400) | 0.192 (0.191-0.193) | 1.40 (1.39-1.42) (0.17, -1.40) |

| Variant | Missed clouds (in the stretch) | Alignment outside the stretch, median / p95 | In the stretch, median / p95 / max | Alignment s per s in the stretch |
| --- | --- | --- | --- | --- |
| NDT_OMP | 4 (0) | 38 / 114 ms | 37 / 105 / 364 ms | 0.33 |
| SMALL_GICP | 181 (164) | **22 / 67 ms** | 45 / 350 / 2966 ms | 0.60 |
| SMALL_GICP, 2 threads | 244 (227) | 24 / 72 ms | 54 / 374 / 4978 ms | 0.67 |

The stretch is 163-259 s after the first expected cloud. There, in every run, the
localizer accepted no scan: the sensor is off the map, and the pose is carried by the
RKO-LIO odometry from the last accepted pose. The stretch is the same in all nine runs.

- **SMALL_GICP fails the check in all five runs, NDT_OMP passes all four.**
- **Coverage is lost off the map.** With no map to fit, SMALL_GICP needs more
  iterations. Its slowest scans take 1-8 s, most of them at the iteration cap, and
  90-93% of its missed clouds fall in the stretch.
- **RMSE is lost off the map too, but not to a worse pose.** The odometry-carried pose
  sinks 1.40 m below GT there, against 0.77 m with NDT_OMP. The odometry is the same:
  the two backends hand it different anchors (*Why the carried pose drifts further*).
- **On the map SMALL_GICP is faster but less accurate.** Its alignment is 22 instead of
  38 ms (median, 67 instead of 114 ms p95), at 0.192 instead of 0.177 m RMSE. In the
  first 20 s it holds a pitch error of +0.27 deg, against +0.01 deg for NDT_OMP.
- **Two threads lose more clouds** (244 instead of 181) and leave the drift as it is.

## Why the carried pose drifts further

Off the map the published pose is the `map -> odom` of the last accepted scan
(163.3 s) times the RKO-LIO `odom` pose. `anchor_difference.py` pairs the poses of two
runs by stamp over the stretch and finds the one rigid transform that maps one
trajectory onto the other:

| Runs | Anchor change roll / pitch / yaw deg | Gap max m | Residual median / max m |
| --- | --- | --- | --- |
| NDT_OMP -> SMALL_GICP (runs 1, 2, 3) | 0.08-0.10 / -0.45 to -0.46 / -0.01 to -0.03 | 0.79-0.85 | 0.007-0.024 / 0.033-0.053 |
| NDT_OMP run 1 -> run 3 | -0.02 / -0.02 / -0.01 | 0.06 | 0.007 / 0.038 |
| SMALL_GICP run 1 -> run 3 | -0.03 / -0.01 / 0.01 | 0.07 | 0.021 / 0.052 |

- **The odometry is the same.** One anchor change of 0.47 deg of tilt takes the
  SMALL_GICP trajectory to the NDT_OMP one within 0.05 m over 88 s. Neither CPU load
  nor the backend changes what RKO-LIO reports.
- **The anchor of SMALL_GICP is not the worse one.** The last accepted scan is off by
  pitch +0.26 deg and roll -0.21 deg with NDT_OMP, pitch -0.01 deg and roll +0.18 deg with
  SMALL_GICP (GT body frame). The odometry itself sinks over the stretch, and
  NDT_OMP's pitch error happens to cancel part of that.
- **So the RMSE of this check rests on the tilt of one scan.** The stretch holds a
  third of the poses. The 0.47 deg anchor change there makes most of the 0.17 m RMSE
  gap between the backends; outside the stretch they differ by 0.015 m. Compare
  variants on the RMSE outside the stretch as well; `summarize_runs.py` reports it.

## Would a better anchor help?

No: on this sequence a more accurate anchor makes the carried pose worse.
`anchor_tilt.py` takes five NDT_OMP runs on `main` (2026-11-02, 0.309-0.352 m off the
map). It replaces the last accepted scan's tilt error with the mean of the last N
accepted scans, or with none, by turning the off-map poses about the anchor position.

| Anchor tilt error | Roll / pitch deg | Off-map RMSE m |
| --- | --- | --- |
| the last accepted scan's, as run | -0.39 / +0.17 to +0.18 | 0.309-0.352 |
| mean of the last 5 accepted scans | -0.12 to -0.14 / +0.11 to +0.15 | 0.424-0.531 |
| mean of the last 10 | -0.05 to 0.00 / +0.11 to +0.14 | 0.508-0.590 |
| mean of the last 30 | -0.06 to -0.01 / +0.10 to +0.12 | 0.518-0.589 |
| none | 0 / 0 | 0.613-0.709 |

- **The anchor is the same in every run.** The last accepted scan is always the one at
  163.0 s, and its tilt error is the same within 0.01 deg. The spread of the off-map
  RMSE between runs comes from the online RKO-LIO odometry, which is not
  deterministic.
- **That scan's roll error is an outlier.** Scan to scan, roll varies by 0.20-0.25 deg
  (standard deviation over the last 30) and pitch by 0.12-0.14 deg. The last scan's
  -0.39 deg roll is 1.4-1.8 standard deviations from the mean of the last 30.
- **But it cancels part of the odometry's own sink.** Averaging it away, or removing
  the tilt error altogether, raises the off-map RMSE by 0.1-0.4 m. The same holds for
  levelling `map -> odom` (`level_map_to_odom`), which on `outdoor_hard_02b` raised the
  RMSE from 0.26 to 0.35 m ([troubleshooting](../../docs/troubleshooting.md)).
- **So the off-map RMSE of this check measures the odometry, and luck in the anchor.**
  Nothing the localizer does with the anchor improves it here, and a change that
  makes it better should not be read as a better anchor. Compare variants on the map
  (`rmse_on_map_m`).

## Decision

On Koide, SMALL_GICP fails the gate's condition that RMSE and coverage are not
materially worse. Its coverage is 88-93% and no run passes the check. Its RMSE outside
the stretch is 0.192 against 0.177 m. Its RMSE within the stretch says nothing about
the backend. The Istanbul result alone does not justify a backend change. NDT_OMP stays
the default, and `param/istanbul_small_gicp.yaml` stays a dataset-specific comparison.

`results.json` holds the per-run numbers (`summarize_runs.py`).

Measured 2026-11-02 on ROS 2 Jazzy, 4 vCPUs. This is one public sequence replayed on a
cloud container, not a physical-robot test.

## Rerun

```bash
# Build RKO-LIO from the rko_lio commit lidar_slam_ros2 pins, as Release, in RKO_WS.
ros2 run lidar_localization_ros2 run_koide_public_bag.py --download --data-dir koide-data \
  --output koide-runs/ndt_1 --rko-executable "$RKO_WS/install/rko_lio/lib/rko_lio/online_node"
# Set registration_method: SMALL_GICP in tools/readme_media/koide_outdoor_hard_02b_online.yaml
# (with --symlink-install the check reads it from the checkout), run again into
# koide-runs/sgicp_1, and alternate.
python3 experiments/koide_backend_comparison/summarize_runs.py koide-data \
  NDT_OMP=koide-runs/ndt_1 SMALL_GICP=koide-runs/sgicp_1 --json results.json
# the anchor tilt (any number of runs)
python3 experiments/koide_backend_comparison/anchor_tilt.py koide-data koide-runs/ndt_1
```
