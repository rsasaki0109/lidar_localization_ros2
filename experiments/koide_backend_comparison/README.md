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
- **RMSE is lost off the map too.** The odometry-carried pose sinks 1.40 m below GT
  there, against 0.77 m with NDT_OMP. The drift repeats to within 0.05 m across runs
  of a backend, and 2 threads leave it unchanged. So it does not come from load on the
  shared cores. SMALL_GICP leaves the map with a pitch error of -0.14 to -0.15 deg,
  against +0.08 to +0.10 deg for NDT_OMP (median over the last 8 s, two runs each).
  That would tilt the carried trajectory, but it is not established as the cause.
- **On the map SMALL_GICP is faster but less accurate.** Its alignment is 22 instead of
  38 ms (median, 67 instead of 114 ms p95), at 0.192 instead of 0.177 m RMSE. In the
  first 20 s it holds a pitch error of +0.27 deg, against +0.01 deg for NDT_OMP.
- **Two threads lose more clouds** (244 instead of 181) and leave the drift as it is.

## Decision

On Koide, SMALL_GICP fails the gate's condition that RMSE and coverage are not
materially worse, on both counts: 88-93% coverage and 0.32-0.41 m RMSE, and no run
passes the check. The Istanbul result alone does not justify a backend change. NDT_OMP stays the default,
and `param/istanbul_small_gicp.yaml` stays a dataset-specific comparison.

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
```
