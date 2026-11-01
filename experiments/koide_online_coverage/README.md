# Koide online check: missed clouds

Question: why did the v1.5.0 Koide `outdoor_hard_02b` check
([docs/koide-online-replay.md](../../docs/koide-online-replay.md#v150-release-verification-2026-11-01))
report 97.6-98.3% diagnostic coverage, where v1.3.0 reported 100%? And how much margin
does the 97% gate leave?

## Setup

- **Runs.** `run_koide_public_bag.py`, full sequence at 1x.
- **Platform.** ROS 2 Jazzy in one 4-vCPU cloud container (Intel Xeon @ 2.80 GHz).
- **Odometry.** Online RKO-LIO 0.3.2, built as Release from the `rko_lio` commit that
  `lidar_slam_ros2` pins (`7e91814`). It is the same binary in every run.
- **Builds.** The v1.5.0 runs use the release tree (`eed5098`, built without a build
  type). The v1.3.0 runs use the tag (`3878a7a`) with `ndt_omp_ros2` built as Release,
  since v1.3.0 does not compile NDT_OMP itself.
- **Order.** Variants alternated run by run. Load was 0.2-4.4 at the start of each run.

A cloud counts as missed when it got no `/alignment_status`, as the check counts it.
`summarize_runs.py` lists the missed clouds and writes `results.json`.

## v1.3.0 and v1.5.0 on the same setup

| Variant | Runs | Failed | Coverage % | RMSE m | Max output gap s |
| --- | --- | --- | --- | --- | --- |
| v1.5.0 | 6 | 2 | 97.9 (95.2-99.1) | 0.263 (0.252-0.275) | 1.4 (0.9-1.9) |
| v1.3.0 | 2 | 0 | 97.5 (97.0-97.9) | 0.267 (0.255-0.278) | 1.4 (1.3-1.5) |

Median (range) over the runs.

- **Not a regression.** On this setup v1.3.0 also misses clouds. The 100% it reported
  came from its setup (Humble, the RKO image, another machine), not from its code.
- **Little margin.** On 4 vCPUs the gate has 1-2 points to spare. Two of the six v1.5.0
  runs fell below it (96.4% and 95.2%), and one v1.3.0 run passed at exactly 97.0%.

## Where the clouds are missed

79-100% of the missed clouds lie between 100 and 132 s after the first expected cloud,
in every run of both versions. Two things stall the localizer there:

- **NDT needs many more iterations.** The median per scan rises from 3-4 to 10-31 in
  the slowest seconds. The alignment time per second of bag then reaches 0.5-2.1 s, more
  than the 4 cores (shared with RKO) keep up with.
- **The local map is rebuilt in the scan callback.** It is rebuilt every 10 m of
  motion. The rebuild at 108.6 s coincides with the longest output gap of each run
  (1.3-1.9 s), and seconds 109, 116 and 124 had no aligned scan in one run.

`local_map_async_rebuild` did not help the 75 m profile as it was. With `scan_max_range`
at 100 m, more than `local_map_radius`, the cached target is never kept past
`local_map_update_distance`, so every rebuild waits. Only 0.2% of the points of
`outdoor_hard_02b` lie beyond 60 m (44 sampled clouds; 1.3% beyond 40 m, p99 at 41.9 m).
At `scan_max_range: 60` the cached target serves until 15 m, and the background rebuild
gets 5 m to finish.

## Profile variants

Same build (v1.5.0), same RKO, alternated:

| Variant | Runs | Failed | Coverage % | RMSE m | p95 error m | Max output gap s |
| --- | --- | --- | --- | --- | --- | --- |
| profile as in v1.5.0 | 6 | 2 | 97.9 (95.2-99.1) | 0.263 (0.252-0.275) | 0.669 (0.560-0.746) | 1.4 (0.9-1.9) |
| `scan_max_range: 60` | 3 | 0 | 97.9 (97.8-98.4) | 0.236 (0.234-0.254) | 0.548 (0.544-0.570) | 1.2 (0.9-1.3) |
| `scan_max_range: 60`, `local_map_async_rebuild: true` | 5 | 0 | 98.4 (97.2-99.2) | 0.238 (0.235-0.239) | 0.593 (0.581-0.616) | 0.9 (0.9-1.5) |

- **The range drives the RMSE.** All 8 runs at 60 m scored 0.234-0.254 m RMSE, against
  0.252-0.275 m before (one run overlaps).
- **The background rebuild shortens the stall.** The longest output gap was 0.9 s in
  three of five runs (1.1 and 1.5 s in the others). Coverage is not clearly higher: the iteration surge remains.
- **Adopted.** The profile now uses both settings: no run failed, RMSE dropped
  0.025 m and the longest gap 0.5 s. With 3 and 5 runs, the coverage gain is not
  established, and the margin over the 97% gate stays 0.2-2.2 points on this machine.

## Open

Why NDT needs 10-31 iterations from 104 to 130 s, where most clouds are missed. The
seed comes from the RKO `odom` TF (`use_odom_tf_prediction`). Find whether that seed is
off there, or whether the scans differ: the filtered point count doubles at 100 and
108 s. A faster convergence there would give the gate real margin on 4 cores.

## Rerun

```bash
# Build RKO-LIO from the rko_lio commit lidar_slam_ros2 pins, as Release, in RKO_WS.
ros2 run lidar_localization_ros2 run_koide_public_bag.py --download --data-dir koide-data \
  --output koide-runs/run1 --rko-executable "$RKO_WS/install/rko_lio/lib/rko_lio/online_node"
# Edit tools/readme_media/koide_outdoor_hard_02b_online.yaml between runs for a variant;
# with --symlink-install the check reads it from the checkout.
python3 experiments/koide_online_coverage/summarize_runs.py koide-data/outdoor_hard_02b_si \
  v1.5.0=koide-runs/run1 v1.5.0=koide-runs/run2 --json results.json
```

Measured 2026-11-01. This is a public-bag replay on one cloud container, not a
physical-robot test.
