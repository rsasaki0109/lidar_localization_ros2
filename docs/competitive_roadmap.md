# Competitive Roadmap

Last updated: 2026-03-11

## Mission

Make `lidar_localization_ros2` a best-in-class open-source ROS 2 package for 3D LiDAR map-based localization on modern CPUs, while keeping the system easy to benchmark and easy to operate.

This document is no longer only a competitor memo. It is the execution plan for getting from the current package to a measured, defensible result set.

## Definition of done

There are three different "done" states. They should not be mixed.

### Done level 1: Build and benchmark ready

This level means:

- the package builds in the local workspace
- at least two backends can be selected and run
- a repeatable benchmark harness exists
- one dataset can be replayed without manual topic poking
- result artifacts are saved to disk in a stable format

Status on 2026-03-11:

- Complete

### Done level 2: First measured result

This level means:

- at least one benchmark dataset exists in the workspace
- `NDT_OMP` and one newer backend can be run on the same input
- pose output, diagnostics, CPU, and memory are captured
- a reference trajectory exists for at least one run
- a result summary can be attached to a commit or report

Status on 2026-03-11:

- Complete

### Done level 3: Strong competitor position

This level means:

- the package has a measurable speed or robustness edge against at least one direct competitor
- relocalization and failure handling are no longer obvious weak points
- diagnostics and covariance are good enough for downstream consumers
- large-map handling is not a blocker for real deployments

Status on 2026-03-11:

- Not started

## Current baseline

The package currently has these traits:

- Registration backends: `NDT`, `GICP`, `NDT_OMP`, `GICP_OMP`, `SMALL_GICP`, `SMALL_VGICP`
- Optional odometry motion prior
- Optional IMU deskew path
- Manual initial pose or fixed startup pose
- Full map loaded into memory
- Alignment diagnostics topic
- Benchmark runner, pose recorder, diagnostic recorder, and trajectory evaluator
- No automatic global relocalization
- No covariance estimation pipeline with documented semantics
- No dynamic map loading
- No benchmark comparison against a direct competitor yet

## Direct competitors

| System | Why it matters | Major capabilities beyond current baseline | Links |
|---|---|---|---|
| Autoware `autoware_ndt_scan_matcher` | Most relevant production-grade competitor in the ROS 2 ecosystem | Monte Carlo initial pose estimation, dynamic map loading, regularization, real-time covariance estimation, richer diagnostics | [Docs](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_ndt_scan_matcher/), [Autoware Core](https://github.com/autowarefoundation/autoware_core) |
| `koide3/hdl_localization` | Mature LiDAR localization baseline with stronger runtime robustness | UKF pose estimation, IMU pose prediction, scan matching status output, relocalization hook | [Repo](https://github.com/koide3/hdl_localization) |
| `koide3/hdl_global_localization` | Fills the largest product gap in this package | Global relocalization using BBS, FPFH + RANSAC, FPFH + TEASER | [Repo](https://github.com/koide3/hdl_global_localization) |

## Technology competitors

These are not always full localizers, but they define the performance ceiling.

| System | Why it matters | Relevant advantage | Links |
|---|---|---|---|
| `koide3/small_gicp` | Best immediate backend upgrade candidate | Optimized and parallel registration pipeline, PCL-compatible interface, low-overhead CPU backend | [Repo](https://github.com/koide3/small_gicp) |
| `koide3/fast_gicp` | Registration speed benchmark | VGICP, CUDA VGICP, NDTCuda | [Repo](https://github.com/koide3/fast_gicp) |
| `koide3/glim` | High-end open-source reference for system design and accuracy | Factor graph, stronger estimation stack, GPU registration options | [Repo](https://github.com/koide3/glim) |
| `NVIDIA-ISAAC-ROS/isaac_ros_map_localization` | Good benchmark for ROS 2 startup and relocalization UX | GPU-accelerated map localization flow | [Repo](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_map_localization) |

## Gap summary

To beat the direct competitors, the package still needs work in five areas:

1. Faster and more accurate registration backend defaults
2. Automatic global relocalization
3. Covariance estimation and more actionable diagnostics
4. Submap or dynamic map loading for scale
5. Better motion prior and explicit failure handling

## What changed recently

The local workspace now has the minimum infrastructure needed to stop arguing abstractly and start measuring:

- optional `small_gicp` and `small_vgicp` backend support
- `alignment_status` diagnostic output
- benchmark harness scripts
- benchmark harness compatibility fix for ROS 2 Humble bag playback duration control
- no-sudo local dependency setup
- a graph-to-benchmark dataset path for producing a first result even without a rosbag
- an official `hdl_localization` sample dataset acquisition path with `rosbag1 -> rosbag2` conversion

That means the next phase is measurement, not more speculative architecture discussion.

## Priority roadmap

### P0: Get the first measured result

This is the current top priority.

The goal is not to prove superiority yet. The goal is to establish a clean, reproducible baseline with artifacts.

Deliverables:

- one benchmark input dataset committed or generated reproducibly
- one `summary.json`
- one `pose_trace.csv`
- one `alignment_status.csv`
- one `resource_trace.csv`
- one `trajectory_eval.json`
- one short written interpretation of the run

Exit criteria:

- the run completes without manual topic interaction
- at least ten cloud frames are processed
- at least one pose estimate is recorded after initialization
- the result directory is reusable for later comparisons

### P1: Replace the registration core

Upgrade the scan matching backend once the baseline exists.

Tasks:

- measure `NDT_OMP` versus `SMALL_GICP`
- measure `SMALL_GICP` versus `SMALL_VGICP`
- expose backend-specific timings and failure diagnostics
- decide whether the default backend should move away from `NDT_OMP`

Expected result:

- lower alignment latency
- better convergence on current CPUs
- a justified default backend, not a guessed one

### P2: Add global relocalization

The package still assumes a good initial pose. This remains the biggest product gap.

Tasks:

- add a relocalization service or action
- start with a CPU-first candidate search path
- use local scan matching only as the refinement stage
- trigger relocalization when diagnostics show a bad or lost state

Expected result:

- recovery after startup error or localization loss
- direct improvement against `hdl_global_localization`
- a clearer story against Autoware production workflows

### P3: Add covariance and stronger diagnostics

Current output quality is still too opaque.

Tasks:

- publish covariance with documented semantics
- publish convergence state, score, overlap-like signals, and alignment time
- distinguish between "bad match", "late map", "missing initial pose", and "runtime overload"
- add thresholds that make failures machine-readable

Expected result:

- easier downstream fusion and arbitration
- easier deployment debugging
- less human guesswork during evaluation

### P4: Add submap or dynamic map loading

Full-map loading is the next clear scaling limit.

Tasks:

- load only nearby map tiles or prebuilt submaps
- update the target map asynchronously
- track tile load latency and tile coverage diagnostics
- ensure scan callbacks do not block on map IO

Expected result:

- lower peak memory
- better startup time
- usable city-scale behavior

### P5: Improve motion prior and failure handling

The current prior is lighter than what stronger systems use.

Tasks:

- move from ad hoc odometry prediction toward EKF or UKF style prediction
- use IMU beyond deskewing when available
- add explicit failure states and recovery transitions
- separate "tracking degraded" from "tracking lost"

Expected result:

- better continuity in sharp turns and low-overlap segments
- less pose jump under transient scan failures

## Immediate execution plan

### Step 1: Produce the first result

This is the shortest path to a meaningful artifact.

Plan:

1. Generate a synthetic benchmark dataset from the local graph directory
2. Build a synthetic map from a short sequence of graph clouds
3. Generate a rosbag2 input with:
   - `/velodyne_points`
   - `/initialpose`
4. Generate a matching reference trajectory CSV
5. Run `benchmark_runner` with `NDT_OMP`
6. Evaluate the recorded pose trace against the reference CSV
7. Save the result directory and write down what happened

Why this path is acceptable:

- there is no local rosbag in the workspace right now
- the graph directory already contains scan PCDs and trajectory estimates
- the result is not a final product benchmark, but it is a valid first systems test
- it is enough to validate the benchmark plumbing before moving to a real rosbag comparison

### Step 2: Repeat with `SMALL_GICP`

Once the first `NDT_OMP` result exists, repeat the exact same dataset with:

- `registration_method: SMALL_GICP`

If it runs cleanly, compare:

- alignment time distribution
- convergence state
- pose RMSE against the same reference
- CPU and memory

### Step 3: Decide the next engineering target

Decision rule after the first two runs:

- if `SMALL_GICP` is clearly faster with similar or better RMSE, make it the main optimization path
- if `SMALL_GICP` is unstable on the synthetic input, fix backend integration before starting relocalization
- if both backends perform similarly, shift effort to relocalization and diagnostics

## First result snapshot

The first measured result now exists in the local workspace.

Dataset shape:

- graph-derived synthetic benchmark
- source graph directory:
  - `/media/autoware/aa/20240425_LX300_aoba/2024-04-25-14-38-46/20240718-223557/graph`
- 12 graph frames
- synthetic map built from those frames
- map point stride: 16
- bag cloud point stride: 2
- local artifact root:
  - `/tmp/lidarloc_first_result`

Measured results:

| Backend | Matched samples | Translation RMSE | Rotation RMSE | Median align time | Max align time | Median fitness | Notes |
|---|---:|---:|---:|---:|---:|---:|---|
| `NDT_OMP` | 7 | 49.15 m | 106.61 deg | 0.714 s | 0.828 s | 3.368 | All recorded alignment diagnostics were non-OK due to score threshold violations |
| `SMALL_GICP` | 4 | 34.32 m | 86.81 deg | 1.045 s | 1.694 s | 3.552 | Fewer matched samples than `NDT_OMP`, but lower RMSE on this synthetic input |

Interpretation:

- The benchmark plumbing works end to end
- The first result is good enough to compare backends mechanically
- The absolute accuracy is poor, so this synthetic benchmark should not be treated as a product-quality result
- The likely issue is dataset realism or pose approximation quality, not benchmark infrastructure
- The next priority is to validate the synthetic map and trajectory assumptions or move to a real rosbag

Immediate next action after this snapshot:

1. sanity-check the synthetic dataset against the graph pose convention
2. repeat the run on a real rosbag as soon as one is available
3. only then start drawing backend quality conclusions

## Real rosbag acquisition snapshot

An official real rosbag is now available in the workspace for the next benchmark stage.

Dataset:

- source: `koide3/hdl_localization` example dataset
- bag URL: `https://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz`
- map source: `https://raw.githubusercontent.com/koide3/hdl_localization/master/data/map.pcd`
- local artifact root:
  - `data/official/hdl_localization`
- converted rosbag2 path:
  - `data/official/hdl_localization/hdl_400_ros2`

Observed properties:

- real recorded bag, not graph-derived synthetic data
- source format: `rosbag1`
- converted playback format: `rosbag2 sqlite3`
- duration: `126.32 s`
- point cloud topic: `/velodyne_points`
- IMU topic: `/gpsimu_driver/imu_data`
- point cloud frame: `velodyne`
- map size: about `2.05 M` points

What this unblocks:

- a real-data replay path that does not depend on graph reconstruction assumptions
- a reproducible external sample that another developer can fetch from upstream
- a direct future comparison point against `hdl_localization`

Publication rule:

- treat this official `hdl_400` sample as the default public benchmark dataset
- keep private local field logs and graph-derived synthetic bags out of published benchmark claims
- cite the upstream repository and dataset URL when publishing results, and avoid mirroring the raw bag unless redistribution terms are explicitly cleared

Public benchmark track update:

- keep `hdl_400` as the direct parity baseline against `hdl_localization`
- add the official Autoware Istanbul localization dataset as the harder public `map-based lidar localization` benchmark
- use the Istanbul pointcloud map together with the localization-only ROS 2 bag and GNSS pose topics for reference evaluation

What is still blocked:

- fair initialization for `lidar_localization_ros2` on this dataset
- direct apples-to-apples comparison against `hdl_localization`, because the original demo uses relocalization
- product-quality conclusions about backend ranking on real data

Immediate next action:

1. derive or estimate a fair initial pose for the first official real-bag run
2. replay `hdl_400_ros2` with `NDT_OMP`
3. replay the same bag with `SMALL_GICP`
4. save both run directories and compare them with `benchmark_compare_runs`

## Official real-bag smoke result

The first real-bag smoke result now exists on the official `hdl_localization` dataset.

Run setup:

- bag: `data/official/hdl_localization/hdl_400_ros2`
- map: `data/official/hdl_localization/map.pcd`
- initial pose: fixed zero pose
- playback window: first `20 s`
- comparison scope: smoke test only, no reference trajectory yet

Artifacts:

- `NDT_OMP` run directory:
  - `/tmp/lidarloc_hdl400_smoke_ndt_v2`
- `SMALL_GICP` run directory:
  - `/tmp/lidarloc_hdl400_smoke_small_gicp`
- comparison JSON:
  - `/tmp/lidarloc_hdl400_smoke_compare.json`

Measured results:

| Backend | Pose rows | Diagnostic rows | Non-OK diagnostics | Median align time | Max align time | Median fitness | Notes |
|---|---:|---:|---:|---:|---:|---:|---|
| `NDT_OMP` | 140 | 139 | 0 | 0.045 s | 0.414 s | 0.0171 | Stable on zero init during the first 20 seconds |
| `SMALL_GICP` | 150 | 149 | 0 | 0.0478 s | 0.815 s | 0.0226 | Also stable on zero init, slightly more frames recorded in the same window |

Interpretation:

- the official real bag is now usable by the current benchmark harness
- a zero-pose start is at least good enough for a short smoke test on this dataset
- both backends produced continuous pose output and clean diagnostics for the first `20 s`
- `NDT_OMP` was slightly faster on median and had a lower worst-case align time in this short window
- `SMALL_GICP` produced slightly more pose and diagnostic rows, but without a reference trajectory that is not yet a quality claim

What this does not prove:

- long-horizon drift behavior
- absolute accuracy on the official sample
- fairness against `hdl_localization`, because the official method still has a relocalization path that this package lacks

Immediate next action after the smoke result:

1. extend the official sample run beyond `20 s`
2. derive a reference trajectory or pseudo-ground-truth path for the official sample
3. repeat the same comparison with `use_imu` enabled if the frame assumptions are validated
4. only then decide whether backend ranking on this real bag is meaningful

## Autoware Istanbul clean snapshot

The Istanbul benchmark flow is now clean enough to trust as a public benchmark harness.

What changed:

- extract the GNSS reference directly from the rosbag2 sqlite file instead of recording it through replay
- resample the GNSS poses onto `/localization/util/downsample/pointcloud` header timestamps
- isolate every benchmark run with its own `ROS_DOMAIN_ID`

Artifacts:

- reference CSV: `/tmp/lidarloc_istanbul_ref_direct_30s.csv`
- initial pose YAML: `/tmp/lidarloc_istanbul_init_direct_30s.yaml`
- run directory: `/tmp/lidarloc_autoware_istanbul_ndt_30s_direct_iso3`
- public HTML report:
  - `/media/autoware/aa/ai_coding_ws/lidarloc_ws/artifacts/public/autoware_istanbul_ndt_30s/index.html`

Run setup:

- dataset: official Autoware Istanbul localization bag
- backend: `NDT_OMP`
- playback window: first `30 s`
- init: first GNSS reference sample
- score gate: `30.0`

Measured results:

| Metric | Value |
|---|---:|
| Pose rows | `189` |
| Diagnostic rows | `204` |
| Matched samples vs GNSS reference | `187` |
| Translation RMSE vs GNSS reference | `10.731 m` |
| Rotation RMSE vs GNSS reference | `141.120 deg` |
| Diagnostic rejects | `16` |
| Median fitness | `16.076` |

Interpretation:

- the Istanbul benchmark harness itself is no longer the blocker
- with a fair initial pose and clean reference extraction, the current `NDT_OMP` configuration still diverges too much
- the next work item is backend and estimator quality on public urban data, not more benchmark plumbing

## Istanbul quick NDT sweep

The first coarse parameter sweep on the clean Istanbul benchmark is complete.

Sweep scope:

- backend: `NDT_OMP`
- window: first `30 s`
- fixed fair initial pose from the extracted GNSS reference
- searched knobs:
  - `ndt_resolution`
  - `voxel_leaf_size`
  - `score_threshold`
  - `scan_max_range`

Artifacts:

- config preset:
  - `param/benchmark/autoware_istanbul_ndt_quick_sweep.json`
- sweep summary:
  - `/tmp/lidarloc_istanbul_ndt_quick_sweep/summary.csv`
- best report:
  - `/media/autoware/aa/ai_coding_ws/lidarloc_ws/artifacts/public/autoware_istanbul_ndt_30s/index.html`

Best run:

| Name | Translation RMSE | Rotation RMSE | Matched samples | Warn diagnostics |
|---|---:|---:|---:|---:|
| `ndt_r1_l02_thr30` | `9.995 m` | `141.132 deg` | `189` | `8` |

What the sweep says:

- `ndt_resolution=2` and `3` did not improve Istanbul
- coarser `voxel_leaf_size` also did not help
- relaxing `score_threshold` to `50` increased accepted updates but made RMSE worse
- the current bottleneck is not simple front-end tuning; it is the overall scan-matching quality on this urban dataset

## Istanbul stable-init finding

The biggest single improvement so far came from the initialization policy, not from the NDT front-end.

Finding:

- the first exported GNSS sample in the Istanbul 30-second window can carry an unstable yaw
- using that first sample as the initial pose pushed the NDT result into a roughly `140 deg` yaw mismatch
- skipping the first `0.05 s` of exported reference samples fixed the initial yaw and changed the benchmark outcome materially

Artifacts:

- stable-init YAML from extractor:
  - `/tmp/lidarloc_istanbul_init_stable_from_extractor.yaml`
- stable-init baseline run:
  - `/tmp/lidarloc_autoware_istanbul_ndt_30s_stable_init`
- stable-init threshold sweep:
  - `/tmp/lidarloc_istanbul_stable_threshold_sweep/summary.csv`
- current public report:
  - `/media/autoware/aa/ai_coding_ws/lidarloc_ws/artifacts/public/autoware_istanbul_ndt_30s/index.html`

Measured result after stable init:

| Config | Translation RMSE | Rotation RMSE | Matched samples |
|---|---:|---:|---:|
| Stable init + `score_threshold=30` | `8.020 m` | `9.321 deg` | `200` |
| Stable init + `score_threshold=5` | `1.253 m` | `0.385 deg` | `108` |
| Stable init + `score_threshold=8` | `1.171 m` | `0.736 deg` | `102` |
| Stable init + `score_threshold=10` | `1.307 m` | `0.982 deg` | `113` |

Interpretation:

- initialization policy matters more than coarse NDT tuning on this dataset
- once the initial yaw is sane, the remaining high-error mode is mostly bad updates leaking through the fitness gate
- the stable-init baseline is now good enough to use as the control for twist-aided prediction

Measured result after linear-only twist sweep:

| Config | Translation RMSE | Rotation RMSE | Matched samples |
|---|---:|---:|---:|
| Twist linear + `score_threshold=5` | `1.740 m` | `0.115 deg` | `105` |
| Twist linear + `score_threshold=6` | `1.163 m` | `0.383 deg` | `109` |
| Twist linear + `score_threshold=7` | `1.970 m` | `0.238 deg` | `108` |
| Twist linear + `score_threshold=8` | `1.378 m` | `0.586 deg` | `110` |
| Twist linear + `score_threshold=10` | `2.895 m` | `0.704 deg` | `116` |

Interpretation:

- linear-only twist prediction is now the best public 30-second setting on Istanbul
- `score_threshold=6` is the current balanced winner because it slightly improves translation over the old best,
  cuts rotation error roughly in half, and increases matched coverage
- this does not solve long-horizon behavior yet, but it is the right public baseline for the next iteration

## Istanbul 60-second extension

After selecting `twist linear + score_threshold=6` as the public 30-second winner, the next question
was whether the same setting merely wins the first few seconds or remains usable once the trajectory
extends beyond the easy startup window.

Run setup:

- dataset: official Autoware Istanbul localization bag
- map: `data/official/autoware_istanbul/pointcloud_map.pcd`
- cloud topic: `/localization/util/downsample/pointcloud`
- twist topic: `/localization/twist_estimator/twist_with_covariance`
- reference source: direct bag extraction from `/sensing/gnss/pose_with_covariance`
- initial pose policy: same stable-init rule, skip the first `0.05 s` before choosing the exported
  pose used for initialization
- localization params:
  - `registration_method=NDT_OMP`
  - `score_threshold=6`
  - `use_twist_prediction=true`
  - `twist_prediction_use_angular_velocity=false`
  - `max_twist_prediction_dt=0.5`

Artifacts:

- 60-second initial pose YAML:
  - `/tmp/lidarloc_istanbul_init_60s_stable.yaml`
- 60-second reference CSV:
  - `/tmp/lidarloc_autoware_istanbul_reference_60s_stable.csv`
- 60-second run directory:
  - `/tmp/lidarloc_autoware_istanbul_twist_linear_thr6_60s`
- 60-second evaluation JSON:
  - `/tmp/lidarloc_autoware_istanbul_twist_linear_thr6_60s/eval.json`
- 60-second drift analysis JSON:
  - `/tmp/lidarloc_autoware_istanbul_twist_linear_thr6_60s/drift_analysis.json`
- 60-second HTML report:
  - `/media/autoware/aa/ai_coding_ws/lidarloc_ws/artifacts/public/autoware_istanbul_ndt_60s/index.html`

Measured result:

| Window | Translation RMSE | Rotation RMSE | Matched samples | Reference samples |
|---|---:|---:|---:|---:|
| `30 s` best public run | `1.163 m` | `0.383 deg` | `109` | `238` |
| `60 s` extension | `2.661 m` | `0.385 deg` | `113` | `538` |

What changed from `30 s` to `60 s`:

- rotation quality stayed essentially flat
- translation error more than doubled
- matched sample count increased only slightly even though the reference window more than doubled
- the failure mode therefore does not look like immediate catastrophic divergence
- instead it looks like a slower translational drift or intermittent under-tracking after the easy
  startup segment

Current interpretation:

- the project has crossed an important threshold: Istanbul no longer fails at initialization
  or at obvious yaw alignment in the first `30 s`
- the remaining weakness is now specifically long-horizon translation consistency
- because rotation remains stable while translation drifts, the next likely bottlenecks are:
  - accepted translational corrections being slightly biased
  - poor update cadence after some frames are rejected
  - map/scan overlap quality degrading in a way that the current scalar fitness gate does not
    fully capture

Practical implication:

- for public reporting, the `30 s` result is strong enough to keep publishing
- for engineering direction, the `60 s` result is more important because it identifies the next
  bottleneck clearly
- backend replacement should stay secondary until this translation drift is characterized

First measured drift interval:

- first translation delta `> 1 m`: `4.7 s`
- first translation delta `> 2 m`: `10.3 s`
- first translation delta `> 3 m`: `10.5 s`
- first translation delta `> 5 m`: `32.8 s`
- first sustained `> 1 m` crossing by rolling median: `5.1 s`

Diagnostic summary around the first drift event (`+-2 s` around `4.7 s`):

- rows: `40`
- warn count: `0`
- error count: `0`
- fitness median: `1.132`
- fitness max: `1.289`
- alignment time median: `0.000124 s`
- alignment time max: `0.000527 s`
- non-converged count: `0`

Interpretation:

- the first drift interval is not a visible "diagnostics turned red" event
- translation begins separating while the current health signals still look good
- this raises the priority of either:
  - a stronger acceptance criterion than scalar fitness alone
  - or a debug view that tracks translational consistency directly

## Istanbul next debug slice

The next Istanbul work should not start from a large parameter search. It should start from the
current best public run and isolate the first interval where translation drift accumulates.

Primary objective:

- find the first time range in the `60 s` run where translation delta begins to separate materially
  while rotation stays near the reference

Why this is the right next step:

- a fresh sweep will mostly rediscover the same `thr6` region
- the benchmark now shows a clear transition from "good startup" to "worse long horizon"
- isolating the first bad interval is more informative than averaging over the whole `60 s`

Concrete debug outputs to capture next:

- translation-delta time series from the `60 s` report
- fitness score around the first sustained growth in delta
- accepted vs rejected update cadence over that same interval
- pose increment magnitude between accepted poses
- point count and alignment time around the first drift segment

Hypotheses to test in order:

1. rejected updates create too large a gap before the next accepted translational correction
2. the twist predictor is helping orientation but under-constraining translation
3. the scalar `fitness_score` threshold is too weak as a guard for urban partial-overlap cases
4. scan range filtering or voxelization is discarding information needed later in the segment

Immediate experiment queue:

1. annotate the first `60 s` report with the first `1 m`, `2 m`, and `3 m` translation-delta crossings
2. export a compact diagnostic summary around the first crossing window
3. compare `twist linear thr6` against `stable_thr8` on the same `60 s` interval to verify that the
   extra drift is not simply a tradeoff hidden by the `30 s` average
4. only after that, decide whether to:
   - tighten the rejection logic further
   - improve the predictor
   - or move toward Autoware-style regularization

Definition of done for the next Istanbul step:

- identify the first drift interval with concrete timestamps
- show whether the interval corresponds to:
  - diagnostics worsening
  - lower accepted update density
  - or clean diagnostics despite increasing translation error
- produce one change that specifically targets that failure mode
- rerun the same `60 s` benchmark and compare against the current `2.661 m / 0.385 deg` baseline

What not to do next:

- do not claim longer-horizon parity on Istanbul yet
- do not switch default backend based only on the current `30 s` public snapshot
- do not add a large new feature before the first long-horizon drift interval is understood

## Official full-bag parity snapshot

A guarded-update pass on the official `hdl_400` bag now closes the long-horizon gap against
`hdl_localization` under the current comparison rules.

Run setup:

- bag: `data/official/hdl_localization/hdl_400_ros2`
- map: `data/official/hdl_localization/map.pcd`
- init: fixed zero pose
- fairness rule: `use_imu:=false`, no relocalization, same map
- baseline: Dockerized `hdl_localization`
- current package changes:
  - reject pose updates when `fitness_score > score_threshold`
  - extrapolate the next initial guess from the last accepted pose delta

Artifacts:

- previous full-bag run:
  - `/tmp/lidarloc_hdl400_full_ndt`
- improved full-bag run:
  - `/tmp/lidarloc_hdl400_full_ndt_guarded`
- baseline output:
  - `/tmp/hdl_localization_hdl400_full`
- updated HTML comparison report:
  - `/tmp/lidarloc_hdl400_report_guarded/index.html`

Measured result against `hdl_localization`:

| Run | Pose rows | Translation RMSE | Rotation RMSE | Endpoint delta | First delta > 1m | Diagnostic warns | Max fitness |
|---|---:|---:|---:|---:|---:|---:|---:|
| Previous full-bag run | 711 | 28.755 m | 47.874 deg | 85.908 m | 53.35 s | 250 | 18.57 |
| Guarded update | 1059 | 0.057 m | 0.926 deg | 0.013 m | not reached | 0 | 1.09 |

Additional checks:

- path length ratio vs `hdl_localization`: `1.0077`
- overlap duration: `126.21 s`
- translation delta never exceeded `1 m`
- rotation delta never exceeded `10 deg`

Interpretation:

- on this official bag and under this no-relocalization rule, `lidar_localization_ros2` is now
  no longer materially worse than `hdl_localization`
- the main fix was not a new backend but refusing bad scan-match updates and keeping a better
  next-frame initial guess
- this result is still dataset-specific and should not yet be generalized to difficult relocalization
  or low-feature cases

Immediate next action after this parity snapshot:

1. repeat the same full-bag run with `SMALL_GICP`
2. add a regression check that fails when full-bag delta exceeds a fixed threshold
3. stress the same logic on a harder bag with larger initial error or repetitive structure

## Benchmark protocol

The benchmark protocol must stay stable across commits.

### Input classes

At minimum, maintain these dataset classes:

- synthetic graph-derived benchmark for smoke testing
- real urban driving rosbag
- low-feature or repetitive-structure rosbag
- large-initial-error relocalization stress rosbag

### Metrics to report

Always capture:

- translation RMSE
- rotation RMSE
- number of recorded poses
- number of diagnostic messages
- median, p95, and p99 CPU usage
- median and peak RSS
- peak HWM memory
- count of non-OK alignment diagnostics

Capture when available:

- relocalization success rate
- relocalization latency
- lost-track rate
- startup-to-first-pose latency

### Result directory contract

Each run directory should contain:

- `pose_trace.csv`
- `alignment_status.csv`
- `resource_trace.csv`
- `summary.json`
- `trajectory_eval.json` when a reference exists
- stdout and stderr logs for system, recorder, diagnostics, and bag playback

## Decision gates

Use these gates before moving to the next major feature:

### Gate A: Benchmark confidence

Required before backend changes are considered "real":

- same input dataset
- same parameter file except for the backend under test
- same bag rate
- same target process pattern
- same evaluation script

### Gate B: Backend adoption

Required before changing the default backend:

- at least two datasets
- no catastrophic failures on either
- `SMALL_GICP` median alignment time better than `NDT_OMP`
- RMSE not materially worse

### Gate C: Product feature shift

Required before pausing backend work and moving to relocalization:

- at least one backend result is stable enough to serve as a baseline
- diagnostics are trustworthy enough to detect degraded behavior

## Risks and mitigations

### Risk: No real rosbag available locally

Impact:

- delays meaningful field comparison

Mitigation:

- use the graph-derived synthetic benchmark as the first artifact
- keep the runner compatible with real rosbag2 so the workflow does not change later

### Risk: Synthetic benchmark is too easy

Impact:

- may overestimate stability and understate real-world failure

Mitigation:

- treat it only as a systems smoke test
- move to real rosbag data immediately after the first artifact exists

### Risk: Full maps are too large for fast iteration

Impact:

- long startup times
- benchmark turnaround becomes poor

Mitigation:

- use small synthetic maps for first-result runs
- move to cropped or tiled maps for later field datasets

### Risk: Dependency friction hides backend results

Impact:

- work stalls in build issues instead of localization quality

Mitigation:

- keep the no-sudo local environment script up to date
- keep benchmark input generation self-contained inside the repo

## Near-term milestone schedule

### Milestone M0: First artifact

Target:

- 2026-03-11

Must include:

- synthetic bag
- synthetic map
- reference trajectory
- one benchmark result directory

### Milestone M1: First backend comparison

Target:

- 2026-03-12

Must include:

- `NDT_OMP` and `SMALL_GICP` on the same input
- one short comparison note

### Milestone M2: Real bag baseline

Target:

- immediately after a real rosbag path is available

Must include:

- at least one real dataset result
- same artifact structure as M0 and M1

## Recommended order of attack

If engineering time is limited, do the work in this order:

1. First measured result
2. `small_gicp` backend comparison
3. Relocalization service
4. Covariance and richer diagnostics
5. Dynamic map loading
6. Stronger motion prior and failure handling

## License notes

This repository is BSD-2-Clause. Prefer Apache-2.0, BSD, and MIT source projects for direct adaptation. Avoid copying GPL code into the core package.
