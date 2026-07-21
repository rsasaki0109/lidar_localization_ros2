# GLIL-style GLIM prior-map localizer

This experiment provides a GLIL-style tightly coupled localizer that keeps GLIM
LiDAR/IMU odometry continuous while adding prior-map constraints to the same fixed-lag
factor graph. It replaces continuously running NDT. The implementation combines the
sliding-window range-inertial localization formulation from Koide et al. (ICRA 2024)
with exact quadratic point-cloud downsampling from Koide et al. (ICRA 2025).

The older split `map -> odom` experiment is retained as an opt-in comparison mode.
New measurements and publication artifacts use the tightly coupled mode.

## Architecture

GLIM produces continuous `odom -> base_link` poses at scan rate. Each fixed-lag update
jointly optimizes IMU preintegration, scan-to-scan GICP to the preceding three frames,
and scan-to-prior-map GICP where overlap is sufficient. Scan factors use the ICRA 2025
deferred exact-coreset state machine: a full linearization is reduced to a weighted
subset that preserves all independent entries of the relative-pose `H`, `b`, and `c`,
then reused only inside an explicit linearization bound.

The graph owns a persistent `odom_from_map` state. Prior-map factors connect it directly
to the same pose states constrained by IMU and scan-to-scan factors. Verified sparse
submap VGICP observations start a new graph epoch to anchor map-degenerate directions;
they do not publish a second correction. The inverse graph state is the sole public
`map -> odom` transform and is bounded to 0.25 m and 2 degrees per update by default.

When map overlap is lost, only map factors are omitted. Range-inertial odometry and
re-stamping of the last valid `map -> odom` continue. `/initialpose` candidates are
independently checked by VGICP over three consecutive frames before a recovery epoch
can enter the graph. Rejected candidates do not reset odometry or TF.

No ground truth is used at runtime. Ground truth is used only by the evaluator and GIF
renderer. Full-map KISS search remains experimental and is disabled by default because
the unseeded outdoor replay found a repeatable structural ambiguity.

The previously published callback and ROS consumer are maintained in these forks;
the exact-coreset GLIM update will be pinned here when the tightly coupled branch is
published:

- [GLIL unofficial core callback](https://github.com/rsasaki0109/glil_unofficial/commit/dd29667938cb56ef59de7f90290ad2dbc613f00c)
- [GLIM ROS external map-to-odom consumer](https://github.com/rsasaki0109/glim_ros2/commit/cd4b2c9eb37a5c12c93d7339ce10167ebaa55288)

The Dockerfile pins the published fork revisions. The design follows the
[ICRA 2024 localization paper](https://arxiv.org/abs/2402.05540),
[ICRA 2025 exact-downsampling paper](https://arxiv.org/abs/2505.01017),
[GLIM paper](https://arxiv.org/abs/2407.10344),
[GLIM extension API](https://github.com/koide3/glim/blob/master/docs/extend.md), and
[KISS-Matcher paper](https://arxiv.org/abs/2409.15615).

## Build

```bash
docker build -t lidarloc/glim-ros2:jazzy-v1.2.2-tightly-coupled \
  experiments/glim_prior_map_localizer
```

The build compiles and tests the GLIM ROS smoother before installing the extension.

## Runtime configuration

The benchmark runner adds `libglim_prior_map_localizer.so` to GLIM's
`extension_modules`. Important environment variables are:

| Variable | Default | Meaning |
| --- | ---: | --- |
| `GLIM_PRIOR_MAP_PATH` | required | ASCII or little-endian binary PLY prior map |
| `GLIM_PRIOR_MAP_TIGHTLY_COUPLED` | false | insert scan-to-map factors into GLIM's fixed-lag graph |
| `GLIM_PRIOR_MAP_FACTOR_CORESET_SIZE` | 32 | exact quadratic coreset target size |
| `GLIM_PRIOR_MAP_FACTOR_MAX_CORRESPONDENCE_M` | 2.0 | map-factor correspondence gate |
| `GLIM_PRIOR_MAP_FACTOR_MIN_OVERLAP_FRACTION` | 0.05 | minimum per-scan map overlap |
| `GLIM_PRIOR_MAP_FACTOR_NUM_THREADS` | 1 | prior-map factor worker count |
| `GLIM_PRIOR_MAP_RECOVERY_CONFIRMATION_FRAMES` | 3 | consistent frames required for recovery |
| `GLIM_PRIOR_MAP_PUBLIC_MAX_TRANSLATION_STEP_M` | 0.25 | public TF translation bound per update |
| `GLIM_PRIOR_MAP_PUBLIC_MAX_ROTATION_STEP_DEG` | 2.0 | public TF rotation bound per update |
| `GLIM_PRIOR_MAP_VOXEL_SIZE_M` | 0.75 | KISS-Matcher resolution |
| `GLIM_PRIOR_MAP_MIN_INLIERS` | 10 | minimum KISS inliers for coarse fallback |
| `GLIM_PRIOR_MAP_BOOTSTRAP_CENTER_X/Y/Z` | unset | approximate startup position in map frame |
| `GLIM_PRIOR_MAP_BOOTSTRAP_YAW_DEG` | unset | approximate startup yaw |
| `GLIM_PRIOR_MAP_BOOTSTRAP_CROP_RADIUS_M` | 60.0 | acquisition crop radius |
| `GLIM_PRIOR_MAP_TRACKING_CROP_RADIUS_M` | 35.0 | tracking crop radius |
| `GLIM_PRIOR_MAP_BOOTSTRAP_SUBMAP_STRIDE` | 2 | acquisition match interval |
| `GLIM_PRIOR_MAP_TRACKING_SUBMAP_STRIDE` | 10 | tracking match interval |
| `GLIM_PRIOR_MAP_TRANSLATION_GAIN` | 0.2 | accepted displacement gain from first anchor |
| `GLIM_PRIOR_MAP_VERTICAL_GAIN` | translation gain | separate accepted/pending Z gain |
| `GLIM_PRIOR_MAP_MAX_LOCAL_TRANSLATION_M` | 1.5 | tracking correction bound |
| `GLIM_PRIOR_MAP_MAX_LOCAL_YAW_DEG` | 10.0 | full 3D rotation bound (legacy name) |
| `GLIM_PRIOR_MAP_ALLOW_GLOBAL_RELOCALIZATION` | true | enable two-submap recovery consensus |
| `GLIM_PRIOR_MAP_MAX_GLOBAL_DISAGREEMENT_M` | 2.0 | recovery translation agreement bound |
| `GLIM_PRIOR_MAP_MAX_GLOBAL_DISAGREEMENT_YAW_DEG` | 10.0 | recovery rotation agreement bound |
| `GLIM_PRIOR_MAP_FULL_GLOBAL_SEARCH` | false | experimental unseeded full-map KISS search |
| `GLIM_PRIOR_MAP_VGICP_VOXEL_RESOLUTION_M` | 1.0 | prior-map VGICP voxel size |
| `GLIM_PRIOR_MAP_VGICP_MAX_ITERATIONS` | 10 | refinement iterations |
| `GLIM_PRIOR_MAP_VGICP_MIN_INLIER_FRACTION` | 0.50 | minimum overlap |
| `GLIM_PRIOR_MAP_VGICP_MAX_NORMALIZED_ERROR` | 5.0 | maximum error per inlier |

In legacy split mode only, GLIM ROS parameter
`glim_ros.external_map_odom_time_constant` controls smoothing and defaults to 2.0 s.

## Legacy split-mode measurements

The results below predate the tightly coupled graph and are retained as the split-mode
baseline. All four outdoor-hard bags were replayed with the exact quadratic coreset
(size 32).
The original policy passed three bags, while `outdoor_hard_02a` exceeded the 2.0 m ATE
limit in both repeats. A targeted 02a variant with vertical gain 1.0 then passed two
independent full repeats without changing the horizontal gain or acceptance gates.

| Bag | Coverage | Translation ATE | Final error | Median 10 m RPE | Rotation ATE | Processing p95 | Gate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `outdoor_hard_01a` | 99.21% | 1.142 m | 3.112 m | 0.157 m | 1.479 deg | 28.3 ms | Pass |
| `outdoor_hard_01b` | 99.04% | 1.539 m | 2.055 m | 0.191 m | 1.598 deg | 53.8 ms | Pass |
| `outdoor_hard_02a` baseline repeat 1 | 99.15% | 2.059 m | 3.078 m | 0.180 m | 1.676 deg | 35.8 ms | ATE fail |
| `outdoor_hard_02a` baseline repeat 2 | 99.15% | 2.023 m | 3.049 m | 0.183 m | 1.715 deg | 37.2 ms | ATE fail |
| `outdoor_hard_02a` Z-bridge repeat 1 | 99.15% | 1.926 m | 2.854 m | 0.181 m | 1.683 deg | 38.5 ms | Pass |
| `outdoor_hard_02a` Z-bridge repeat 2 | 99.15% | 1.795 m | 2.827 m | 0.181 m | 1.703 deg | 30.4 ms | Pass |
| `outdoor_hard_02b` | 99.03% | 0.908 m | 1.412 m | 0.177 m | 1.765 deg | 47.9 ms | Pass |

Every run had zero TF jumps and zero unauthorized resets. Registration stopped being
accepted during each sequence, after which the last map correction stayed frozen while
GLIM odometry carried the output. The maximum translation jump was 0.189 m on 01a and
at most 0.017 m on the other measured runs. In the Z-bridge repeats, submap 30 entered
global consensus and updated only Z; no global XY candidate was promoted. Both planar
runs passed, but repeat 2 still measured 9.21 m non-planar 3D ATE and 14.29 m final
error. Full 3D drift and kidnapped-pose recovery therefore remain unverified. The
authoritative recorded values are in [`results.json`](results.json).

A follow-up four-bag regression used vertical gain 1.0 everywhere. 01a passed at
1.106 m ATE, 02b passed at 1.097 m, and the two previously recorded 02a repeats passed
at 1.926 m and 1.795 m. 01b passed one repeat at 1.867 m and failed one at 2.219 m.
Cross-composition showed that failed repeat remained at 2.210 m with the baseline map
anchor, whereas baseline raw odometry remained passing at 1.562 m with the Z-bridge
anchor. The variance therefore came primarily from raw GLIM odometry, not the anchor,
but the Z bridge remains opt-in until repeatability and non-planar drift improve.

The raw 01b variance was reproduced from the first scan and traced to parallel ordering
in random-grid preprocessing and the odometry optimizer. Setting both thread counts to
one made two full 302 s raw odometry dumps byte-identical. Both full outputs passed at
1.407079 m ATE, 1.957 m final error, and 0.187 m median 10 m RPE; processing p95 was
44.3 ms and 47.5 ms. Prior-map VGICP still produced a small run-to-run Z-anchor
difference, but it changed planar ATE by less than one micrometre.

The converter composes the canonical raw GLIM poses with the recorded live
`external_map_odom.txt` history. This evaluates the transform actually published by
the live policy and avoids introducing a second offline correction model.

## Reproduce a measured run

```bash
python3 scripts/run_koide_glim_odometry_benchmark.py \
  --bag "$DATA/sequences/outdoor_hard_01a" \
  --reference "$DATA/benchmark/outdoor_hard_01a/reference.csv" \
  --prior-map "$DATA/map_outdoor_hard.ply" \
  --prior-map-bootstrap-center -86.040205 -8.857126 -11.043077 \
  --prior-map-bootstrap-yaw-deg -82.0063 \
  --prior-map-tightly-coupled \
  --tightly-coupled-num-threads 8 \
  --image lidarloc/glim-ros2:jazzy-v1.2.2-tightly-coupled \
  --allow-image-mismatch \
  --output /tmp/glil_outdoor_hard_01a
```

Do not combine `--prior-map-vertical-gain` with tightly coupled mode. Sparse verified
graph epochs already use the configured horizontal gain and full vertical observation.

### Reproduce the startup recovery integration test

Append the following options to the 01a command above. The pose is an independently
generated map-frame candidate for the first stabilized fixed-lag window. The delay is
wall-clock time and is specific to the measured one-thread container profile.

```bash
  --tightly-coupled-num-threads 1 \
  --playback-duration-sec 15 \
  --requested-duration-sec 15 \
  --inject-initial-pose \
    -86.775441 -8.903109 -11.123637 \
    -0.006297 -0.009488 -0.716092 0.697913 \
  --initial-pose-delay-sec 6.5
```

The accepted smoke run validated frames 28--30 consecutively and logged
`activated independently verified recovery generation=1 as map state m1`. An
inconsistent candidate was rejected after three frames without creating a map state.
Both cases retained zero TF jumps and zero unauthorized resets. Full recovery latency
and throughput are measured separately on the kidnap bags.

## Remaining acceptance work

1. Record clean full 01a/01b/02a/02b tightly coupled replays after competing CPU jobs
   finish, including CPU/RSS and non-planar metrics.
2. Run the indoor and outdoor kidnap bags plus an injected-displacement recovery case.
3. Pin and publish the updated GLIM/GLIM ROS forks and benchmark image.
4. Regenerate the GLIL-only gallery GIFs from the accepted runs.
