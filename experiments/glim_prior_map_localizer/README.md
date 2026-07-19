# GLIL-style GLIM prior-map localizer

This experiment keeps GLIM LiDAR/IMU odometry continuous and applies prior-map
localization only through a smoothed `map -> odom` transform. It replaces continuously
running NDT; it does not deform GLIM's factor graph or rewrite the raw odometry.

## Architecture

GLIM produces `odom -> base_link` at scan rate. At a sparse submap interval, this
extension crops the prior PLY map around the current prediction and runs CPU VGICP.
Normal tracking uses direct VGICP; KISS-Matcher is invoked only for initial acquisition
or after direct VGICP rejects the candidate. A large recovery correction requires two
independent submaps to agree before it is accepted.

An accepted registration is converted into a `map -> odom` observation. The output:

- keeps the configured startup rotation fixed;
- defines its translation reference from the first accepted observation;
- applies 20% of each later translation displacement from that reference by default;
- optionally uses a separate vertical gain and, during global-consensus pending, updates
  only Z while keeping the last accepted XY and rotation;
- transitions with a 2 s first-order smoother in GLIM ROS; and
- freezes and re-stamps the last valid transform while map matching is rejected.

No ground truth is used at runtime. Ground truth is used only by the evaluator and GIF
renderer. Full-map KISS search remains experimental and is disabled by default because
the unseeded outdoor replay found a repeatable structural ambiguity.

The callback and ROS consumer are maintained in two forks:

- [GLIL unofficial core callback](https://github.com/rsasaki0109/glil_unofficial/commit/dd29667938cb56ef59de7f90290ad2dbc613f00c)
- [GLIM ROS external map-to-odom consumer](https://github.com/rsasaki0109/glim_ros2/commit/cd4b2c9eb37a5c12c93d7339ce10167ebaa55288)

The Dockerfile pins both commits. The design follows the
[GLIM paper](https://arxiv.org/abs/2407.10344),
[GLIM extension API](https://github.com/koide3/glim/blob/master/docs/extend.md), and
[KISS-Matcher paper](https://arxiv.org/abs/2409.15615).

## Build

```bash
docker build -t lidarloc/glim-ros2:jazzy-v1.2.2-live-map-odom \
  experiments/glim_prior_map_localizer
```

The build compiles and tests the GLIM ROS smoother before installing the extension.

## Runtime configuration

The benchmark runner adds `libglim_prior_map_localizer.so` to GLIM's
`extension_modules`. Important environment variables are:

| Variable | Default | Meaning |
| --- | ---: | --- |
| `GLIM_PRIOR_MAP_PATH` | required | ASCII or little-endian binary PLY prior map |
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

GLIM ROS parameter `glim_ros.external_map_odom_time_constant` controls smoothing and
defaults to 2.0 s.

## Measured full replays

All four outdoor-hard bags were replayed with the exact quadratic coreset (size 32).
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
  --config param/odometry/glim_koide_outdoor_gicp6500_coreset \
  --image lidarloc/glim-ros2:jazzy-v1.2.2-live-map-odom \
  --output /tmp/glil_outdoor_hard_01a
```

Add `--prior-map-vertical-gain 1.0` for the measured 02a Z-bridge policy.

## Remaining acceptance work

1. Resolve the remaining non-planar Z drift and the 01b raw-odometry run-to-run variance
   before making the vertical policy the default.
2. Measure seeded and unseeded false-match and kidnapped-pose recovery cases.
3. Confirm that two-submap recovery reacquires a deliberately displaced pose without
   discontinuity or a false map anchor.
4. Compare whole-process CPU and accuracy with the validated GLIM+NDT bridge before
   changing the default localizer.
