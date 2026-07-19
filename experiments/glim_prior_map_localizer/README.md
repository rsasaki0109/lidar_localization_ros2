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

## Measured full replay

The live architecture passed every current completion gate on the full 380 s
`outdoor_hard_01a` replay using the exact quadratic coreset (size 32):

| Metric | Result |
| --- | ---: |
| Output coverage | 99.21% |
| Translation ATE RMSE | 1.142 m |
| Final translation error | 3.112 m |
| Median 10 m translation RPE | 0.157 m |
| Rotation ATE RMSE | 1.479 deg |
| Median 10 m rotation RPE | 0.676 deg |
| Processing p95 | 28.3 ms |
| Maximum pose gap | 0.100 s |
| Maximum translation jump | 0.189 m |
| TF jumps / unauthorized resets | 0 / 0 |

Map registration was accepted through submap 60. Later candidates were rejected or
had no points in the crop; the map correction then stayed frozen while GLIM odometry
continued to provide a smooth output. This proves the separation and failure behavior,
but it is not yet evidence for the other three outdoor-hard bags or kidnapped-pose
recovery. The authoritative recorded values are in
[`results.json`](results.json).

The converter composes the canonical raw GLIM poses with the recorded live
`external_map_odom.txt` history. This evaluates the transform actually published by
the live policy and avoids introducing a second offline correction model.

## Reproduce the measured run

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

## Remaining acceptance work

1. Repeat full replays for `outdoor_hard_01b`, `outdoor_hard_02a`, and
   `outdoor_hard_02b`.
2. Measure seeded and unseeded false-match and kidnapped-pose recovery cases.
3. Confirm that two-submap recovery reacquires a deliberately displaced pose without
   discontinuity or a false map anchor.
4. Compare whole-process CPU and accuracy with the validated GLIM+NDT bridge before
   changing the default localizer.
