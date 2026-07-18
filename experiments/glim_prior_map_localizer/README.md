# GLIM prior-map localizer experiment

This experiment makes prior-map localization part of GLIM's global factor
graph. It is intended to replace continuously running NDT, not GLIM odometry.

## Design

The extension keeps GLIM as the continuous LiDAR/IMU odometry front end. At a
sparse submap interval it crops the prior PLY map around GLIM's prediction and
uses that prediction to initialize GLIM's own CPU VGICP implementation. Normal
tracking does not run KISS-Matcher. KISS is used only to acquire an unknown
frame or as a fallback after direct VGICP loses lock. Accepted registrations
are inserted as fixed-map `IntegratedVGICPFactor`s, matching GLIM's native
global-mapping formulation rather than converting a coarse match into an
arbitrarily strong pose prior.

When an approximate startup pose is supplied, the extension transforms GLIM's
X(0) initial value into the map frame before its first iSAM2 update. GLIM's own
gauge-fixing damping factor then anchors the correct frame from the beginning;
later map factors are bounded refinements, not an ill-conditioned 80+ metre
late graph shift.

The policy has two modes:

- Tracking: accept a well-supported pose only when its full 3D correction from
  GLIM's prediction is bounded.
- Recovery: a large correction is not accepted until registrations from two
  independent GLIM submaps produce consistent map-from-GLIM-world anchors.
  In a seeded tracking crop this supports reattachment after local lock loss;
  unseeded recovery still requires explicit full-map mode.

Full-map KISS alone is not yet accepted as a production kidnap-recovery path;
the measured unseeded replay found a repeatable structural ambiguity, so this
remains an experimental opt-in.

No ground truth is used at runtime. Ground truth or a previously saved GLIM
pose may be used only by the offline evaluator. Tracking uses a local prior-map
crop and runs at submap rate, not scan rate. Full-map KISS search is
experimental and disabled by default; an unseeded saved-submap search took
29.55 seconds and repeated a structurally ambiguous match.

The extension uses GLIM's documented global-mapping callbacks rather than
patching GLIM itself. Relevant primary sources:

- [GLIM paper](https://arxiv.org/abs/2407.10344)
- [GLIM extension API](https://github.com/koide3/glim/blob/master/docs/extend.md)
- [KISS-Matcher paper](https://arxiv.org/abs/2409.15615)
- [KISS-Matcher implementation](https://github.com/MIT-SPARK/KISS-Matcher)

## Build and offline check

    docker build -t lidarloc/glim-ros2:jazzy-v1.2.2-prior-map \
      experiments/glim_prior_map_localizer

    docker run --rm \
      -v /path/to/glim_dump:/dump:ro \
      -v /path/to/map.ply:/map.ply:ro \
      lidarloc/glim-ros2:jazzy-v1.2.2-prior-map \
      /usr/local/bin/glim_prior_map_offline_match /dump/000050 /map.ply 0.5

The offline output is one JSON object containing KISS validity/inliers, VGICP
overlap/error, runtime, the refined matrix, and its difference from the pose
stored in the GLIM dump. Add `direct` after the crop radius to initialize VGICP
from the stored GLIM pose instead of the KISS pose. The stored-pose error is an
evaluation diagnostic, not a runtime gate.

## Runtime configuration

Add libglim_prior_map_localizer.so to extension_modules in GLIM's
config_ros.json, mount the prior map, and set GLIM_PRIOR_MAP_PATH.

Important environment variables and defaults:

| Variable | Default | Meaning |
| --- | ---: | --- |
| GLIM_PRIOR_MAP_PATH | required | ASCII or little-endian binary PLY prior map |
| GLIM_PRIOR_MAP_VOXEL_SIZE_M | 0.75 | KISS-Matcher resolution |
| GLIM_PRIOR_MAP_MIN_INLIERS | 10 | minimum KISS inliers when coarse fallback is used |
| GLIM_PRIOR_MAP_BOOTSTRAP_CROP_RADIUS_M | 60.0 | pre-alignment search radius |
| GLIM_PRIOR_MAP_BOOTSTRAP_CENTER_X/Y/Z | unset | approximate initial map position |
| GLIM_PRIOR_MAP_BOOTSTRAP_YAW_DEG | unset | approximate initial map yaw |
| GLIM_PRIOR_MAP_MAX_BOOTSTRAP_TRANSLATION_M | 10.0 | seeded-candidate correction bound |
| GLIM_PRIOR_MAP_TRACKING_CROP_RADIUS_M | 35.0 | local tracking-map radius |
| GLIM_PRIOR_MAP_BOOTSTRAP_SUBMAP_STRIDE | 2 | pre-alignment match interval |
| GLIM_PRIOR_MAP_TRACKING_SUBMAP_STRIDE | 10 | post-alignment match interval |
| GLIM_PRIOR_MAP_FULL_GLOBAL_SEARCH | false | experimental full-map mode |
| GLIM_PRIOR_MAP_MAX_LOCAL_TRANSLATION_M | 1.5 | single-submap tracking correction bound |
| GLIM_PRIOR_MAP_MAX_LOCAL_YAW_DEG | 10.0 | full 3D rotation bound (legacy name) |
| GLIM_PRIOR_MAP_ALLOW_GLOBAL_RELOCALIZATION | true | enable two-submap recovery |
| GLIM_PRIOR_MAP_MAX_GLOBAL_DISAGREEMENT_M | 2.0 | recovery-anchor agreement |
| GLIM_PRIOR_MAP_MAX_GLOBAL_DISAGREEMENT_YAW_DEG | 10.0 | 3D rotation agreement |
| GLIM_PRIOR_MAP_VGICP_VOXEL_RESOLUTION_M | 1.0 | prior-map VGICP voxel size |
| GLIM_PRIOR_MAP_VGICP_MAX_ITERATIONS | 10 | refinement iterations |
| GLIM_PRIOR_MAP_VGICP_MIN_INLIER_FRACTION | 0.50 | minimum overlap (same as GLIM pose-graph default) |
| GLIM_PRIOR_MAP_VGICP_MAX_NORMALIZED_ERROR | 5.0 | maximum VGICP error per inlier |
| GLIM_PRIOR_MAP_VGICP_MAX_COARSE_DELTA_M | 2.0 | maximum KISS-to-VGICP translation change |
| GLIM_PRIOR_MAP_VGICP_MAX_COARSE_DELTA_DEG | 5.0 | maximum KISS-to-VGICP rotation change |

## Measured status

On `outdoor_hard_01a` (380 s), the direct-VGICP plus two-submap consensus
replay achieved 99.21% coverage, 2.196 m translation ATE, 4.434 m end error,
0.313 m median 10 m translation RPE, 46.9 ms processing p95, and no TF jumps.
It improves the earlier coarse-KISS pose-prior prototype (3.379 m ATE, 7.668 m
end error) and demonstrated a real consensus reattachment at submaps 30/40,
followed by normal tracking through submap 80.

It is not yet a production win: the GLIM coreset baseline remains better at
1.696 m ATE and 0.182 m RPE. The remaining issue is that global-map factors
alter the corrected trajectory even though GLIM's local odometry is already
more accurate. The next design gate is therefore a separately smoothed
`map -> odom` correction output that never deforms GLIM odometry. Do not use
this experiment to replace the validated localizer yet.

## Acceptance gates

Before enabling this in measured replays:

1. Compile and unit-test the factor acceptance policy.
2. Measure registration success, error, and wall time on saved submaps from all
   four Koide sequences, including false-match and kidnapped-pose cases.
3. Run a 60-second GLIM-only replay and verify factor insertion and TF
   continuity.
4. Run all full sequences, compare ATE/recovery with the current hybrid, and
   measure whole-process CPU. Do not remove NDT until these gates pass.
