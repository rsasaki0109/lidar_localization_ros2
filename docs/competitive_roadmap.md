# Competitive Roadmap

Last updated: 2026-05-25

This document defines the competitive direction for `lidar_localization_ros2`. It is intentionally
shorter than the benchmark logs: use it to decide what to build next, not to replay every experiment.

## Mission

Make `lidar_localization_ros2` a practical open-source ROS 2 package for 3D LiDAR map-based
localization that is easy to launch, easy to benchmark, and honest about its validated limits.

## Current Position

What is already in place:

- ROS 2 LiDAR localization node with lifecycle support
- Nav2 launch helpers for localization-only and full navigation bringup
- recommended Nav2 preset: `param/nav2_ndt_urban.yaml`
- selectable backends: `NDT`, `GICP`, `NDT_OMP`, `GICP_OMP`, `SMALL_GICP`, `SMALL_VGICP`
- local-map crop, guarded update logic, recovery diagnostics, and reinitialization request output
- public replay/regression scripts and rosbag benchmark tooling
- experimental artifact-first relocalization pipeline for validated dry-run `/initialpose` commands

Current validated public boundary:

- release regression snapshot: `2026-05-22`, commit `2a5f11f`, `overall_pass=true`
- Istanbul `60 s` no-IMU safety check: `1.176 m` translation RMSE, `0.393 deg` rotation RMSE
- HDL `60 s` IMU safety check, two-repeat median: pose rows `558.5 -> 553.5`
- Nav2 reinitialization supervisor `150 s`: requested rows `944 -> 7`

Current limits:

- Istanbul is not the main benchmark dataset for IMU or production robustness claims
- long-horizon public `LiDAR + IMU + GT` evaluation still needs a controlled dataset track
- real Jetson + MID-360 hardware behavior needs separate sensor, thermal, vibration, and extrinsics validation
- global relocalization is not production-ready; v1.1 is an artifact-first dry-run evaluation path
- covariance semantics are still not strong enough to be a main competitive claim

See [v1_status.md](v1_status.md), [public_validation_log.md](public_validation_log.md), and
[v1_1_relocalization.md](v1_1_relocalization.md) for the detailed validation boundary.

## Competitors

Direct competitors:

| System | Why it matters | Main gap to close |
|---|---|---|
| Autoware `autoware_ndt_scan_matcher` | Most relevant ROS 2 production localizer | richer diagnostics, covariance, map loading, production workflows |
| `koide3/hdl_localization` | Mature LiDAR localization baseline | runtime robustness, UKF-style estimation, relocalization hooks |
| `koide3/hdl_global_localization` | Direct reference for global relocalization | BBS / FPFH / RANSAC / TEASER-style global search |

Technology references:

| System | Why it matters |
|---|---|
| `koide3/small_gicp` | CPU registration backend quality and speed reference |
| `koide3/fast_gicp` | VGICP / CUDA / NDTCuda performance reference |
| `koide3/glim` | stronger estimation-stack and factor-graph design reference |
| `NVIDIA-ISAAC-ROS/isaac_ros_map_localization` | GPU localization and relocalization UX reference |

## Strategy

Do not try to beat every competitor at once. The near-term competitive angle is:

- simpler ROS 2 integration than heavyweight stacks
- reproducible public benchmark commands
- conservative Nav2 defaults with clear failure signals
- fast iteration on recovery and relocalization through artifacts before runtime claims

The project should avoid claims that are not backed by repeatable public artifacts.

## Dataset Strategy

Different datasets have different jobs:

| Dataset | Current role | Next action |
|---|---|---|
| Autoware Istanbul | no-IMU urban replay and Nav2 regression guard | keep as safety/regression only |
| HDL sample | public IMU smoke and throughput guard | keep as an IMU pipeline safety check, not final ranking |
| Boreas | public `LiDAR + IMU + GT` candidate, currently diagnostic only | fix prediction/map-split behavior before using it for ranking |
| Koide hard localization | next controlled public benchmark candidate; local indoor `60 s` and outdoor `120 s` runs pass; outdoor `180 s` has a reproducible relocalization request window and an oracle-free route-proximity reset artifact | validate route-proximity ordering beyond the single failure window, then rank backends without making IMU claims until calibration is controlled |

Istanbul should stay in the release suite, but it should not drive the next main research direction.

## Roadmap

### Now

1. Keep the release regression green.
2. Make README and docs user-friendly enough that a new user can find the right command quickly.
3. Pick and validate the next real benchmark track from Boreas or Koide-style public data.
4. Close v1.1 around the dry-run relocalization command artifact, not automatic reset publication.
5. Keep Istanbul as a no-IMU/Nav2 regression guard, not the main benchmark target.
6. Keep `param/nav2_ndt_urban.yaml` conservative.

Done when:

- `run_release_regression_suite.sh` remains the release boundary
- docs point users to one clear command per workflow
- Boreas or Koide has a clear map/reference/initial-pose workflow for backend comparison
- v1.1 wording stays limited to artifact-first relocalization evaluation

### Next

1. Strengthen measurement acceptance beyond scalar fitness score.
2. Improve recovery diagnostics so drift can be detected before a large pose error accumulates.
3. Validate the Koide `outdoor_hard_01a` route-proximity relocalization artifact beyond the single `180 s` request window.
4. Compare `NDT_OMP`, `SMALL_GICP`, and `SMALL_VGICP` on Koide after that failure boundary has a controlled recovery story.
5. Start a runtime relocalization prototype only after the artifact-first path is stable.
6. Define covariance output semantics that downstream consumers can trust.

Done when:

- default backend and recovery settings are justified by public data
- diagnostics distinguish bad match, missing map/initial pose, weak overlap, stale prediction, and overload
- covariance is documented enough to be consumed by fusion or arbitration logic

### Later

1. Add dynamic map loading or tile/submap support.
2. Evaluate GPU or accelerated backends only after CPU behavior is well characterized.
3. Turn guarded relocalization publication into a controlled runtime feature.
4. Add hardware-focused validation for Jetson + MID-360 and other robot-specific integrations.

Done when:

- city-scale map size is not a startup or memory blocker
- runtime relocalization has safe trigger, candidate, scoring, publish, and post-reset validation rules
- hardware validation is measured separately from public replay validation

## Decision Gates

Backend default changes require:

- at least two public datasets
- same bag, map, reference, and evaluation command
- no catastrophic failure on either dataset
- median or p95 alignment time improvement, or a clear robustness improvement
- RMSE and matched-sample coverage not materially worse

Relocalization runtime claims require:

- candidate generation without reference-oracle ordering
- registration scoring from runtime-available inputs
- reset publication guarded by explicit state and validation checks
- post-reset recovery evidence
- a regression test that fails on unsafe publication or false acceptance

Nav2 default changes require:

- demo smoke still passes
- replay smoke still passes
- release regression still passes
- no regression in reinitialization request handling

## Benchmark Claim Rules

Safe claims:

- public replay regression passes for the documented boundary
- Nav2 launch and replay smoke paths are validated
- Istanbul is a no-IMU/Nav2 regression check
- v1.1 can produce validated dry-run reset command artifacts
- MID-360 bringup path is launch/config/build oriented unless hardware data is provided

Unsafe claims:

- long-horizon urban replay is solved
- Istanbul proves IMU or production robustness
- global relocalization is production-ready
- automatic reset publication is part of the default public path
- `SMALL_GICP` or `SMALL_VGICP` is default-ready without current public comparisons
- private or NC dataset results are open benchmark results

## Recommended Order

1. Documentation and branch hygiene
2. Release regression maintenance
3. Boreas or Koide benchmark-track selection
4. Artifact-first relocalization v1.1 closeout
5. Backend comparison on public datasets
6. Covariance and richer diagnostics
7. Runtime relocalization
8. Dynamic map loading
9. Hardware validation

## Useful Links

- [benchmarking.md](benchmarking.md): benchmark and regression commands
- [v1_status.md](v1_status.md): validated scope and known limits
- [v1_1_relocalization.md](v1_1_relocalization.md): relocalization MVP boundary
- [public_validation_log.md](public_validation_log.md): recorded public validation snapshots
- [mid360_legged_jetson.md](mid360_legged_jetson.md): MID-360 legged robot bringup notes

## License Notes

This repository is BSD-2-Clause. Prefer Apache-2.0, BSD, and MIT source projects for direct
adaptation. Avoid copying GPL code into the core package.
