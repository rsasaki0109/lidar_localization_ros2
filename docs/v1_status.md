# v1 Status

This document defines what `v1.0.0` means for this repository: what is included, what has been
validated, and what should not be claimed yet.

## Included

`v1.0.0` includes:

- ROS 2 LiDAR localization node with lifecycle support
- localization-only and full Nav2 launch files
- recommended Nav2 preset: `param/nav2_ndt_urban.yaml`
- selectable registration backends: `NDT`, `GICP`, `NDT_OMP`, `GICP_OMP`, `SMALL_GICP`, `SMALL_VGICP`
- pointcloud-map loading from `.pcd` and `.ply`
- local-map crop for large pointcloud maps
- odom, twist, and guarded IMU-preintegration prediction paths
- alignment diagnostics on `/alignment_status`
- reinitialization request output on `/reinitialization_requested`
- optional external supervisor that can republish `/initialpose` from the Nav2 wrapper
- rosbag benchmark, replay smoke, public regression, and release regression helpers
- artifact-first experimental relocalization tools for dry-run reset command evaluation
- MID-360 legged robot bringup launch/config/check helpers

## Validated Scope

The v1 packaging pass validated these paths in this workspace:

- package build through `colcon build --symlink-install --packages-up-to lidar_localization_ros2`
- localizer launch and lifecycle activation
- Nav2 wrapper launch argument resolution
- self-contained Nav2 demo smoke using the built-in odom/localization demo path
- replay smoke using the real localizer helper path
- public regression suite entry point
- reinitialization supervisor regression entry point
- release regression entry point
- experiment suite entry point
- mapless-public dataset scaffolding entry point
- MID-360 launch/config/build path

Use [../README.md](../README.md) for the short command list and [benchmarking.md](benchmarking.md)
for benchmark and regression commands.

## Current Public Snapshot

Latest recorded public validation:

- date: `2026-05-22`
- commit: `2a5f11f`
- release regression: `overall_pass=true`
- Istanbul `60 s` no-IMU safety check: `1.176 m` translation RMSE, `0.393 deg` rotation RMSE
- HDL `60 s` IMU safety check, two-repeat median: pose rows `558.5 -> 553.5`
- Nav2 reinitialization supervisor `150 s`: requested rows `944 -> 7`

This is public replay and controlled Nav2 regression validation. It is not a Jetson + MID-360
hardware result.

See [public_validation_log.md](public_validation_log.md) for the recorded snapshot history.

## Recommended Preset

Use `param/nav2_ndt_urban.yaml` unless you are intentionally running an experiment.

Rationale:

- it is the most conservative tested Nav2-facing preset
- more aggressive recovery presets improved some intermediate diagnostics but regressed longer replay behavior
- it keeps `/reinitialization_requested` observable without making automatic reset publication a core runtime claim
- it is covered by the current release regression path

Experimental presets should stay experimental until they win on public replay and do not regress
Nav2 smoke or release regression behavior.

## Safe Claims

It is reasonable to say:

- the package provides ROS 2 map-based 3D LiDAR localization with Nav2 launch helpers
- the recommended preset has public replay and smoke validation within the documented boundary
- public regression and release regression scripts exist for repeatable checks
- Istanbul is used as a no-IMU/Nav2 regression guard, not as the main IMU benchmark
- the project has an artifact-first relocalization evaluation pipeline
- MID-360 support is a launch/config/build-oriented bringup path

## Known Limits

- Long-horizon robustness on stronger public `LiDAR + IMU + GT` data is not established yet.
- Istanbul has no IMU stream for IMU-preintegration claims and should not be treated as the main benchmark dataset.
- Smoke and replay success do not replace real-robot validation with real odom, real TF, measured extrinsics, vibration, and thermal conditions.
- The current relocalization work is not production-grade global relocalization.
- The guarded reset publisher is experimental and opt-in.
- Occupancy-map generation from PCD is route-crop oriented and is not a general mapper.
- Covariance semantics are not strong enough yet to be a headline downstream-fusion claim.

## Do Not Claim

Do not claim:

- universal recovery from prolonged reject streaks
- production-grade long-horizon urban operation
- production-ready global relocalization
- automatic reset publication in the default public benchmark path
- `SMALL_GICP` or `SMALL_VGICP` as the default-ready backend without current public comparisons
- Jetson + MID-360 hardware validation unless real hardware evidence is provided

## v1.1 MVP Endpoint

v1.1 is closed around one public endpoint:

> validated dry-run `/initialpose` command artifact derived from NDT_OMP-scored route-grid candidates,
> with `published_count=0` in the default benchmark path

Starter manifests:

- `param/benchmark/v1_1_boreas_localizer_only.example.yaml` — health / candidate scaffolding only
- `param/benchmark/v1_1_boreas_dry_run_endpoint.example.yaml` — full no-publish MVP chain

Smoke check:

```bash
source scripts/setup_local_env.sh
scripts/run_v1_1_relocalization_smoke.sh
```

Outside the v1.1 MVP endpoint:

- guarded publisher (`publish_relocalization_reset_commands.py --execute`)
- post-reset observation as an accepted-recovery claim
- automatic runtime recovery

## Next Work

The next technical work should focus on:

- keeping release regression green
- promoting a stronger public benchmark track based on Boreas or Koide-style data
- comparing backends on public datasets before changing defaults
- strengthening diagnostics and covariance semantics
