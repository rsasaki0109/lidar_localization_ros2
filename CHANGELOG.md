# Changelog

## Unreleased

### Added

- `scripts/run_public_demo.sh` for the 30-minute Autoware Istanbul public demo path
- `scripts/run_public_validation_dashboard.sh` for Markdown/HTML public validation dashboards
- `param/benchmark/v1_1_boreas_dry_run_endpoint.example.yaml` for the v1.1 MVP dry-run endpoint chain
- `scripts/run_v1_1_relocalization_smoke.sh` to guard the v1.1 claim boundary

### Reliability / Claim Boundary

- add `docs/frame_contract.md` for `map` / `odom` / `base_link` expectations and issue #58 / #27 guidance
- publish TF immediately on `/initialpose` and add explicit frame/map diagnostics for issue #47
- add `docs/troubleshooting.md` for bringup failures and `/alignment_status` interpretation (issues #70, #35, #43, #48)
- add `docs/map_alignment.md` for map frame / initial pose alignment (issues #75, #44)
- add `docs/pose_covariance.md` for `/pcl_pose` covariance semantics (issue #72)
- document Istanbul public demo RMSE run variance; identical seed/map can yield ~`1.2 m` or late-run drift outliers
- guard `use_odom` integration until initial pose is valid and keep pose finite for NDT init guess
- ignore non-finite `/initialpose` payloads and stop eager scan replay during pose reset
- v1.1 relocalization is documented as a validated dry-run `/initialpose` command artifact only
- guarded reset publication and post-reset observation remain experimental and outside the public MVP endpoint
- automatic runtime recovery and production-grade global relocalization are explicitly out of scope

### ROS 2 Distro Support

- README badges and support matrix now show **Jazzy-first / Humble-compatible**
- GitHub Actions workflow updated to `actions/checkout@v4` and `push` on `main`
- Jazzy CI still builds against `ndt_omp_ros2` `humble` branch until a dedicated Jazzy branch exists upstream

## 1.0.0 - 2026-03-30

Initial repo-ready Nav2-focused release.

- added Nav2-oriented launch flows with `nav2_lidar_localization.launch.py` and `nav2_navigation.launch.py`
- added smoke and replay helpers including `run_nav2_demo_smoke`, `run_nav2_replay_smoke`, and `send_nav2_goal.py`
- added occupancy map generation from PCD with `generate_occupancy_map_from_pcd.py`
- added benchmarking and comparison helpers for rosbag-based localization evaluation
- added local-map crop, twist-based prediction, borderline seed rejection gate, diagnostics, and replay-oriented utilities
- fixed `NDT_OMP` runtime keep-alive issues seen in replay and fixed the shutdown-only `SIGINT` crash path
- set the recommended Nav2 preset to `param/nav2_ndt_urban.yaml`

Known limitation at this release:

- long-duration urban replay robustness is improved enough for smoke and controlled validation, but not yet solved well enough to claim fully stable long-horizon operation in all cases
