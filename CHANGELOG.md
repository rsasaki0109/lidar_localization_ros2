# Changelog

## Unreleased

### Added

- `failure_category` key on `/alignment_status` distinguishing
  `missing_map` / `missing_initial_pose` / `weak_overlap` / `bad_match` /
  `stale_prediction` / `overload` / `healthy`, with `*_active` companion flags
  and three `diagnostics_*` tuning parameters (docs: `troubleshooting.md`)
- `scripts/analyze_pose_covariance_calibration.py` to calibrate `/pcl_pose`
  covariance against ground-truth replays
- G2 runtime global localization service: `scripts/global_localization_node.py`
  answers `std_srvs/Trigger` queries on `~/query` with map-wide BBS_2D
  candidates on `~/candidates` (opt-in node; default launch unchanged), with
  ROS-free query logic in `scripts/global_localization_query.py`
- `scripts/render_global_localization_demo_gif.py` renders the
  kidnapped-start -> global localization -> tracking-resume demo GIF
- `scripts/diagnose_local_map_crop_coverage.py` decides offline whether a Boreas
  `local_map_crop_too_small` cliff is a map-coverage problem or prediction-driven
  divergence, by counting map points within `local_map_radius` of every
  ground-truth pose (no ROS / no localizer); core unit-tested in
  `test/test_local_map_crop_coverage.py`
- G3 guarded automatic reinitialization: `scripts/reinitialization_supervisor_node.py`
  connects `/reinitialization_requested` to the G2 query service and republishes
  `/initialpose` only when explicit safety guards pass (opt-in node; default
  launch unchanged). The decision logic is the ROS-free state machine in
  `scripts/reinitialization_supervisor_policy.py` -- candidate-score floor, minimum
  reset spacing, post-reset recovery evidence, and a non-self-resetting attempt
  ceiling -- regression-tested against unsafe-publication and false-acceptance
  sequences in `test/test_reinitialization_supervisor_policy.py`
- `docs/global_localization.md`: operations guide for running the G2 service and
  the G3 supervisor (commands, parameters, and the supervisor's safety guards)

### Changed

- `make_bbs_relocalization_attempts.py` branch-and-bound search is ~8x faster
  (precomputed per-yaw/level integer offset tables + adaptive FFT hit maps)
  with exactly identical candidate output
- `/pcl_pose` covariance default model is now `error_floor`, calibrated against
  ground truth on Koide outdoor_hard_01a and Autoware Istanbul (per-axis 2σ
  coverage ≥ 96 % on the calibration runs); the v1.1 heuristic remains
  available as `pose_covariance_mode: fitness_scaled` (docs:
  `pose_covariance.md`, issue #72)
- Documented the single-threaded-executor invariant that serializes the node's
  shared pose state without locks, so a future executor change cannot silently
  introduce a data race (issue #54)

## 1.1.0 - 2026-06-11

Reliability sprint, Jazzy support, and the artifact-first relocalization /
global-localization foundation. Release boundary:
`run_release_regression_suite.sh` `overall_pass=true` on a Jazzy build
(2026-06-11, commit `64da698`); see `docs/public_validation_log.md`.

### Added

- `scripts/run_public_demo.sh` for the 30-minute Autoware Istanbul public demo path
- `scripts/run_public_validation_dashboard.sh` for Markdown/HTML public validation dashboards
- `param/benchmark/v1_1_boreas_dry_run_endpoint.example.yaml` for the v1.1 MVP dry-run endpoint chain
- `scripts/run_v1_1_relocalization_smoke.sh` to guard the v1.1 claim boundary
- `docs/global_localization_roadmap.md` defining the G1/G2/G3 global localization phases
- `docs/development_plan.md` dated execution plan linked from README and the roadmap
- `scripts/make_map_grid_relocalization_attempts.py` map-wide candidate baseline that needs no route prior
- `scripts/make_bbs_relocalization_attempts.py` BBS_2D branch-and-bound candidate generator over the occupancy-grid pyramid

### Reliability / Claim Boundary

- add `docs/frame_contract.md` for `map` / `odom` / `base_link` expectations and issue #58 / #27 guidance
- publish TF immediately on `/initialpose` and add explicit frame/map diagnostics for issue #47
- add `docs/troubleshooting.md` for bringup failures and `/alignment_status` interpretation (issues #70, #35, #43, #48)
- add `docs/map_alignment.md` for map frame / initial pose alignment (issues #75, #44)
- add `docs/pose_covariance.md` for `/pcl_pose` covariance semantics (issue #72)
- document Istanbul public demo RMSE run variance; identical seed/map can yield ~`1.2 m` or late-run drift outliers
- tune public Istanbul 60 s preset with `local_map_crop` and `recovery_retry_from_last_pose` (`r3`, `gap<=1 s`, `seed<=15 m`)
- reconfirm public regression suite after drift tuning; Istanbul gate `1.00 m` and HDL IMU checks pass (2026-06-10)
- enable `recovery_retry_from_last_pose` on Koide outdoor manifests; `180 s` acceptance rises from `71` to `473+` ok rows
- reconfirm Koide outdoor manifests after recovery tuning; acceptance holds on `180 s` reconfirm (`486/753 ok_rows`) with run-to-run RMSE variance
- fix `boreas_localization_120s.yaml` dataset paths; baseline smoke shows `37/961 ok_rows` with `local_map_crop_too_small` bottleneck
- add `param/boreas_ndt_velodyne.yaml` and Boreas cliff sweeps; first `12 s` tracks (~`0.04 m`) before fitness reject streaks, `local_map_radius=300` improves 60 s RMSE to `39.5 m`
- tune Boreas preset throughput (`voxel 1.5`, `ndt threads 8`, `cloud_queue_depth 1`); 120 s matched rises to `45` with align median `0.018 s`
- add Boreas seed-management sweep; `post_reject_strict` + `rejected_seed_update` + `max_twist_prediction_dt=0.1` improves 60 s matched to `49` / RMSE `45.7 m`; 120 s confirm matched `52` / RMSE `47.8 m`
- guard `use_odom` integration until initial pose is valid and keep pose finite for NDT init guess
- ignore non-finite `/initialpose` payloads and stop eager scan replay during pose reset
- v1.1 relocalization is documented as a validated dry-run `/initialpose` command artifact only
- guarded reset publication and post-reset observation remain experimental and outside the public MVP endpoint
- automatic runtime recovery and production-grade global relocalization are explicitly out of scope

### ROS 2 Distro Support

- README badges and support matrix now show **Jazzy-first / Humble-compatible**
- GitHub Actions workflow updated to `actions/checkout@v4` and `push` on `main`
- Jazzy CI still builds against `ndt_omp_ros2` `humble` branch until a dedicated Jazzy branch exists upstream
- `scripts/setup_local_env.sh` resolves the ROS distro (`LIDAR_LOCALIZATION_ROS_DISTRO`, `ROS_DISTRO`, humble, jazzy) instead of hardcoding Humble
- HDL sample fetch patches rosbag2 `offered_qos_profiles` metadata per distro so `ros2 bag play` works on Jazzy
- public regression suite passes on a Jazzy build (2026-06-11): Istanbul `0.673 m` translation RMSE, HDL medians `468.5 / 459.0` pose rows
- Nav2 plugin type names switched from the Humble lookup form (`nav2_navfn_planner/NavfnPlanner`, `nav2_behaviors/*`) to the `::` type form required by Iron+ and accepted by Humble
- `param/nav2_humble_pointcloud.yaml` works with Jazzy bringup: explicit `plugin_lib_names` removed (double registration crashes Jazzy `bt_navigator`), `collision_monitor` and minimal `docking_server` sections added for the Iron+ lifecycle set
- Nav2 reinit supervisor regression prints route-bounded occupancy-map bootstrap commands when the map artifact is missing on a fresh workspace

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
