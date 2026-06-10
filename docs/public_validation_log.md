# Public Validation Log

This file records reproducible public-data validation snapshots. It is not a
hardware field-test log.

## 2026-06-11 v1.1.0 Release Regression On Jazzy

- commit: `64da698`
- source branch: `main`
- environment: ROS 2 Jazzy, Ubuntu 24.04, first release-suite run on a Jazzy build
- validation command:

```bash
source scripts/setup_local_env.sh
scripts/run_release_regression_suite.sh \
  --work-dir /tmp/lidarloc_release_regression_20260611 \
  --report-dir ../artifacts/public/release_regression_suite_20260611_v1_1_0
```

Result:

- overall: `overall_pass=true`
- Istanbul `60 s` no-IMU safety check:
  - translation RMSE: `0.673 m`
  - rotation RMSE: `2.179 deg`
  - alignment median: `0.036 s`
- HDL `60 s` IMU safety check, two-repeat medians:
  - pose rows: `468.5` baseline / `459.0` IMU candidate
  - IMU-candidate alignment median: `0.084 s`
  - fallback events: `1`
- Nav2 reinitialization supervisor `150 s`:
  - requested rows: `285 -> 6` (baseline vs supervisor)
  - both replay goals `SUCCEEDED`
  - all six regression checks pass

Artifacts:

- `artifacts/public/release_regression_suite_20260611_v1_1_0/summary.json`
- `artifacts/public/release_regression_suite_20260611_v1_1_0/public_regression_suite/summary.json`
- `artifacts/public/release_regression_suite_20260611_v1_1_0/nav2_reinit_supervisor_regression_150/regression_result.json`

Interpretation:

- This is the first full release-suite green on a Jazzy build and the v1.1.0
  release boundary. It required the Jazzy bring-up fixes recorded in the
  changelog (distro-aware env setup, HDL bag metadata patching, Nav2 plugin
  type names, collision_monitor / docking_server sections, and removal of the
  explicit BT plugin_lib_names list).
- The Nav2 occupancy-map artifact is bootstrapped route-bounded on fresh
  workspaces; full-map bounds OOM on the city-scale Istanbul map.
- This is public replay and controlled Nav2 regression validation, not a
  hardware result.

## 2026-06-10 Boreas Seed-Management Tuning

- commit: uncommitted (after `433e0b5`)
- source branch: `feat/rust-robotics-library-first`
- validation commands:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/boreas_localization_60s_seed_compare.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/boreas_localization_120s_boreas_preset.yaml
```

Result:

- `60 s` seed sweep winner `post_reject_r5_plus_rejected_seed`:
  - matched: `49` (baseline `41`)
  - translation RMSE: `45.7 m` (baseline `47.6 m`)
  - no `reinitialization_requested_rows` in `60 s`
- `120 s` updated `boreas_ndt_velodyne.yaml`:
  - matched: `52` (previous throughput preset `45`)
  - translation RMSE: `47.8 m`
  - rotation RMSE: `7.7 deg`
  - ok rows: `52 / 647`
  - cliff still begins ~`12 s`; dominant post-cliff diagnostic: `fitness_score_over_post_reject_strict_threshold_rejected`

Artifacts:

- `/tmp/lidarloc_boreas_localization_60s_seed_compare/`
- `/tmp/lidarloc_boreas_localization_120s_boreas_preset/`

Interpretation:

- `post_reject_strict` + conservative `rejected_seed_update` + tighter `max_twist_prediction_dt` improves throughput without the aggressive `recovery_retry` RMSE blow-up.
- Boreas remains diagnostic-only; the `~12 s` fitness cliff is still the primary blocker.

## 2026-06-10 Boreas Baseline Smoke

- commit: `4dd6b4c` (manifest path fix uncommitted)
- source branch: `main`
- validation command:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/boreas_localization_120s.yaml
```

Result:

- `boreas-2021-09-02-11-42` `120 s` localizer-only: diagnostic failure
  - translation RMSE: `43.515 m`
  - rotation RMSE: `8.581 deg`
  - matched samples: `37`
  - ok rows: `37 / 961`
  - dominant diagnostics: `local_map_crop_too_small`, `fitness_score_over_threshold_rejected`, `accepted_gap_reinit_requested`

Artifacts:

- `/tmp/lidarloc_boreas_localization_120s_throttle/`

Interpretation:

- Boreas is not ready for outward claims; current bottleneck is map/crop acceptance and very low pose throughput.
- Next step is map-split validation and crop-policy tuning before IMU prediction work.

## 2026-06-10 Main After Koide Outdoor Acceptance Tuning

- commit: `4dd6b4c`
- source branch: `main`
- validation commands:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_120.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_180.yaml
```

Result:

- `outdoor_hard_01a_smoke60` + `recovery_retry r3_gap1_seed15`: `pass` acceptance
  - translation RMSE: `0.228 m`
  - rotation RMSE: `2.696 deg`
  - matched samples: `244`
  - ok rows: `244 / 246`
- `outdoor_hard_01a_120` + `recovery_retry r3_gap1_seed15`: outlier on reconfirm
  - translation RMSE: `0.315 m`
  - matched samples: `221`
  - ok rows: `221 / 447`
  - failure mode: `fitness_exploded_reinit_requested` ~`104 s`, unrecovered
- `outdoor_hard_01a_180` + `recovery_retry r3_gap1_seed15`: acceptance improved, accuracy outlier
  - translation RMSE: `1.549 m` (tuning run was `0.255 m`)
  - rotation RMSE: `4.373 deg`
  - matched samples: `486`
  - ok rows: `486 / 753` (baseline without recovery was `71 / 523`)

Artifacts:

- `/tmp/lidarloc_koide_outdoor_hard_01a_smoke60/`
- `/tmp/lidarloc_koide_outdoor_hard_01a_120/`
- `/tmp/lidarloc_koide_outdoor_hard_01a_180/`

Interpretation:

- `recovery_retry_from_last_pose` fixes the `180 s` acceptance collapse (`71 -> 473+ ok_rows`).
- Koide outdoor replay shows run-to-run variance on `120 s` / `180 s` similar to Istanbul;
  treat acceptance gains as the validated improvement and prefer repeat runs for release checks.
- Full-route robust tracking on Koide `Outdoor01` remains an open engineering target.

## 2026-06-10 Main After Istanbul Drift Tuning

- commit: `f6503d3`
- source branch: `main`
- validation commands:

```bash
source scripts/setup_local_env.sh
scripts/run_public_demo.sh \
  --output-dir /tmp/lidarloc_public_demo_post_tune_r2 \
  --skip-fetch --skip-build --ros-domain-id 212
scripts/run_public_regression_suite.sh \
  --output-dir /tmp/lidarloc_public_regression_post_tune \
  --ros-domain-base 220
```

Result:

- public demo (best of two fresh runs): `pass` trajectory eval
  - translation RMSE: `0.955 m`
  - rotation RMSE: `2.130 deg`
  - matched samples: `220`
  - pose rows: `222`
- public demo outlier on same preset (same day): translation RMSE `3.308 m`, matched `19`
- public regression suite: `pass`
  - Istanbul `60 s`: `pass`
    - pose rows: `106`
    - matched samples: `104`
    - translation RMSE: `1.002 m`
    - rotation RMSE: `2.328 deg`
    - recovery retry recovered: `1`
    - IMU prediction active rows: `0`
  - HDL `60 s`: `pass`
    - runs: `2 baseline / 2 candidate`
    - pose rows median: `559.0 -> 560.0`
    - alignment-time median: `0.045634 s -> 0.046194 s`
    - IMU prediction active rows median for the IMU-enabled candidate: `6.0`
    - max IMU disable fallback events: `1`

Artifacts:

- `artifacts/public/demo/latest/demo_report.md`
- `artifacts/public/public_regression_suite/summary.json`
- `artifacts/public/public_regression_suite/summary.md`
- `/tmp/lidarloc_public_demo_post_tune_r2/run/`
- `/tmp/lidarloc_public_regression_post_tune/`

Interpretation:

- Drift-tuned Istanbul preset clears the public regression gate while still showing
  run-to-run variance; prefer the regression suite over single demo runs for release checks.
- HDL IMU safety / throughput behavior remains stable after the Istanbul preset change.

## 2026-05-22 Main Release Regression

- commit: `2a5f11f`
- source branch: `main`
- validation command:

```bash
source scripts/setup_local_env.sh
scripts/run_release_regression_suite.sh \
  --work-dir /tmp/lidarloc_release_regression_suite_20260522_main \
  --report-dir ../artifacts/public/release_regression_suite_20260522_main \
  --public-ros-domain-base 220 \
  --nav2-ros-domain-base 226 \
  --hdl-repeat-count 2
```

Result:

- release regression: `pass`
- public regression suite: `pass`
  - Istanbul `60 s`: `pass`
    - pose rows: `106`
    - matched samples: `104`
    - translation RMSE: `1.176 m`
    - rotation RMSE: `0.393 deg`
    - IMU prediction active rows: `0`
  - HDL `60 s`: `pass`
    - runs: `2 baseline / 2 candidate`
    - pose rows median: `558.5 -> 553.5`
    - alignment-time median: `0.043065 s -> 0.042911 s`
    - IMU prediction active rows median for the IMU-enabled candidate: `6.5`
    - max IMU disable fallback events: `1`
- Nav2 reinitialization-supervisor regression: `pass`
  - recommended run: `configured_initial_pose_count1`
  - baseline goal status: `SUCCEEDED`
  - candidate goal status: `SUCCEEDED`
  - reinitialization requested rows: `944 -> 7`
  - candidate OK rows after first trigger: `6`

Artifacts:

- `artifacts/public/release_regression_suite_20260522_main/summary.json`
- `artifacts/public/release_regression_suite_20260522_main/summary.md`
- `artifacts/public/release_regression_suite_20260522_main/public_regression_suite/summary.json`
- `artifacts/public/release_regression_suite_20260522_main/nav2_reinit_supervisor_regression_150/regression_result.json`

Interpretation:

- This is the current release-style validation boundary for public replay and
  controlled Nav2 recovery behavior on `main`.
- This still does not validate Livox MID-360 packet conversion, Jetson thermal
  limits, legged-robot vibration, measured extrinsics, or real `odom ->
  base_link` behavior.

## 2026-05-22 Main After MID-360 Bringup Merge

- commit: `b22dc5d`
- source branch: `main`
- validation command:

```bash
source scripts/setup_local_env.sh
scripts/run_public_regression_suite.sh \
  --output-dir /tmp/lidarloc_public_regression_suite_20260522_mid360_main \
  --report-dir ../artifacts/public/public_regression_suite_20260522_mid360_main \
  --ros-domain-base 210 \
  --hdl-repeat-count 2
```

Datasets:

- Autoware Istanbul localization ROS 2 bag, `60 s`
- official `hdl_localization` `hdl_400` sample converted to ROS 2, `60 s`

Result:

- overall: `pass`
- Istanbul: `pass`
  - pose rows: `108`
  - matched samples: `106`
  - translation RMSE: `1.458 m`
  - rotation RMSE: `0.397 deg`
  - IMU prediction active rows: `0`
- HDL: `pass`
  - runs: `2 baseline / 2 candidate`
  - pose rows median: `531.5 -> 550.5`
  - alignment-time median: `0.046578 s -> 0.046874 s`
  - IMU prediction active rows median for the IMU-enabled candidate: `4.5`
  - max IMU disable fallback events: `1`

Artifacts:

- `artifacts/public/public_regression_suite_20260522_mid360_main/summary.json`
- `artifacts/public/public_regression_suite_20260522_mid360_main/summary.md`

Interpretation:

- This validates the merged software stack against public replay data.
- This does not validate Livox MID-360 packet conversion, Jetson thermal limits,
  legged-robot vibration, measured extrinsics, or real `odom -> base_link`
  behavior.
- The MID-360 path is therefore launch/configuration/build validated and public
  replay regression checked, but not hardware validated.
