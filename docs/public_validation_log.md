# Public Validation Log

This file records reproducible public-data validation snapshots. It is not a
hardware field-test log.

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
