# Public Validation Log

This file records reproducible public-data validation snapshots. It is not a
hardware field-test log.

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
