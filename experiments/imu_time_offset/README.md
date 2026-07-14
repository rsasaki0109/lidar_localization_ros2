# LiDAR-IMU time-offset experiment

This experiment tests whether a fixed timestamp correction is identifiable and safe enough for
a bounded runtime A/B test. It does not change production parameters or global localization.

The approach follows LI-Init's motion-alignment initialization and LI-Calib's continuous-time
treatment of asynchronous measurements:

- [Robust Real-time LiDAR-inertial Initialization](https://arxiv.org/abs/2202.11006)
- [Targetless Calibration of LiDAR-IMU System Based on Continuous-time Batch Estimation](https://arxiv.org/abs/2007.14759)

The analyzer integrates gyro measurements over consecutive LiDAR/reference intervals while
jointly fitting rotation and bias. A fine offset sweep is supplemented by a quadratic sub-grid
estimate, the score at zero offset, the width of the near-optimal basin, and independent
30-second window estimates. A runtime candidate must improve rotational RMSE over zero by 2%,
have a one-percent basin no wider than 30 ms, and remain stable within 10 ms MAD across windows.

Run all 11 Koide sequences with:

```bash
python3 experiments/imu_time_offset/run_dataset_analysis.py \
  --data-root /media/sasaki/aiueo/datasets/koide_hard_localization \
  --output-dir /media/sasaki/aiueo/datasets/koide_hard_localization/generated/imu_time_offset_20260714
```

## Decision

Do not promote timestamp correction to production. The offline sweep found stable 1--4 ms
estimates on most normal sequences and a much larger 46 ms estimate on `outdoor_kidnap_a`.
That largest, most identifiable candidate was used as the bounded runtime gate. Compared with
zero offset over the same 30-second replay, 46 ms increased translation RMSE from 0.1147 m to
0.2012 m and rotation RMSE from 0.378 degrees to 1.204 degrees. Final translation error rose
from 0.284 m to 1.097 m and final rotation error from 0.365 degrees to 7.058 degrees.

The offline objective aligns gyro rotation with the ground-truth trajectory, but improving that
objective does not guarantee better interaction with the current preintegration, scan timing,
and NDT correction loop. The runtime parameter candidate was therefore removed. Production
behavior remains unchanged; only the offline identifiability analyzer and recorded rejection
evidence are retained.
