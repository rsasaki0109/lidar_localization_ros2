# Scan-bounded dual IMU queue experiment

Date: 2026-07-14

This candidate follows the LIO-SAM separation between an optimization IMU
queue and a prediction/repropagation queue. It also preserves optimized
velocity and bias across accepted LiDAR updates, integrates only causal samples
through the scan timestamp, and keeps the consistency gate in front of the NDT
seed.

## Bugs found before the passing probe

1. `gyro_noise_density` and `accel_noise_density` are continuous densities in
   `/sqrt(Hz)`, but the discrete measurement variance omitted division by
   `dt`. At 200 Hz this made the IMU factor roughly 200 times overconfident.
2. Koide `outdoor_hard_01a` publishes Livox acceleration in `g`, not m/s².
   Offline LI-Init-style analysis measured the required scale as `9.80665`.
3. Resetting velocity to zero at every correction produced metre-scale
   open-loop errors. The candidate transfers optimized velocity/bias and uses a
   bounded LiDAR finite-difference velocity only when a reset is unavoidable.

All three issues have analytical or behavioral regression tests. The
dataset-specific acceleration scale remains an explicit parameter; it is not
auto-guessed at runtime.

## Same-window result

Dataset: Koide `outdoor_hard_01a`, first 30 seconds. Both IMU modes used
`imu_accel_scale=9.80665`; only the candidate enabled the dual queue.

| Metric | Current single stream | Dual queue candidate |
| --- | ---: | ---: |
| Finite open-loop comparisons | 40 | 43 |
| Translation prediction median | 0.988 m | 0.064 m |
| Rotation prediction median | 4.633 deg | 0.763 deg |
| IMU seed rows | 2/46 | 38/47 |
| Reject streak max | 8 | 0 |
| Pose coverage | 22.80/30 s (76.0%) | 29.40/30 s (98.0%) |
| Translation RMSE | 0.242 m | 0.079 m |
| Rotation RMSE | 2.486 deg | 1.084 deg |
| Translation end error | 1.391 m | 0.057 m |
| Rotation end error | 14.314 deg | 0.354 deg |
| Alignment latency median | 0.297 s | 0.272 s |
| Alignment latency p95 | 0.412 s | 0.391 s |

Candidate artifacts are on the external SSD under
`generated/imu_dual_queue_covariance_probe_20260714`; the single-stream control
is under `generated/imu_single_queue_covariance_probe_20260714`.

## Decision

The initial probe was followed by three repeats on each of three fixtures.

| Fixture | Candidate median (worst) coverage | Translation RMSE | Rotation RMSE | End translation | End rotation | Reject max | Seed ratio | Latency p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `outdoor_hard_01a`, 0–30 s | 98.0% (98.0%) | 0.085 (0.085) m | 1.134 (1.185) deg | 0.054 (0.057) m | 0.330 (0.350) deg | 0 | 85.4% (84.8%) | 0.323 (0.336) s |
| `outdoor_hard_01a`, 85–112 s | 96.7% (96.7%) | 0.073 (0.081) m | 0.895 (1.061) deg | 0.031 (0.034) m | 0.517 (0.535) deg | 0 | 69.0% (67.5%) | 0.442 (0.457) s |
| `outdoor_hard_02a`, 0–30 s | 98.7% (98.3%) | 0.095 (0.096) m | 1.051 (1.141) deg | 0.117 (0.125) m | 1.101 (1.122) deg | 0 | 86.0% (74.5%) | 0.323 (0.375) s |

The remaining nondeterministic failure was traced to a `1.000281 s` scan
interval being rejected by an exact `1.000000 s` guard. A 10 ms timestamp/
scheduler tolerance admits this boundary case while still rejecting genuine
`1.1 s` gaps. Accepted scans without causal IMU coverage now advance through a
pose-only anchor rather than erasing optimized velocity and bias.

The dual queue passed all nine repeats and is promoted to
`imu_dual_queue_enabled=true`. The comparison runner still provides
`imu_preintegration` as the explicit legacy single-stream control.

Continuous-time deskew remains default-off. With the promoted dual queue it
applied to at least 87.5% of `outdoor_hard_01a` rows and passed three repeats,
but on `outdoor_hard_02a` its median translation RMSE regressed from `0.095 m`
to `0.146 m` across three repeats. The `imu_dual_queue_deskew` runner mode is
retained only for reproducible future experiments.

Repeated artifacts are on the external SSD under `runs/`:
`imu_jitter_tolerance_ab_20260714`, `imu_jitter_corner_ab_20260714`,
`imu_jitter_outdoor_hard_02a_ab_20260714`,
`imu_dual_queue_deskew_ab_20260714`, and
`imu_dual_queue_deskew_outdoor_hard_02a_20260714`.
