# IMU seed consistency gate

This experiment checks an IMU-predicted pose against the accepted LiDAR pose
before the prediction is allowed to seed registration. A valid comparison must
pass both the translation and rotation thresholds for five consecutive accepted
updates. Missing, non-finite, or failed evidence revokes permission immediately.

## Koide probe (2026-07-14)

Dataset: `outdoor_hard_01a`, first 30 seconds. The run artifacts are stored on
the external SSD at `generated/imu_seed_gate_probe_20260714`.

```bash
scripts/run_koide_hard_imu_deskew_smoke.sh \
  --data-dir /media/sasaki/aiueo/datasets/koide_hard_localization \
  --output-dir /media/sasaki/aiueo/datasets/koide_hard_localization/generated/imu_seed_gate_probe_20260714 \
  --duration 30 --ros-domain-id 190 \
  --mode lidar_only --mode imu_preintegration
```

The gate thresholds were 0.5 m and 5 degrees with five required consecutive
passes. The IMU prediction was never selected as a registration seed:

- 32 valid accepted-pose comparisons (37 diagnostic rows carried finite values)
- translation error: 0.0057 m minimum, 0.9952 m median, 1.3620 m maximum
- rotation error: 0.0238 degrees minimum, 5.2486 degrees median, 23.5381 degrees maximum
- maximum consecutive pass count: 4/5
- IMU seed source: 0/44 alignment rows
- fallback count: 0

The gated run therefore remained open-loop with respect to registration. Its
trajectory metrics must not be interpreted as an IMU accuracy gain: the two
independent NDT runs had different pose counts even though IMU seed use was
zero, demonstrating material run-to-run/runtime-scheduling variation on this
fixture.

## Decision

Keep the gate and its diagnostics. Do not enable the state-retaining or
time-buffered IMU seed prototypes that previously caused large Koide drift.
Next, fix scan-time-bounded integration and initial velocity/bias observability,
then repeat this open-loop test. Only allow an IMU seed when multiple windows
and sequences satisfy the same predeclared thresholds.
