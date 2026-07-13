# Koide IMU rotation prediction experiment

This experiment checks whether gyro-only relative rotation can safely seed local NDT tracking.
It does not add global localization and does not integrate acceleration or position.

`run_dataset_analysis.py` analyzes all 11 Koide sequences. It compares raw identity axes,
signed-axis permutations, and a fitted Wahba rotation; searches LiDAR/IMU time offset; reports
gravity convention, apparent bias, static-TF agreement, and timestamp coverage. The standalone
`imu_rotation_prediction.hpp` candidate uses bounded interpolation, trapezoidal SO(3)
integration, and coverage/duration/sample-gap rejection. Its test can be run with:

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexperiments/imu_yaw_prediction \
  experiments/imu_yaw_prediction/test_imu_rotation_prediction.cpp -o /tmp/test_imu_rotation_prediction
/tmp/test_imu_rotation_prediction
```

The full offline result is stored outside the repository at
`/media/sasaki/aiueo/datasets/koide_hard_localization/generated/imu_yaw_validation_20260714`.
The compact result and runtime A/B gates are in `results.json`.

## Decision

Do not promote this candidate to runtime. Indoor `indoor_easy_01` regressed from 0.053 m to
1.314 m translation RMSE and from 1.68 to 2.42 degrees rotation RMSE. Outdoor runs were close,
but `outdoor_kidnap_a` also slightly worsened final rotation error. The full-loop indoor runs
were therefore stopped by the early regression gate rather than spending additional runtime on
an already rejected method. The production LiDAR-only path and global-localization behavior are
unchanged.

One contaminated run is intentionally excluded: an interrupted benchmark left a rosbag process
on the same ROS domain, producing alternating scan timestamps about 64 seconds apart. Clean runs
used isolated domains and no surviving replay processes.
