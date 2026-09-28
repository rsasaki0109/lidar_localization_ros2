# Rejected Koide IMU rotation prediction experiment

The compact historical A/B results remain in [results.json](results.json).
The gyro-only seed candidate was rejected: indoor translation RMSE increased
from 0.053 m to 1.314 m, and final rotation error also regressed on the outdoor
kidnap sequence. These historical results have not been rerun by this cleanup.
No runtime parameters or production code use the candidate.

The unbuilt header and standalone test were removed after rejection. Their
exact sources and the original notes remain in Git at
`d3c621deb52338cd2ae4d2612927a1e69622a614:experiments/imu_yaw_prediction/`.
A SHA-256-verified working archive is also stored at
`/media/sasaki/aiueo2/jeplo_data/experiments/go2_rejected_imu_source_archive/archive`.

The original full analysis location recorded by the experiment was
`/media/sasaki/aiueo/datasets/koide_hard_localization/generated/imu_yaw_validation_20260714`.
The original notes exclude one contaminated replay with alternating timestamps;
that exclusion remains part of the historical evidence, not a new measurement.
