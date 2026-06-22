# IMU Estimation

This page documents the IMU-aided estimation already implemented in the package,
in response to the recurring requests in
[#36 (imu preintegration)](https://github.com/rsasaki0109/lidar_localization_ros2/issues/36)
and
[#77 (utilization of IMU angular velocity and the introduction of an
estimator)](https://github.com/rsasaki0109/lidar_localization_ros2/issues/77).
Both are implemented; this page is the map of what exists, how to turn it on, and
what is and is not validated.

## What is implemented

The node offers a **layered** set of prediction/estimation options, cheapest
first. Each is opt-in; the default bringup uses none of them (pure NDT/GICP
tracking) so the IMU path never affects users who do not enable it.

| Layer | Parameter | What it does | Addresses |
| --- | --- | --- | --- |
| Twist prediction | `use_twist_prediction` (+ `twist_prediction_use_angular_velocity`) | Constant-twist motion prediction between scans, using the IMU/odom angular velocity to rotate the seed | #77 (angular velocity) |
| Twist EKF | `use_twist_ekf` | A small constant-velocity EKF on the twist, feeding both the prediction seed and the published covariance | #77 (estimator) |
| **IMU preintegration + smoother** | `use_imu` + `use_imu_preintegration` | On-manifold IMU preintegration (Forster et al. 2017) inside a sliding-window Gauss-Newton optimizer with 6-DOF NDT priors and online gyro/accel bias estimation | #36, #77 |
| Twist GTSAM smoother | `use_gtsam_smoother` | Sliding-window odometry+NDT smoother (no raw IMU) | #77 (estimator) |

The headline item is the **IMU preintegration estimator**:

- `include/lidar_localization/imu_preintegration.hpp` — on-manifold
  preintegration of gyro+accel into a relative `(delta_p, delta_v, delta_R)`
  with covariance propagation and first-order bias Jacobians, pure Eigen.
- `include/lidar_localization/imu_gtsam_smoother.hpp` — a sliding-window
  Gauss-Newton optimizer over per-pose state `[p, v, rpy]` (9 DOF) plus a shared
  `[bias_gyro, bias_accel]` (6 DOF), with preintegration factors between poses
  and Huber-robust NDT priors weighted by alignment fitness.
- `include/lidar_localization/imu_preintegration_guard_policy.hpp` — the safety
  layer (below).

## Correctness evidence

The preintegration math is the subtle, easy-to-get-wrong core, so it has a
numerical regression test, `test/test_imu_preintegration.cpp`, that pins it
against closed-form answers and a finite-difference check:

- `reset()` is identity; out-of-range `dt` (≤0 or >0.5 s sensor dropouts) is
  ignored;
- **pure rotation** (constant gyro, zero specific force) reproduces
  `delta_R = Exp(omega·t)` exactly, with zero `delta_p`/`delta_v`;
- **pure translation** (constant specific force) reproduces `delta_v = a·t`,
  `delta_p = ½·a·t²` exactly (the discrete scheme is exact for constant accel);
- the **residual is zero** for a world trajectory built to be consistent with
  the integrated specific force and gravity;
- the **bias Jacobians match finite differences**: the first-order
  `correctByBias()` update reproduces a full re-integration at a perturbed bias
  to within the O(δ²) truncation error — this validates all five
  `d_*/d_bias` accumulators together;
- the propagated covariance is symmetric, positive-semidefinite, and grows with
  integration time.

The test is pure Eigen (no ROS), so it also builds and runs standalone:

```bash
g++ -std=c++17 -I include -I /usr/include/eigen3 \
    test/test_imu_preintegration.cpp -o /tmp/t && /tmp/t
```

It is registered in `CMakeLists.txt` and runs under `colcon test` as
`imu_preintegration`.

## Safety: prediction/correction guard and reset

A LiDAR localizer must not let a bad IMU integration drag the pose. The guard
policy (`imu_preintegration_guard_policy.hpp`) enforces that:

- IMU preintegration is used as the registration seed/prediction path; the
  accepted LiDAR registration measurement remains the published pose anchor;
- if the IMU prediction and the NDT correction disagree by more than
  `imu_prediction_correction_guard_translation_m` (2.0 m) or
  `imu_prediction_correction_guard_yaw_deg` (10.0°), or the smoother update is
  non-finite / implausibly large, the node publishes the LiDAR measurement pose
  and resets the IMU smoother to that measurement before continuing;
- fallback mode is reserved for an already-disabled IMU preintegration state;
- the guard is itself unit-tested (`test_imu_preintegration_guard_policy.cpp`).

This is the same design principle as the rest of the stack (see
[pose_covariance.md](pose_covariance.md), the G3 reinitialization guard): a
component that feeds itself must not be able to diverge unbounded.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `use_imu` | `false` | Enable the legacy IMU correction path |
| `use_imu_preintegration` | `true` | Use the guarded preintegration smoother from `/imu`, independent of `use_imu` |
| `imu_preintegration_use_base_frame_transform` | `false` | Transform IMU samples into the base frame before integrating |
| `use_continuous_time_deskew` | `false` | Experimental default-off hook that deskews the pre-voxel scan using per-point timing and IMU-predicted relative motion |
| `continuous_time_deskew_reference_time_sec` | `0.0` | Scan-relative reference time for the deskewed cloud; `0.0` means earliest point |
| `imu_gyro_noise_density` | `0.01` | rad/s/√Hz |
| `imu_accel_noise_density` | `0.1` | m/s²/√Hz |
| `imu_gyro_random_walk` | `0.0001` | rad/s²/√Hz |
| `imu_accel_random_walk` | `0.001` | m/s³/√Hz |
| `imu_bias_prior_sigma_gyro` | `0.01` | gyro bias prior σ |
| `imu_bias_prior_sigma_accel` | `0.1` | accel bias prior σ |
| `imu_ndt_sigma_z` / `_roll` / `_pitch` | `0.1` / `0.05` / `0.05` | NDT prior σ on the axes IMU constrains most |
| `imu_prediction_correction_guard_translation_m` | `2.0` | reset IMU smoother if IMU-vs-NDT disagree beyond this |
| `imu_prediction_correction_guard_yaw_deg` | `10.0` | reset IMU smoother if IMU-vs-NDT yaw disagree beyond this |

Set the noise densities from your IMU datasheet (or an Allan-variance run) for
best results; the defaults are MEMS-grade ballpark values.

## How to enable

```bash
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    localization_param_dir:=/path/to/params.yaml \
    imu_topic:=/your/imu
```

`lidar_localization.launch.py` exposes an `imu_topic` argument (default `/imu`)
that is remapped onto the node's `/imu` subscription, so no manual remapping is
needed. In the params file:

```yaml
/**:
  ros__parameters:
    use_imu: false
    use_imu_preintegration: true
```

Set `use_imu: true` only when you also want the legacy IMU correction path. The
first-bringup config generator exposes this as `--imu-mode preintegration`,
`--imu-mode legacy`, or `--imu-mode both`.

The MID-360 launch (`mid360_legged_localization.launch.py`) already wires
`imu_topic:=/livox/imu` as a worked example. When
`imu_preintegration_use_base_frame_transform: true`, publish
`base_frame_id -> imu_frame_id` through the launch `imu_tf_*` arguments or an
external TF publisher, then run the generated doctor command with
`--require-imu-base-tf`.

For continuous-time / deskew development, also verify the LiDAR stream before
tuning estimator math:

```bash
ros2 run lidar_localization_ros2 check_lidar_localization_bringup.py \
  --profile mid360 \
  --require-imu \
  --require-imu-base-tf \
  --require-cloud-time-field
```

That check only verifies data readiness. It requires a per-point timing field on
`PointCloud2` (`time`, `timestamp`, `offset_time`, or `t`) so future deskew code
can interpolate motion inside a scan.

The C++ helper `extractPointRelativeTimesSeconds()` in
`point_cloud_conversion.hpp` now turns those fields into per-point relative
times normalized to the scan's earliest point. It accepts absolute
`timestamp` fields and relative `offset_time` fields, preserves invalid points
as `NaN`, and reports valid/invalid counts. This is still infrastructure: the
default registration path remains scan-to-map NDT/GICP until a real deskew
stage is wired in.

`convertSensorCloudToTimedXyzi()` preserves those relative times with the same
indices as the converted `pcl::PointXYZI` cloud, and scan preparation now carries
the matching vector through range filtering. If voxel filtering is enabled, the
registration cloud is a centroid cloud, so the final registration cloud is not
reported as time-aligned yet. A real deskew stage should run before voxel
filtering.

`continuous_time_deskew_policy.hpp` adds the pure geometry primitive for that
stage: given a scan-start-to-scan-end relative motion and per-point relative
times, it interpolates translation/rotation and rewrites points into a chosen
scan reference time. The runtime hook is default-off. For generated bringup,
enable it explicitly:

```bash
ros2 run lidar_localization_ros2 create_lidar_localization_config.py \
  --profile mid360 \
  --map-path /absolute/path/to/map.pcd \
  --enable-continuous-time-deskew \
  --use-sim-time \
  --output /tmp/mid360_deskew.yaml
```

The generated doctor command will require IMU data, `base_frame -> imu_frame`
when the profile uses base-frame IMU integration, and a per-point cloud timing
field. `--use-sim-time` only affects the printed launch command, which is useful
for bag replay. For manual launch:

```bash
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  use_continuous_time_deskew:=true \
  use_imu_preintegration:=true
```

When enabled, the localizer only applies deskew if per-point scan timing is
ready, IMU preintegration is initialized, and the pre-voxel scan still has a
time vector aligned with the cloud. The relative motion currently comes from
the IMU smoother's latest optimized pose to its current prediction, so treat it
as an experimental scan-interval approximation until validated on bags with
ground truth.

## Runtime diagnostics

`/alignment_status` includes IMU preintegration state next to the alignment
result:

| Key | Use |
| --- | --- |
| `imu_preintegration_enabled` | confirms the parameter path is active |
| `imu_preintegration_status` | explains disabled, waiting, stale, active, non-finite, or fallback states |
| `imu_has_new_samples` | `true` when the scan had IMU samples newer than the previous scan update |
| `registration_seed_source` | seed used for registration, such as `imu_preintegration`, `twist_prediction`, `previous_delta`, or `current_pose` |
| `imu_received_sample_count` / `imu_integrated_sample_count` | samples seen vs samples accepted into preintegration for the current scan interval |
| `imu_transform_failure_count` | samples skipped because `base_frame <- imu_frame` TF was unavailable |
| `imu_invalid_dt_count` / `imu_last_dt_sec` | timestamp health for preintegration samples |
| `imu_last_sample_age_sec` | scan-to-IMU timestamp lag |
| `imu_integration_window_sec` | IMU time available for the current scan interval |
| `scan_time_status` | continuous-time / deskew data readiness for per-point timing |
| `scan_time_field` | point field selected for timing, such as `offset_time`, `time`, or `timestamp` |
| `scan_time_duration_sec` | observed scan span after normalizing per-point times |
| `scan_time_valid_point_count` / `scan_time_invalid_point_count` | points with readable vs unreadable per-point timing |
| `deskew_ready` | `true` only when per-point scan timing and IMU preintegration are both usable |
| `deskew_readiness_status` | first blocker for future deskew work, or `deskew_ready` |
| `continuous_time_deskew_enabled` / `continuous_time_deskew_applied` | whether the default-off runtime hook is enabled and used for the latest scan |
| `continuous_time_deskew_status` | applied/skip reason for the runtime hook |
| `continuous_time_deskew_point_count` | points deskewed before optional voxel filtering |
| `continuous_time_deskew_skipped_invalid_time_count` / `continuous_time_deskew_clamped_time_count` | invalid or out-of-range per-point times handled by the hook |

Use these before tuning noise values. If `imu_preintegration_status` is
`imu_preintegration_waiting_for_imu`, fix topic remapping or driver timestamps
first. If it is `imu_preintegration_fallback_mode`, inspect the correction guard
messages and the LiDAR alignment quality around the transition.

For a quick runtime smoke test during rosbag replay:

```bash
ros2 run lidar_localization_ros2 validate_lidar_localization_imu.py \
  --duration-sec 30 \
  --min-imu-active-ratio 0.5 \
  --require-imu-seed-source
```

For the experimental deskew hook:

```bash
ros2 run lidar_localization_ros2 validate_lidar_localization_imu.py \
  --duration-sec 30 \
  --min-imu-active-ratio 0.5 \
  --require-imu-seed-source \
  --require-deskew-applied
```

If you already recorded `alignment_status.csv` with
`benchmark_diagnostic_recorder`, analyze it offline:

```bash
ros2 run lidar_localization_ros2 validate_lidar_localization_imu.py \
  --alignment-csv /path/to/alignment_status.csv \
  --output-md /tmp/imu_validation.md
```

This is a code-path and data-health check. It shows whether preintegration and
deskew were active, skipped, or in fallback; trajectory accuracy still needs a
bag/reference comparison.

For the full three-way run:

```bash
ros2 run lidar_localization_ros2 run_lidar_localization_imu_comparison.py \
  --bag-path /absolute/path/to/bag \
  --map-path /absolute/path/to/map.pcd \
  --profile mid360 \
  --output-dir /tmp/lidarloc_imu_compare
```

This creates separate `lidar_only`, `imu_preintegration`, and `deskew` benchmark
directories. Add `--reference-csv /absolute/path/to/reference.csv` to generate
trajectory RMSE JSONs as well. Without reference, the output is still useful for
code-path health, alignment timing, fitness, and resource comparison, but it is
not an accuracy validation. Start from `comparison.md` in the output directory.
The runner checks input paths and rosbag2 `metadata.yaml` topic names before
starting ROS processes; use `--print-only` to review generated configs and
commands without requiring local bag/map files.
It writes `run_commands.sh` for manual reproduction. If you edit or rerun parts
of the output manually, regenerate only the Markdown report with:

```bash
ros2 run lidar_localization_ros2 run_lidar_localization_imu_comparison.py \
  --output-dir /tmp/lidarloc_imu_compare \
  --report-only
```

For a public real-bag smoke that stages Koide `outdoor_hard_01a` and runs the
same three-way comparison:

```bash
scripts/run_koide_hard_imu_deskew_smoke.sh --download
```

Latest local runtime smoke (`2026-06-21`, 60 s Koide `outdoor_hard_01a`,
`/tmp/lidarloc_koide_wrapper_allmodes_smoke60_strict`):

- `lidar_only`: `translation_rmse_m=0.163`, `rotation_rmse_deg=2.137`
- `imu_preintegration`: IMU active / seed source `34.4%`, fallback `0`,
  `translation_rmse_m=0.082`, `rotation_rmse_deg=0.810`
- `deskew`: IMU active / seed source `35.4%`, deskew applied `31.2%`,
  fallback `0`, `translation_rmse_m=0.081`, `rotation_rmse_deg=0.771`

The wrapper enables an open-loop strict score gate for Koide to reject stale
local minima after long accepted-pose gaps. This confirms the real-bag code
paths are active and bounded on that smoke window. It is not a claim that IMU
preintegration or deskew always improves accuracy; longer windows and
calibration/extrinsics control still matter.

Longer-window boundary check (`2026-06-21`, same bag, 120 s,
`/tmp/lidarloc_koide_wrapper_allmodes_smoke120_strict`):

- `imu_preintegration`: `translation_rmse_m=0.087`, `rotation_rmse_deg=0.855`,
  but pose output only reached `21.4 s` of the requested `120 s`
- `deskew`: `translation_rmse_m=0.161`, `rotation_rmse_deg=1.926`,
  pose output only reached `32.7 s`, deskew applied `10.5%`

So the 60 s smoke is a useful runtime safety check, but 120 s still exposes a
tracking/relocalization boundary. The comparison report now includes `Pose
rows` and `Last pose s` so short coverage cannot be mistaken for long-horizon
accuracy.

## Validation state (honest limits)

- The preintegration **math** is verified (the regression test above) and the
  guard prevents divergence.
- The estimator is exercised by the Koide real-bag smoke above and the HDL IMU
  smoke / throughput check in the release suite. These are *pipeline safety* and
  short-window health gates, **not** final backend rankings (see
  [competitive_roadmap.md](competitive_roadmap.md)).
- A longer controlled public **LiDAR + IMU + GT** accuracy benchmark is still
  open. Until that lands, the IMU estimator is offered as a working,
  math-verified, guarded option, **not** as a general accuracy improvement over
  pure NDT. Do not cite it as an accuracy claim without a controlled run.

### Turnkey A/B benchmark (idle-ready)

A controlled Koide A/B is committed and ready to run the moment the machine is
quiet (timing/accuracy on this shared box is only trustworthy at load < ~5):

- `param/benchmark/koide_hard_localization_outdoor_hard_01a_full380_ndt_omp_imu_off.yaml`
- `param/benchmark/koide_hard_localization_outdoor_hard_01a_full380_ndt_omp_imu_on.yaml`

Both are the Phase 1 NDT_OMP winner config on the real `outdoor_hard_01a` bag
(which carries `/livox/imu`, 75947 samples over 380 s, plus GT); they differ
**only** in `use_imu_preintegration` (and `output_dir` / `ros_domain_id`), so any
metric delta is attributable to the Forster preintegration smoother. Run both
back-to-back and compare:

```bash
ros2 run lidar_localization_ros2 benchmark_from_manifest --manifest \
  param/benchmark/koide_hard_localization_outdoor_hard_01a_full380_ndt_omp_imu_off.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest --manifest \
  param/benchmark/koide_hard_localization_outdoor_hard_01a_full380_ndt_omp_imu_on.yaml
```

**Read the metrics the Phase 1 way** (docs/phase1_koide_backend_comparison.md): a
lower translation RMSE alone is a trap on this hard sequence — an estimator can
look better while tracking far fewer poses. Judge on **ok-rate, longest lost
window, and pose count first**, then RMSE among the survivors. Only after this
run lands may the preintegration path be described as an accuracy win (or not)
on Koide; until then the limits above stand.

## Related docs

- [pose_covariance.md](pose_covariance.md) — the twist-EKF hybrid covariance path
- [interfaces.md](interfaces.md) — topics, including the optional `/imu` input
- [mid360_legged_jetson.md](mid360_legged_jetson.md) — a real IMU bringup
- [competitive_roadmap.md](competitive_roadmap.md) — dataset/validation strategy
