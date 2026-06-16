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

## Safety: prediction/correction guard and fallback

A LiDAR localizer must not let a bad IMU integration drag the pose. The guard
policy (`imu_preintegration_guard_policy.hpp`) enforces that:

- if the IMU prediction and the NDT correction disagree by more than
  `imu_prediction_correction_guard_translation_m` (2.0 m) or
  `imu_prediction_correction_guard_yaw_deg` (4.0°), or the smoother update is
  non-finite / implausibly large, the node enters **fallback mode** and reverts
  to non-IMU prediction until the disagreement clears;
- the guard is itself unit-tested (`test_imu_preintegration_guard_policy.cpp`).

This is the same design principle as the rest of the stack (see
[pose_covariance.md](pose_covariance.md), the G3 reinitialization guard): a
component that feeds itself must not be able to diverge unbounded.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `use_imu` | `false` | Master enable for the IMU input |
| `use_imu_preintegration` | `true` | Use the preintegration smoother (only active when `use_imu`) |
| `imu_preintegration_use_base_frame_transform` | `false` | Transform IMU samples into the base frame before integrating |
| `imu_gyro_noise_density` | `0.01` | rad/s/√Hz |
| `imu_accel_noise_density` | `0.1` | m/s²/√Hz |
| `imu_gyro_random_walk` | `0.0001` | rad/s²/√Hz |
| `imu_accel_random_walk` | `0.001` | m/s³/√Hz |
| `imu_bias_prior_sigma_gyro` | `0.01` | gyro bias prior σ |
| `imu_bias_prior_sigma_accel` | `0.1` | accel bias prior σ |
| `imu_ndt_sigma_z` / `_roll` / `_pitch` | `0.1` / `0.05` / `0.05` | NDT prior σ on the axes IMU constrains most |
| `imu_prediction_correction_guard_translation_m` | `2.0` | fallback if IMU-vs-NDT disagree beyond this |
| `imu_prediction_correction_guard_yaw_deg` | `4.0` | fallback if IMU-vs-NDT yaw disagree beyond this |

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
    use_imu: true
    use_imu_preintegration: true
```

The MID-360 launch (`mid360_legged_localization.launch.py`) already wires
`imu_topic:=/livox/imu` as a worked example.

## Validation state (honest limits)

- The preintegration **math** is verified (the regression test above) and the
  guard prevents divergence.
- The estimator is exercised by the **HDL IMU smoke / throughput check** in the
  release suite, which is a *pipeline safety* gate, **not** an accuracy ranking
  (see [competitive_roadmap.md](competitive_roadmap.md)).
- A controlled public **LiDAR + IMU + GT** accuracy benchmark is still open
  (the Boreas / Koide tracks). Until that lands, the IMU estimator is offered as
  a working, math-verified, guarded option — **not** as a validated accuracy
  improvement over pure NDT on a specific dataset. Do not cite it as an accuracy
  claim without a controlled run.

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
