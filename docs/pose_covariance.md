# Pose Covariance Semantics

This document explains the covariance published on `/pcl_pose` for issue
[#72](https://github.com/rsasaki0109/lidar_localization_ros2/issues/72).

Since 2026-06-12 the default model is **calibrated against ground truth on
public datasets** (Koide `outdoor_hard_01a`, Autoware Istanbul) and is intended
to be consumable by fusion/arbitration logic — with the explicit limits in
[What the covariance can and cannot promise](#what-the-covariance-can-and-cannot-promise).

## Output topic

| Item | Value |
| --- | --- |
| Topic | `/pcl_pose` |
| Type | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| Frame | `header.frame_id = global_frame_id` (default: `map`) |

Covariance is filled only when a scan alignment is **accepted** and the pose is
published. Rejected alignments do not update the published covariance.

## Matrix layout

`pose.covariance` is a 36-element row-major array for
`(x, y, z, roll, pitch, yaw)`. Diagonal terms are indices 0, 7, 14, 21, 28, 35;
off-diagonal terms are zero except in the twist-EKF hybrid path.

## Fill modes

Implementation: `fillPoseCovariance()` in `lidar_localization_component.cpp`,
policy in `pose_covariance_policy.hpp`. Select with `pose_covariance_mode`:

| `pose_covariance_mode` | Behavior |
| --- | --- |
| `error_floor` (default) | calibrated floor + fitness slope, see below |
| `fitness_scaled` | legacy v1.1 heuristic (`0.01 * max(1, fitness)` etc.) |

### Mode `error_floor` (default since 2026-06-12)

```text
std = clamp(floor + per_fitness * fitness_score, floor, max)
variance = std^2
```

| Axis | floor | slope (per fitness unit) | max | Parameters |
| --- | --- | --- | --- | --- |
| x, y | `0.2 m` | `0.1 m` | `5.0 m` | `covariance_xy_floor_std_m`, `covariance_xy_std_per_fitness_m`, `covariance_xy_max_std_m` |
| z | `0.3 m` | shares xy slope | shares xy max | `covariance_z_floor_std_m` |
| yaw | `2.0 deg` | `1.0 deg` | `30 deg` | `covariance_yaw_floor_std_deg`, `covariance_yaw_std_per_fitness_deg`, `covariance_yaw_max_std_deg` |
| roll, pitch | `1.5 deg` | shares yaw slope | shares yaw max | `covariance_roll_pitch_floor_std_deg` |

### Why a floor model

Calibration against ground truth (`scripts/analyze_pose_covariance_calibration.py`,
report archived at
[`experiments/pose_covariance/calibration_2026_06_12.json`](../experiments/pose_covariance/calibration_2026_06_12.json))
showed that **within the accepted-fitness range the actual error is dominated by
a per-dataset floor, not by the fitness score**:

| Run | matched poses | abs xy error p68 at fitness < 0.5 | p68 at fitness 2–4 |
| --- | --- | --- | --- |
| Koide outdoor_hard_01a (180 s) | 328 | 0.18 m | 0.17 m |
| Koide outdoor_hard_01a (repeat) | 186 | 0.18 m | 0.24 m |
| Autoware Istanbul (60 s) | 114 | 0.26 m | ~3 m (n=2, degenerate) |

This confirms the observation in #72 that fitness is not linearly related to
error across environments. The defaults target ~1 sigma per-axis coverage on
the worst observed floor (Istanbul) and are conservative on tighter maps:

| Run | per-axis xy coverage 1σ / 2σ / 3σ | yaw coverage 1σ / 2σ / 3σ |
| --- | --- | --- |
| Koide (180 s) | 0.95 / 0.98 / 0.99 | 0.90 / 0.99 / 1.00 |
| Koide (repeat) | 0.94 / 1.00 / 1.00 | 0.86 / 0.96 / 1.00 |
| Istanbul | 0.69 / 0.98 / 0.99 | 0.96 / 1.00 / 1.00 |

For comparison, the legacy `fitness_scaled` mode measured 1σ xy coverage of
0.42–0.59 (Koide) and 0.46 (Istanbul) — systematically overconfident.

### Mode `fitness_scaled` (legacy v1.1)

Kept for compatibility. Base diagonal variances at `fitness <= 1.0`:
x/y `0.01`, z `0.05`, roll/pitch `0.001`, yaw `0.0005`, each multiplied by
`max(1.0, fitness_score)`. Not calibrated; treat as a relative hint only.

### Twist EKF hybrid

When `use_twist_ekf` is enabled and initialized, `x`, `y`, `z` variances (and
the `x-y` cross terms) and `yaw` come from the EKF state covariance. Roll and
pitch come from the active mode's model.

## What the covariance can and cannot promise

Can (in `error_floor` mode, on environments comparable to the calibration data):

- per-axis x/y/yaw 2σ bounds that held for ≥ 96 % of accepted poses on the
  calibration runs
- monotone loosening as match quality (fitness) degrades

Cannot:

- bound the **systematically biased tail**: degenerate geometry can produce
  accepted poses with multi-meter error at *low* fitness (observed: 4.2 m at
  fitness 0.5–1.0 on Koide). No fitness-based covariance can represent this;
  arbitration logic must also watch `/alignment_status`
  (`failure_category`, `consecutive_rejected_updates`,
  `reinitialization_requested`).
- claim NDT posterior uncertainty or a Cramér–Rao bound (a Hessian-inversion
  path was evaluated by the community in #72 and found unreliable in practice)
- transfer to environments with a worse error floor than the calibration data
  without re-running the calibration (see below)

## Recalibrating for your platform

1. Run a benchmark replay with ground truth
   (see [benchmarking.md](benchmarking.md)) to get `pose_trace.csv`,
   `alignment_status.csv`, `trajectory_eval.json`.
2. `python3 scripts/analyze_pose_covariance_calibration.py mylabel=/path/to/run`
3. Pick floors near the reported p68 errors at low fitness; verify per-axis
   coverage with `--model`.
4. Override the `covariance_*` parameters.

## Practical usage guidance

| Use case | Recommendation |
| --- | --- |
| RViz / logging | safe |
| `robot_localization` / EKF fusion | usable in `error_floor` mode; gate inputs with `/alignment_status` `failure_category != healthy` as well |
| Arbitration between two pose sources | usable for 2σ-level comparisons; never trust covariance alone in degenerate geometry |
| Safety-critical gating | covariance is not sufficient; use `/alignment_status` reject reasons and `reinitialization_requested` |

## Related docs

- [troubleshooting.md](troubleshooting.md) — `/alignment_status` field guide,
  `failure_category` taxonomy
- [benchmarking.md](benchmarking.md) — public replay evaluation
- [reliability_roadmap.md](reliability_roadmap.md) — issue triage
