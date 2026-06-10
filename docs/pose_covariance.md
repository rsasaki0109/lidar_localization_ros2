# Pose Covariance Semantics

This document explains the covariance published on `/pcl_pose` for issue
[#72](https://github.com/rsasaki0109/lidar_localization_ros2/issues/72).

It describes **current v1.1 behavior**. The values are useful as a coarse
confidence hint and diagnostics aid; they are **not** a calibrated sensor-fusion
measurement model yet.

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
`(x, y, z, roll, pitch, yaw)`:

| Index | Element |
| --- | --- |
| 0, 1, 2 | `x` variance / `x-y` / `x-z` |
| 6, 7, 8 | `y-x` / `y` variance / `y-z` |
| 12, 13, 14 | `z-x` / `z-y` / `z` variance |
| 21, 28, 35 | `roll`, `pitch`, `yaw` variances (diagonal terms used) |

Off-diagonal rotation terms are zero in the default fitness-based path.

## Two fill modes

Implementation: `fillPoseCovariance()` in `lidar_localization_component.cpp`,
policy in `pose_covariance_policy.hpp`.

### Mode 1: fitness-scaled heuristic (default)

Used when twist EKF output is **not** active.

Base diagonal variances (at `fitness_score <= 1.0`):

| Axis | Variance |
| --- | --- |
| x | `0.01` |
| y | `0.01` |
| z | `0.05` |
| roll | `0.001` |
| pitch | `0.001` |
| yaw | `0.0005` |

Scaling:

```text
scale = max(1.0, fitness_score)
variance_axis *= scale   # for x, y, z, roll, pitch, yaw as implemented
```

Interpretation:

- lower `fitness_score` → tighter covariance
- poor matches above threshold should be rejected before publish; if accepted,
  covariance grows with `fitness_score`

`/initialpose` published at startup uses zero covariance until the first accepted
scan update.

### Mode 2: twist EKF hybrid

Used when `use_twist_ekf` is enabled and the EKF is initialized.

- `x`, `y`, `z` variances (and `x-y` cross term) come from the EKF state covariance
- `roll`, `pitch`, `yaw` variances still use the fitness-scaled heuristic

This mode is **not** the default on the public Istanbul no-IMU benchmark path.

## What the covariance is not

Do **not** assume:

- NDT posterior uncertainty or Cramér–Rao bound
- full 6×6 pose uncertainty from scan geometry
- compatibility with `robot_localization` defaults without retuning
- interchangeability with GNSS `/pose_with_covariance` noise models

The matrix is a **pragmatic output placeholder** so downstream nodes have finite
values and can observe relative tightening/loosening with match quality.

## Practical usage guidance

| Use case | Recommendation |
| --- | --- |
| RViz / logging | safe |
| Compare runs qualitatively | compare trend with `/alignment_status` `fitness_score` |
| Arbitration between two pose sources | not yet; treat as experimental |
| `robot_localization` fusion | requires custom noise tuning; not documented as supported |
| Safety-critical gating | use `/alignment_status` and explicit reject reasons instead |

Better gating signals today:

- `/alignment_status` message and keys (`fitness_score`, `consecutive_rejected_updates`)
- `reinitialization_requested` on `/reinitialization_request`

## Example inspection

```bash
source scripts/setup_local_env.sh
ros2 topic echo /pcl_pose --once
ros2 topic echo /alignment_status --once
```

Compare `fitness_score` from `/alignment_status` with the diagonal entries on
`/pcl_pose`. After a few accepted updates, larger fitness should correspond to
larger position variances in fitness-scaled mode.

## Roadmap note

Issue #72 asks for clearer covariance semantics. v1.1 closes the documentation
gap and explicitly **does not** claim fusion-grade calibration. Future work may:

- expose which fill mode is active in diagnostics
- add optional static covariance overrides for known platforms
- document a supported `robot_localization` integration pattern once validated on
  public benchmarks

## Related docs

- [troubleshooting.md](troubleshooting.md) — `/alignment_status` field guide
- [benchmarking.md](benchmarking.md) — public replay evaluation
- [reliability_roadmap.md](reliability_roadmap.md) — issue triage
