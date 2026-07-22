# Troubleshooting

This guide covers the most common bringup failures reported in open issues
[#70](https://github.com/rsasaki0109/lidar_localization_ros2/issues/70),
[#35](https://github.com/rsasaki0109/lidar_localization_ros2/issues/35),
[#43](https://github.com/rsasaki0109/lidar_localization_ros2/issues/43), and
[#48](https://github.com/rsasaki0109/lidar_localization_ros2/issues/48).

For frame-tree and TF questions, see [frame_contract.md](frame_contract.md).
For open-issue triage, see [reliability_roadmap.md](reliability_roadmap.md).

## Symptom: Node Starts But Pose Never Updates

This is the most common "it launches but does not localize" pattern.

### Minimum Bringup Checklist

Work through these in order:

1. **Map loaded**
   - `use_pcd_map:=true` with a valid `map_path`, or a `/map` publisher is running
   - log should show `Map Size <N>` with `N > 0`
2. **Initial pose set**
   - publish `/initialpose` with `header.frame_id = map`
   - RViz **2D Pose Estimate** is the usual path; see [frame_contract.md](frame_contract.md)
3. **LiDAR cloud arriving**
   - `ros2 topic hz <cloud_topic>` shows a steady rate
   - point cloud has `x`, `y`, `z` fields (`PointCloud2`)
4. **Frames consistent**
   - `base_frame_id` matches your robot TF
   - lidar frame is reachable from `base_link` if TF is used

Quick smoke:

```bash
source scripts/setup_local_env.sh
ros2 launch lidar_localization_ros2 nav2_lidar_localization.launch.py \
  map_path:=/absolute/path/to/map.ply \
  cloud_topic:=/your/points

# other terminal
ros2 topic echo /alignment_status --once
ros2 topic echo /pcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

If `/pcl_pose` never updates, inspect `/alignment_status` next.

## Reading `/alignment_status`

Topic: `/alignment_status` (`diagnostic_msgs/msg/DiagnosticArray`)

Each update publishes one status named `lidar_localization_ros2/alignment`.
The `message` field is the primary reject reason. Key-value pairs carry context.

### Startup flags (check first)

| Key | Meaning when `false` |
| --- | --- |
| `map_received` | map not loaded yet; wait for `map_path` load or `/map` |
| `initialpose_received` | no valid `/initialpose` in `map` frame yet |

### IMU preintegration state

When `use_imu_preintegration` is enabled, these keys explain why the IMU seed is
or is not being used:

| Key | Meaning |
| --- | --- |
| `imu_preintegration_status` | one of `imu_preintegration_prediction_active`, `imu_preintegration_waiting_for_imu`, `imu_preintegration_waiting_for_smoother_initialization`, `imu_preintegration_transform_unavailable`, `imu_preintegration_non_finite_sample`, `imu_preintegration_invalid_delta_time`, `imu_preintegration_waiting_for_new_imu`, `imu_preintegration_stale_imu`, `imu_preintegration_prediction_non_finite`, or `imu_preintegration_fallback_mode` |
| `imu_received_sample_count` / `imu_integrated_sample_count` | IMU samples seen vs samples actually integrated for the current scan interval |
| `registration_seed_source` | registration seed source for the scan, such as `imu_preintegration`, `twist_prediction`, `previous_delta`, `current_pose`, or `not_selected` |
| `imu_skipped_sample_count` | samples skipped before integration |
| `imu_transform_failure_count` | samples skipped because `base_frame <- imu_frame` TF was unavailable |
| `imu_invalid_dt_count` / `imu_last_dt_sec` | samples skipped because timestamps were duplicated, reversed, or too far apart |
| `imu_last_sample_age_sec` | scan stamp minus latest IMU stamp; large values mean the IMU stream is late or stopped |
| `imu_integration_window_sec` | IMU time available since the previous scan update |
| `imu_preintegration_fallback_mode` | `true` after the correction guard disables IMU preintegration for the run |

If the status is `imu_preintegration_waiting_for_imu`, check `/imu` remapping and
the bringup doctor's `--require-imu` check. If it is
`imu_preintegration_stale_imu`, check timestamp synchronization before tuning
NDT or IMU noise. If `imu_received_sample_count` is nonzero but
`imu_integrated_sample_count` is zero, inspect the skip counters before tuning
noise values.

For `--imu-mode preintegration`, successful scan updates should normally report
`registration_seed_source=imu_preintegration`. If IMU counters are healthy but
the seed source is `previous_delta` or `twist_prediction`, check whether the IMU
smoother is initialized, whether new IMU samples arrived after the previous
scan, and whether `imu_preintegration_status` is active.

### Per-point scan time state

These keys show whether the incoming `PointCloud2` stream has usable per-point
timing for future continuous-time / deskew work. They are diagnostics only; the
default registration path remains scan-to-map NDT/GICP.

| Key | Meaning |
| --- | --- |
| `scan_time_status` | one of `scan_time_range_ready`, `scan_time_field_missing`, `scan_time_field_invalid`, or `scan_time_range_too_large` |
| `scan_time_field` | field selected from `time`, `t`, `timestamp`, or `offset_time` |
| `scan_time_duration_sec` | observed scan span after normalizing per-point times |
| `scan_time_valid_point_count` / `scan_time_invalid_point_count` | points with readable vs unreadable timing |
| `deskew_ready` | `true` only when scan timing and IMU preintegration are both ready for a future deskew stage |
| `deskew_readiness_status` | first blocker for continuous-time deskew inputs, such as `deskew_scan_time_field_missing`, `deskew_imu_transform_unavailable`, or `deskew_ready` |
| `continuous_time_deskew_enabled` / `continuous_time_deskew_applied` | whether the experimental runtime hook is enabled and used on the latest scan |
| `continuous_time_deskew_status` | applied/skip reason, such as `continuous_time_deskew_applied`, `continuous_time_deskew_scan_time_not_ready`, or `continuous_time_deskew_waiting_for_new_imu` |
| `continuous_time_deskew_point_count` | points deskewed before optional voxel filtering |
| `continuous_time_deskew_skipped_invalid_time_count` / `continuous_time_deskew_clamped_time_count` | invalid or out-of-range per-point times kept safe by the hook |

Run the bringup doctor with `--require-cloud-time-field` when you want this to
be a hard gate. `create_lidar_localization_config.py` enables deskew by default for
profiles that use IMU preintegration and prints a doctor command that requires IMU
data plus per-point cloud timing. Use `--no-enable-continuous-time-deskew` to opt out.
LiDAR-only profiles keep running without deskew, and the doctor reports invalid or
oversized timing as a warning.

During bag replay, `validate_lidar_localization_imu.py` summarizes these same
diagnostics over a time window and fails if IMU preintegration is not active
often enough. When generating a bag replay launch command with
`create_lidar_localization_config.py`, add `--use-sim-time` so localization
uses `/clock`:

```bash
ros2 run lidar_localization_ros2 validate_lidar_localization_imu.py \
  --duration-sec 30 \
  --min-imu-active-ratio 0.5 \
  --require-imu-seed-source
```

`--require-imu-seed-source` is the strict check for "IMU preintegration is
actually seeding registration"; it fails unless enough samples report
`registration_seed_source=imu_preintegration`.

### Failure category (one-key triage)

The `failure_category` key classifies the current state into a single dominant
cause so you do not have to reverse-engineer the message:

| Category | Meaning | First thing to check |
| --- | --- | --- |
| `healthy` | alignment accepted and timely | nothing |
| `missing_map` | no map loaded | `map_path` / `/map` topic |
| `missing_initial_pose` | no valid `/initialpose` yet | send `/initialpose` in `map` frame |
| `weak_overlap` | too few points to constrain the match (scan side or `local_map_crop_too_small`) | scan filters, FOV, initial pose vs map crop |
| `bad_match` | registration did not converge or score is over the effective threshold | initial pose, map alignment, sensor frame |
| `stale_prediction` | no measurement accepted for a while; pose is coasting on prediction | upstream cause of rejects; consider re-sending `/initialpose` |
| `overload` | alignment is slower than the configured budget | CPU load, `ndt_num_threads`, voxel sizes |

Categories are prioritized in the order above; the companion boolean keys
`weak_overlap_active`, `bad_match_active`, `stale_prediction_active`, and
`overload_active` stay `true` for every condition that holds, so co-occurring
causes remain visible. Thresholds are parameters:
`diagnostics_weak_overlap_min_filtered_points` (default `100`),
`diagnostics_stale_prediction_min_gap_sec` (default `2.0`),
`diagnostics_overload_alignment_time_sec` (default `0.3`, `<= 0` disables).
These keys are diagnostics only — they never change acceptance behavior.

### Common `message` values

| Message | Likely cause | What to try |
| --- | --- | --- |
| `ok` | last alignment accepted | localization is running |
| `scan_missing_xyz_field` | cloud is not `PointCloud2` with x/y/z | fix driver conversion (common with custom lidar bridges) |
| `filtered_scan_empty` | crop/range filter removed all points | check `min_scan_range` / `max_scan_range`, sensor FOV |
| `local_map_crop_too_small` | predicted pose is outside local map crop | wrong initial pose or map frame offset; re-send `/initialpose` |
| `registration_not_converged` | NDT/GICP did not converge | initial pose too far from truth; try closer seed |
| `fitness_score_over_threshold_rejected` | match quality too poor | fix initial pose, map alignment, or sensor frame |
| `fitness_score_over_threshold_consistency_recovered` | borderline reject overridden by consistency gate | monitor; may recover if environment is consistent |
| `recovery_retry_from_last_pose_recovered` | retry from last good pose succeeded | transient glitch; usually OK |
| `gtsam_update_rejected` / `ekf_update_rejected` | backend smoother rejected the update | often follows repeated measurement rejects |

### Useful numeric keys

| Key | How to use it |
| --- | --- |
| `fitness_score` | lower is better; compare to `effective_score_threshold` |
| `consecutive_rejected_updates` | high streak means seed or map is wrong |
| `seed_translation_since_accept_m` | large drift since last accept → open-loop problem |
| `filtered_point_count` | very low → scan preprocessing or crop issue |
| `reinitialization_requested` | `true` means node wants a new `/initialpose` |

Watch live:

```bash
ros2 topic echo /alignment_status
```

For benchmark runs, record diagnostics to CSV with `benchmark_diagnostic_recorder`
(see [benchmarking.md](benchmarking.md)).

## Symptom: Direct Positioning Got Stuck (#35)

"Stuck" usually means repeated rejects, not a hung process.

1. echo `/alignment_status` and note `message` + `consecutive_rejected_updates`
2. if `fitness_score_over_threshold_rejected`:
   - re-check initial pose in RViz (roughly correct position and yaw)
   - confirm map and trajectory are in the same coordinate convention
3. if `local_map_crop_too_small`:
   - initial pose is far from where scans actually are
   - or `local_map_radius` is too small for your map scale
4. if `registration_not_converged`:
   - move initial pose closer manually, then let scan matching refine

Do not expect automatic relocalization in v1.1 runtime; use a new `/initialpose`
when `reinitialization_requested` stays true.

## Symptom: Map Not Visible in RViz (#43, #48)

Two different map paths exist:

| Source | Topic / param | When it appears |
| --- | --- | --- |
| PCD/PLY file | `map_path` + `use_pcd_map:=true` | after node loads file; also publishes `/initial_map` |
| External map server | `/map` subscription | when another node publishes the map |

RViz tips:

- set **Fixed Frame** to `map`
- add **PointCloud2** display on `/initial_map` or `/map` depending on your launch
- live localization cloud is separate from the static map topic

If the map displays but localization fails, the issue is usually initial pose or
scan frame mismatch, not map publishing.

## Symptom: Pose Looks Offset From Reality (#44, #75)

See the full map-alignment guide in [map_alignment.md](map_alignment.md).

Separate these cases:

1. **Consistent offset** (always wrong by ~same translation/rotation)
   - map build frame vs localization frame mismatch
   - initial pose yaw error
   - check map origin and `/initialpose` seed
2. **Drift over time**
   - tuning / environment issue; see public benchmark limits in [benchmarking.md](benchmarking.md)
   - Istanbul public replay can show late-run drift even when early error is `< 0.1 m`
   - public preset enables `recovery_retry_from_last_pose` (`r3`, `gap<=1 s`, `seed<=15 m`); inspect
     `recovery_retry_from_last_pose_recovered` and late-run `seed_translation_since_accept_m`
3. **Jumps after rejects**
   - inspect `/alignment_status` reject streak before the jump

Collect before filing an issue:

- map format and `map_path`
- `/initialpose` used
- 3–5 `/alignment_status` samples around failure
- `ros2 run tf2_ros tf2_echo map base_link`

## Pose Covariance Questions (#72)

`/pcl_pose` publishes `geometry_msgs/msg/PoseWithCovarianceStamped`. Semantics
are documented in [pose_covariance.md](pose_covariance.md).

Short version:

- the default `error_floor` mode is calibrated against ground truth on public
  datasets (per-axis 2σ coverage ≥ 96 % on the calibration runs) and is usable
  for EKF fusion, with documented limits
- it still cannot bound systematically biased degenerate matches; gate fusion
  inputs with `/alignment_status` `failure_category` as well
- the legacy v1.1 heuristic remains available as
  `pose_covariance_mode: fitness_scaled`

## Sensor-Specific Notes

| Sensor / setup | Check |
| --- | --- |
| Livox MID-360 | [mid360_legged_jetson.md](mid360_legged_jetson.md), `check_lidar_localization_bringup.py --profile mid360` |
| Ouster / Velodyne / RoboSense | set `cloud_topic` and lidar `frame_id` in launch YAML |
| Nav2 full stack | need 3D map + 2D `map_yaml` + `odom -> base_link`; see README Nav2 section |

## Crash / TF Issues (Sprint 1 fixes)

| Symptom | Doc |
| --- | --- |
| RViz "No map frame" / 2D pose crash | [frame_contract.md](frame_contract.md) — TF now published on `/initialpose` |
| `use_odom:=true` crash at startup | set `/initialpose` before odom integrates; use current `main` |
| Strange odom TF tree | pick Mode A (`map->base_link`) or Mode B (`map->odom` + external odom) |

## Still Stuck?

1. run the public demo to verify your environment can reproduce a known-good path:

   ```bash
   source scripts/setup_local_env.sh
   scripts/run_public_demo.sh
   ```

2. open a GitHub issue with:
   - ROS distro
   - launch command
   - `/alignment_status` message and key values
   - whether `map_received` and `initialpose_received` are `true`
