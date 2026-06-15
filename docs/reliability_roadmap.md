# Reliability Roadmap

This document classifies open GitHub issues for the v1.1 reliability sprint.
It is an engineering triage sheet, not a promise that every issue will be fixed immediately.

Last triaged: 2026-06-10

## Priority Order

1. crash on startup / initial pose / `use_odom`
2. TF / frame contract confusion
3. map loading / alignment / RViz display
4. covariance semantics
5. docs / FAQ / sensor-specific questions
6. research enhancements

## Issue Matrix

| Issue | Title | Category | Priority | Status | Next action |
| --- | --- | --- | --- | --- | --- |
| [#76](https://github.com/rsasaki0109/lidar_localization_ros2/issues/76) | initial pose result process died | crash | P0 | released in v1.1.0 | non-finite `/initialpose` rejected; no eager scan replay on reset |
| [#47](https://github.com/rsasaki0109/lidar_localization_ros2/issues/47) | rviz no map frame; node dies on 2D pose | crash | P0 | released in v1.1.0 | publish TF on `/initialpose`; clearer frame/map diagnostics |
| [#56](https://github.com/rsasaki0109/lidar_localization_ros2/issues/56) | crash with `use_odom:=true` | crash | P0 | released in v1.1.0 | skip odom before initial pose; guard non-finite pose integration |
| [#58](https://github.com/rsasaki0109/lidar_localization_ros2/issues/58) | strange tf_tree with odom frame | frame / TF | P1 | documented | see [frame_contract.md](frame_contract.md) Mode A vs B |
| [#27](https://github.com/rsasaki0109/lidar_localization_ros2/issues/27) | use odom TF instead of odom topic | frame / TF | P1 | documented | odom topic vs odom TF clarified; TF-only mode not implemented |
| [#55](https://github.com/rsasaki0109/lidar_localization_ros2/issues/55) | `odom_frame_id_` defined but not used | frame / TF | P2 | open | code audit + doc alignment |
| [#75](https://github.com/rsasaki0109/lidar_localization_ros2/issues/75) | map alignment | map | P1 | documented | see [map_alignment.md](map_alignment.md) |
| [#68](https://github.com/rsasaki0109/lidar_localization_ros2/issues/68) | MGRS map not displayed | map | P2 | documented | float32 precision limit on absolute MGRS/UTM coords; recenter to a local frame ([map_alignment.md](map_alignment.md)) |
| [#48](https://github.com/rsasaki0109/lidar_localization_ros2/issues/48) | map showing alongside live data in rviz | map | P2 | documented | see [troubleshooting.md](troubleshooting.md) map visibility section |
| [#44](https://github.com/rsasaki0109/lidar_localization_ros2/issues/44) | localization pose offset | map | P2 | documented | see [map_alignment.md](map_alignment.md) offset vs drift table |
| [#72](https://github.com/rsasaki0109/lidar_localization_ros2/issues/72) | pose covariance | covariance | P2 | documented | see [pose_covariance.md](pose_covariance.md); no fusion claim yet |
| [#70](https://github.com/rsasaki0109/lidar_localization_ros2/issues/70) | program starts but positioning fails | docs / diagnostics | P1 | documented | see [troubleshooting.md](troubleshooting.md) bringup checklist |
| [#43](https://github.com/rsasaki0109/lidar_localization_ros2/issues/43) | map publishing clarification | docs | P3 | documented | see [troubleshooting.md](troubleshooting.md) map visibility section |
| [#41](https://github.com/rsasaki0109/lidar_localization_ros2/issues/41) | how to reduce drift? | docs | P3 | documented | see [benchmarking.md](benchmarking.md) Istanbul drift tuning section |
| [#33](https://github.com/rsasaki0109/lidar_localization_ros2/issues/33) | about ros2 humble | docs | P3 | open | README now documents Jazzy-first / Humble-compatible |
| [#34](https://github.com/rsasaki0109/lidar_localization_ros2/issues/34) | different sensor like Ouster | docs | P3 | open | point to `cloud_topic` / frame-id configuration |
| [#49](https://github.com/rsasaki0109/lidar_localization_ros2/issues/49) | ERROR run with Rslidar | docs | P3 | open | collect sensor-specific launch params |
| [#35](https://github.com/rsasaki0109/lidar_localization_ros2/issues/35) | direct positioning got stuck | docs / diagnostics | P2 | documented | see [troubleshooting.md](troubleshooting.md) stuck positioning section |
| [#52](https://github.com/rsasaki0109/lidar_localization_ros2/issues/52) | different results | docs / diagnostics | P3 | open | require same map / seed / bag window for comparisons |
| [#37](https://github.com/rsasaki0109/lidar_localization_ros2/issues/37) | high CPU usage | docs | P3 | open | document backend cost and crop settings |
| [#54](https://github.com/rsasaki0109/lidar_localization_ros2/issues/54) | `corrent_pose_with_cov_stamped_ptr_` locking | enhancement | P3 | open | code audit if reproduced under concurrency |
| [#25](https://github.com/rsasaki0109/lidar_localization_ros2/issues/25) | specifying lidar frame id | docs | P3 | open | already configurable; improve launch examples |
| [#50](https://github.com/rsasaki0109/lidar_localization_ros2/issues/50) | enhance stability | enhancement | P4 | open | track through public regression, not ad-hoc tuning |
| [#77](https://github.com/rsasaki0109/lidar_localization_ros2/issues/77) | IMU angular velocity estimator | enhancement | P4 | open | **implemented** (twist prediction / twist EKF / IMU preintegration smoother); documented in [imu_estimation.md](imu_estimation.md). Open only on a controlled LiDAR+IMU+GT accuracy ranking |
| [#36](https://github.com/rsasaki0109/lidar_localization_ros2/issues/36) | imu preintegration | enhancement | P4 | open | **implemented** (Forster on-manifold preintegration + sliding-window smoother + guard); math verified by `test_imu_preintegration`; documented in [imu_estimation.md](imu_estimation.md). Open only on a controlled accuracy ranking |

## Sprint 1 Targets

The first reliability sprint should stay narrow:

1. `#76` `/initialpose` crash or process death
2. `#56` `use_odom:=true` crash
3. `#58` / `#27` frame contract documentation and smoke coverage

Done criteria for sprint 1:

- each P0 issue has either a reproduced test, a documented workaround, or a merged fix

### 2026-06-10 Sprint 1 patch notes

- `#56` / `#76` shared startup race:
  - `odomReceived()` now waits for a valid initial pose and finite current pose
  - non-finite odom integration is rolled back instead of poisoning NDT init guess
  - `initialPoseReceived()` no longer immediately replays a cached pre-reset scan
- unit coverage: `test_odom_integration_policy`
- `#58` / `#27` frame confusion:
  - added [frame_contract.md](frame_contract.md)
  - README now links expected `map` / `odom` / `base_link` behavior
- `#47` RViz initial pose:
  - `/initialpose` now publishes `map` TF immediately when possible
  - frame mismatch / map-not-ready warnings are explicit in node logs
- Sprint 2 troubleshooting (`#70`, `#35`, `#43`, `#48`):
  - added [troubleshooting.md](troubleshooting.md) with bringup checklist and `/alignment_status` guide
  - README links troubleshooting from Frames And TF / Read More
- Sprint 2 map / covariance (`#75`, `#44`, `#72`):
  - added [map_alignment.md](map_alignment.md) for map frame, seed pose, and offset vs drift
  - added [pose_covariance.md](pose_covariance.md) for `/pcl_pose` covariance semantics
  - Istanbul RMSE outlier (`4.74 m`) traced to late-run drift on identical seed/map; fresh rerun `1.21 m`
- Istanbul drift tuning (`#41`):
  - public preset now uses `local_map_crop` + `recovery_retry r3_gap1_seed15`
  - repeat compare median translation `1.17 m` vs baseline `1.66 m`; see [benchmarking.md](benchmarking.md)
- README or `docs/benchmarking.md` links to this roadmap
- fixed issues get a one-line note in `CHANGELOG.md` Unreleased reliability section

## Repro Template

Use this minimum checklist before changing runtime code:

```bash
source scripts/setup_local_env.sh
ros2 launch lidar_localization_ros2 nav2_lidar_localization.launch.py \
  map_path:=/absolute/path/to/map.ply \
  cloud_topic:=/your/points \
  use_odom:=false

# in another terminal
ros2 topic echo /alignment_status --once
ros2 run tf2_ros tf2_echo map base_link
```

For initial-pose crashes, also record:

- whether `map`, `odom`, and `base_link` frames exist
- whether the map path loads successfully
- whether `/alignment_status` reports a reject reason before death
