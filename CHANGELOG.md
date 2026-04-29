# Changelog

## 1.0.0 - 2026-03-30

Initial repo-ready Nav2-focused release.

- added Nav2-oriented launch flows with `nav2_lidar_localization.launch.py` and `nav2_navigation.launch.py`
- added smoke and replay helpers including `run_nav2_demo_smoke`, `run_nav2_replay_smoke`, and `send_nav2_goal.py`
- added occupancy map generation from PCD with `generate_occupancy_map_from_pcd.py`
- added benchmarking and comparison helpers for rosbag-based localization evaluation
- added local-map crop, twist-based prediction, borderline seed rejection gate, diagnostics, and replay-oriented utilities
- fixed `NDT_OMP` runtime keep-alive issues seen in replay and fixed the shutdown-only `SIGINT` crash path
- set the recommended Nav2 preset to `param/nav2_ndt_urban.yaml`

Known limitation at this release:

- long-duration urban replay robustness is improved enough for smoke and controlled validation, but not yet solved well enough to claim fully stable long-horizon operation in all cases
