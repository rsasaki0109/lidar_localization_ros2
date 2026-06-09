# Frame And TF Contract

This document answers common frame-tree confusion reported in issues
[#58](https://github.com/rsasaki0109/lidar_localization_ros2/issues/58) and
[#27](https://github.com/rsasaki0109/lidar_localization_ros2/issues/27).

## Default Frame IDs

| Parameter | Default | Role |
| --- | --- | --- |
| `global_frame_id` | `map` | Localization output frame |
| `odom_frame_id` | `odom` | Odometry parent frame when map->odom TF is enabled |
| `base_frame_id` | `base_link` | Robot base frame |

`/initialpose` must use `header.frame_id = global_frame_id` (default: `map`).

## Two Supported TF Modes

### Mode A: direct `map -> base_link` (default)

- `enable_map_odom_tf: false`
- the localizer publishes `map -> base_link`
- this is the simplest bringup path for localization-only replay

### Mode B: `map -> odom` with external `odom -> base_link`

- `enable_map_odom_tf: true`
- the localizer publishes only `map -> odom`
- another node must already publish `odom -> base_link` (wheel odom, `robot_localization`, replay helper, etc.)
- if `odom -> base_link` is missing, TF lookup fails and map->odom is not published

Use Mode B for Nav2-style stacks. Use Mode A for quick rosbag localization smoke.

## Odom Topic vs Odom TF

These are different inputs:

| Input | Parameter | What it does |
| --- | --- | --- |
| `/odom` topic | `use_odom: true` | integrates twist into the localization pose estimate before scan matching |
| `odom -> base_link` TF | external publisher | required only when `enable_map_odom_tf: true` |

Issue [#27](https://github.com/rsasaki0109/lidar_localization_ros2/issues/27) asks for odom TF instead of the odom topic.
Today the supported path is:

- pose prediction from `/odom` topic when `use_odom: true`
- map->odom composition from TF when `enable_map_odom_tf: true`

There is no built-in "consume only odom TF, ignore odom topic" mode.

## Nav2 Bringup Checklist

For `nav2_navigation.launch.py`:

1. provide `map_path` (3D localizer map) and `map_yaml` (2D occupancy map)
2. provide `/odom` or enable `publish_identity_odom:=true` only for smoke tests
3. send `/initialpose` in `map` frame before expecting localization
4. keep `cloud_topic`, `base_frame_id`, and lidar TF consistent

## Quick Diagnostics

```bash
source scripts/setup_local_env.sh

# frames present?
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link

# initial pose frame correct?
ros2 topic echo /initialpose --once

# localization health
ros2 topic echo /alignment_status --once
```

## RViz 2D Pose Estimate Checklist (issue #47)

1. set RViz **Fixed Frame** to `map`
2. wait until the pointcloud map is visible (or `use_pcd_map` load finished)
3. use **2D Pose Estimate** (publishes `/initialpose`)
4. confirm the tool uses `map` frame (not `odom` or `base_link`)
5. check the node log:
   - `published map -> base_link TF from initial pose` means RViz can resolve `map`
   - `initial pose ignored: frame_id ...` means the frame is wrong
   - `initial pose accepted before map is ready` means pose was stored early; wait for map load

## Common Failure Patterns

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| RViz: "No map frame" | fixed frame not set to `map`, or TF not published yet | set RViz fixed frame to `map`; publish `/initialpose` once (recent builds publish `map` TF immediately) |
| `/initialpose` ignored | `header.frame_id` is not `map` | republish with `frame_id: map` |
| node dies right after 2D pose | stale scan replay or bad odom startup order | use current `main`; check logs for non-finite pose / map-not-ready warnings |
| strange tf tree with odom | mixed Mode A and Mode B publishers | pick one mode; do not publish both `map->base_link` and `map->odom` unless intentional |
| `use_odom:=true` crash at startup | odom before initial pose | fixed in recent builds; ensure initial pose is set first |
| `enable_map_odom_tf:=true` but no map->odom | missing `odom->base_link` TF | publish wheel odom / identity odom / replay odom first |

## Related Parameters

```yaml
global_frame_id: map
odom_frame_id: odom
base_frame_id: base_link
enable_map_odom_tf: false
use_odom: false
```

For Nav2 urban preset defaults, see `param/nav2_ndt_urban.yaml`.
