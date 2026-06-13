# Map Alignment

This guide answers map-frame and pose-offset questions reported in issues
[#75](https://github.com/rsasaki0109/lidar_localization_ros2/issues/75),
[#44](https://github.com/rsasaki0109/lidar_localization_ros2/issues/44), and
[#68](https://github.com/rsasaki0109/lidar_localization_ros2/issues/68).

For bringup failures, see [troubleshooting.md](troubleshooting.md).
For TF expectations, see [frame_contract.md](frame_contract.md).

## What “map alignment” means here

Localization assumes:

1. the **point cloud map** is expressed in `global_frame_id` (default: `map`)
2. the **initial pose** (`/initialpose` or `set_initial_pose`) is in the same frame
3. incoming scans are transformed into `base_frame_id` before matching

If any of these disagree, pose looks offset, crops fail, or registration rejects repeat.

## Supported map inputs

| Format | Parameter | Notes |
| --- | --- | --- |
| PCD | `map_path` + `use_pcd_map:=true` | primary path; intensity optional |
| PLY | `map_path` + `use_pcd_map:=true` | same loader path as PCD |
| External `PointCloud2` on `/map` | `use_pcd_map:=false` | map must already use `global_frame_id` |

MGRS tiles, GeoTIFF, or projected raster maps are **not** loaded directly.
Convert to a Cartesian point cloud in the frame you intend to localize in, then
set that frame as `global_frame_id`.

## Coordinate conventions

### Local Cartesian map frame (typical)

Most rosbag replay setups — including the public Autoware Istanbul benchmark —
use a fixed local East-North-Up style frame baked into the map build:

- map coordinates are large absolute values (for example `x ≈ 66459`, `y ≈ 43620`)
- `/initialpose` must use the **same origin and axis convention** as the map file
- GNSS reference poses in the Istanbul bag are already in that convention

### Very large coordinates and float32 precision (issue #68)

The map is loaded into a `pcl::PointCloud<pcl::PointXYZI>`, whose XYZ are **32-bit
floats**. float32 keeps ~24 bits of mantissa, so the spacing between representable
values (and therefore the position quantization of every map point) grows with the
coordinate magnitude:

| Coordinate magnitude | float32 step (quantization) |
| --- | --- |
| `~66,000` (Istanbul local frame) | `~0.008 m` (fine) |
| `~500,000` (UTM easting) | `~0.03 m` |
| `~4,000,000` (UTM/MGRS northing) | `~0.25 m` |
| `~8,000,000` | `~0.5 m` |

So a map kept in **absolute MGRS/UTM coordinates** (six- to eight-digit northings)
is silently snapped to a 0.25–0.5 m grid the moment it is loaded — registration can
never do better than that floor, and in RViz the cloud looks coarse or, more often,
**does not appear at all**: RViz/Ogre also renders in float32 with the camera at the
origin, so a cloud millions of metres away is beyond the default view and clipping
planes (this is the symptom reported in #68).

**Fix: localize in a recentered frame, not in absolute MGRS/UTM.** Subtract a fixed
origin offset from the map so its coordinates are small (a few km at most), and use
that same offset for `/initialpose`, any GNSS reference poses, and when interpreting
`/pcl_pose`. This is exactly what the public setups already do — the Istanbul map is
a local ENU frame (`x ≈ 66459`), and the Boreas maps are GT-aligned
`*_loc_frame.pcd` rebuilds rather than absolute coordinates. MGRS tiles are **not**
supported directly (see *Supported map inputs* above); convert and recenter first.

### Common mismatch patterns

| Symptom | Likely cause | What to check |
| --- | --- | --- |
| Consistent XY offset | map built in a different datum or origin shift | compare map build log vs `/initialpose` |
| Consistent yaw offset | initial pose heading wrong by constant angle | re-send `/initialpose` in RViz |
| Offset grows over time | registration drift, not static map misalignment | see `/alignment_status` reject streak |
| `local_map_crop_too_small` | pose seed far outside map bounds | initial pose or map path wrong |
| Map visible, pose never moves | frame id mismatch on `/initialpose` | `header.frame_id` must equal `global_frame_id` |
| Map not displayed in RViz / coarse-looking cloud | absolute MGRS/UTM coordinates exceed float32 precision | recenter the map to a local origin (see *Very large coordinates*, issue #68) |

## Minimum alignment checklist

Work through these before tuning NDT parameters:

1. **Confirm map frame**
   - loaded map log shows `Map Size <N>` with `N > 0`
   - if subscribing to `/map`, verify `header.frame_id == global_frame_id`
2. **Seed pose in map frame**
   - `/initialpose` uses `header.frame_id = map` (or your `global_frame_id`)
   - for benchmark replay, use the extracted YAML from the reference bag window
3. **Verify lidar extrinsics**
   - cloud frame reaches `base_link` through TF or is already in `base_link`
4. **Sanity-check against a known pose**
   - compare first accepted `/pcl_pose` to a reference pose at the same timestamp
   - on Istanbul public data, `benchmark_extract_pose_reference_from_rosbag2` writes both reference CSV and initial-pose YAML from the same bag window

Quick commands:

```bash
source scripts/setup_local_env.sh

# map loaded?
ros2 topic echo /initial_map --once | head

# initial pose frame
ros2 topic echo /initialpose --once

# output pose vs TF
ros2 topic echo /pcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

## Using the public Istanbul reference seed

The demo and regression scripts derive seed pose and GT from the **same bag window**:

```bash
ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 \
  --bag-path data/official/autoware_istanbul/localization_rosbag \
  --pose-topic /sensing/gnss/pose_with_covariance \
  --sample-topic /localization/util/downsample/pointcloud \
  --bag-duration 60 \
  --initial-pose-skip-sec 0.05 \
  --output-csv /tmp/autoware_istanbul_reference_60s.csv \
  --output-initial-pose-yaml /tmp/autoware_istanbul_initial_pose_60s.yaml
```

Template parameters live in `param/public_istanbul_60s_benchmark.yaml`.
The runner merges that template with the extracted initial pose and resolved `map_path`.

If your custom map was **not** built in the same frame as your live sensors,
do not reuse this seed verbatim — align or transform the map first.

## Aligning a custom map

Recommended workflow:

1. **Pick one localization frame** (`map`, site ENU, or robot-centric local frame).
2. **Build the dense map in that frame** (SLAM / mapping tool export to PCD/PLY).
3. **Publish or load with `use_pcd_map:=true`** and keep `global_frame_id` consistent.
4. **Set `/initialpose`** near a recognizable structure in the map.
5. **Inspect `/alignment_status`**:
   - `ok` with low `fitness_score` → alignment is plausible
   - `local_map_crop_too_small` → pose still outside map support
   - repeated `fitness_score_over_threshold_rejected` → yaw/translation seed still wrong

For Nav2, also provide a 2D occupancy map whose origin matches the 3D map frame
within your stack’s expected tolerance.

## Separating map alignment from registration tuning

| Observation | Map alignment issue | Registration tuning issue |
| --- | --- | --- |
| Wrong by ~same transform everywhere | yes | unlikely |
| Wrong only after many seconds | unlikely | yes (drift) |
| Wrong only in part of the route | partial map coverage / wrong tile | yes |
| `/alignment_status` mostly `ok` but RViz looks shifted | TF display / fixed frame | check RViz fixed frame and map topic |
| `/alignment_status` mostly rejects from first scan | yes | maybe threshold, but seed first |

## Collect before opening an issue

- map format, how it was built, and `map_path`
- `global_frame_id`, `base_frame_id`, `cloud_topic`
- `/initialpose` payload (frame + pose)
- 3–5 `/alignment_status` samples around the first reject
- whether offset is **constant** or **growing**

## Related docs

- [troubleshooting.md](troubleshooting.md) — “Pose Looks Offset From Reality”
- [benchmarking.md](benchmarking.md) — public Istanbul reference extraction
- [frame_contract.md](frame_contract.md) — `map` / `odom` / `base_link` contract
