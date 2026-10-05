# Site Setup (Repeat Route)

One-page workflow for bringing up **map-based LiDAR localization with automatic
cold start and guarded recovery** on a site where you already have a mapping run.

Use this when the robot follows a **known route** (warehouse aisle, outdoor loop,
etc.). Supply a reference trajectory CSV so G2 seeds candidates near the route
instead of searching the full map (avoids corridor alias on hard outdoor maps).

**Limit:** route-crop selects reference poses by the scan's timestamp, so it only
re-localizes scans from the mapping run itself (replays and benchmarks). A robot running
on a later day is outside that time span, and route-crop reports
`scan is ... s outside the reference trajectory` instead of guessing. For a live robot,
G2 then uses map-wide BBS over the occupancy grid quickstart generates from the map
(or the one passed with `--occupancy-map`).

## What you need

| Asset | Purpose |
| --- | --- |
| 3D map `.pcd` or `.ply` | NDT localizer target |
| Mapping-run reference CSV | Route-crop G2 candidates (`stamp_sec`, pose columns) |
| Occupancy YAML (optional) | Map-wide BBS for scans outside the mapping run; quickstart generates one from the 3D map when omitted |
| LiDAR (+ IMU for MID-360) | Live `/cloud` and optional `/imu` |

## 1. Create the reference CSV

From a mapping bag and a TUM/GT trajectory aligned to the map frame:

```bash
ros2 run lidar_localization_ros2 tum_trajectory_to_pose_reference_csv_for_rosbag2.py \
  --trajectory /path/to/mapping_trajectory.txt \
  --bag /path/to/mapping_bag \
  --output-csv /path/to/site_reference.csv \
  --output-initial-pose-yaml /path/to/site_initial_pose.yaml
```

The CSV must match the format used by `make_route_grid_relocalization_attempts.py`
(columns: `stamp_sec`, `position_x/y/z`, `orientation_x/y/z/w`).

Tip: reuse the benchmark reference under `benchmark/<sequence>/reference.csv` when
you already have a Koide-style dataset layout.

## 2. Start localization (one command)

```bash
ros2 run lidar_localization_ros2 quickstart.py \
  --profile mid360 \
  --map /absolute/path/to/map.pcd \
  --reference-csv /absolute/path/to/site_reference.csv \
  --global-seed-z -11.05 \
  --cloud-topic /livox/points \
  --imu-topic /livox/imu
```

What this launches:

1. Core localizer (NDT against the 3D map)
2. G2 with **route-crop** candidates + 3D NDT scoring
3. Guarded **G3** supervisor (re-query + `/initialpose` when tracking is lost)
4. Startup manager (saved pose → global search → RViz fallback)

Initialization order: explicit pose → saved pose (same map hash) → route-crop
global search → RViz **2D Pose Estimate**.

Disable automatic recovery if you only want cold start:

```bash
  --no-g3-recovery
```

Dry-run (print launch command only):

```bash
  --dry-run --no-rviz
```

## 3. Check that it is working

| Check | Command / topic |
| --- | --- |
| Pose publishing | `ros2 topic echo /pcl_pose --once` (or `/localization/pose_with_covariance` on MID-360) |
| Tracking health | `ros2 topic echo /alignment_status --once` → `failure_category: healthy` |
| Cold-start status | `ros2 topic echo /startup_initialization/status --once` (JSON) |
| TF | `ros2 run tf2_ros tf2_echo map base_link` (standalone) or `map odom` (Nav2) |

Bringup smoke test (runs automatically 3 s after launch unless disabled):

```bash
ros2 run lidar_localization_ros2 check_lidar_localization_bringup.py \
  --profile mid360 --duration-sec 5
```

## 4. When something fails

| Symptom | What to do |
| --- | --- |
| `no_safe_automatic_source` in startup status | Add `--reference-csv` or set pose in RViz |
| `map_mismatch` | Map file changed since pose was saved; ignore old state or delete `~/.local/state/lidar_localization_ros2/<map>.json` |
| `weak_candidate_rejected` / G3 never resets | Robot moving during G2 query — pause or slow down; check `registration_fitness` in G2 JSON (outdoor maps often need fitness ≤ ~5 to publish and ≤ ~2 to confirm) |
| `failure_category: bad_match` | Wrong seed or map overlap; re-seed in RViz |
| `failure_category: stale_prediction` | Tracking lost — G3 should query if `--g3-recovery` (default) |
| High CPU | Reduce `voxel` / NDT threads in generated config; see [troubleshooting.md](troubleshooting.md) |

G2 service manual query (debug):

```bash
ros2 service call /global_localization_node/query std_srvs/srv/Trigger
```

Inspect JSON `candidate_source` (`route_crop`), `runtime_sec`, top candidate
`registration_fitness`, and `score`.

## 5. Optional: bag replay validation

Replay a recorded sequence through the recovery stack as shown in
[global_localization.md](global_localization.md#replaying-a-bag-through-the-recovery-stack).

## Related docs

- [quickstart.md](quickstart.md) — all CLI flags
- [global_localization.md](global_localization.md) — G2/G3 parameters
- [frame_contract.md](frame_contract.md) — TF and topic names
- [troubleshooting.md](troubleshooting.md) — `failure_category` detail
