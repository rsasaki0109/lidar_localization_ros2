# Global Localization

Operations guide for the guarded global-localization stack: recovering a usable
pose with **no** (or a lost) initial pose, and optionally re-seeding tracking
automatically when it diverges. This is the *how to run it* companion to the
design/status document, [global_localization_roadmap.md](dev/global_localization_roadmap.md).

Quickstart enables guarded cold-start global initialization by default whenever a
matching occupancy map is supplied. Without that required asset it remains inactive;
`--no-auto-initialize` disables it explicitly. The continuous G3 lost-tracking recovery
supervisor still uses the dedicated `global_localization_recovery.launch.py` bringup,
and never publishes a reset unless its safety guards pass.

## The three pieces

| Stage | What it does | Entry point |
| --- | --- | --- |
| G1 | Map-wide candidate generation (BBS_2D branch-and-bound over an occupancy grid), artifact-first / offline | `scripts/make_bbs_relocalization_attempts.py` |
| G2 | Runtime service that exposes G1 on demand: ask, get ranked candidates back | `scripts/global_localization_node.py` |
| G3 | Guarded *automatic* reinitialization: watch the lost-tracking signal, query G2, and re-seed `/initialpose` behind safety gates | `scripts/reinitialization_supervisor_node.py` |

G1 and G2 need an occupancy grid built from the map PCD
(`ros2 run lidar_localization_ros2 generate_occupancy_map_from_pcd`). G3 needs a running localization
node *and* a running G2 node.

## G2: on-demand relocalization service

Run the service node against the same map and cloud topic the localizer uses:

```bash
python3 scripts/global_localization_node.py --ros-args \
  -p occupancy_yaml:=/path/to/occupancy.yaml \
  -p cloud_topic:=/velodyne_points \
  -p global_frame_id:=map
```

Trigger a query (it uses the latest scan it has seen):

```bash
ros2 service call /global_localization_node/query std_srvs/srv/Trigger
```

The response `message` is JSON (`candidate_count`, `scan_point_count`,
`runtime_sec`, `scan_stamp_sec`, `scan_age_sec`, `candidate_age_sec`, and the
`top` candidate `x`/`y`/`yaw_deg`/`score`). The full ranked list is also
published once as a `geometry_msgs/PoseArray` on
`/global_localization_node/candidates`. To relocalize manually, publish the top
candidate to `/initialpose`. On moving bags, `candidate_age_sec` is the first
thing to check when a plausible candidate still fails to lock.

Key parameters (see the node for the full list): `z_min_m` / `z_max_m` (scan
height band in the levelled base frame, launch args `g2_z_min_m` / `g2_z_max_m`,
default 0.5 / 5.0 m; the startup log line reports it), `sparse_band_min_points`
(launch arg `g2_sparse_band_min_points`, node default 0 = off, quickstart 512: when the band holds fewer
2D points than this, counted before the `max_scan_points` cap, the query searches
again from the sensor plane up with 1024 points, 32 candidates and NDT-refined poses;
each answer reports `sparse_band_search`), `max_scan_points`, `angular_resolution_deg`, `pyramid_depth`,
`max_candidates`, `nms_radius_m` (node default 2.0 m; launch arg `g2_nms_radius_m`,
3.0 m; quickstart `--global-nms-radius-m`, 3.0 m, or 1.5 m with odometry prediction),
`registration_refine_candidates` (node param;
launch arg `g2_registration_refine_candidates`, default `false`). When enabled,
`g2_ndt_score` publishes each candidate's NDT-refined pose instead of the raw BBS
cell center — required for the HDL hdl_400 recovery replay (raw poses up to ~3 m
off) but harmful on Koide, where refinement can snap aliased hypotheses to
locally-perfect alignments and collapse walk candidates onto one pose. Lower
`nms_radius_m` keeps more near-by or alternate-yaw candidates for G3 candidate
walking; higher values return more spatially diverse candidates. The suppression runs
on the 2D search score, before NDT refinement: where an aliased place 1-3 m away scores
higher on the grid, the right pose is suppressed and never refined, however much better
it registers. On the JEPLO Go2 cross-session replays (with 3 m, 15 map pairs,
1,175 scans) the right pose was among the candidates for 67% of the scans and on top
for 64%; with 1.5 m, 78% and 72%, and the queries took a median 0.46 s instead of
1.16 s. On the EIL_Stairs map with EIL_Box's scans, the true pose registered 4.5 times
better than the returned top (fitness 0.033 against 0.149, median) in 42 of the 48
scans it was missing from, and the top was a place 1-2 m along the room. Closer
candidates also show the startup's nearby-alternative check
([`ambiguous_registration_retry`](quickstart.md#troubleshooting)) the aliases that 3 m
hid, so there are more hand-offs to the operator where a room repeats every 1.5-2 m
(the startup's `global_consensus_near_alias_samples` takes some of them back; quickstart
sets 1.5 m and 3 near-alias views by default when odometry prediction is on, see
[the A/B evaluation](benchmark.md#defaults-with-odometry-prediction)); see
[a map from another session](go2_mid360_quickstart.md#a-map-from-another-session) for
the settings and
[the measurements](benchmark.md#eil_stairs--eil_box-why-starts-were-wrong-and-the-fix).
Query latency
on a validated window is a few seconds; see the roadmap for the speed/coverage
envelope. Most of it is NDT scoring, and preparing the scoring target (map crop
within `ndt_local_map_radius`, voxel filter, NDT cells, k-d tree) costs several
times the alignment. Candidates whose scan, plus 10 m, fits inside one crop share it:
on the Koide outdoor maps this scored 8 candidates 2.9x and 16 candidates 3.7x
faster, with identical fitness.

Three settings make the scoring fast on a map the scan reaches across (launch
arguments `g2_ndt_local_map_radius`, `g2_ndt_num_threads` and `g2_ndt_scan_max_range`;
node parameters without the `g2_` prefix). Their defaults keep the behavior above:

- **`ndt_local_map_radius: 0.0`** (default 150) scores every candidate against the
  whole map. Its NDT target is prepared once, when the node starts, instead of a crop
  per candidate on every query. The node holds that target: about 240 MiB more on the
  3.1 million-point Boreas map.
- **`ndt_num_threads`** (default 1) threads each alignment and its fitness score. The
  KDTREE neighbour search always runs on one thread; the default search is DIRECT7.
- **`ndt_scan_max_range`** (default 0 = off) scores only the scan points within this
  range. Returns from beyond the mapped area otherwise dominate the fitness: on Boreas
  the true pose scored 300-770 with every point and 0.05-0.08 within 80 m.

[On Boreas](#on-the-boreas-fixture-map-wide-queries) they bring a query from 19 s to
2 s.

While a query runs, the node publishes JSON progress on
`/global_localization_node/status` (`std_msgs/String`: `query_id`, `phase`
`started`/`search`/`scoring`/`done`, `done`/`total` counts (`total` is the
number of candidates being scored), `elapsed_sec`). Watch it during a slow query instead of waiting blind:

```bash
ros2 topic echo /global_localization_node/status
```

### Route-crop candidates (opt-in, WP3)

On routes where map-wide BBS returns along-corridor aliases (Koide outdoor north/
south corridor), seed G2 from a reference trajectory near the query scan stamp
instead of searching the full occupancy grid. This skips the BBS search entirely
and is typically much faster; pair with `enable_registration_scoring:=true` so
NDT fitness ranks the route-local hypotheses.

```bash
python3 scripts/global_localization_node.py --ros-args \
  -p candidate_source:=route_crop \
  -p reference_csv:=/path/to/reference.csv \
  -p map_path:=/path/to/map.pcd \
  -p enable_registration_scoring:=true \
  -p cloud_topic:=/livox/points
```

Reference CSV format matches `make_route_grid_relocalization_attempts.py` (columns:
`stamp_sec`, `position_x/y/z`, `orientation_x/y/z/w`). Generate from a mapping run
with `tum_trajectory_to_pose_reference_csv_for_rosbag2.py`. Key parameters:
`route_time_radius_sec`, `route_min_spacing_m`, `route_max_poses`, and the
`*_offsets_*` lists. `global_localization_recovery.launch.py` passes them on as
`g2_candidate_source`, `g2_reference_csv` and `g2_route_*`. The Koide `outdoor_hard_01a`
recovery used:

```bash
ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py \
  cloud_topic:=/livox/points imu_topic:=/livox/imu \
  map_path:=/path/to/map_outdoor_hard.ply \
  localization_param_dir:=/path/to/koide_outdoor.yaml \
  g2_candidate_source:=route_crop \
  g2_reference_csv:=/path/to/outdoor_hard_01a/reference.csv \
  g2_enable_registration_scoring:=true \
  base_frame_id:=livox_frame lidar_frame_id:=livox_frame \
  publish_lidar_tf:=false use_imu_preintegration:=true use_sim_time:=true \
  supervisor_reset_default_z_m:=-11.046818 supervisor_prefer_reset_default_z_m:=true
```

Keep `supervisor_enable_seed_motion_compensation` at its default, `false`, here. A
route-crop query takes 10-20 s, and extrapolating the seed over that latency moved it
tens of metres on a moving bag.

### 3D-BBS as a G2 engine (evaluated 2026-11-21)

G2 searches a 2D occupancy grid made from the 3D map, so in a building with stairs the
room, the stairs and the hall below fall onto the same cells. [3D-BBS](https://github.com/KOKIAOKI/3d_bbs)
(Aoki, Koide et al., ICRA 2024) runs the branch-and-bound over a 3D voxel hierarchy
instead and returns x, y, z and yaw. It was evaluated offline as a replacement for the
2D search, on CPU (no GPU here). Method, scripts and the
reproduction are in [experiments/3d_bbs_eval](../experiments/3d_bbs_eval/README.md).
It is now available as an opt-in engine:
[Using 3D-BBS](#using-3d-bbs-global_search_engine-bbs3d).

- **Licence: MIT.** Usable from this BSD-2 package, also vendored, if its notice is
  kept. Kept external here.
- **Build:** CPU library in 3.6 s (3 files), needs Eigen 3.4, OpenMP, Boost headers and
  Boost.Filesystem, which the library uses but does not link. One answer per search
  (the best pose and its score); the evaluation adds a top-k variant.
- **Same scans and maps** as `g2_offline_probe.py` (one scan per second, levelled with
  the ground-truth attitude), the cross-session suite's G2 settings, and the same NDT
  refinement and ranking for 3D-BBS's candidates. 3D-BBS: 0.5 m finest voxels, 6 levels,
  scan 1-30 m at 0.5 m, the map's whole bounding box in x, y, z, 4 threads. Success:
  top answer within 1 m and 5 degrees (Boreas: 2 m). `Long_Stairs` map → `Outdoor2`
  is scored against the odometry moved into the ground-truth frame and NDT-registered to
  the map every second (corrections at most 0.12 m), so it covers the corridor, the
  stairs and the hall 5 m below, not only the motion-capture room. The Long_Stairs scans
  are no longer on disk, so `Outdoor2` map → `Long_Stairs` was not run.

JEPLO Go2, 14 map → scan pairs, 1,161 queries. Starts are the startup replay
(`decide_startup` from every query, ground truth as odometry): wrong / right / operator.

| method | success | answer > 1.5 m off in height | median / p95 / max query (s) | peak RSS | starts wrong / right / operator |
|---|---|---|---|---|---|
| G2 (2D BBS + NDT on 16) | 763 (66%) | 65 | 1.1 / 3.1 / 4.7 | 150-310 MB | 41 / 789 / 163 |
| 3D-BBS, its single answer | 833 (72%) | 25 | 0.19 / 7.5 / 201 | 150-1,470 MB | 160 / 807 / 26 |
| 3D-BBS single answer + NDT | 865 (75%) | 25 | 0.26 / 7.6 / 201 | 150-1,470 MB | 95 / 823 / 75 |
| 3D-BBS top 8 (3 m apart, 10 s limit) + NDT | **921 (79%)** | 12 | 2.9 / 10.2 / 10.2 | 150-390 MB | **39** / **846** / 108 |

By map:

| map → scans | G2 | 3D-BBS + NDT | 3D-BBS top 8 + NDT |
|---|---|---|---|
| room maps (EIL_Box, EIL_Mix, EIL_Stairs), 10 pairs, 734 queries | 513, 0.9 s | 588, 0.1 s | 619, 1.5 s |
| Long_Stairs map → room runs (EIL_Box, Mask1, Stairs), 264 | 113, 1.8 s | 123, 4.2 s | 149, 10 s |
| Long_Stairs map → Outdoor2 0-165 s, 163 | 137, 2.9 s | 154, 1.1 s | 153, 10 s |

(successes, median query time). In the room maps an answer far off in height is NDT
drifting from a wrong start, not another floor; G2 has 24 of them there.

- **Stairs and the hall below: 3D-BBS's case.** `Long_Stairs` map → `Outdoor2`, by
  segment (success / queries, answers on the wrong level):

  | segment | G2 | 3D-BBS + NDT | 3D-BBS top 8 + NDT |
  |---|---|---|---|
  | room, 0-50 s | 50/50 | 44/50 | 50/50 |
  | corridor, 50-125 s | 71/75, 1 | 75/75 | 75/75 |
  | stairs, 125-145 s | 13/20, **5** | **20/20** | 20/20 |
  | hall 5 m below, 145-165 s | 3/18, **12** | **15/18**, 3 | 8/18 (10 timed out) |

  On the stairs and in the hall G2 put 17 answers on another level, 3D-BBS 3 (in the
  last 3 s, where the robot leaves the part of the hall the map covers). Its weak spot
  there is time: hall queries took 15 s at the median and up to 201 s, and its queue
  peaked at 1.5 GB. The 10 s limit caps both but returns nothing for 10 hall queries.
- **The aliased pairs are not solved.** EIL_Box map → EIL_Stairs: G2 59/135, 3D-BBS
  43/135 with NDT on its single answer and 69/135 with top 8; EIL_Box → EIL_Mix:
  14, 15, 16 of 24. In all three, most misses are the same room turned by 180 degrees
  (G2 50, 3D-BBS 70 and 54 queries on EIL_Box → EIL_Stairs). That is the scene (a box
  platform in the map where the scans have a stair module, in a near-symmetric room),
  not the 2D grid, and the 3D score
  barely separates them (median 0.98 for right, 0.95 for wrong answers; on the
  Long_Stairs map 0.99 for both).
- **A room scan on a large multi-level map** (Long_Stairs map → room runs): the 3D search
  also may pick another level. 3D-BBS alone put 21 of 264 answers more than 1.5 m off in
  height, G2 23, top 8 + NDT 8.
- **One answer is not enough for the startup checks.** The nearby-alternative and
  fitness-ratio checks need other candidates; with the single 3D-BBS answer the replay
  published 95-160 wrong starts against G2's 41. Top 8 at least 3 m apart (G2's
  suppression radius) restores them: 39 wrong, and the most right starts. With 1 m
  suppression the top 8 nearly always held a refined alternative 1-5 m away, and most
  starts went to the operator.
- **Boreas** (the 1 km map is not on disk; the 120 s fixture's 545 x 367 m map, 3.1 M
  points, cross-season scans, 17 queries 7 s apart, 2 m / 5 degrees, G2 with the fast
  settings of [the map-wide Boreas queries](#on-the-boreas-fixture-map-wide-queries)):
  G2 12/17 at 4.5 s median (9.8 s max, 570 MB), 3D-BBS (1 m voxels, scan at 2 m) + NDT
  **17/17** at 8.2 s median (52 s max, 790 MB, 1.7 s to build the voxel map), top 8
  17/17 at 30 s median (60 s limit).
- **Cost on CPU:** the voxel maps take 20-110 ms to build for the JEPLO maps and 1.7 s
  for Boreas. Query time is fast where the scan is distinctive (0.1 s in the rooms) and
  unbounded where it is not: an open hall or a room scan on a big map expands far more
  nodes, with time and queue memory to match. Asking for the k best leaves costs a
  multiple of the single answer. On one thread (every 5th query of EIL_Mix → EIL_Box
  and Long_Stairs → Outdoor2) the search took 3.4 times as long at the median (0.9 and
  2.6 s), and 2 of the 33 Long_Stairs → Outdoor2 queries, in the hall, hit a 60 s limit.

**Recommendation.** Worth adding as an **optional** G2 engine for multi-level maps, not
as the default: on stairs and lower floors it is right where the 2D grid is not, and on
Boreas it placed every scan. It needs (1) a top-k with G2's 3 m suppression so the
startup's alternative checks keep working, (2) a hard time limit and a queue cap (the
library's timeout, or a node budget), and (3) the same NDT refinement and ranking G2
already has. The licence and the dependency (one small library, Eigen/OpenMP/Boost) are
not obstacles. The costs are CPU time on open or repetitive scenes (seconds to minutes
without a limit, so a limit that sometimes returns nothing) and memory during the
search (up to 1.5 GB here). It does not help with the aliased rooms that fooled G2; the
startup checks remain what stops those.

### Using 3D-BBS (`global_search_engine: bbs3d`)

Opt-in; the default stays `bbs2d`, whose answers are unchanged (checked query by query
against the evaluation above on two pairs). Use it for maps with more than one level:
stairs, a mezzanine, a hall below a floor. Keep `bbs2d` on flat maps, on small
computers, and where queries must return in a second or two.

```bash
ros2 run lidar_localization_ros2 quickstart.py --map site.pcd --global-search-engine bbs3d
# or: ros2 launch ... g2_global_search_engine:=bbs3d   (quickstart / global_localization_recovery launch)
# or: global_localization_node.py --ros-args -p global_search_engine:=bbs3d -p map_path:=site.pcd ...
```

What changes: the map-wide search runs over a 3D voxel hierarchy of `map_path` instead
of the 2D grid and returns x, y, z and yaw. Its `bbs3d_top_k` best poses, each at least
`bbs3d_min_separation_m` from a better one, go through the same NDT refinement (always
on for bbs3d) and fitness ranking as the 2D candidates, so the startup's consensus and
nearby-alternative checks work unchanged. The answer JSON gains `global_search_engine`
and `search_info` (`elapsed_ms`, `timed_out`, `queue_capped`, `answers`, ...).

| parameter (node) | default | meaning |
|---|---|---|
| `global_search_engine` | `bbs2d` | `bbs3d` for the 3D search (launch arg `g2_global_search_engine`, quickstart `--global-search-engine`) |
| `bbs3d_top_k` | 8 | answers per query (`g2_bbs3d_top_k`) |
| `bbs3d_min_separation_m` | 3.0 | answers at least this far apart (3D); 1 m left a refined alternative near nearly every top, and most starts went to the operator |
| `bbs3d_relative_score_floor` / `bbs3d_min_score_ratio` | 0.9 / 0.0 | after the best answer, others need 90% of its score; absolute floor as a share of scan points |
| `bbs3d_timeout_sec` | 10.0 | hard limit of one search (`g2_bbs3d_timeout_sec`, quickstart `--bbs3d-timeout-sec`); it returns the answers found so far, possibly none |
| `bbs3d_max_queue_nodes` | 4,000,000 | cap of open nodes (32 bytes each); the search stops like on the time limit |
| `bbs3d_num_threads` | 4 | OpenMP threads (`g2_bbs3d_num_threads`, quickstart `--bbs3d-threads`) |
| `bbs3d_min_level_res_m` / `bbs3d_max_level` | 0.5 / 6 | finest voxel and coarser levels (`g2_bbs3d_min_level_res_m`; 1.0 for large outdoor maps) |
| `bbs3d_map_leaf_m`, `bbs3d_scan_leaf_m`, `bbs3d_scan_min_range_m`, `bbs3d_scan_max_range_m`, `bbs3d_max_tilt_rad` | 0.1, 0.5, 1, 30, 0.02 | map and scan thinning, scan range, roll/pitch searched around the levelled scan |

One search runs at a time: a query that arrives while one runs gets no candidates and
`search_error: busy` instead of queueing (the node's service is single-threaded anyway,
so this matters only for callers that share the engine). Searches cannot pile up or
grow without bound: the time limit and the node cap bound each one.

If `bbs3d` is requested but cannot run (the `bbs3d_cpp` module was not built, `map_path`
is empty or does not load), the node logs an **error** at startup, answers with `bbs2d`,
and every answer carries `global_search_engine: bbs2d` and
`global_search_engine_error`. Invalid `bbs3d_*` values (k < 1, a time limit <= 0, ...)
refuse to start the node. An occupancy grid is still required, for that fallback.

**Build.** No new dependency: the module `bbs3d_cpp` is built whenever pybind11 is
found (like `bbs_cpp`), from three unmodified files of 3D-BBS vendored in
[third_party/bbs3d](../third_party/bbs3d/README.md) (MIT; the licence is installed with
the package) and the top-k search in `include/lidar_localization/bbs3d_topk_search.hpp`.
They need Eigen and the Boost hash headers, which PCL already brings; OpenMP when found.
Vendoring was chosen over an optional external library because 3D-BBS is not packaged
for ROS or Ubuntu: a user would have to build and install it, and fix its missing
Boost.Filesystem link, for an engine they may never use; the subset is 260 lines and
builds in seconds, without the file I/O that needed Boost.Filesystem.

**Through the node's code path** (`eval_global_search.py --method g2 --config
'{"global_search_engine": "bbs3d"}'`, i.e. `GlobalLocalizationEngine.query`, same scans,
maps and suite settings as the evaluation; 2026-11-22):

| map → scans, queries | bbs2d: success, median query | bbs3d: success, median query | evaluation, top 8 3 m apart |
|---|---|---|---|
| Long_Stairs → Outdoor2 0-165 s, 163 | 137, 2.9 s | **153**, 10.1 s | 153 |
| EIL_Box → EIL_Stairs, 135 | 59, 0.7 s | 69, 1.4 s | 69 |
| EIL_Mix → EIL_Box, 103 | 78, 1.2 s (evaluation run) | 88, 3.5 s | 88 |

bbs3d gave the evaluation's top answer on every query and the same full candidate list
on every query that did not hit the time limit (Long_Stairs → Outdoor2: 135 of 163; the
other 28 stopped at 10 s, where the answers found so far depend on CPU speed). The
startup replay starts at the same query with the same pose in all three. bbs2d gave
the evaluation's G2 candidate lists exactly on every query of Long_Stairs → Outdoor2 and
EIL_Box → EIL_Stairs. On Long_Stairs → Outdoor2 by segment, bbs3d / bbs2d: room 50/50 /
50/50, corridor 75/75 / 71/75, stairs 20/20 / 13/20 (5 on another level), hall 5 m
below 8/18 (10 queries without an answer at the time limit) / 3/18 (12 on another
level).

Live startup replays (`run_benchmark.py`, the cross-session suite with
`--global-search-engine=bbs3d`, RKO-LIO odometry, one run each):

| case | bbs2d | bbs3d |
|---|---|---|
| Long_Stairs_map__Outdoor2 | right, first pose 15.1 s, 0.02 m median | right, first pose 37.8 s (three 10 s queries), 0.02 m median |
| EIL_Box_map__EIL_Stairs, 2 runs | right in both, first pose 11.6 s | one wrong start, 3.4 m off; one handed to the operator after 6 queries |

On the aliased room the live bbs3d start went wrong once where bbs2d was right twice:
the third query's search kept a single answer above 90% of the best score, so there was
no alternative for the nearby-alternative check to compare with, and two answers agreed
after the odometry motion. The offline replay above starts right on this pair for both
engines; two live runs are too few to rank them there, but bbs3d is not a fix for
aliased rooms, and the 2D search stays the better choice on single-level maps.

**Cost.** Voxel maps, once per map load: 22 ms and 0.6 MB (EIL_Box, 90 k points after
0.1 m thinning), 32 ms and 0.7 MB (EIL_Mix), 132 ms and 6.2 MB (Long_Stairs, 386 k
points); RSS after loading +17, +29 and +70 MB (mostly PCL reading the 0.4-1.2 M point
file). The node logs both at startup. During a search the open nodes take 32 bytes
each: up to 2.2 M nodes (70 MB, plus the queue's growth) in the hall within 10 s, and the
4 M cap bounds a search at about 128-256 MB. Peak RSS of the whole query process stayed
as in the evaluation (184-374 MB with the NDT scorer). Query time on 4 threads is 0.1-
3 s where the scan is distinctive and the full 10 s on open or repetitive scenes; on
one thread it was 3.4 times as long in the evaluation. The 10 s limit returns fewer
answers, or none, on a slower or busier computer: on this shared 4-vCPU machine
108 of 163 Long_Stairs → Outdoor2 searches hit it (98 in the evaluation).

**Small computers** (the Go2 `mid360_legged_low_resource` class): the voxel maps are
small, but the search is CPU-bound and runs next to the localizer. If bbs3d is needed
there, keep `bbs3d_num_threads` below the core count, lower `bbs3d_max_queue_nodes`
(1,000,000 bounds a search at about 32-64 MB), and expect more searches to end at the
limit with fewer answers; the startup then needs more queries before two agree.
Raising `bbs3d_timeout_sec` trades startup time for answers. Not measured on such
hardware.

**Not verified:** a GPU build (the vendored subset is CPU only), Boreas through the node
path, Foxy/Humble builds of `bbs3d_cpp` (CI covers them), and real hardware.

## G3: guarded automatic reinitialization

The localization node already raises `/reinitialization_requested`
(`std_msgs/Bool`) from its C++ recovery supervisor when it judges tracking lost.
The supervisor node consumes that signal, queries G2, and republishes
`/initialpose` from the top candidate — **only** when every safety guard passes.

```bash
python3 scripts/reinitialization_supervisor_node.py --ros-args \
  -p query_service:=/global_localization_node/query \
  -p alignment_status_topic:=/alignment_status \
  -p initialpose_topic:=/initialpose
```

Wiring:

```
/reinitialization_requested (Bool) ──▶ supervisor
/alignment_status (DiagnosticArray) ─▶ supervisor   (reads the fitness_score key)
        supervisor ──call──▶ /global_localization_node/query (Trigger)
        supervisor ──pub───▶ /initialpose (PoseWithCovarianceStamped)
```

### Safety guards

All of the *decision* logic lives in the ROS-free state machine
`scripts/reinitialization_supervisor_policy.py` and is regression-tested in
`test/test_reinitialization_supervisor_policy.py`. The guards exist because a
closed recovery loop can diverge catastrophically if a confidently-wrong
candidate is allowed to be published repeatedly (the Phase 3 lesson). They are:

| Parameter | Guard |
| --- | --- |
| `min_candidate_score` | A reset is never published from a candidate scoring below this (no *unsafe publication*). |
| `min_seconds_between_attempts` | Two resets are never closer than this. |
| `request_debounce_sec` | The request must persist this long before the first query (ignores transient blips). |
| `recovery_fitness_threshold` | After a reset the supervisor waits for `/alignment_status` fitness back under this as *recovery evidence*. Launch arg `supervisor_recovery_fitness_threshold` (default `1.5`). **Do not loosen** on repetitive outdoor maps without GT cross-check: threshold `2.0` on Koide 180 s produced a rubric PASS that ground truth showed was an along-route aliased false confirm (~25–28 m off); see [g3_live_closed_loop.md](dev/g3_live_closed_loop.md) (Koide 180 s boundary characterization, 2026-07-03). |
| `recovery_confirmation_samples` | The low-fitness recovery evidence must arrive for this many consecutive fresh, stable-tracking diagnostic samples before standing down. |
| `settle_timeout_sec` | If recovery is not observed within this long, the attempt is counted failed. |
| `max_attempts` | Hard ceiling on attempts for one continuous problem; it only resets on confirmed recovery or when the request clears — never as a side effect of attempting — then the supervisor **gives up** and surfaces an error for an operator rather than looping. |
| `query_timeout_sec` | A query that returns nothing within this long is abandoned (counts against `max_attempts`). |
| `confirm_cross_check` / `cross_check_mismatch_m` | Fitness alone confirms along-corridor aliases on self-similar maps, so after a settle-confirm (and after a localizer *self-cleared* recovery inside an episode whose reset never confirmed) the supervisor issues one more G2 query and compares the fix against the localizer pose at the fix scan stamp. Mismatch above the threshold (default `5.0` m) — or a trustworthy fix with **no** pose near its stamp — resumes the episode (`alias_confirmed` overrides the request flag) and immediately reseeds from the verify fix (`cross_check_reseed`, the freshest fix with fresh fix-to-fix velocity). A weak fix (below `min_candidate_score`) does not confirm either, and neither a weak fix nor one that fails the distinctiveness gate (`max_registration_fitness_ratio`) reseeds or is walked: the episode cools down and queries again. Launch args `supervisor_confirm_cross_check` (default `true`), `supervisor_cross_check_mismatch_m`. |
| `seed_motion_wall_fallback` | Wall-clock seed motion compensation (pose-delta / trusted-velocity paths) produced wrong seeds three distinct ways (kidnapped-history velocity, corner extrapolation, drift-onset velocity) and is **off by default**; only the sim-clock fix-to-fix path (two consecutive G2 fixes on bag-stamped scan times) compensates. Launch arg `supervisor_seed_motion_wall_fallback` (default `false`). |

Two related localizer-side guards (component params, not supervisor):
`imu_prediction_correction_guard_warmup_accepts` (default `5`) stands the IMU
prediction-correction guard down until N corrections are accepted after an
`/initialpose` — the smoother re-initializes with velocity 0, so on a moving
platform the guard would otherwise reject every correct post-reset NDT correction.
An accepted `/initialpose` is also **authoritative**: scans stamped before it are
dropped, alignment results seeded before it are discarded (generation counter),
and every anchor (`last_accepted_pose`, prediction state, smoothers) moves with
it — otherwise an in-flight scan can silently undo the reset.

An `/initialpose` the supervisor did not publish (an operator's, in the map frame
with a finite pose) ends its episode, with the reason `external_reset` in the event
log: the reset it was settling or checking is no longer the localizer's pose. A
request still raised afterwards starts a new, debounced episode with a fresh attempt
count. Before, the supervisor carried the old episode on: on Boreas a check query
answered on a scan from before the operator's reset set `alias_confirmed`, and 28 s
later the supervisor reset a localizer that was tracking at 96% inliers to a pose
12.6 m behind it.

After any terminal outcome (recovery or give-up) the supervisor waits for
`/reinitialization_requested` to de-assert before it will act again, so a request
line that stays (or is stuck) high cannot re-fire it — it is edge-triggered on the
*next* problem.

### Status

G3 logic is complete and tested offline, and as of 2026-07-04 the Koide
real-kidnap recovery is ground-truth-validated end to end: kidnap sticks, one
reseeded reset chain, `recovery_confirmed` verified by the post-confirm
cross-check, 26.4 s GT-recovered window (canonical 120 s run). The run-level GT
gate (`check_recovery_pose_gt.py`, which requires the trace to *end* recovered)
stays red because of two problems outside G3 — localizer corner fragility and
BBS ambiguity on the north stretch; no run on 2026-07-04 produced a false
confirm. Full defect chain and artifacts in
[g3_live_closed_loop.md](dev/g3_live_closed_loop.md) ("Real-kidnap re-baseline");
remaining work in [global_localization_roadmap.md](dev/global_localization_roadmap.md)
(G3 section).

### On the Boreas fixture, after a 10 s stop

`tools/benchmark/soak_replay.py --companion` runs the two recovery nodes beside the
soaked localizer (`global_localization_recovery.launch.py start_localization:=false`),
with a grid built from the fixture map by `generate_occupancy_map_from_pcd` and the
launch defaults. Two laps with a 10 s stop, 50 s and 30 s into the lap, with
`use_imu_preintegration: true`, on 2026-11-08:

- **The loop closes, but finds nothing.** The supervisor queried G2 1.5 s after the
  reinitialization request. Each answer took 18-21 s, and its candidates fit the scan
  with registration fitness 50-657, too close to each other to pass the aliasing
  check. Nothing was published, and no lap recovered. No wrong reset was published.
- **The map-wide search misses there.** Offline, the BBS alone takes 0.5-1.8 s. Of
  scans at 40, 50 and 61 s into the lap none had the right pose among its 16
  candidates, with height bands of 0.5-5, -1-3 and -1.5-1.5 m, 256 or 1024 scan points
  and 10 or 5 deg: tree-lined stretches score 0.86-0.93 at wrong places. At 20 and 100 s
  the right pose ranks first with the band at -1-3 m.
- **The registration scoring takes the rest.** At 6 m/s the vehicle has moved 110-130 m
  by the time an answer comes. The next section profiles it and cuts it to 2 s.
- **A search near a close prediction works, but after 10 s none is close.**
  - Over the grid cropped to 40 m around a prediction 15 m and 20 deg off the truth,
    the BBS takes 0.4-0.7 s and ranks the right pose first at 40 and 50 s and second
    at 61 s.
  - After the 10 s stop, though, the localizer's IMU retry seed, bridged across the
    gap, was 26-94 m and 61 deg off.
  - Around the last accepted pose, over 70 m in every direction and every heading, the
    right pose ranked first for one scan of three. When the search ran the vehicle was
    64-94 m away, and a wider window brings the ambiguity back.
  - Searching the grid from inside the localizer, around its prediction (40 m, 45 deg),
    took nothing in 52 searches.
- **The IMU samples during the stop are what is missing.** With them, the IMU
  prediction carries LiDAR gaps of 5 s. Larger FastDDS shared-memory buffers (a
  16384-entry port queue, a 128 MB segment) did not keep them either, with the replay
  publishing clouds and IMU from one participant. Without an odometry source that
  keeps running (`use_odom_tf_prediction`), a 10 s stop of a moving vehicle ends in a
  reinitialization request. With one it recovers: see the next section.

### On the Boreas fixture, map-wide queries

Sixteen queries with no prior, on scans 5 to 115 s into the fixture, on 2026-11-12
with 4 vCPUs. An answer counts as published when it passes the supervisor's score and
ambiguity checks, and as right within 2 m and 10 deg of the ground truth.

| Settings | Median | Worst | Right / wrong / none published |
| --- | --- | --- | --- |
| Launch defaults | 19.0 s | 24.0 s | 0 / 0 / 16 |
| `g2_ndt_local_map_radius:=0.0 g2_ndt_num_threads:=4 g2_ndt_scan_max_range:=80.0` | 1.8 s | 3.2 s | 6 / 2 / 8 |
| and `g2_registration_refine_candidates:=true` | 2.0 s | 3.2-3.6 s | 8 / 0 / 8 |

- **Where the 19 s went** (one query, 16 candidates): 13-14 s prepared a scoring
  target for each candidate (crop 0.8 s, voxel filter 3.3 s, NDT cells 3.4 s, the
  fitness search tree 6.0 s), 3-5 s aligned, and the BBS took 0.9-1.0 s. The scan
  reaches 234 m, so no two candidates could share a 150 m crop, and each crop held
  45-66% of the map.
- **Why nothing was published before:** with every scan point the true pose's fitness
  was 300-770, the worst of the 16 at 20 s. Within 80 m it is 0.05-0.21, against
  0.9-5.2 at wrong places.
- **The two wrong answers without refinement** were the right place, 2.1-2.9 m off:
  the BBS cell pose, not yet aligned. With refinement they are within 0.74 m.
- **The search still misses half the scans.** The right pose is among the 16 BBS
  candidates for 8 of the 16 scans, before and after; at 28, 35, 40, 50, 55, 61, 90
  and 115 s it is not, and no answer is published there. At those places the best
  wrong candidate scores 0.13-0.85 and only the ambiguity check stops it.
- **One scan is fragile:** the scan 0.1 s later flipped 20 s and 70 s between right
  and none.
- **Accumulating scans did not help.** Matching the last 10 or 21 scans (1-2 s),
  even placed by their ground-truth motion, never put the right pose among the
  candidates at 40, 50 or 61 s, and took 2.7-3.5 s.
- **On Koide** (`outdoor_hard_01a`, 10 scans) the fast settings give the same answers
  as the defaults, 2 right, 1 wrong and 7 none, in 0.82 s instead of 2.57 s at the
  median. The wrong one, 193 m off at 95 s, is published with the defaults too.

Not measured: range caps other than 80 m, and the HDL replay with threads.

### On the Boreas fixture, the closed loop with the fast settings

The same 10 s stop as above, with the supervisor and the fast settings
(`g2_ndt_local_map_radius:=0.0 g2_ndt_num_threads:=4 g2_ndt_scan_max_range:=80.0
g2_registration_refine_candidates:=true`), `use_imu_preintegration: true`, ten stops
at 10, 30, 50, 60 and 70 s into the lap, on 2026-11-12:

- **No stop recovered**, 0 of 10. With the launch defaults the one stop run did not
  either: two answers in 16.8 and 20.2 s, both withheld.
- **The answers were right, and too old.** All 14 resets the supervisor published were
  0.02-0.50 m from the truth at their scan's stamp. They reached the localizer 2.5-24 s
  later, 12.6-134 m behind a vehicle at 5-8 m/s. The localizer restarts at the reset's
  stamp with velocity 0 and no IMU history, so every later scan was seeded at the reset
  pose and rejected (fitness 14-23 against 6, 22-59% inliers).
- **No wrong pose was accepted,** and no aliased answer was published: the ambiguity
  check withheld every wrong top answer (42-280 m off).
- **Answers are slower in the loop:** 3.7 s at the median and up to 14.4 s while the
  localizer is lost and uses 1.3-3.2 cores, against 2.0 s when it tracks.
- **Walking to the next candidate reuses the old answer:** after the 20 s settle
  timeout the second candidate was published 24 s old, 134 m off.
- **`supervisor_enable_seed_motion_compensation:=true` made it worse:** 0 of 5
  recovered, with resets up to 653 m off. It takes a velocity from any two consecutive
  answers, withheld wrong ones included (20.8 and 25.6 m/s from a pair 214 m apart).
- With the `external_reset` handling, a repeat of five stops published 7 resets, all
  with a request active and no scan accepted in the 3 s before. None recovered.

So the fast settings stay opt-in. Without odometry, a reset from a moving vehicle
recovers a track only once it is brought up to the scan it is applied to: answering
faster, or compensating the answer's motion, leaves it too old (next section).
`enable_stale_reset_replay` does it in the localizer, from the scans it kept.

### On the Boreas fixture, a stale reset replayed through the kept scans

With `enable_stale_reset_replay: true` (a localizer parameter, default `false`) the
localizer keeps its prepared scans of the last `stale_reset_replay_max_age_sec`
(20 s). An `/initialpose` stamped `stale_reset_replay_min_age_sec` (0.5 s) or more
before the scan it is applied to is tracked through the kept scans from its stamp on,
as in normal tracking:

- The first three scans, then one per 0.2 s, each seeded from the one before with
  the motion tracked so far, and kept only under `score_threshold`.
- Each scan callback replays for about 0.2 s and keeps its own scan, so the replay
  catches up with the newest scan. The reset then moves there, with the tracked motion
  as the next seed and the tracked velocity in the IMU smoother. That next scan goes
  through the normal measurement gate. Nothing is published during the replay.
- The reset is applied as before when its own scan, or one within 0.25 s after it,
  was not kept, or when the replay rejects more than 3 scans in a row, accepts fewer
  than 90%, rejects the last scan, or falls 20 s behind.

The closed loop of the previous section (fast settings, `use_imu_preintegration:
true`, ten stops at 10, 30, 50, 60 and 70 s into the lap), on 2026-11-17 with 4 vCPUs
shared with another job:

| Localizer | Stops recovered | Wrong pose accepted | Largest error after recovery |
| --- | --- | --- | --- |
| main, three stops at 30, 70 and 10 s | 0 of 3 | none | - |
| `enable_stale_reset_replay: true` | 5 of 10 | none | 0.34 m (0.36 m in the lap without a stop) |

- **Each recovered stop tracked to the end of its lap**, every scan accepted. The
  replays took resets 1.2-9.2 s older than the scan through 7-39 scans in 0.21-3.6 s,
  and moved them 3.8-82 m.
- **The time to recover is the time to an answer.** From the end of the stop to the
  first accepted scan took 30-58 s: the reinitialization request comes 15-27 s after
  the stop, and the first right answer 2-34 s after the request.
- **No wrong pose was accepted.** The supervisor published one wrong reset in each
  run, 98 m off on main and 35 m off with the replay, at registration fitness 1.51
  and 1.00 with nothing within half of it elsewhere. The replay rejected the first 4
  scans after the second and applied it as before; the localizer rejected every scan
  after both.
- **Not recovered:** two replays lost track in the 20 deg/s turn after 95 s, with
  resets 10.4 and 11.9 s old (11 of 15 and 34 of 38 scans accepted); one reset 12.7 s
  old came without its scan kept (the lost localizer had dropped it); the answer of one
  stop came after the lap ended; and the wrong reset.
- **Why not answer faster or search:** offline, NDT on a scan from the pose 0.1 s
  earlier (0.3-0.8 m behind) converged on 21 of 21 scans, from 0.25 s earlier
  (0.7-2.0 m) on 20, from 0.5 s earlier on 5, from 1 s earlier on none. An answer
  aligned again on the newest scan is still as old as that query when it is applied.
  A BBS search around the reset over 10 m/s times its age, refined by NDT, proposed the
  right pose for 24 of 33 scans 2.5-10 s on and one 2.3 m off, but took 0.8-39 s over
  every heading at 2 deg.
- **Steps:** replaying every scan took 44 ms per scan; steps of 0.5 s lost track in a
  tightening turn (55 to 66 deg of heading in a second), 0.2 s held. With one long
  replay instead of 0.2 s per callback, the next scan came 3.4 s after the replayed
  one even with `cloud_queue_depth: 50`, and was rejected.
- **No regression without a stop:** one lap on its own gives an RMSE of 0.132 m with
  the preset, the same with the replay on, and 0.127 m with
  `use_imu_preintegration: true` and the replay on.

Not measured: odometry (`use_odom_tf_prediction`) together with the replay, other
datasets, and the memory the kept scans take (about 2000 points each on Boreas).

### On Koide, with odometry, after a 10 s stop

The same stop with an odometry source that keeps running:
`run_koide_public_bag.py --stall START_SEC:DURATION_SEC` freezes only the localizer
while RKO-LIO and the player run. On `outdoor_hard_02b` with the online profile
(`use_odom_tf_prediction: true`), one stop per run, on 2026-11-11:

| Stop | Where | Runs | First accepted scan after it | Largest error in the next 30 s |
| --- | --- | --- | --- | --- |
| 10 s from 30 s | in a turn of 13 deg/s | 2 | 0.19 and 0.27 s | 0.19 m |
| 10 s from 110 s | on a straight | 2 | 1.02 and 0.94 s | 0.39 and 0.53 m |
| 2 s from 30 s | in a turn | 1 | 0.02 s | 0.15 m |
| 2 s from 110 s | on a straight | 1 | 0.79 s | 0.54 m |

- **Every run recovered by itself.** No run requested reinitialization, and no pose
  was more than 0.54 m off. The whole-run RMSE is 0.239-0.249 m, against 0.240-0.242 m
  without a stop.
- **The odometry carries the gap.** The first accepted scan was seeded from the
  odometry transform 13.0 m (turn) and 18.1-18.2 m (straight) from the last accepted
  pose, and corrected by 0.12-0.37 m.
- **What is rejected first:** on the straight, one or two clouds queued during the stop
  and seeded from the previous motion (fitness above 11), and the first scan from the
  odometry seed (fitness 8.15 against the threshold of 6). The stretch from 112 s
  rejects 10-12 scans per 10 s without a stop too, and reaches 0.56 m there.
- A 10 s stop loses 3% of the diagnostic stamps, so the check's coverage gate (97%)
  reports FAIL at 96.8-96.9%. A stalled run is not a release check.

Not measured: a stop with the odometry source stopped too, and a stop off the map
(163-259 s).

### Replaying a bag through the recovery stack

The scripted replay harnesses were removed with the offline tooling (6825f05); the
runs recorded above used them. To replay a bag yourself, build the occupancy grid from
the same map, start the three-node stack, and play the bag on `/clock`:

```bash
ros2 run lidar_localization_ros2 generate_occupancy_map_from_pcd \
  --pcd /path/to/map.pcd --output-dir maps --map-name site

ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py \
  localization_param_dir:=/path/to/params.yaml use_sim_time:=true \
  cloud_topic:=/livox/points imu_topic:=/livox/imu \
  base_frame_id:=livox_frame lidar_frame_id:=livox_frame publish_lidar_tf:=false \
  occupancy_yaml:=maps/site.yaml map_path:=/path/to/map.pcd \
  g2_registration_seed_z_m:=<sensor height in the map frame>

ros2 bag play /path/to/bag --clock
```

With an external odometry front end (`odom -> base` TF), add
`enable_map_odom_tf:=true use_odom_tf_prediction:=true publish_bridge_pose_when_lost:=true`.
`supervisor_recovery_fitness_threshold` (default `1.5`) is the production value; change
it only for boundary characterization.
