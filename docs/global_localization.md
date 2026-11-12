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
`max_candidates`, `nms_radius_m`, `registration_refine_candidates` (node param;
launch arg `g2_registration_refine_candidates`, default `false`). When enabled,
`g2_ndt_score` publishes each candidate's NDT-refined pose instead of the raw BBS
cell center — required for the HDL hdl_400 recovery replay (raw poses up to ~3 m
off) but harmful on Koide, where refinement can snap aliased hypotheses to
locally-perfect alignments and collapse walk candidates onto one pose. Lower
`nms_radius_m` keeps more near-by or alternate-yaw candidates for G3 candidate
walking; higher values return more spatially diverse candidates. Query latency
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

Not measured: the fast settings in the closed loop with the supervisor, range caps
other than 80 m, and the HDL replay with threads.

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
