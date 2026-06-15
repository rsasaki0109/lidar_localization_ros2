# Global Localization (experimental)

Operations guide for the opt-in global-localization stack: recovering a usable
pose with **no** (or a lost) initial pose, and optionally re-seeding tracking
automatically when it diverges. This is the *how to run it* companion to the
design/status document, [global_localization_roadmap.md](global_localization_roadmap.md).

Everything here is **opt-in**. None of these nodes are part of the default launch,
and none of them publish or change anything until you explicitly run them and (for
the supervisor) the safety guards pass. The core localization node is unchanged.

## The three pieces

| Stage | What it does | Entry point |
| --- | --- | --- |
| G1 | Map-wide candidate generation (BBS_2D branch-and-bound over an occupancy grid), artifact-first / offline | `scripts/make_bbs_relocalization_attempts.py` |
| G2 | Runtime service that exposes G1 on demand: ask, get ranked candidates back | `scripts/global_localization_node.py` |
| G3 | Guarded *automatic* reinitialization: watch the lost-tracking signal, query G2, and re-seed `/initialpose` behind safety gates | `scripts/reinitialization_supervisor_node.py` |

G1 and G2 need an occupancy grid built from the map PCD
(`scripts/generate_occupancy_map_from_pcd.py`). G3 needs a running localization
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
`runtime_sec`, and the `top` candidate `x`/`y`/`yaw_deg`/`score`). The full ranked
list is also published once as a `geometry_msgs/PoseArray` on
`/global_localization_node/candidates`. To relocalize manually, publish the top
candidate to `/initialpose`.

Key parameters (see the node for the full list): `z_min_m` / `z_max_m` (scan
height band), `max_scan_points`, `angular_resolution_deg`, `pyramid_depth`,
`max_candidates`, `nms_radius_m`. Query latency on a validated window is a few
seconds; see the roadmap for the speed/coverage envelope.

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
| `recovery_fitness_threshold` | After a reset the supervisor waits for `/alignment_status` fitness back under this as *recovery evidence* before standing down. |
| `settle_timeout_sec` | If recovery is not observed within this long, the attempt is counted failed. |
| `max_attempts` | Hard ceiling on attempts for one continuous problem; it only resets on confirmed recovery or when the request clears — never as a side effect of attempting — then the supervisor **gives up** and surfaces an error for an operator rather than looping. |
| `query_timeout_sec` | A query that returns nothing within this long is abandoned (counts against `max_attempts`). |

After any terminal outcome (recovery or give-up) the supervisor waits for
`/reinitialization_requested` to de-assert before it will act again, so a request
line that stays (or is stuck) high cannot re-fire it — it is edge-triggered on the
*next* problem.

### Status

G3 logic is complete and tested offline; a live/replay smoke on the Koide
kidnapped-start window and the post-reset recovery-evidence capture are still
pending an idle machine. Track it in
[global_localization_roadmap.md](global_localization_roadmap.md) (G3 section).
