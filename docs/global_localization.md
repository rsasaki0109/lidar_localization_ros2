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
`runtime_sec`, `scan_stamp_sec`, `scan_age_sec`, `candidate_age_sec`, and the
`top` candidate `x`/`y`/`yaw_deg`/`score`). The full ranked list is also
published once as a `geometry_msgs/PoseArray` on
`/global_localization_node/candidates`. To relocalize manually, publish the top
candidate to `/initialpose`. On moving bags, `candidate_age_sec` is the first
thing to check when a plausible candidate still fails to lock.

Key parameters (see the node for the full list): `z_min_m` / `z_max_m` (scan
height band), `max_scan_points`, `angular_resolution_deg`, `pyramid_depth`,
`max_candidates`, `nms_radius_m`, `registration_refine_candidates` (node param;
launch arg `g2_registration_refine_candidates`, default `false`). When enabled,
`g2_ndt_score` publishes each candidate's NDT-refined pose instead of the raw BBS
cell center — required for the HDL hdl_400 recovery replay (raw poses up to ~3 m
off) but harmful on Koide, where refinement can snap aliased hypotheses to
locally-perfect alignments and collapse walk candidates onto one pose. Lower
`nms_radius_m` keeps more near-by or alternate-yaw candidates for G3 candidate
walking; higher values return more spatially diverse candidates. Query latency
on a validated window is a few seconds; see the roadmap for the speed/coverage
envelope.

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
| `recovery_fitness_threshold` | After a reset the supervisor waits for `/alignment_status` fitness back under this as *recovery evidence*. Launch arg `supervisor_recovery_fitness_threshold` (default `1.5`). **Do not loosen** on repetitive outdoor maps without GT cross-check: threshold `2.0` on Koide 180 s produced a rubric PASS that ground truth showed was an along-route aliased false confirm (~25–28 m off); see [g3_live_closed_loop.md](g3_live_closed_loop.md) (Koide 180 s boundary characterization, 2026-07-03). |
| `recovery_confirmation_samples` | The low-fitness recovery evidence must arrive for this many consecutive fresh, stable-tracking diagnostic samples before standing down. |
| `settle_timeout_sec` | If recovery is not observed within this long, the attempt is counted failed. |
| `max_attempts` | Hard ceiling on attempts for one continuous problem; it only resets on confirmed recovery or when the request clears — never as a side effect of attempting — then the supervisor **gives up** and surfaces an error for an operator rather than looping. |
| `query_timeout_sec` | A query that returns nothing within this long is abandoned (counts against `max_attempts`). |
| `confirm_cross_check` / `cross_check_mismatch_m` | Fitness alone confirms along-corridor aliases on self-similar maps, so after a settle-confirm (and after a localizer *self-cleared* recovery inside an episode whose reset never confirmed) the supervisor issues one more G2 query and compares the fix against the localizer pose at the fix scan stamp. Mismatch above the threshold (default `5.0` m) — or a trustworthy fix with **no** pose near its stamp — resumes the episode (`alias_confirmed` overrides the request flag) and immediately reseeds from the verify fix (`cross_check_reseed`, the freshest fix with fresh fix-to-fix velocity). A weak fix (below `min_candidate_score`) fails open: G2 must not override the localizer with garbage. Launch args `supervisor_confirm_cross_check` (default `true`), `supervisor_cross_check_mismatch_m`. |
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
[g3_live_closed_loop.md](g3_live_closed_loop.md) ("Real-kidnap re-baseline");
remaining work in [global_localization_roadmap.md](global_localization_roadmap.md)
(G3 section).

### Replay harnesses

Kidnapped-recovery closed-loop replays (prepare assets, launch recovery stack, inject
kidnap, play bag, run health rubric):

| Scenario | Prepare | Replay | Regression wrapper |
| --- | --- | --- | --- |
| Koide `outdoor_hard_01a` | `scripts/prepare_koide_hard_relocalization_assets.sh` | `scripts/run_koide_g3_recovery_replay.sh` | `scripts/run_koide_g3_recovery_regression.sh` |
| HDL `hdl_400_ros2` | `scripts/prepare_hdl_recovery_assets.sh` | `scripts/run_hdl_g3_recovery_replay.sh` | `scripts/run_hdl_g3_recovery_regression.sh` |

Both wrappers write `recovery_health.json` and `regression_result.json` under
`artifacts/public/<scenario>_g3_recovery_regression` by default and skip gracefully
when the dataset is absent.

The Koide replay script accepts `--recovery-fitness-threshold X` (default `1.5`), passed
through as launch arg `supervisor_recovery_fitness_threshold` on
`global_localization_recovery.launch.py`. Use it only for boundary characterization; the
default is the intended production value. Example:

```bash
scripts/run_koide_g3_recovery_replay.sh --skip-prepare --duration-sec 180 --rate 0.4
scripts/run_koide_g3_recovery_replay.sh --skip-prepare --duration-sec 180 --rate 0.4 \
  --recovery-fitness-threshold 2.0 --output-dir /tmp/lidarloc_koide_g3_recovery_180s_thr20
```

The three-node recovery bringup (`ros2 launch ... global_localization_recovery.launch.py`)
also exposes `supervisor_recovery_fitness_threshold` directly when not using the harness.
