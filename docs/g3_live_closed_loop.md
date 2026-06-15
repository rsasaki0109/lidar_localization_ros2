# G3 Live Closed-Loop Result (Koide outdoor_hard_01a)

Date: 2026-06-15. This is the live exercise of the G3 post-reset recovery-evidence
gate from [global_localization_roadmap.md](global_localization_roadmap.md): the
three-node bringup (`launch/global_localization_recovery.launch.py`) run against the
real Koide `outdoor_hard_01a` bag + map, with a kidnap injected at a healthy location
so the guarded automatic-reinitialization loop has to recover.

## Setup

- Bringup: `global_localization_recovery.launch.py` (localizer + G2 + G3 supervisor),
  `use_sim_time:=true`, `cloud_topic:=/livox/points`, real `map_outdoor_hard.ply` and
  `outdoor_hard_occupancy.yaml`.
- Localizer params: smoke config (`nav2_ndt_urban` + Livox overrides, `score_threshold
  6.0`), with the reinitialization trigger lowered (`reinitialization_trigger_threshold
  0.40`, `reinitialization_trigger_gap_scale_sec 10`) so the request fires promptly in a
  bounded window. Lowering the trigger to expose the request is the same technique the
  repo's own `..._180_reinit090` diagnostic manifest uses; it tunes the *localizer's*
  request, not the G3 loop under test.
- Supervisor: `min_candidate_score 0.6`, `max_attempts 3`, `query_timeout_sec 30`,
  `settle_timeout_sec 20` (patient enough for the ~10 s BBS query).
- Bag played at rate 0.5 to halve vehicle motion during the BBS query (reduce
  reset-seed staleness). Correct pose seeded after `/clock`; a wrong `/initialpose`
  (`-36, -58.8`) injected at sim t+22 s to break tracking at a healthy location.

## What happened

| Phase | Evidence |
| --- | --- |
| Healthy tracking | 134 accepted `/pcl_pose`, fitness 0.06–0.2, pose advancing (-86,-9)→(-108,-5.5) |
| Kidnap (sim 848) | fitness explodes 222→428, `/pcl_pose` freezes (no accepts), `/reinitialization_requested` asserts |
| Supervisor loop | `idle→await_query→query_issued`, then **3 guarded resets**, then **give-up** |

Candidate quality vs ground truth (the decisive finding):

| Attempt | sim | GT (true) | G2 candidate (BBS score) | error |
| --- | --- | --- | --- | --- |
| 1 | ~862 | (-107.2, 17.7) | (-45.9, -74.6) score 0.990 | **~107 m** |
| 2 | ~880 | (-109.7, 40.3) | (53.1, -71.6) score 0.988 | **~190 m** |
| 3 | ~898 | (-97.3, 54.3) | (-102.9, 53.6) score 0.998 | **~5.5 m (correct)** |

After three attempts without confirmed recovery (fitness never returned under
`recovery_fitness_threshold 1.5`), the supervisor logged
`reinitialization gave up (recovery_failed_exhausted) after 3 attempt(s); operator
intervention needed` and latched `exhausted`.

## Conclusions

1. **The G3 supervisor mechanism is validated end-to-end against a real stack.** Every
   state transition fired correctly: detect → debounce → query G2 → score-guard →
   guarded `/initialpose` publish → recovery-wait → retry → `max_attempts` ceiling →
   safe give-up with an operator alert. This exercises both the publish path *and* the
   recovery-failure path of the recovery-evidence gate.

2. **The safety ceiling earned its keep, live.** Three confidently-wrong-or-stale
   candidates did **not** produce an unbounded reset loop — `max_attempts` + give-up
   contained it. This is the Phase 3 closed-loop lesson (a closed loop must not loop
   forever on confident-wrong input) validated in the G3 context, not just in the
   offline policy test.

3. **Recovery was blocked by candidate *quality*, not by the supervisor.** G2's BBS_2D
   returned high-confidence but grossly wrong candidates on 2 of 3 attempts (107 m and
   190 m off at BBS score ≈ 0.99) — perceptual aliasing on the self-similar outdoor map.
   The correct pose did appear (attempt 3, 5.5 m at score 0.998) but too late, after the
   ceiling and inside the closing bag window. **The BBS score does not separate right
   from wrong**: 0.99 labelled a 190 m error and 0.998 labelled the 5.5 m hit.

## Fix: ranked-candidate walk (implemented 2026-06-15)

The first of the two remedies below is now implemented. The insight is that the
localizer's own NDT fitness *is* the registration oracle the BBS score is not, so the
supervisor should let it adjudicate the ranked list rather than trusting rank 0:

- **Walk the ranked candidate list within one query** (done). G2 now returns the full
  ranked `candidates` list in its `~/query` reply (not just `top`). The supervisor
  publishes the best candidate, waits `settle_timeout_sec` for recovery, and if the
  localizer's fitness does not drop, publishes the *next* candidate from the same query
  — using the localizer's fitness to reject aliased poses. Walking within a query does
  **not** spend a `max_attempts` slot (only re-querying does), so the ceiling stays a
  true bound on queries while one query can try all the poses it found. A wrong reset is
  rejected by the localizer (high fitness, no accepted pose), so walking cannot cause
  the Phase 3 acceptance blowup. Covered by `test_reinitialization_supervisor_policy.py`
  (walk recovers on a lower-ranked candidate without spending attempts; the list-exhaust
  path still respects `max_attempts`; the score floor still truncates the walk) and by
  the ROS integration test (the node walks 0→1 within one query and recovers).
- **Per-candidate registration scoring in G2** (still future work). Have G2 score each
  candidate by NDT/GICP fitness against the 3D map at the candidate pose, so the ranking
  itself reflects registration quality — the complement to the walk for when the correct
  pose is not even in the BBS top-K.

Query latency remains a secondary factor: attempt 3 was correct but stale by the time it
was published. The G1 optimization note's coarse-yaw / C++ path applies here.

## Live walk run (2026-06-15, second run) — the blocker is query latency, not candidates

Re-running the kidnap with the walk enabled corrected the earlier "candidate quality"
diagnosis. The walk itself is now **live-validated**: the supervisor walked all 16
candidates from one query (`candidate 1/16` … `16/16`, each a `next_candidate`
transition, each published by index), kept every guard, and only then re-queried — the
designed behaviour, on the real stack.

And the candidate generation was actually *fine*: the BBS **top candidate (rank 1) was
the true pose** — `(-107.1, 18.44)` at score `1.0`, versus ground truth `(-107.2, 17.7)`
at the query-issue time, ~0.8 m off. The walk published it first.

It still did not recover, and the reason is **BBS query latency**:

- the `~/query` call reported `runtime_sec: 23.376` — the BBS search took ~23 s;
- the candidate is computed against the scan captured at query-*issue* time, but is
  published ~23 s later. At the 0.4× replay rate that is ~9 s of sim time, during which
  the (still-driving) vehicle moved from `(-107, 18)` to ~`(-108, 30)` — so the
  rank-1-correct candidate was ~12–17 m **stale** by the time it was published;
- the localizer seeded there, could not register the now-moved scan (fitness stayed
  ~31), and the counter shows `/pcl_pose` briefly at `(-107.1, 18.4)` with high fitness
  and no accepted pose. Every other candidate is from the same stale scan, so the walk
  cannot rescue it.

**Refined conclusion.** Candidate generation and ranking were not the blocker here (BBS
put the truth at rank 1); the blocker is that a ~23 s query makes *any* candidate ~15 m
stale on a moving vehicle, outside the registration basin. The walk is validated and
keeps its guarantees, but live recovery needs a **fresh** candidate. So the primary next
lever is **G2 query speed** — the coarse-yaw (~9 s) setting or the C++ port already noted
in the G1 optimization entry of the roadmap, and/or exposing G2's search parameters so a
faster (coarser) configuration can be selected for the runtime loop. Per-candidate
registration scoring in G2 remains useful (it would reject a stale candidate faster), but
it cannot manufacture a fresh one — query latency is the thing to cut.

## Fast-G2 run (2026-06-15, third run) — query sped up, but walk latency re-staled

Exposing G2's search cost as launch arguments and running with a coarse config
(`g2_angular_resolution_deg:=10`, `g2_max_scan_points:=256`) cut the query from ~23 s
to **~4–8 s** (`runtime_sec` 7.559 then 4.387) — the speed knob works. Recovery still did
not complete, for a subtler reason and with a harness caveat:

- **Walk latency re-introduces staleness.** Even with a fast query, a near-correct
  candidate that is not rank 1 is published late: the walk spends `settle_timeout_sec`
  (6 s here) on each earlier candidate first. The near-correct candidate `(-107.9, 12.0)`
  was rank 3, so it was published ~20 s after the query issued — stale again by the
  vehicle's motion, fitness stayed ~29, rejected. So the latency budget is *query +
  rank × settle*, not just the query.
- **Harness caveat for this run.** The healthy baseline did not establish: the one-shot
  seed publisher exited before the localizer's `/initialpose` subscription finished
  discovery (a transient-local race that only bit at this slower replay rate), so the
  localizer had no lock before the kidnap. The kidnap seed (published by the longer-lived
  injector) *was* received. This makes the third run a weak recovery test, but the query
  timing and the walk-latency observation above stand on their own.

**Net.** On a *moving* vehicle the total scan→seed latency (query + walk) must stay under
the time the vehicle takes to leave the registration basin. The levers are: cut the query
hard (C++ BBS, or coarser still) *and* keep the true pose at rank 1 (so no walk latency);
or motion-compensate the published seed forward by the measured latency; or scope G3
recovery to low-speed / stationary kidnaps and say so. The ranked-candidate walk stays the
right mechanism for *aliasing* (true pose present but not at rank 0); it is not a remedy
for *staleness*, which is a latency problem. The test harness should also republish the
seed until the localizer acknowledges, to remove the discovery race.

## Clean run (2026-06-15, fourth run) — the decisive blocker is the localizer re-lock

With the seed harness fixed (the publisher now holds the transient-local latch instead of
exiting), the fourth run had everything the previous ones lacked at once: a **healthy
baseline** (seed received, 106 accepted poses, fitness 0.07–0.23), a **fast query**
(`runtime_sec` 5.365 then 3.595), and a **correct, fresh, rank-1 candidate**. Query 1's
top candidate was `(-106.10, 9.04, 150°)`; ground truth at the query time (sim 855) was
`(-106.8, 8.6, 156°)` — **0.8 m and 6° off, in full 3-DOF**, published ~5 s after the
scan it was computed from.

It still did not re-lock. After the candidate was published, the localizer logged
`initialPoseReceived`, immediately went `tracking -> recovering (reject_measurement)`, and
streamed `fitness score is over 6.0` continuously; the counter shows `/pcl_pose` sitting
at the seed `(-106.1, 9.0)` with fitness ~27 and no accept — at a pose that is ~0.8 m from
truth and where the **same area scored fitness 0.23 during healthy tracking minutes
earlier**. Across the whole post-kidnap window there were 28 `initialPoseReceived` (every
reset) and 517 over-threshold rejections, with only 2 brief `recovering -> tracking`
accepts.

**This eliminates every hypothesis upstream of the localizer.** Candidate generation
(rank-1 correct in 3-DOF), ranking, the walk, query latency, and yaw were all adequate
here, yet recovery failed. The decisive blocker is in the **localizer's own post-reset
re-lock path**: it does not re-converge to low fitness from a correct externally-published
`/initialpose`, even though it tracks that exact area at fitness 0.2 in steady state. The
likely mechanics are the same ones the Boreas root-cause flagged — the alignment
`init_guess` / local-map-crop centering during the `recovering` state not following an
externally-injected reset pose — but confirming that needs **code-level investigation of
the C++ component**, not more black-box replays. The five live runs have isolated the
blocker by elimination; that investigation is the next step (Task #16, re-scoped).

## Root cause and fix (2026-06-15) — RECOVERY ACHIEVED

The code investigation found the blocker upstream of the localizer after all, and it is
mundane: **the reset was published with z = 0**. A G2 candidate is 2D (`x, y, yaw`, with
`z = seed_z_m = 0`) and the supervisor's `_publish_reset` set only `x / y` + yaw, leaving
`position.z = 0`. On the Koide outdoor map the true z is ~-11 m, so every reset seeded the
pose ~11 m above ground -- far outside the NDT z-basin -- and the lock never took even when
`x / y / yaw` were correct. The fourth run's rank-1 candidate was 0.8 m / 6 deg off in
`x / y / yaw` and *still* held fitness ~27 for exactly this reason. The HDL demo worked
only because its map is flat (z ~ 0), so z = 0 happened to be right.

**Fix.** The supervisor subscribes to the localizer pose (`/pcl_pose`) and carries its
`z / roll / pitch` onto the 2D candidate, overriding only `x / y / yaw`. The vehicle's
height and attitude drift slowly even while xy tracking is lost, so the last pose is a
good carry-over (falls back to `reset_default_z_m` if no pose seen yet).

**Result -- the recovery-evidence gate is met.** Re-running the clean kidnap with the fix,
the reset now publishes `z ~ -9.4 / -7.3` (carried from the pose), and the loop recovers
end-to-end:

```
published /initialpose reset to (-76.50, 55.24, z=-7.33, 90.0 deg) score=1.0 [candidate 1/16]
supervisor: none -> standdown (recovery_confirmed)
supervisor: none -> idle (standdown_cleared)
```

After that reset the localizer locked: `/pcl_pose` resumed and fitness fell to **0.10**
(from ~30+ while lost), the supervisor saw recovery, stood down, and re-armed to idle for
the next problem. So the full G3 closed loop -- kidnap -> tracking lost ->
`/reinitialization_requested` -> G2 query -> ranked-candidate walk with z carried from the
pose -> a candidate locks -> fitness recovers -> `recovery_confirmed` -> stand down /
re-arm -- is validated live on the Koide window.

**Remaining (quality, not correctness).** Recovery here took two queries and a full
16-candidate walk (~140 s of replay) before a candidate matched the by-then-current
position, because query latency + walk latency keep the candidates somewhat stale; cutting
the BBS query further (C++ / coarser) and per-candidate registration scoring in G2 would
make recovery faster and first-try. The test harness should also republish the seed until
the localizer acknowledges (the transient-local discovery race still occasionally drops the
healthy baseline at slow replay rates). But the gate question -- *does tracking recover
after the guarded reset* -- is now answered yes.

## Speedup lever 1 -- bound the walk, re-query on a fresher scan

The dominant cost in that run was **not** the query itself but the walk: the first query
(at t+8 s) returned 16 candidates clustered around `(95, -63)` and `(-65, -78)`, none of
which was the true pose `(-76, 55)` -- the right place simply was not in that scan's
candidate set. The supervisor nonetheless walked all 16 at `settle_timeout_sec` (6 s) each,
burning **~95 s**, before cooling down and re-querying. The *second* query (on a newer, more
distinctive scan) returned the true pose at rank 1 with score 1.0 and locked immediately.

So past the top few BBS occupancy maxima, a fresh query beats a deeper walk into a stale
list. The policy now caps the walk at `max_walk_candidates` (default 4): after the top few
candidates fail to lock, it treats the whole query as a miss and re-queries (spending one
attempt) instead of grinding through ranks 5-16. On this run that turns ~95 s of doomed
walking into ~24 s, then a fresh query -- the one that actually recovered. The cap is a pure
policy change (`reinitialization_supervisor_policy.py`), unit-tested in
`test_reinitialization_supervisor_policy.py` (`test_walk_is_bounded_by_max_walk_candidates`,
`test_high_max_walk_candidates_restores_full_list_walk`), and exposed as the
`supervisor_max_walk_candidates` launch arg. Set it high to restore walking the full list.

Note this does *not* help when the true candidate is genuinely in the list but ranked deep
on registration quality -- that is what per-candidate registration scoring in G2 (lever 2,
not yet built) would fix. The cap addresses the more common stale-query case observed here,
where no rank would have recovered and the time is better spent on a new scan.

## Speedup lever 2 -- forward-compensate the seed for query latency

The fourth (clean) run isolated the residual blocker after the re-lock fix: even a *correct
rank-1 fix goes stale*. A G2 BBS query takes ~5-23 s, and the candidate it returns is a fix
of the scan taken when the query was *issued*; by the time the supervisor publishes the
reset the vehicle has driven several metres on, so the seed lands behind the vehicle, outside
the registration basin, and the lock does not take. The true lever here is not candidate
*quality* but *latency*: close the gap between where the fix says the vehicle was and where it
is when the seed is applied.

Two ways to close it: make the query instant (the G2 C++ port -- the root fix, profiled and
planned in [g2_bbs_speedup.md](g2_bbs_speedup.md)), or *forward-compensate* the seed by the
measured latency. This lever does the latter, and it
has to work on the Koide bag, which carries no twist/odom topic (only `/livox` lidar+imu). So
the velocity is inferred from the data the loop already has: **successive query fixes**. Each
query returns an absolute map-frame position at its issue time, so the displacement between
two consecutively published fixes, divided by the wall time between their issues, is a direct
map-frame velocity; the next seed is pushed ahead by `velocity * (now - query_issue_time)`.
Monotonic wall time is used consistently for both the velocity `dt` and the latency, so the
result is correct under `use_sim_time` and any constant bag-replay rate (the rate cancels).

Safety is the same shape as the rest of the guard: a perceptual-aliasing wrong candidate (the
first run saw a 190 m-off candidate scored 0.99) would imply an absurd speed, so any estimate
above `max_seed_speed_mps` (default 30) is rejected and the raw candidate is published
unchanged; the latency is clamped to `max_seed_latency_sec` (default 30) so a pathologically
slow query cannot extrapolate a first-order model arbitrarily far. Compensation therefore only
ever fires when two consecutive fixes are mutually consistent -- exactly when the velocity is
trustworthy -- and a bad estimate is always a no-op, so this can never *worsen* a publish. It
is off by default (`enable_seed_motion_compensation` / `supervisor_enable_seed_motion_compensation`
launch arg), since it only helps a moving vehicle with slow queries.

The compensation math is pure and unit-tested in `reinitialization_supervisor_policy.py`
(`estimate_seed_velocity`, `forward_compensate_xy`) with `test_reinitialization_supervisor_policy.py`
pinning the velocity estimate, the within-query-walk rejection, the implausible-speed
rejection, the latency clamp, and the invalid-estimate no-op. Live verification on Koide
(does the compensated seed lock where the raw one went stale) is the remaining idle-machine
step.
