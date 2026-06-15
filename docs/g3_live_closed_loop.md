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
