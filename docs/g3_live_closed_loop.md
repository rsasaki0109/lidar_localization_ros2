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

**Live validation of the walk on this exact kidnap window is still pending** — the unit
and integration tests prove the supervisor walks and recovers given a ranked list whose
list contains the true pose; whether the real BBS top-K at this kidnap location actually
contains it (and within the staleness budget) is the next thing to confirm on the bag.
