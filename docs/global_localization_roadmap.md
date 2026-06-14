# Global Localization Roadmap

Last updated: 2026-06-11

This document defines how global localization (pose estimation without a user-provided
initial pose) enters this repository without breaking the claim discipline in
[competitive_roadmap.md](competitive_roadmap.md).

## Position

Today the localizer requires `/initialpose` (or a map-frame seed) before tracking starts.
The v1.1 relocalization pipeline ([v1_1_relocalization.md](v1_1_relocalization.md)) already
proves the downstream chain: candidate poses -> registration scoring -> reset candidate
selection -> validated dry-run `/initialpose` command. Its current candidate source is a
route corridor, which assumes a prior route artifact.

Global localization replaces that candidate source with a map-wide search. Everything
downstream of candidate generation is reused unchanged. This keeps the new work small,
benchmarkable, and inside the existing claim boundary.

Reference implementation: `koide3/hdl_global_localization` (BSD-2-Clause; BBS, FPFH +
RANSAC engines). License is compatible with direct adaptation.

## Phases

### G1: map-wide candidate generation, artifact-first

Goal: replace the route-grid candidate stage with engines that need no route prior.

1. `MAP_GRID` baseline: `scripts/make_map_grid_relocalization_attempts.py` samples seed
   poses from occupied map cells with discretized yaw and emits the existing
   `relocalization_attempts.csv` / `relocalization_candidates.csv` contract. This is the
   brute-force coverage baseline that the smarter engines must beat.
2. `BBS_2D` engine: `scripts/make_bbs_relocalization_attempts.py` runs an admissible
   branch-and-bound search over a max-pooled pyramid of the 2D occupancy grid from
   `scripts/generate_occupancy_map_from_pcd.py`, matching the bag scan nearest to each
   request window and emitting top-K candidates best-first in the shared CSV contract.
3. `FPFH_RANSAC` engine: PCL FPFH features + RANSAC alignment as a full-3D fallback.
4. All engines emit the existing `relocalization_attempts.csv` schema so the v1.1
   scoring, selection, and dry-run command stages run unmodified.

Done when:

- Koide `outdoor_hard_01a` `180 s` request window: map-wide candidates reach a refined
  pose comparable to the route-proximity artifact, without `oracle_rank` and without a
  route prior
- at least one additional failure window (or kidnapped-start scenario) is evaluated
- engine runtime and candidate counts are recorded in the benchmark artifacts

#### 2026-06-11 first G1 evaluation on Koide outdoor_hard_01a

Same-run comparison on one reproduced request window plus one synthetic
kidnapped-start scan (machine was shared; absolute replay numbers are not
comparable to the recorded boundary):

- kidnapped-start, healthy scan: `BBS_2D` rank-1 candidate refines through the
  standard NDT stage to `0.69 m / 1.0 deg` from ground truth with no route
  prior and no initial pose. The BBS recipe that made this work: minimum-range
  filter (Livox zero-filled invalid returns), sparse structure grid
  (`--obstacle-height-m 1.5 --min-points-per-cell 10 --inflate-radius-m 0`),
  1-cell dilation tolerance, 5 deg yaw sampling, spatial NMS. Runtime ~14 min
  per window in pure python (optimization is future work).
- the reproduced `180 s` failure window is not a candidate-generation problem:
  even the ground-truth pose scores NDT fitness `9.6` against the map there
  (gate `6.0`), and BBS misses accordingly. Route-proximity candidates reach
  `2.4 m` in oracle terms but the registration gate rejects all of them too.
  Recovery at such windows belongs to retry/last-pose (already shipped) and to
  multi-criteria acceptance (Phase 3), not to a better global engine.
- `MAP_GRID` stays the coverage baseline (`31.7 m` best at a 512-candidate cap
  on this map); useful as a fallback set, not as a primary engine.

#### 2026-06-12 BBS runtime optimization

`branch_and_bound_candidates` was rewritten around two exact-equivalence
optimizations, removing the "~14 min per window" blocker for G2:

- node coordinates are always multiples of `2^level`, so the per-node floor
  decomposes into a block index plus a `(yaw, level)` integer offset table,
  precomputed once and deduplicated with counts;
- per `(yaw, level)` pair the scorer starts with direct gathers and switches to
  a one-shot FFT cross-correlation hit map for the whole level once the pair is
  hot (hit counts are integers, so rounding restores exact values).

Measured: synthetic 1200x1200 / 72-yaw / top-64 search 81.3 s -> 9.6 s (8.5x);
HDL smoke window end-to-end 19.9 s -> 8.1 s with byte-identical candidate CSVs
(the remainder is bag/map loading). The remaining floor is the Python
branch-and-bound loop itself (~580 k heap pops); if G2 needs sub-second
queries, that loop is the next target (C++ or batched expansion).

### G2: runtime global localization service

Goal: expose G1 as an on-demand ROS 2 service, still without automatic publication.

1. `global_localization_node` (or component) offering a `~/query` service: input latest
   scan, output ranked pose candidates with registration scores.
2. Candidates feed the existing dry-run command artifact path; publication stays manual.
3. Add a replay smoke that calls the service on a known failure window.

Done when:

- service returns a usable candidate on the validated G1 windows in bounded time
- no change to default launch behavior; the service is opt-in

#### 2026-06-13 G2 service node and end-to-end demo

`scripts/global_localization_node.py` implements the on-demand service:
`std_srvs/Trigger` on `~/query` runs the (8x-optimized) BBS_2D search on the
latest scan from the localization cloud topic and publishes ranked candidates
as a `geometry_msgs/PoseArray` on `~/candidates` (transient local). The pure
query logic lives in `scripts/global_localization_query.py` (unit tested
without rclpy). Opt-in only: nothing changes unless the node is started and
the service is called.

End-to-end demo on the HDL sample bag (kidnapped start, `set_initial_pose:
false`): play bag -> pause player -> `~/query` answers in 23 s with 16
candidates (top score 0.998) -> top candidate published as `/initialpose` ->
resume -> NDT localization tracks the full remaining bag (858 accepted poses
around the loop). Rendered as a GIF by
`scripts/render_global_localization_demo_gif.py` from the recorded artifacts.
The pause is honest about query latency; G3 automation will need either the
~9 s coarse-yaw setting or the C++ port noted in the G1 optimization entry.

### G3: guarded automatic reinitialization

> How to run the G2 service and the G3 supervisor: see
> [global_localization.md](global_localization.md).

Goal: connect `/reinitialization_requested` to the G2 service behind the existing
recovery supervisor, gated by the relocalization runtime rules already defined in
[competitive_roadmap.md](competitive_roadmap.md) Decision Gates:

- candidate generation without reference-oracle ordering
- registration scoring from runtime-available inputs
- reset publication guarded by explicit state and validation checks
- post-reset recovery evidence
- a regression test that fails on unsafe publication or false acceptance

G3 starts only after G1/G2 artifacts are stable on at least two public failure scenarios.

#### 2026-06-14 G3 guard policy and supervisor node (logic complete, runtime pending)

The loop is now wired in code. The localization component already raises
`/reinitialization_requested` from its C++ recovery supervisor; the new
`reinitialization_supervisor_node.py` consumes it, calls the G2 `~/query` service,
and republishes `/initialpose`. The safety-critical part -- *whether* it is allowed
to publish -- is a ROS-free state machine, `reinitialization_supervisor_policy.py`,
so it can be tested without a live stack:

- candidate-score floor: a reset is never published from a candidate scoring below
  `min_candidate_score` (the "reset publication guarded by explicit checks" gate);
- minimum reset spacing and a non-self-resetting attempt ceiling, so a
  confidently-wrong candidate cannot loop -- the Phase 3 closed-loop blowup shape;
- post-reset recovery evidence: after a reset the supervisor waits for alignment
  fitness back under `recovery_fitness_threshold` before standing down, and gives
  up after `max_attempts` rather than retrying forever;
- edge-triggered re-arming: after recovery or give-up it waits for the request to
  de-assert, so a stuck-high request line cannot re-fire it.

`test/test_reinitialization_supervisor_policy.py` is the required "regression test
that fails on unsafe publication or false acceptance": ten adversarial sequences
covering transient blips, weak candidates, false acceptance, spacing, budget reset
on recovery, and the exhausted latch. All pass.

**Runtime glue validated (2026-06-14):** the supervisor node now runs under ROS.
A first launch surfaced and fixed a shutdown bug (an external SIGTERM raised an
uncaught `ExternalShutdownException`). `test/test_reinitialization_supervisor_node_ros.py`
is an rclpy integration smoke (skipped without a sourced ROS env) that fakes the
localizer + G2 service and asserts the full path end-to-end: the node receives
`/reinitialization_requested` + `/alignment_status`, debounces, calls the G2
`~/query` service, parses the candidate JSON, and publishes `/initialpose`
(correct pose + covariance) before entering `settling`. So the supervisor's own
job -- decide and publish a guarded reset -- is validated end-to-end.

**3-node bringup authored (2026-06-14):** `launch/global_localization_recovery.launch.py`
now brings up all three nodes together -- the core localizer (via the existing
`lidar_localization.launch.py`), the G2 `global_localization_node`, and the G3
`reinitialization_supervisor_node` -- on a shared `cloud_topic` / `global_frame_id`,
with the supervisor wired to the G2 `~/query` service and the localizer's
`/alignment_status` and `/reinitialization_requested`. The supervisor's typed guards
(`min_candidate_score`, `max_attempts`) are exposed as launch arguments (coerced with
`ParameterValue` so the node's int/float parameter types are respected). The two
supervisor files are now installed via `CMakeLists.txt` so they resolve as
`ros2 run` / launch executables. `ros2 launch ... --show-args` validates the full
argument graph (including the included localizer's pass-through args).

**Live closed-loop exercised (2026-06-15):** the three-node bringup was run against the
real `outdoor_hard_01a` bag + map with a kidnap injected at a healthy location. Full
write-up in [g3_live_closed_loop.md](g3_live_closed_loop.md). Two results:

- *The G3 supervisor mechanism is validated end-to-end against a real stack.* Detect →
  debounce → query G2 → score-guard → guarded `/initialpose` publish → recovery-wait →
  retry → `max_attempts` ceiling → safe give-up with an operator alert all fired
  correctly. The safety ceiling contained three confidently-wrong-or-stale candidates
  without an unbounded reset loop — the Phase 3 lesson validated live, not just offline.
- *Recovery did not complete, and the cause is candidate quality, not the supervisor.*
  G2's BBS_2D returned high-confidence but grossly wrong candidates (107 m and 190 m off
  at BBS score ≈ 0.99) on 2 of 3 attempts; the correct pose (5.5 m, score 0.998) arrived
  too late. The BBS occupancy score does not separate right from wrong.

So the recovery-evidence gate had a concrete, evidence-backed blocker: the supervisor
trusted the BBS score, which does not separate right from wrong.

**Ranked-candidate walk implemented (2026-06-15):** the supervisor now walks the ranked
candidate list from a single query, using the localizer's NDT fitness as the
registration oracle the BBS score is not — publish the best, and if fitness does not
recover within `settle_timeout_sec`, publish the next-best from the same query. Walking
does not spend a `max_attempts` slot (only re-querying does), so the ceiling still bounds
queries while one query can try every pose it found. G2 returns the full ranked
`candidates` list in its reply; the policy and node carry a candidate index; the new
behaviour is regression-tested (policy walk + list-exhaust + score-floor cases, plus a
ROS integration test that walks 0→1 and recovers). The complementary piece — G2 scoring
each candidate by NDT/GICP fitness so the *ranking* reflects registration quality — and
the BBS query-latency/staleness remain the next G3 work; live validation of the walk on
the kidnap window is pending. See [g3_live_closed_loop.md](g3_live_closed_loop.md).

**Recovery-evidence gate met (2026-06-15).** The final blocker turned out to be that the
reset was published with z = 0: G2 candidates are 2D and the supervisor left
`position.z = 0`, ~11 m above the true Koide ground, outside the NDT z-basin — so even an
x/y/yaw-correct candidate never locked. The supervisor now carries z / roll / pitch from
`/pcl_pose` onto the candidate. With that fix the full loop recovers live on the Koide
kidnap window: after the guarded reset (`recovery_confirmed`) the localizer re-locks and
fitness falls to ~0.1, then the supervisor stands down and re-arms. So *post-reset
recovery evidence* — the last G3 Decision Gate — is now satisfied with a real localizer.
Remaining work is quality, not correctness: faster/first-try recovery (cut BBS query
latency, per-candidate registration scoring in G2).

## Non-Goals For Now

- production-grade kidnapped-robot recovery claims
- GPU global search (BBS/FPFH CPU behavior must be characterized first)
- making global localization part of the default bringup path

## Benchmark Hooks

- Koide hard localization `outdoor_hard_01a` request windows (existing manifests)
- kidnapped-start variant: same bag, no `/initialpose`, global engine must bootstrap
- Istanbul stays a no-IMU regression guard and is not a global localization target
