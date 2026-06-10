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

### G2: runtime global localization service

Goal: expose G1 as an on-demand ROS 2 service, still without automatic publication.

1. `global_localization_node` (or component) offering a `~/query` service: input latest
   scan, output ranked pose candidates with registration scores.
2. Candidates feed the existing dry-run command artifact path; publication stays manual.
3. Add a replay smoke that calls the service on a known failure window.

Done when:

- service returns a usable candidate on the validated G1 windows in bounded time
- no change to default launch behavior; the service is opt-in

### G3: guarded automatic reinitialization

Goal: connect `/reinitialization_requested` to the G2 service behind the existing
recovery supervisor, gated by the relocalization runtime rules already defined in
[competitive_roadmap.md](competitive_roadmap.md) Decision Gates:

- candidate generation without reference-oracle ordering
- registration scoring from runtime-available inputs
- reset publication guarded by explicit state and validation checks
- post-reset recovery evidence
- a regression test that fails on unsafe publication or false acceptance

G3 starts only after G1/G2 artifacts are stable on at least two public failure scenarios.

## Non-Goals For Now

- production-grade kidnapped-robot recovery claims
- GPU global search (BBS/FPFH CPU behavior must be characterized first)
- making global localization part of the default bringup path

## Benchmark Hooks

- Koide hard localization `outdoor_hard_01a` request windows (existing manifests)
- kidnapped-start variant: same bag, no `/initialpose`, global engine must bootstrap
- Istanbul stays a no-IMU regression guard and is not a global localization target
