# v1.1 Relocalization MVP

This document fixes the v1.1 endpoint for the experimental relocalization work.

The v1.1 target is not automatic runtime recovery. The target is an artifact-first,
publicly reproducible pipeline that turns relocalization request windows into a
validated dry-run `/initialpose` command artifact without publishing a reset in the
default benchmark path.

## Decision

Freeze new relocalization helper work for v1.1 and consolidate the existing chain around one
public endpoint:

> A validated dry-run `/initialpose` command artifact derived from NDT_OMP-scored
> route-grid relocalization candidates, with no publish in the default benchmark path.

This endpoint is strong enough to show that candidate generation, scan resolution,
registration scoring, reset candidate selection, and command validation are connected.
It is also narrow enough to avoid claiming autonomous or production-grade recovery.

## Public Stage Boundary

| Stage | Runtime-like? | Offline diagnostic? | Dry-run only? | Can publish? | Public v1.1 claim? | Main artifacts |
| --- | --- | --- | --- | --- | --- | --- |
| Health summary | No | Yes | Yes | No | Yes | `health_summary.json/md` |
| Route-grid candidates | Partly | Yes | Yes | No | Yes | `relocalization_attempts.csv`, `relocalization_candidates.csv` |
| Reference-oracle scoring | No | Yes | Yes | No | Diagnostic only | `relocalization_candidate_scores.csv` |
| NDT_OMP registration scoring | Partly | Yes | Yes | No | Yes | `relocalization_registration_scores_ndt.csv` |
| Ordering comparison | No | Yes | Yes | No | Diagnostic only | `ordering_comparison/*` |
| Reset candidate plan | Partly | Yes | Yes | No | Yes | `relocalization_reset_candidate_plan.csv/json/md` |
| Plan validation | No | Yes | Yes | No | Yes | `relocalization_reset_candidate_plan_validation.json/md` |
| Dry-run reset command | Partly | Yes | Yes | No | MVP endpoint | `relocalization_reset_commands.csv/json/md` |
| Command validation | No | Yes | Yes | No | MVP endpoint | `relocalization_reset_commands_validation.json/md` |
| Guarded publisher | Yes | No | Default yes | Yes with `--execute` | No | `relocalization_reset_command_execution.csv/json/md` |
| Post-reset observation | Evaluation only | Yes | No | No | Future controlled test | `relocalization_reset_execution_observation.csv/json/md` |

## Public Pipeline

The public-facing story is four stages.

1. Candidate generation

   Route-corridor candidates are generated for request windows. The route corridor is an
   offline public-regression artifact, not a runtime claim.

2. Registration scoring

   Candidate rows are converted into registration jobs, nearest scans are resolved from the
   bag, and NDT_OMP scores the candidate initial poses. A gate pass is a scorer artifact, not
   an accepted reset.

3. Reset candidate selection

   A dry-run reset candidate plan is selected from registration-derived fields only:
   convergence, score, refinement delta, yaw delta, and optional score margin. Runtime-like
   selection must not depend on `oracle_rank`.

4. Safe reset artifact

   The selected refined pose is converted into a dry-run `/initialpose` command artifact and
   validated for topic, frame, finite pose values, near-unit quaternion, non-oracle provenance,
   `dry_run=true`, and `published_count=0`.

## Diagnostic And Experimental Utilities

The following pieces remain useful, but they are not the v1.1 public endpoint.

- Reference-oracle scoring measures whether the candidate set contains a plausible pose. It is an
  offline upper-bound diagnostic, not a runtime relocalization score.
- Ordering comparison contrasts runtime-like `candidate_index` ordering with diagnostic
  `oracle_rank` ordering. It should stay bounded and off the default public path.
- `publish_relocalization_reset_commands.py` is an experimental guarded execution utility. It
  does not publish unless `--execute` is passed, and it is not required for the v1.1 MVP claim.
- `observe_relocalization_reset_execution.py` is the acceptance evaluator for future controlled
  execution tests. It is not the v1.1 public MVP endpoint.

## Current Evidence

The current Boreas localizer-only artifact chain exercises the v1.1 endpoint without publishing:

- `candidate_index_top1` produced one scored row with NDT score `2.158257` and a passed local gate.
- `oracle_rank_top1` produced one scored row with NDT score `13.994764` and no gate pass.
- `candidate_index_top5` scored five rows, all with passed local gates, with median score
  `2.175548`.
- `oracle_rank_top5` scored five rows, with zero passed gates and median score `15.483539`.
- The reset candidate plan selected candidate `2` with score `1.331940` and kept
  `accepted=false`.
- The dry-run command artifact targeted `/initialpose`, frame `map`, pose
  `x=75.816933`, `y=11.294861`, `z=0.653252`, `yaw=0.248475`, and kept
  `published_count=0`.
- Command validation passed for the valid artifact and rejected an altered non-dry-run artifact.
- The guarded publisher, when run without `--execute`, reported `published_count=0` with
  `execute_flag_required`.

This is evidence for a guarded artifact chain. It is not evidence for autonomous relocalization or
post-reset recovery success.

The current Koide `outdoor_hard_01a` 180 s failure-boundary diagnostic adds a sharper recovery
target:

- The lowered-threshold `180_reinit090` run produced one request window with `264`
  `/reinitialization_requested` rows and no recovered request window.
- Route-grid generation for that request produced `90` candidates, and reference-oracle scoring
  found candidate `49` at `0.000 m` translation error.
- Candidate-index top-32 NDT_OMP scoring was unsafe as a reset source: the lowest registration score
  was candidate `6` with oracle score `26.293 m`.
- Route-proximity top-32 NDT_OMP scoring is oracle-free and selected candidate `49` first, with
  registration score `0.984596`.
- Reset candidate validation and dry-run command validation both passed for `route_proximity` with
  `published_count=0`.

This shows that the failure window contains a plausible reset pose and that the artifact chain can
carry it to a validated dry-run `/initialpose` command. It does not show automatic runtime
relocalization, because reset publication is still disabled and the route corridor is an offline
evaluation artifact.

## Dataset Roles

### Istanbul

Use as the daily regression and Nav2 replay stability guard.

Safe claim:

- short public regression and controlled Nav2 recovery regression pass

Unsafe claim:

- long-horizon urban replay is solved

### Boreas

Use as the next public dataset coverage target.

The first Boreas target is localizer-only artifact reproducibility: health summary, route-grid
candidates, registration scoring, reset candidate selection, and validated dry-run reset command
artifacts. Do not turn this into a broad robustness claim yet.

### Koide Outdoor01

Use as a public-map failure-boundary diagnostic.

Keep original public-map results separate from diagnostic-map results. The useful claim is boundary
characterization, not robust full-route tracking on the original public map.

For the `outdoor_hard_01a` 180 s boundary, `route_proximity` is the first oracle-free ordering that
selects the known good reset candidate. The next useful task is validating that ordering on more
request windows or replacing the offline route corridor with a runtime candidate source.

### GLIL

Treat as competitor context or plumbing smoke unless a clearly public dataset target is defined.

## Safe Claim Language

Safe v1.1 wording:

> v1.1 adds an experimental artifact-first relocalization evaluation pipeline. It can generate
> route-corridor reset candidates, resolve scans, score candidates with NDT_OMP registration, select
> and validate a reset candidate, and produce a validated dry-run `/initialpose` command artifact.
> Actual reset publication is guarded, opt-in only, and not part of the default public benchmark
> path.

Safe shorter wording:

> experimental artifact-first relocalization evaluation with validated dry-run reset command
> artifacts

## Do Not Claim

- global relocalization solved
- automatic recovery
- robust relocalization
- runtime relocalization
- production-ready reset
- Boreas relocalization success
- oracle-based candidate ordering works at runtime
- `SMALL_GICP` is default-ready
- reliable covariance estimation

## v1.1 Closeout

For v1.1, stop adding scoring, selection, or recovery heuristics. The remaining work is closeout:

1. Keep release regression green.
2. Keep the v1.1 public story centered on the dry-run command validation endpoint.
3. Treat guarded publication and post-reset observation as controlled follow-up work.
4. Add a no-publish smoke runner only if it is a thin wrapper around existing artifacts and adds no
   new scoring or selection behavior.
