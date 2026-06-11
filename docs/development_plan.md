# Development Plan

Last updated: 2026-06-11

This document is the working development plan for `lidar_localization_ros2`. It is more
detailed and more dated than [competitive_roadmap.md](competitive_roadmap.md): the roadmap
defines direction and decision gates, this plan tracks the concrete phases, their order,
their done criteria, and the current execution state. When the two disagree, the roadmap's
claim rules and decision gates win.

Related documents:

- [competitive_roadmap.md](competitive_roadmap.md): mission, competitors, decision gates, claim rules
- [reliability_roadmap.md](reliability_roadmap.md): issue triage matrix and sprint notes
- [global_localization_roadmap.md](global_localization_roadmap.md): G1/G2/G3 global localization phases
- [v1_status.md](v1_status.md): what v1.0.0 means and its validated boundary
- [public_validation_log.md](public_validation_log.md): recorded validation snapshots

## Current State (2026-06-11)

What is true right now:

- `v1.0.0` was released 2026-03-30. Since then `main` has accumulated a large unreleased
  reliability batch: the Sprint 1/2 crash fixes (#76, #56, #47), the frame/troubleshooting/
  map-alignment/covariance documentation set, Istanbul drift tuning, Koide outdoor recovery
  tuning, and Boreas diagnostic work.
- The 2026-05-22 release regression snapshot (`overall_pass=true`) plus the 2026-06-10
  drift-tuning reconfirm are the latest recorded green boundaries on the Humble overlay.
- New on 2026-06-11: the public regression suite is green on a ROS 2 Jazzy build for the
  first time (`overall_pass=true`): Istanbul 60 s no-IMU translation RMSE `0.673 m`,
  rotation RMSE `2.179 deg`, alignment median `0.036 s`; HDL 60 s two-repeat medians
  `468.5` baseline / `459.0` IMU-candidate pose rows, IMU-candidate alignment median
  `0.084 s`, fallback events `1`. The long Nav2 reinitialization-supervisor regression is
  the remaining release-suite stage (see execution log below).
- The P0 crash issues are fixed on `main` but still open on GitHub, which makes the
  package look more broken than it is to a new user. Closing them is gated on tagging the
  release that contains the fixes.
- Local development now also runs on ROS 2 Jazzy: `scripts/setup_local_env.sh` resolves
  the distro instead of hardcoding Humble, and the HDL sample fetch patches rosbag2
  metadata per distro. Build, 32 unit tests, and the experiment suite pass on Jazzy.
- Global localization work has started: the `MAP_GRID` candidate baseline
  (`scripts/make_map_grid_relocalization_attempts.py`) generates map-wide seed candidates
  without a route prior and feeds the unchanged v1.1 scoring/dry-run chain.

Known execution constraints:

- The shared development machine often runs other heavy workloads. The release
  regression's timing gates (HDL alignment median <= 0.10 s, pose-row floors) are only
  meaningful on an idle machine; under load ~37 the suite fails on throughput, not on
  correctness. Performance-gated validation must check `load < ~5` first.
- Jazzy builds still use the `ndt_omp_ros2` `humble` branch, matching CI.
- `run_release_regression_suite.sh --resume` reuses failed run outputs; delete the failed
  subdirectories from the work dir before resuming.
- The Nav2 reinitialization-supervisor regression depends on a workspace-local artifact
  that is not shipped in the repo: `artifacts/public/autoware_istanbul_60s_nav2_map/
  istanbul_60s_nav2_map.yaml`. On a fresh workspace it must be bootstrapped once from the
  Istanbul map with `scripts/generate_occupancy_map_from_pcd.py --pcd
  data/official/autoware_istanbul/pointcloud_map.pcd --output-dir
  artifacts/public/autoware_istanbul_60s_nav2_map --map-name istanbul_60s_nav2_map`.
  Folding this bootstrap into the regression entry script is an open cleanup item.

## Phase 0: v1.1.0 Release Closeout

Goal: ship the accumulated reliability work as `v1.1.0` and clean up the issue tracker so
the public state of the project matches the actual state of `main`.

Steps:

1. Run `scripts/run_release_regression_suite.sh` on an idle machine and confirm
   `overall_pass=true`. Status: the public-suite half is green on Jazzy as of 2026-06-11
   (numbers in Current State); the Nav2 reinitialization-supervisor half is pending the
   occupancy-map bootstrap described in the execution constraints. Earlier same-day
   attempts under load ~37 failed only on throughput gates, not correctness; Istanbul RMSE
   passed even then (1.39 m vs the 6.0 m gate).
2. Move the `Unreleased` section of `CHANGELOG.md` to `1.1.0`, bump `package.xml` to
   `1.1.0`, record the validation snapshot in
   [public_validation_log.md](public_validation_log.md), and tag `v1.1.0`.
3. Close fixed P0 issues #76, #56, #47 with the fix commits, the release tag, and a short
   verification recipe. They already carry "fixed on main" comments; the close comment
   only needs to add "released in v1.1.0".
4. Sweep the documented issues (#58, #27, #70, #75, #44, #72, #48, #43, #41, #35): reply
   with the matching doc link (frame_contract, troubleshooting, map_alignment,
   pose_covariance, benchmarking) and close with an invitation to reopen if the doc does
   not resolve the report.

Done when:

- `v1.1.0` tag exists and points at a commit with a green release regression
- the issue tracker has no open issue that is already fixed or already documented
- `reliability_roadmap.md` statuses are updated from "fix in main" to "released"

## Phase 1: Koide Benchmark Track

Goal: make the Koide hard-localization dataset the controlled public benchmark for
recovery and backend ranking, replacing Istanbul as the research driver.

Steps:

1. Validate the route-proximity relocalization artifact beyond the single 180 s request
   window on `outdoor_hard_01a` (additional windows, and at minimum one different segment
   or seed perturbation).
2. Once the failure boundary has a controlled recovery story, run the backend comparison
   (`NDT_OMP` vs `SMALL_GICP` vs `SMALL_VGICP`) with identical bag/map/eval commands.
3. Apply the roadmap's backend decision gate (two public datasets, no catastrophic
   failure, alignment-time or robustness win, RMSE not materially worse) before changing
   any default.

Done when:

- recovery behavior on Koide outdoor is reproducible across at least two failure windows
- a backend ranking table exists with public commands and artifacts
- default backend/recovery settings are justified by that data, or explicitly kept

## Phase 2: Boreas Recovery

Goal: turn Boreas from a diagnostic-only track into a usable public `LiDAR + IMU + GT`
benchmark. Current state is not usable for ranking: 60 s RMSE ~45 m, with
`local_map_crop_too_small` and fitness-reject streaks as the dominant failure modes,
even though the first ~12 s track at ~0.04 m error.

Steps:

1. Root-cause the `local_map_crop_too_small` bottleneck (suspected map-split / crop-center
   interaction with the prediction path).
2. Separate the reject-streak trigger: stale prediction vs score-threshold mismatch, using
   the existing diagnostic sweep manifests.
3. Re-evaluate seed management (`post_reject_strict`, `rejected_seed_update`,
   `max_twist_prediction_dt`) once tracking survives past the current cliff.

Done when:

- Boreas 120 s runs hold tracking without reject-streak collapse
- RMSE is in a range where backend and IMU comparisons are meaningful
- Boreas earns a defined role in the regression or benchmark suite

## Phase 3: Measurement Acceptance, Diagnostics, Covariance

Goal: move acceptance and failure detection beyond the scalar fitness score, so drift is
caught before large pose error accumulates and downstream consumers can trust the outputs.

Steps:

1. Multi-criteria measurement acceptance (score + correction magnitude + overlap +
   prediction staleness), developed through the `experiments/` comparison framework first.
2. Diagnostics that distinguish bad match, missing map/initial pose, weak overlap, stale
   prediction, and overload.
3. Covariance semantics for `/pcl_pose` that fusion/arbitration consumers can rely on
   (the real fix for #72), building on [pose_covariance.md](pose_covariance.md).

Done when the roadmap's "Next" done-criteria for diagnostics and covariance hold.

## Global Localization Track (G1 -> G3)

Runs in parallel with Phases 1-3; detailed phases live in
[global_localization_roadmap.md](global_localization_roadmap.md).

- G1 (artifact-first map-wide candidates): `MAP_GRID` baseline implemented 2026-06-11
  (unit-tested, verified on the Istanbul map and against the registration job generator).
  A `BBS_2D` implementation (multi-resolution occupancy pyramid with admissible
  branch-and-bound over x/y/yaw, scan input from the bag at the request-window trigger) is
  being drafted and reviewed; it lands only with passing unit tests for bound
  admissibility and top-K ordering. Next after that: evaluate both engines on the Koide
  180 s window against the route-proximity artifact, then `FPFH_RANSAC` as the full-3D
  fallback.
- G2 (runtime query service): opt-in service returning ranked candidates; starts after G1
  beats or matches route-proximity on the validated windows.
- G3 (guarded automatic reinitialization): connects `/reinitialization_requested` to G2
  behind the recovery supervisor; gated by the roadmap's relocalization runtime rules.

## Later

Unchanged from the roadmap: dynamic map loading / tiling, GPU backends after CPU
characterization, Jetson + MID-360 hardware validation as a separate track.

## Operating Rules

- `run_release_regression_suite.sh` remains the release boundary; behavior or parameter
  changes also run `run_experiment_suite.py` and the public regression suite.
- Performance-gated suites run only on an idle machine (see execution constraints).
- No public claim without a repeatable public artifact; the claim rules in
  [competitive_roadmap.md](competitive_roadmap.md) apply to every phase above.
- BSD/MIT/Apache sources only for direct adaptation (e.g. `hdl_global_localization` is
  BSD-2 and safe to reference).

## Execution Log

### 2026-06-11: Jazzy bring-up and first Jazzy-green public suite

What happened, in order, on a fresh Jazzy-only workspace:

1. Workspace bootstrap: `repo` + `build_ws` (with `ndt_omp_ros2` `humble` branch) +
   `local_prefix`, per [local_build.md](local_build.md). `colcon build` succeeded; all 32
   unit tests and the experiment suite passed.
2. Two environment fixes were needed and are now on `main`:
   - `setup_local_env.sh` hardcoded Humble; it now resolves
     `LIDAR_LOCALIZATION_ROS_DISTRO` / `ROS_DISTRO` / humble / jazzy in that order.
   - The HDL sample fetch patched rosbag2 metadata into the Humble-only string form,
     which Jazzy's rosbag2 (metadata version 9) rejects with a yaml-cpp `bad conversion`
     at `offered_qos_profiles`; the patch is now distro-aware.
3. First release-regression attempt ran while the machine carried unrelated workloads
   (load ~37 on 20 hardware threads). Every timing-gated check failed (HDL alignment
   median `0.26-0.32 s` vs the `0.10 s` gate, pose rows ~95 vs 250) while correctness
   held (processes survived, Istanbul RMSE `1.39 m` vs the `6.0 m` gate). Lesson recorded
   in the execution constraints: performance gates are meaningless on a loaded machine.
4. A `--resume` retry silently reused the broken HDL outputs and failed again in
   seconds; the failed subdirectories must be deleted first. Also recorded above.
5. After load dropped to ~7, a clean rerun produced the first Jazzy-green public suite:
   `overall_pass=true`, Istanbul `0.673 m` / `2.179 deg` / alignment median `0.036 s`,
   HDL medians `468.5 / 459.0` pose rows with IMU-candidate alignment median `0.084 s`.
   These numbers are comparable to the recorded Humble boundary (`1.176 m` Istanbul,
   `~550` HDL rows) and better on Istanbul RMSE for this run.
6. The release suite then stopped at the Nav2 reinitialization-supervisor stage: the
   occupancy-map artifact `istanbul_60s_nav2_map.yaml` is workspace-local and absent on a
   fresh checkout. Regeneration from the public Istanbul PCD is in progress; folding the
   bootstrap into the entry script is an open cleanup item.

Release decision: `v1.1.0` tags only after the full release suite (public + Nav2
supervisor) is green in one run. The public-suite half is no longer the blocker.

### 2026-06-11: First G1 evaluation on Koide (after v1.1.0)

- Dataset bootstrapped on a fresh workspace: `outdoor_hard_01a` sequence (1.64 GiB,
  Zenodo transfers need resume+retry), maps, GT converted via
  `tum_trajectory_to_pose_reference_csv_for_rosbag2.py`.
- The `180 s` route-proximity artifact manifest reproduced a request window, but the
  registration stage rejected all 32 candidates — including the ground-truth pose, which
  scores NDT fitness `9.6` vs the `6.0` gate at that location. The window is adversarial
  for any scan-matching method; the fixed scalar gate is the weakest layer (Phase 3
  evidence).
- `BBS_2D`, after tuning on this data (min-range filter for Livox invalid returns,
  sparse structure grid, dilation tolerance, 5 deg yaw, spatial NMS), solves a synthetic
  kidnapped-start on a healthy scan end-to-end: rank-1 candidate -> NDT -> `0.69 m /
  1.0 deg`, no priors. G2 has a working basis.
- `MAP_GRID` had a yaw-aliasing bug in its candidate cap (stride collapsed every
  candidate to one heading); fixed with a regression test.
- Detailed numbers live in
  [global_localization_roadmap.md](global_localization_roadmap.md).

### 2026-06-11: Phase 3 opened with the measurement_acceptance experiment

The Koide failure window turned Phase 3 into the evidence-backed priority, so it
started the same day:

- new `experiments/measurement_acceptance` track with six fixtures taken from the
  reproduced failure window (healthy row, degraded onset, degraded streak, stale
  prediction, total loss, synthetic fresh-prediction jump)
- `correction_conditioned` (fitness threshold + correction/staleness cross-check)
  passes 6/6; the runtime-equivalent `fixed_threshold` baseline passes 3/6 — it
  rejects the rows that start the 300-row reject streak and accepts the aliased
  jump
- next step: validate the winning policy against full replay (does accepting the
  degraded-onset rows keep RMSE bounded end-to-end on Koide and Istanbul?), then
  consider the `measurement_gate_policy` C++ change behind the public regression
  gates

#### 2026-06-12 closed-loop replay result: fixture winner fails end-to-end

The `correction_conditioned` policy was implemented as an opt-in
`degraded_tracking_acceptance` gate (unit-tested against the same fixture
values) and A/B-replayed on Koide `outdoor_hard_01a` `180 s` on an idle
machine:

- baseline (scalar gate): translation RMSE `0.582 m`, ok rows `329/539`
- degraded acceptance: translation RMSE `62.7 m`, ok rows `152/535`

Root cause: in closed loop the correction-vs-prediction cross-check is
self-referential. Each accepted degraded measurement becomes the next
prediction, so a drifting pose keeps producing small corrections and keeps
"confirming" itself; and because the streak budget counts *rejections*, every
acceptance resets it, making the budget unbounded in degraded regions. Single-
step fixtures cannot expose either effect.

Consequences (the runtime change was reverted; the experiment stays):

1. the `measurement_acceptance` experiment needs sequence fixtures that replay
   a window of consecutive samples with closed-loop feedback, not single rows
2. the next policy iteration must bound *consecutive degraded accepts* with a
   counter that only a genuinely below-threshold measurement resets, and/or
   bound the cumulative correction absorbed during a degraded phase
3. fixture-level benchmark scores are necessary but not sufficient; replay
   validation stays mandatory before any runtime gate change

### 2026-06-11: Global localization G1 start

- `MAP_GRID` baseline merged: map-wide seed candidates (occupied-cell centroids with a
  per-cell ground-percentile z and discretized yaw) feeding the unchanged v1.1
  registration-scoring and dry-run command chain. Verified on the Istanbul map (21,172
  cells at 20 m spacing, deterministic striding to the candidate cap) and against
  `make_registration_relocalization_jobs.py` with `candidate_index` ordering.
- `BBS_2D` drafting started the same day (see the track section above for the landing
  criteria).

### 2026-06-12: Degraded acceptance closed — definitive negative after two more replays

The bounded redesign from the 2026-06-12 failure analysis (`bounded_degraded`: budget of
consecutive degraded accepts that only a clean acceptance resets, plus a cumulative
correction cap) was implemented as a runtime gate and replay-validated twice on the Koide
`outdoor_hard_01a` 180 s manifest. Both rounds regressed; all numbers are NDT, score gate
6.0, same machine:

| run | translation RMSE | rotation RMSE | ok rows |
| --- | --- | --- | --- |
| baseline, scalar gate (night run) | 0.582 m | 1.81 deg | 329/539 |
| baseline, scalar gate (same-day rerun, load ~10) | 0.216 m | 2.30 deg | 179 ok-published |
| bounded round 1 (budget 3, cumulative 5 m) | 0.618 m | 6.10 deg | 193/566 |
| bounded round 2 (round 1 + fitness cap 12 + reset guard) | 6.80 m | 28.6 deg | 179/557 |

Round-1 forensics (alignment rows 185-205) found two new failure modes beyond the
2026-06-12 analysis: a below-threshold "ok" (fitness 1.75) carrying a 2.1 m jump refilled
the budget mid-transient, and fitness 18.7/31.9 rows were accepted because only the
correction was checked. Round 2 fixed both (max fitness 12, budget reset requires
correction <= 1 m) and still diverged: eleven accepts in the supposedly safe 6.3-8.7
fitness band — the very band the "GT scores ~9.6" motivation argued for — dragged the
anchor ~1-2 m per accept and the run got lost 30 s earlier than baseline. The same-day
baseline rerun at equal load (0.216 m) rules out machine-load confounding.

Conclusion: on degenerate geometry the over-threshold registrations are systematically
biased, not noisy, so any acceptance of them feeds bias into the prediction anchor.
Three closed-loop falsifications (correction_conditioned 62.7 m, bounded r1, bounded r2)
close this direction. The scalar gate + coast-on-rejection (+ existing recovery retry) is
the replay-validated winner; recovery inside the hard window belongs to the
relocalization/retry track (G2/G3), not to measurement acceptance.

Artifacts kept: the falsified hypotheses are now encoded in the experiment — the two
"should_accept" fixtures are flipped to `should_reject` with the falsifying evidence in
their descriptions, a new `koide_bounded_replay_regression_sequence` (built from the real
round-1 trace) requires rejecting gross fits and the exhausted tail, and the sequence
harness gained `must_reject_indices`. With the corrected expectations `fixed_threshold`
ranks first (85.3) — the offline ranking finally agrees with the replays. The C++ gate
change was reverted both times; `main` never shipped any degraded acceptance.

Phase 3 step 1 (multi-criteria acceptance) is closed as a negative result. Remaining
Phase 3 work: diagnostics taxonomy (step 2) and covariance semantics (step 3).

### 2026-06-12: Diagnostics taxonomy (Phase 3 step 2)

`/alignment_status` now publishes a `failure_category` key that classifies each update
into the roadmap's five distinguishable causes — `missing_map`, `missing_initial_pose`,
`weak_overlap`, `bad_match`, `stale_prediction`, `overload` — plus `healthy`, with
companion `*_active` booleans so co-occurring conditions stay visible (e.g. a bad match
that has also gone stale reports `bad_match` as the category and both flags true).

Design points:

- Pure policy header `alignment_failure_taxonomy.hpp` (`classifyAlignmentFailure`),
  wired through `alignment_status_policy.hpp` so the component only passes parameters.
  Classification reuses the resolved reinitialization gap, so staleness is detected even
  when the caller cannot provide `accepted_gap_sec` directly.
- Weak overlap uses the scan-side proxy (`filtered_point_count <
  diagnostics_weak_overlap_min_filtered_points`, default 100, mirroring
  `local_map_min_points`) and the map-side `local_map_crop_too_small` status.
- Priority order is missing inputs > weak overlap > bad match > stale prediction >
  overload; thresholds are the three new `diagnostics_*` parameters.
- Diagnostics only: acceptance behavior is untouched, so this is exempt from the
  replay-gate rule (no gate semantics changed); unit tests cover every category,
  the priority order, and boundary values.

Documented for users in [troubleshooting.md](troubleshooting.md) ("Failure category").
Remaining Phase 3 work: covariance semantics (step 3, the promised real fix for #72).

### 2026-06-12: Calibrated /pcl_pose covariance (Phase 3 step 3, closes the #72 promise)

The default covariance model is now `error_floor`, calibrated against ground
truth using the replay artifacts already on disk (two Koide outdoor_hard_01a
baselines + Istanbul 60 s; 628 matched poses total) via the new
`scripts/analyze_pose_covariance_calibration.py`.

Key empirical findings (report archived at
`experiments/pose_covariance/calibration_2026_06_12.json`):

- Within the accepted range, error is dominated by a per-dataset floor (Koide
  ~0.18 m, Istanbul ~0.26 m p68), not by fitness — quantifying the #72 thread's
  intuition that fitness is not linearly related to error across environments.
- The legacy `fitness_scaled` model is systematically overconfident: per-axis
  1σ xy coverage 0.42–0.59 measured against ground truth.
- The calibrated defaults (`std_xy = 0.2 + 0.1·fitness` m clamped to 5 m,
  `std_yaw = 2° + 1°·fitness` clamped to 30°, z floor 0.3 m, roll/pitch floor
  1.5°) achieve per-axis xy coverage 0.69–0.95 at 1σ and ≥ 0.96 at 2σ on all
  three runs, slightly conservative by design on tighter maps.
- The biased tail is explicitly out of scope: Koide has accepted poses with
  4.2 m error at fitness 0.5–1.0 (the same degenerate-geometry bias that killed
  degraded acceptance). Documentation directs fusion consumers to also gate on
  `failure_category`.

Implementation: `ErrorFloorCovarianceParams` + `makeErrorFloorPoseCovariance` /
`makeEkfErrorFloorPoseCovariance` in `pose_covariance_policy.hpp`, mode switch
`pose_covariance_mode` (`error_floor` default, `fitness_scaled` legacy), eight
`covariance_*` parameters, unit tests for clamps/NaN/EKF-hybrid paths.
`pose_covariance.md` rewritten with the calibration tables, a recalibration
recipe for user platforms, and an explicit can/cannot-promise list. Covariance
does not affect acceptance or trajectory, so the replay gate does not apply.

## Suggested Order Of Work

1. Phase 0 (release + issue hygiene) — small, high leverage, mostly waiting on an idle
   machine for the regression run.
2. G1 evaluation on Koide + Phase 1 window validation — these share the same dataset and
   manifests, so they advance together.
3. Phase 2 Boreas root-cause, interleaved as long-running sweeps.
4. BBS_2D engine, then Phase 3 acceptance/diagnostics work feeding G2/G3 readiness.
