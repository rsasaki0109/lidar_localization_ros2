# Development Plan

Last updated: 2026-06-13

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

## Current State (2026-06-13)

What is true right now:

- `v1.1.0` is released and tagged (commit `668fff2`): the accumulated reliability batch
  (Sprint 1/2 crash fixes #76/#56/#47, the frame/troubleshooting/map-alignment/covariance
  documentation set, Istanbul drift tuning, Koide recovery tuning, Boreas diagnostics) is
  shipped, `package.xml` is at `1.1.0`, and the fixed P0 issues are closed. Phase 0 is
  done.
- Development and the public regression suite run on ROS 2 Jazzy (first Jazzy-green
  recorded 2026-06-11: Istanbul 60 s no-IMU translation RMSE `0.673 m`, rotation
  `2.179 deg`, alignment median `0.036 s`; HDL two-repeat medians `468.5`/`459.0` pose
  rows). `scripts/setup_local_env.sh` resolves the distro and the HDL fetch patches
  rosbag2 metadata per distro.
- Phase 3 (measurement acceptance, diagnostics, covariance) is complete. Step 1
  (multi-criteria acceptance) closed as a replay-falsified **negative result** after three
  closed-loop A/B replays; step 2 added the `/alignment_status` `failure_category`
  taxonomy; step 3 replaced the `/pcl_pose` covariance with a ground-truth-calibrated
  `error_floor` model, closing the #72 promise. All on `main`; unit-test count is 36
  C++/Python policy tests.
- Global localization has reached G2. `MAP_GRID` and the admissible `BBS_2D`
  branch-and-bound engine are merged; `BBS_2D` was sped up ~8x with byte-identical output;
  and the opt-in `global_localization_node.py` runtime service (`~/query` ->
  `~/candidates`) is validated end-to-end on the HDL bag with a kidnapped start. A demo GIF
  of the full kidnapped-start -> relocalize -> resume flow is in the README.
- Open work, in priority order, is the Phase 1 Koide backend comparison (needs an idle
  machine), G3 guarded automatic reinitialization (needs faster queries), and Phase 2
  Boreas recovery. None are started.

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

## Issue Tracker State (2026-06-13)

Twelve issues are open. None are regressions in shipped behavior; they split into
"already answered by the v1.1 doc set", "small actionable code items", "needs real
investigation", and "future enhancement track". Closing or commenting on GitHub is done
by the maintainer; this table is the triage record.

| # | Title | Disposition |
| --- | --- | --- |
| 25 | specifying the lidar frame id | Answered — `frame_contract.md` documents frame expectations; close with pointer |
| 33 | about ros2 humble (ouster remap, inaccurate) | Answered — sensor remap + `frame_contract.md`/`troubleshooting.md`; Humble/Jazzy matrix shipped; close with pointer |
| 34 | use different sensor (Ouster etc.) | Answered — same sensor/frame guidance as #33; close with pointer |
| 37 | CPU占用率爆炸 | Answered — `failure_category: overload` diagnostic + throughput tuning (`voxel`, `ndt threads`, `cloud_queue_depth`) in `troubleshooting.md`; close with pointer |
| 49 | ERROR run with Rslidar | Answered — point-type / remap guidance; close with pointer (ask for log if it recurs) |
| 50 | enhance stability (try/catch) | Partially shipped — Phase 0 crash-survival fixes (#76/#56/#47) + Phase 3 diagnostics; reply with what shipped, keep open for broader hardening |
| 55 | `odom_frame_id_` defined but not used | Resolved (2026-06-14) — stale report; the field is wired into the `map -> odom` TF path (`publishMapToOdomTransform`), no code change needed |
| 54 | `corrent_pose_with_cov_stamped_ptr_` not locked | Resolved (2026-06-14) — the node runs on a `SingleThreadedExecutor` with the default mutually-exclusive callback group, so the shared pose pointer is serialized without a lock; documented the invariant in the node + header + CHANGELOG so a future executor change can't regress it silently |
| 52 | different results (degrades each restart, worse after reboot) | Needs investigation — restart-to-restart degradation suggests state/seed leak, not documented run variance; keep open |
| 68 | MGRS map not displayed in Rviz | Needs investigation — large-coordinate PCD likely hits float32 precision in the map path; reporter offered to implement, give a hint and keep open |
| 77 | IMU angular velocity + estimator | Future enhancement track (IMU); keep open |
| 36 | imu preintegration | Future enhancement track (IMU), overlaps #77; keep open |

The two cheap code wins (#55, #54) were investigated on 2026-06-14: both turned out
to be non-bugs under the shipped configuration (#55 already wired; #54 protected by the
single-threaded executor) and were closed with documenting changes rather than fixes.
The doc-answered set (#25/#33/#34/#37/#49) is ready to close once the maintainer posts
the pointer comments; #52 and #68 want a reproduction before any claim.

Branch hygiene: `origin` carries ten non-`main` branches. Five are stale merged feature
branches fully contained in `main` (`codex/mid360-policy-split`,
`codex/public-validation-log`, `codex/release-validation-log`,
`feat/public-demo-validation-dashboard`, `fix/tf`) and are deletion candidates.
`feature/small_gicp` is *not* a backend integration — its one unmerged commit is only a
2024 README edit, so the Phase 1 `SMALL_GICP` work starts fresh, not from this branch.
The distro branches (`dashing`, `foxy`, `humble`, `jazzy`) are merged into `main` but kept
as user-facing checkout points for those ROS distros and are not deleted.

## Phase 0: v1.1.0 Release Closeout — DONE (2026-06-11)

Goal was to ship the accumulated reliability work as `v1.1.0` and clean up the issue
tracker so the public state matches `main`. Completed:

- `v1.1.0` tagged at `668fff2`; `package.xml` bumped to `1.1.0`; the `CHANGELOG.md`
  `Unreleased` section moved to `1.1.0`; validation snapshot recorded in
  [public_validation_log.md](public_validation_log.md).
- Fixed P0 issues #76/#56/#47 closed with the fix commits, the release tag, and a
  verification recipe; the documented-issue sweep (#58/#27/#70/#75/#44/#72/#48/#43/#41/#35)
  answered with the matching doc links and closed.
- `reliability_roadmap.md` statuses moved from "fix in main" to "released".

A follow-on `Unreleased` batch has since accumulated on `main` (diagnostics taxonomy,
calibrated covariance, BBS speedup, the G2 service); it will roll into the next tagged
release once Phase 1 / G3 work reaches a natural boundary.

## Phase 1: Koide Benchmark Track — IN PROGRESS

Goal: make the Koide hard-localization dataset the controlled public benchmark for
recovery and backend ranking, replacing Istanbul as the research driver.

Status: the first G1 evaluation on `outdoor_hard_01a` ran 2026-06-11 (see execution log).
It established that the 180 s window is a recovery problem, not a candidate-generation
problem (GT pose scores fitness 9.6 there). The backend comparison (step 2) is the main
open item and is **blocked on an idle machine** — it needs `load < ~5` for meaningful
alignment-time numbers, and the shared machine is usually busy.

Steps:

1. Validate the route-proximity relocalization artifact beyond the single 180 s request
   window on `outdoor_hard_01a` (additional windows, and at minimum one different segment
   or seed perturbation).
2. Once the failure boundary has a controlled recovery story, run the backend comparison
   (`NDT_OMP` vs `SMALL_GICP` vs `SMALL_VGICP`) with identical bag/map/eval commands. While
   collecting these replays, confirm the diagnostics added in Phase 3 (`failure_category`)
   and the calibrated `error_floor` covariance also surface in the outputs.
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

### Root-cause analysis (2026-06-14, code-level; data run still pending)

Read the crop path end to end to narrow step 1 before spending an idle-machine run.
Findings, all from the shipped code and `param/boreas_ndt_velodyne.yaml`:

- **The crop is centered on the *predicted* seed, not the last accepted pose.** The
  primary attempt is `runAlignmentAttempt(init_guess, init_guess, ...)`
  (`lidar_localization_component.cpp:1969`), and `init_guess` is the twist/IMU
  prediction (`:1710-1753`). `setInputTargetForPose` crops `full_map_cloud_ptr_`
  around that center (`:1810`). Only the *recovery retry* re-centers on
  `last_accepted_pose_matrix_` (`:1980`). So a runaway prediction starves its own
  target cloud — a positive-feedback divergence, exactly the closed-loop shape
  Phase 3 warned about.
- **`local_map_crop_too_small` is a lagging symptom here, not a primary cause.** With
  `local_map_radius: 300` and `local_map_min_points: 100`, a 300 m disk around any
  on-map center holds far more than 100 points. For the count to fall under 100 the
  center must be roughly the map radius (~300 m) off the mapped area — i.e. the pose
  has *already* diverged. So the reported `local_map_crop_too_small` is downstream of
  whatever starts the divergence, not an independent map problem at the cliff instant.
- **The crop-failure guard is what freezes the run into a flat 45 m plateau.** After a
  streak of crop failures the component activates `crop_failure_guard`, resets the
  prediction to the last accepted pose, and then *drops every subsequent scan until a
  new initial pose arrives* (`:1503-1521`). With no auto-reinitialization wired in the
  Boreas config, the estimate is frozen at the last good pose while the vehicle keeps
  driving — RMSE then grows linearly to ~45 m. (G3, just landed, is the mechanism that
  could break this freeze automatically; Boreas is its natural second test scenario.)

This leaves two candidate triggers for the *initial* divergence, with opposite fixes:

  (A) **prediction divergence** — a twist-prediction glitch or a single bad accepted
      match throws the seed, and the predicted-center crop then guarantees the next
      reject; fix is estimator-side (crop around last-accepted instead of the raw
      seed, and/or harden the twist/reject path).
  (C) **map coverage** — the Boreas map (`*_rebuild_loc_frame.pcd`, a GT-aligned
      rebuild) does not cover the full 60 s route, so once the vehicle passes the
      mapped region the crop around the *true* pose legitimately falls below
      `min_points`; fix is map-side (extend / retile the map).

`scripts/diagnose_local_map_crop_coverage.py` decides (A) vs (C) offline, with no ROS
and no localizer: it counts map points within `local_map_radius` of every ground-truth
pose and reports the first time that count drops below `local_map_min_points`. If the
true trajectory stays covered, a real `local_map_crop_too_small` is prediction-driven
(A); if coverage collapses at the cliff time, it is a map-split (C). The counting core
is unit-tested in `test/test_local_map_crop_coverage.py`. **Blocked only on data**: the
Boreas map PCD and reference CSV live on the `...` mount, which is
not currently attached; run the diagnostic the moment it is back to fix the root cause
before any runtime change (no unreplayed estimator edits — the Phase 3 rule).

## Phase 3: Measurement Acceptance, Diagnostics, Covariance — DONE (2026-06-12)

Goal was to move acceptance and failure detection beyond the scalar fitness score, so
drift is caught before large pose error accumulates and downstream consumers can trust the
outputs. All three steps are complete; see the execution log for full evidence.

1. **Multi-criteria acceptance — closed as a negative result.** Two policies that won the
   offline fixture ranking (`correction_conditioned`, then the bounded redesign
   `bounded_degraded`) both diverged in closed-loop replay (62.7 m, then 6.80 m). On
   degenerate geometry, over-threshold registrations are systematically biased, not noisy,
   so accepting any of them feeds bias into the prediction anchor. The scalar gate +
   coast-on-rejection + existing recovery retry is the replay-validated winner; recovery
   inside the hard window belongs to the relocalization track (G2/G3), not acceptance. The
   falsified hypotheses are encoded as `experiments/measurement_acceptance` regression
   fixtures + a closed-loop sequence harness so the offline ranking now agrees with the
   replays.
2. **Diagnostics taxonomy — shipped.** `/alignment_status` publishes a `failure_category`
   key (`missing_map`, `missing_initial_pose`, `weak_overlap`, `bad_match`,
   `stale_prediction`, `overload`, `healthy`) plus `*_active` co-occurrence flags, in the
   pure `alignment_failure_taxonomy.hpp` policy. Documented in `troubleshooting.md`.
3. **Calibrated covariance — shipped, closes #72.** `/pcl_pose` defaults to an
   `error_floor` model calibrated against ground truth (per-axis 2σ coverage ≥ 0.96 on the
   calibration runs vs 0.42–0.59 1σ for the old heuristic). The biased tail is documented
   as out of scope, directing fusion consumers to also gate on `failure_category`. Recipe
   and limits in `pose_covariance.md`.

This makes the roadmap's "Next" diagnostics and covariance done-criteria hold.

## Global Localization Track (G1 -> G3)

Runs in parallel with Phases 1-3; detailed phases live in
[global_localization_roadmap.md](global_localization_roadmap.md).

- **G1 (artifact-first map-wide candidates) — DONE.** `MAP_GRID` baseline (2026-06-11,
  unit-tested, verified on the Istanbul map) and the admissible `BBS_2D` branch-and-bound
  engine (multi-resolution occupancy pyramid over x/y/yaw, with unit tests for bound
  admissibility and top-K ordering) are merged. `BBS_2D` solves a synthetic kidnapped
  start end-to-end (rank-1 -> NDT -> 0.69 m / 1.0 deg). The Koide 180 s failure window is
  *not* a candidate-generation problem — even the GT pose scores fitness 9.6 there — so it
  belongs to recovery/retry, not the engine. `FPFH_RANSAC` (full-3D fallback) is the only
  remaining G1 sub-item and is not started.
- **G2 (runtime query service) — DONE (2026-06-13).** `BBS_2D` was sped up ~8x with
  byte-identical output (per-(yaw,level) integer offset tables + adaptive FFT hit maps),
  removing the "~14 min/window" blocker. The opt-in `scripts/global_localization_node.py`
  answers `std_srvs/Trigger` on `~/query` with map-wide candidates on `~/candidates`; pure
  query logic in `global_localization_query.py` (rclpy-free, unit-tested). Validated
  end-to-end on the HDL bag: kidnapped start -> query (23 s, 16 candidates, top score
  0.998) -> top as `/initialpose` -> NDT tracks the full remaining bag (858 poses). Demo
  GIF in the README and `scripts/render_global_localization_demo_gif.py`.
- **G3 (guarded automatic reinitialization) — next.** Connect `/reinitialization_requested`
  to the G2 service behind the recovery supervisor, gated by the roadmap's relocalization
  runtime rules (candidate generation without oracle ordering, runtime-available scoring,
  guarded reset publication, post-reset recovery evidence). Prerequisite: faster queries
  for automation — either the ~9 s coarse-yaw setting or a C++ port of the BBS heap loop.

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

### 2026-06-12: BBS_2D 8.5x speedup with exact output equivalence

The G2 blocker ("~14 min per window in pure python") is addressed in
`make_bbs_relocalization_attempts.py` without changing a single output bit:
per-(yaw, level) integer offset tables (node coordinates are always multiples
of `2^level`, so the floor decomposes) plus adaptive FFT hit maps for hot
(yaw, level) pairs. Synthetic 1200x1200/72-yaw/top-64: 81.3 s -> 9.6 s; HDL
smoke window: 19.9 s -> 8.1 s end-to-end with byte-identical candidate CSVs
(verified against the pre-change code on the same machine and load). Details in
[global_localization_roadmap.md](global_localization_roadmap.md). Remaining
floor: the Python heap loop (~580 k pops) — revisit if G2 needs sub-second
queries.

### 2026-06-13: G2 runtime service + kidnapped-start demo

The G2 service node landed (`8a04314`) and the demo GIF was added to the README
(`595b96a`):

- `scripts/global_localization_node.py`: opt-in node, `std_srvs/Trigger` on `~/query`
  runs the optimized `BBS_2D` search on the latest scan from the localization cloud topic
  and publishes ranked candidates as a `geometry_msgs/PoseArray` on `~/candidates`
  (transient local). Never part of the default launch; nothing is published unless the
  service is called. Pure query logic in `scripts/global_localization_query.py` is
  rclpy-free and unit-tested (`test_global_localization_query.py`).
- End-to-end validation on the HDL sample bag with `set_initial_pose: false`: play ->
  pause player -> `~/query` answers in 23 s with 16 candidates (top score 0.998) -> top
  candidate published as `/initialpose` -> resume -> NDT tracks the full remaining bag
  (858 accepted poses around the loop). The pause is honest about the 23 s query latency.
- `scripts/render_global_localization_demo_gif.py` renders the recorded artifacts into the
  README GIF (kidnapped start -> BBS_2D candidates -> `/initialpose` -> tracking resumes).
- Shared-machine recording recipe (now in memory): isolate with a dedicated
  `ROS_DOMAIN_ID`, drive the run with `rosbag2_player` pause/resume to hide query latency.

G2 is done; G3 automation needs faster queries (coarse-yaw ~9 s or a C++ heap-loop port).

### 2026-06-13: Issue and branch triage

Triaged the twelve open issues into the table in "Issue Tracker State" above: five are
answered by the shipped v1.1 doc set and ready to close with pointers (#25/#33/#34/#37/#49),
two are small actionable code items (#55 unused `odom_frame_id_`, #54 missing lock on
`corrent_pose_with_cov_stamped_ptr_`), two need a reproduction before any claim (#52
restart-degradation, #68 MGRS/large-coordinate map), and the IMU-estimator pair
(#77/#36) plus the broader stability ask (#50) stay open as enhancement work. No open
issue is a regression in shipped behavior.

Branch audit (`git fetch --prune`): of ten non-`main` remote branches, five merged feature
branches are stale and fully in `main` (`codex/mid360-policy-split`,
`codex/public-validation-log`, `codex/release-validation-log`,
`feat/public-demo-validation-dashboard`, `fix/tf`). `feature/small_gicp` carries only a
stale 2024 README commit (not the backend integration its name implies). The distro
branches (`dashing`/`foxy`/`humble`/`jazzy`) are kept as user checkout points.

### 2026-06-14: #55 / #54 closed as non-bugs with documentation

Followed up on the two "small code wins" from the triage. Both turned out to be stale
or already-safe under the shipped configuration, so neither warranted a behavioral
change:

- **#55 (`odom_frame_id_` unused)** — stale. The field is read in
  `publishMapToOdomTransform` to look up `odom -> base` and broadcast the
  `map -> odom` TF whenever `enable_map_odom_tf_` is set. Nothing to remove.
- **#54 (`corrent_pose_with_cov_stamped_ptr_` unlocked)** — not a race. `main` spins the
  node on a `SingleThreadedExecutor` (`lidar_localization_node.cpp`) and no callback
  opts into a Reentrant group, so every subscription/timer/service callback runs in one
  mutually-exclusive group on one thread; the shared pose pointer is serialized by the
  executor, not a lock. The real risk is that this invariant was implicit, so a later
  switch to `MultiThreadedExecutor` (or a Reentrant group) would silently introduce a
  data race. Made the invariant explicit instead of adding a now-redundant mutex:
  comments at the executor construction site and the member declaration, plus a
  CHANGELOG note.

Comment/doc-only changes, no rebuild required.

**Phase 1 correction (found while reading this code):** the SMALL_GICP *and* SMALL_VGICP
backends are already fully wired, not a "start fresh" item as previously noted.
`registration_backend_policy.hpp` maps `SMALL_GICP`/`SMALL_VGICP`, and
`lidar_localization_component.cpp:998-1013` constructs `small_gicp::RegistrationPCL`,
selects GICP vs VGICP via `smallGicpRegistrationType`, and sets epsilon / correspondence
randomness / max-correspondence-distance / `vgicp_voxel_resolution_` / thread count
before assigning `registration_`. `CMakeLists.txt` already does
`find_package(small_gicp QUIET CONFIG)` and defines `LIDAR_LOCALIZATION_HAVE_SMALL_GICP`
+ links `small_gicp::small_gicp` when found; without the dependency the backend is a
clean `RCLCPP_ERROR` + exit. So Phase 1 is a **build-and-benchmark** task, not an
implementation one: install `small_gicp`, rebuild, then run NDT_OMP vs SMALL_GICP vs
SMALL_VGICP on an idle machine (still gated on `load < ~5`).

### 2026-06-14: G3 guarded automatic reinitialization (logic complete)

Built the consumer side of the global-localization recovery loop, which was the
named next item after G1/G2. The producer (`/reinitialization_requested` from the C++
recovery supervisor) already existed; G3 adds the part that closes the loop while
respecting the Phase 3 lesson that a closed loop must not be able to reset its own
bounds. Two new files:

- `scripts/reinitialization_supervisor_policy.py` — a ROS-free, deterministic state
  machine (`decide(params, state, obs)`) that owns every safety decision: a candidate
  must clear `min_candidate_score` to be published, resets are spaced by
  `min_seconds_between_attempts`, after a reset it requires alignment fitness back under
  `recovery_fitness_threshold` as recovery evidence, and the attempt counter is a true
  ceiling (`max_attempts`) that only clears on confirmed recovery or request release —
  never as a side effect of attempting. After a terminal outcome it waits for the request
  to de-assert (edge-triggered), so a stuck-high request line cannot re-fire it.
- `scripts/reinitialization_supervisor_node.py` — the thin ROS shell (opt-in, not in the
  default launch): subscribes `/reinitialization_requested` + `/alignment_status`
  (reads the `fitness_score` diagnostic), calls the G2 `~/query` Trigger service, and
  publishes `/initialpose` from the top candidate only when the policy says so.

`test/test_reinitialization_supervisor_policy.py` is the roadmap-required regression
test that fails on unsafe publication or false acceptance: 10 adversarial sequences
(transient blip, pure tracking, happy-path single reset + recovery, weak-candidate
never published, bounded queries, false-acceptance no-loop, reset spacing, budget reset
on recovery between episodes, exhausted latch, purity). All green. **Pending** (machine-
or runtime-bound, not logic): a live/replay smoke on the Koide kidnapped-start window
and capture of the post-reset recovery evidence for the roadmap's evidence gate.

## Suggested Order Of Work

Phase 0, Phase 3 (all three steps), G1, the BBS speedup, and G2 are complete. Remaining,
in priority order:

1. **Phase 1 Koide backend comparison** (`NDT_OMP` vs `SMALL_GICP` vs `SMALL_VGICP`) on an
   idle machine — needs `load < ~5`; this is the main blocker, since the shared machine is
   usually busy. Reuse the existing Koide manifests; confirm the new `failure_category` and
   `error_floor` covariance columns also appear in the replay outputs while there.
2. **G3 guarded automatic reinitialization** — wire `/reinitialization_requested` to the
   G2 service behind the recovery supervisor. Machine-independent design work can start
   now; the runtime path needs the faster query (coarse-yaw or C++ port) first.
3. **Phase 2 Boreas recovery** — root-cause the `local_map_crop_too_small` cliff;
   long-running sweeps interleave well with the above.
4. **Next release tag** once Phase 1 or G3 reaches a boundary, rolling up the current
   `Unreleased` batch.
