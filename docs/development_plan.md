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
  drift-tuning reconfirm are the latest recorded green boundaries.
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

## Phase 0: v1.1.0 Release Closeout

Goal: ship the accumulated reliability work as `v1.1.0` and clean up the issue tracker so
the public state of the project matches the actual state of `main`.

Steps:

1. Run `scripts/run_release_regression_suite.sh` on an idle machine and confirm
   `overall_pass=true`. Status: blocked on machine availability; build, unit tests, and
   experiment suite already pass; Istanbul RMSE passed even under load (1.39 m vs 6.0 m
   gate) while throughput gates did not.
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
  Next: evaluate it on the Koide 180 s window against the route-proximity artifact, then
  implement `BBS_2D` to cut the candidate count by orders of magnitude, then `FPFH_RANSAC`
  as the full-3D fallback.
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

## Suggested Order Of Work

1. Phase 0 (release + issue hygiene) — small, high leverage, mostly waiting on an idle
   machine for the regression run.
2. G1 evaluation on Koide + Phase 1 window validation — these share the same dataset and
   manifests, so they advance together.
3. Phase 2 Boreas root-cause, interleaved as long-running sweeps.
4. BBS_2D engine, then Phase 3 acceptance/diagnostics work feeding G2/G3 readiness.
