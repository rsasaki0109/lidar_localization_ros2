# Decisions

## Process Rule

- new behavior work lands as multiple comparable variants first
- abstraction is only allowed after two or more variants share the same irreducible boundary
- `docs/interfaces.md`, `docs/experiments.md`, and `docs/decisions.md` are regenerated from experiment results, not handwritten

## Borderline Measurement Gate

- adopted variant for this problem: `seed_conditioned_borderline`
- design family: seed-aware rule table
- rationale: highest combined score (`90.16`) under the shared fixture/evaluation contract
- benchmark score: `100.0`
- readability score: `56.8`
- extensibility score: `94.0`
- nearest alternative: `post_reject_strict` at `78.16`
- generated_from: `scripts/run_borderline_gate_experiments.py`

## IMU Correction Guard

- adopted variant for this problem: `absolute_threshold`
- design family: functional threshold rule
- rationale: highest combined score (`90.4`) under the shared fixture/evaluation contract
- benchmark score: `100.0`
- readability score: `58.0`
- extensibility score: `94.0`
- nearest alternative: `score_budget` at `70.68`
- generated_from: `scripts/run_imu_guard_experiments.py`

## Multi-Criteria Measurement Acceptance

- adopted variant for this problem: `fixed_threshold`
- design family: scalar fitness threshold (runtime baseline)
- rationale: highest combined score (`85.3`) under the shared fixture/evaluation contract
- benchmark score: `87.5`
- readability score: `79.0`
- extensibility score: `85.0`
- nearest alternative: `bounded_degraded` at `73.82`
- generated_from: `scripts/run_measurement_acceptance_experiments.py`

## Recovery Action Selection

- adopted variant for this problem: `guarded_last_pose_retry`
- design family: guarded retry rule table
- rationale: highest combined score (`88.92`) under the shared fixture/evaluation contract
- benchmark score: `100.0`
- readability score: `50.6`
- extensibility score: `94.0`
- nearest alternative: `conservative_drop` at `65.32`
- generated_from: `scripts/run_recovery_action_experiments.py`

## Reinitialization Trigger

- adopted variant for this problem: `gap_streak_score_reinit`
- design family: scorecard threshold
- rationale: highest combined score (`89.44`) under the shared fixture/evaluation contract
- benchmark score: `100.0`
- readability score: `53.2`
- extensibility score: `94.0`
- nearest alternative: `failure_kind_eager_reinit` at `78.08`
- generated_from: `scripts/run_reinit_trigger_experiments.py`

## Startup False-Convergence Integrity Monitor

- runtime promotion: `none`
- leading comparator: `peak_innovation` at `74.82`
- reason: No candidate passes every repeated closed-loop fixture: correction-budget variants false-trigger on indoor_easy_02_live_r02 and miss indoor_kidnap_01_live_r02, while peak and fitness variants miss kidnaps.
- generated_from: `scripts/run_startup_integrity_experiments.py`

## Go2 Twist Reception and Per-Scan Prediction (2026-09-25)

Adopt `631bb51`: receive twist in a dedicated mutually exclusive callback group,
using the existing state lock, and retain one immutable twist observation for
each admitted scan. Registration can release the lock without blocking velocity
reception or changing the observation between seed prediction and rejected-scan
advancement. No parameters, prediction arithmetic, or acceptance thresholds change.

The callback-only alternative regressed the normal Box output gap from 0.60 to
3.10 seconds. Its traces showed different seed/advance velocities in 56/56 rejected
scans. The per-scan snapshot removes that inconsistency; an actual-method fixture
also verifies that an intervening velocity update affects only the next scan.

The trace-free candidate passed the five indoor leave-one-out Go2 runs, Box and
Mask2 ten times each, and twelve synthetic-fault runs. All normal scans were
processed; normal output gaps and coverage matched the baseline. Required Release
checks passed: 65 CTest tests, 232 Python tests with 66 subtests, and two bringup
help checks. These experiments use local data, GT initialization/map alignment,
IMU/EKF disabled, and fixed inputs/settings; they are not an open-benchmark claim.

This is a concurrency correctness fix with observed throughput benefit, not a
solution to overload or slipping. With the node restricted to one CPU and replay
at four times real time, a trace-disabled paired run improved input processing
from 37.9% to 78.9% and output coverage from 7.5% to 68.2%, but maximum error was
still 2.66 m and the maximum output gap worsened from 9.80 to 10.10 seconds.
Both arms failed the absolute accuracy target. Biased-twist Box faults retain a
3.10-second gap. Historical repeat-failure causality is unproven; single stress
runs do not establish statistical superiority. Recovery improvements remain open.

Local evidence lives under `/media/sasaki/aiueo2/jeplo_data/experiments/`:
`go2_scan_twist_production`, `go2_scan_twist_production_repeats`,
`go2_scan_twist_production_faults`, and `go2_scan_twist_production_stress` contain
source/runtime/input hashes, metrics, and completion records. Preserve the
isolated candidate and baseline runtimes. Main-workspace rebuild and replay are
recorded separately in `go2_scan_twist_adoption`.
