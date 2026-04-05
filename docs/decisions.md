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
- generated_from: `/media/autoware/aa/ai_coding_ws/lidarloc_ws/repo/scripts/run_borderline_gate_experiments.py`

## IMU Correction Guard

- adopted variant for this problem: `absolute_threshold`
- design family: functional threshold rule
- rationale: highest combined score (`90.4`) under the shared fixture/evaluation contract
- benchmark score: `100.0`
- readability score: `58.0`
- extensibility score: `94.0`
- nearest alternative: `score_budget` at `70.68`
- generated_from: `/media/autoware/aa/ai_coding_ws/lidarloc_ws/repo/scripts/run_imu_guard_experiments.py`

## Recovery Action Selection

- adopted variant for this problem: `guarded_last_pose_retry`
- design family: guarded retry rule table
- rationale: highest combined score (`88.92`) under the shared fixture/evaluation contract
- benchmark score: `100.0`
- readability score: `50.6`
- extensibility score: `94.0`
- nearest alternative: `conservative_drop` at `65.32`
- generated_from: `/media/autoware/aa/ai_coding_ws/lidarloc_ws/repo/scripts/run_recovery_action_experiments.py`

## Reinitialization Trigger

- adopted variant for this problem: `gap_streak_score_reinit`
- design family: scorecard threshold
- rationale: highest combined score (`89.44`) under the shared fixture/evaluation contract
- benchmark score: `100.0`
- readability score: `53.2`
- extensibility score: `94.0`
- nearest alternative: `failure_kind_eager_reinit` at `78.08`
- generated_from: `/media/autoware/aa/ai_coding_ws/lidarloc_ws/repo/scripts/run_reinit_trigger_experiments.py`
