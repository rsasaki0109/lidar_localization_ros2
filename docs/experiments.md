# Experiments

## Operating Rules

- same interface inside each problem
- same fixture set inside each problem
- same benchmark pass/fail rubric inside each problem
- same readability and extensibility heuristics across problems

## Borderline Measurement Gate

Reject truly bad borderline measurements without regressing clean short-gap updates.

| Variant | Design | Benchmark | Readability | Extensibility | Overall |
|---|---|---:|---:|---:|---:|
| `seed_conditioned_borderline` | seed-aware rule table | 100.0 | 56.8 | 94.0 | 90.16 |
| `post_reject_strict` | state-escalation rule table | 80.0 | 56.8 | 94.0 | 78.16 |
| `fixed_threshold_55` | functional fixed threshold | 60.0 | 65.4 | 94.0 | 67.88 |

### Fixture Outcomes

#### `seed_conditioned_borderline`
- `clean_borderline_should_pass`: pass=`True` decision=`accept` reason=`within_seed_conditioned_thresholds`
- `clear_good`: pass=`True` decision=`accept` reason=`within_seed_conditioned_thresholds`
- `clear_over_threshold`: pass=`True` decision=`reject` reason=`over_effective_threshold`
- `drifted_borderline_bad`: pass=`True` decision=`reject` reason=`over_borderline_seed_gate`
- `post_reject_low_seed_should_pass`: pass=`True` decision=`accept` reason=`within_seed_conditioned_thresholds`

#### `post_reject_strict`
- `clean_borderline_should_pass`: pass=`True` decision=`accept` reason=`within_dynamic_thresholds`
- `clear_good`: pass=`True` decision=`accept` reason=`within_dynamic_thresholds`
- `clear_over_threshold`: pass=`True` decision=`reject` reason=`over_effective_threshold`
- `drifted_borderline_bad`: pass=`True` decision=`reject` reason=`over_post_reject_strict_threshold`
- `post_reject_low_seed_should_pass`: pass=`False` decision=`reject` reason=`over_post_reject_strict_threshold`

#### `fixed_threshold_55`
- `clean_borderline_should_pass`: pass=`False` decision=`reject` reason=`over_fixed_5_5`
- `clear_good`: pass=`True` decision=`accept` reason=`within_thresholds`
- `clear_over_threshold`: pass=`True` decision=`reject` reason=`over_effective_threshold`
- `drifted_borderline_bad`: pass=`True` decision=`reject` reason=`over_fixed_5_5`
- `post_reject_low_seed_should_pass`: pass=`False` decision=`reject` reason=`over_fixed_5_5`

## IMU Correction Guard

Disable the IMU-preintegration path early enough to avoid poisoning localization, without false positives on no-IMU traces.

| Variant | Design | Benchmark | Readability | Extensibility | Overall |
|---|---|---:|---:|---:|---:|
| `absolute_threshold` | functional threshold rule | 100.0 | 58.0 | 94.0 | 90.40 |
| `score_budget` | pipeline scorecard | 66.7 | 59.4 | 94.0 | 70.68 |
| `streak_guard` | stateful OOP streak detector | 66.7 | 52.0 | 84.0 | 67.20 |

### Fixture Outcomes

#### `absolute_threshold`
- `hdl_healthy_candidate_r01`: pass=`True` decision=`trigger_at_3` reason=`correction_yaw_guard`
- `hdl_unstable_candidate_r02`: pass=`True` decision=`trigger_at_3` reason=`correction_translation_guard`
- `istanbul_no_imu`: pass=`True` decision=`no_trigger` reason=`must_not_trigger`

#### `score_budget`
- `hdl_healthy_candidate_r01`: pass=`False` decision=`no_trigger` reason=`missing_trigger`
- `hdl_unstable_candidate_r02`: pass=`True` decision=`trigger_at_3` reason=`score_budget_exceeded`
- `istanbul_no_imu`: pass=`True` decision=`no_trigger` reason=`must_not_trigger`

#### `streak_guard`
- `hdl_healthy_candidate_r01`: pass=`False` decision=`no_trigger` reason=`missing_trigger`
- `hdl_unstable_candidate_r02`: pass=`True` decision=`trigger_at_4` reason=`hazard_streak_reached`
- `istanbul_no_imu`: pass=`True` decision=`no_trigger` reason=`must_not_trigger`

## Recovery Action Selection

Choose whether to keep open-loop prediction, reuse a rejected seed, or retry from the last accepted pose after a failed measurement.

| Variant | Design | Benchmark | Readability | Extensibility | Overall |
|---|---|---:|---:|---:|---:|
| `guarded_last_pose_retry` | guarded retry rule table | 100.0 | 50.6 | 94.0 | 88.92 |
| `conservative_drop` | minimal baseline | 50.0 | 91.6 | 85.0 | 65.32 |
| `rejected_seed_reuse` | seed-reuse rule table | 25.0 | 43.2 | 94.0 | 42.44 |

### Fixture Outcomes

#### `guarded_last_pose_retry`
- `long_gap_collapse_should_not_retry`: pass=`True` decision=`advance_prediction_only` reason=`accepted_gap_too_large`
- `seed_reuse_candidate_should_not_promote`: pass=`True` decision=`advance_prediction_only` reason=`insufficient_reject_streak`
- `short_gap_rejected_measurement_retryable`: pass=`True` decision=`retry_from_last_pose` reason=`within_retry_guard`
- `short_gap_target_unavailable_retryable`: pass=`True` decision=`retry_from_last_pose` reason=`within_retry_guard`

#### `conservative_drop`
- `long_gap_collapse_should_not_retry`: pass=`True` decision=`advance_prediction_only` reason=`no_recovery`
- `seed_reuse_candidate_should_not_promote`: pass=`True` decision=`advance_prediction_only` reason=`no_recovery`
- `short_gap_rejected_measurement_retryable`: pass=`False` decision=`advance_prediction_only` reason=`no_recovery`
- `short_gap_target_unavailable_retryable`: pass=`False` decision=`advance_prediction_only` reason=`no_recovery`

#### `rejected_seed_reuse`
- `long_gap_collapse_should_not_retry`: pass=`True` decision=`advance_prediction_only` reason=`not_rejected_measurement`
- `seed_reuse_candidate_should_not_promote`: pass=`False` decision=`reuse_rejected_seed` reason=`within_seed_reuse_limits`
- `short_gap_rejected_measurement_retryable`: pass=`False` decision=`reuse_rejected_seed` reason=`within_seed_reuse_limits`
- `short_gap_target_unavailable_retryable`: pass=`False` decision=`advance_prediction_only` reason=`not_rejected_measurement`

## Reinitialization Trigger

Decide when bounded local recovery should stop and the system should escalate to full reinitialization.

| Variant | Design | Benchmark | Readability | Extensibility | Overall |
|---|---|---:|---:|---:|---:|
| `gap_streak_score_reinit` | scorecard threshold | 100.0 | 53.2 | 94.0 | 89.44 |
| `failure_kind_eager_reinit` | event-driven rule table | 80.0 | 65.4 | 85.0 | 78.08 |
| `never_reinit` | minimal baseline | 60.0 | 91.6 | 85.0 | 71.32 |

### Fixture Outcomes

#### `gap_streak_score_reinit`
- `long_gap_crop_failure_should_reinit`: pass=`True` decision=`trigger_reinit` reason=`reinit_score_exceeded`
- `long_gap_hopeless_reject_should_reinit`: pass=`True` decision=`trigger_reinit` reason=`reinit_score_exceeded`
- `seed_reuse_candidate_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`reinit_score_safe`
- `short_gap_rejected_measurement_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`reinit_score_safe`
- `short_gap_target_unavailable_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`reinit_score_safe`

#### `failure_kind_eager_reinit`
- `long_gap_crop_failure_should_reinit`: pass=`True` decision=`trigger_reinit` reason=`target_unavailable`
- `long_gap_hopeless_reject_should_reinit`: pass=`True` decision=`trigger_reinit` reason=`fitness_exploded`
- `seed_reuse_candidate_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`continue_local_recovery`
- `short_gap_rejected_measurement_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`continue_local_recovery`
- `short_gap_target_unavailable_should_not_reinit`: pass=`False` decision=`trigger_reinit` reason=`target_unavailable`

#### `never_reinit`
- `long_gap_crop_failure_should_reinit`: pass=`False` decision=`stay_in_local_recovery` reason=`stay_in_local_recovery`
- `long_gap_hopeless_reject_should_reinit`: pass=`False` decision=`stay_in_local_recovery` reason=`stay_in_local_recovery`
- `seed_reuse_candidate_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`stay_in_local_recovery`
- `short_gap_rejected_measurement_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`stay_in_local_recovery`
- `short_gap_target_unavailable_should_not_reinit`: pass=`True` decision=`stay_in_local_recovery` reason=`stay_in_local_recovery`
