# Interfaces

## Stable Boundary

- `repo/src/` and `repo/include/` remain the runtime core.
- `repo/experiments/` is discardable space for competing implementations.
- promotion rule: only the winning behavior graduates into core, never the whole experiment scaffold.

## Borderline Measurement Gate

Reject truly bad borderline measurements without regressing clean short-gap updates.

### Minimal Interface

- `reset() -> None`
- `step(sample: GateSample) -> GateDecision`

### `GateSample` fields

- `index`
- `fitness_score`
- `effective_score_threshold`
- `seed_translation_since_accept_m`
- `consecutive_rejected_updates`

### `GateDecision` fields

- `reject_measurement`
- `reason`
- `score`

### Shared Fixtures

- `clean_borderline_should_pass`: Distilled 30 s style case. Borderline fitness with a clean seed should still pass; a global 5.5 threshold is too strict here.
- `clear_good`: Real Istanbul accepted sample. Even with a large seed drift, a clearly good measurement should pass.
- `clear_over_threshold`: Real Istanbul sample with fitness clearly over the effective threshold. Every sane gate should reject it.
- `drifted_borderline_bad`: Real Istanbul borderline sample. Fitness is below 6.0 but the seed has already drifted far from the last accepted pose, so the measurement should be rejected.
- `post_reject_low_seed_should_pass`: Distilled short-gap replay case. Recent rejects alone are not enough to justify strict gating when the seed is still close to the last accepted pose.

### Candidate Families

- `seed_conditioned_borderline`: seed-aware rule table
- `post_reject_strict`: state-escalation rule table
- `fixed_threshold_55`: functional fixed threshold


## IMU Correction Guard

Disable the IMU-preintegration path early enough to avoid poisoning localization, without false positives on no-IMU traces.

### Minimal Interface

- `reset() -> None`
- `step(sample: GuardSample) -> GuardDecision`

### `GuardSample` fields

- `index`
- `status`
- `fitness_score`
- `alignment_time_sec`
- `imu_prediction_active`
- `consecutive_rejected_updates`
- `seed_translation_since_accept_m`
- `seed_yaw_since_accept_deg`
- `accepted_gap_sec`
- `correction_translation_m`
- `correction_yaw_deg`

### `GuardDecision` fields

- `disable_imu_preintegration`
- `reason`
- `score`

### Shared Fixtures

- `hdl_healthy_candidate_r01`: Healthy HDL IMU-enabled trace excerpt. The guard may cut IMU around the observed divergence event, but it must not fire before sample 3.
- `hdl_unstable_candidate_r02`: Unstable HDL IMU-enabled trace excerpt. The guard should cut IMU before the first over-threshold reject at sample 5.
- `istanbul_no_imu`: Autoware Istanbul excerpt with no IMU activity. Any IMU guard trigger here is a false positive.

### Candidate Families

- `absolute_threshold`: functional threshold rule
- `score_budget`: pipeline scorecard
- `streak_guard`: stateful OOP streak detector


## Recovery Action Selection

Choose whether to keep open-loop prediction, reuse a rejected seed, or retry from the last accepted pose after a failed measurement.

### Minimal Interface

- `reset() -> None`
- `step(sample: RecoverySample) -> RecoveryDecision`

### `RecoverySample` fields

- `index`
- `failure_kind`
- `fitness_score`
- `correction_translation_m`
- `correction_yaw_deg`
- `seed_translation_since_accept_m`
- `accepted_gap_sec`
- `consecutive_rejected_updates`

### `RecoveryDecision` fields

- `action`
- `reason`
- `score`

### Shared Fixtures

- `long_gap_collapse_should_not_retry`: Real 180 s collapse window. Once the open-loop gap and drift have exploded, retrying from the last accepted pose should be suppressed.
- `seed_reuse_candidate_should_not_promote`: Real 60 s borderline reject that satisfied the experimental seed-reuse rule, but later benchmarking showed seed-only reuse should not be the chosen action here.
- `short_gap_rejected_measurement_retryable`: Distilled from the 60 s retry-success window. After a short reject streak with a bounded open-loop gap, retrying from the last accepted pose is the desired action.
- `short_gap_target_unavailable_retryable`: Distilled short-gap local-map-crop failure. The predicted seed has drifted, but the last accepted pose is still recent enough that a retry is preferable to blind open-loop continuation.

### Candidate Families

- `guarded_last_pose_retry`: guarded retry rule table
- `conservative_drop`: minimal baseline
- `rejected_seed_reuse`: seed-reuse rule table


## Reinitialization Trigger

Decide when bounded local recovery should stop and the system should escalate to full reinitialization.

### Minimal Interface

- `reset() -> None`
- `step(sample: ReinitSample) -> ReinitDecision`

### `ReinitSample` fields

- `index`
- `failure_kind`
- `fitness_score`
- `seed_translation_since_accept_m`
- `accepted_gap_sec`
- `consecutive_rejected_updates`

### `ReinitDecision` fields

- `trigger_reinitialization`
- `reason`
- `score`

### Shared Fixtures

- `long_gap_crop_failure_should_reinit`: Real 180 s crop-collapse onset. At this point local recovery has already failed and reinitialization should be requested.
- `long_gap_hopeless_reject_should_reinit`: Real 180 s run just before crop failure. The open-loop gap and drift have already exploded, so the system should request reinitialization even before the crop fully fails.
- `seed_reuse_candidate_should_not_reinit`: Real 60 s borderline reject that later benchmarking marked as a bad seed-reuse direction, but still not a case for full reinitialization.
- `short_gap_rejected_measurement_should_not_reinit`: Real 60 s retry-success window before recovery. This is still inside bounded local recovery and should not escalate to full reinitialization.
- `short_gap_target_unavailable_should_not_reinit`: Distilled short-gap crop failure. When the last accepted pose is still recent, the system should retry locally before requesting reinitialization.

### Candidate Families

- `gap_streak_score_reinit`: scorecard threshold
- `failure_kind_eager_reinit`: event-driven rule table
- `never_reinit`: minimal baseline
