from __future__ import annotations

from dataclasses import dataclass

from experiments.recovery_action.interface import RecoveryDecision
from experiments.recovery_action.interface import RecoverySample


@dataclass(frozen=True)
class Config:
    min_rejections: int = 3
    max_accepted_gap_sec: float = 1.0
    max_seed_translation_m: float = 15.0


class GuardedLastPoseRetryStrategy:
    name = "guarded_last_pose_retry"
    design = "guarded retry rule table"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: RecoverySample) -> RecoveryDecision:
        if sample.failure_kind not in {"rejected_measurement", "target_unavailable", "not_converged"}:
            return RecoveryDecision("advance_prediction_only", "unsupported_failure_kind")
        if sample.consecutive_rejected_updates < self.config.min_rejections:
            return RecoveryDecision("advance_prediction_only", "insufficient_reject_streak")
        if sample.accepted_gap_sec > self.config.max_accepted_gap_sec:
            return RecoveryDecision("advance_prediction_only", "accepted_gap_too_large")
        if sample.seed_translation_since_accept_m > self.config.max_seed_translation_m:
            return RecoveryDecision("advance_prediction_only", "seed_translation_too_large")
        return RecoveryDecision("retry_from_last_pose", "within_retry_guard")
