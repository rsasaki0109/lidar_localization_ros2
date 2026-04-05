from __future__ import annotations

from dataclasses import dataclass

from experiments.recovery_action.interface import RecoveryDecision
from experiments.recovery_action.interface import RecoverySample


@dataclass(frozen=True)
class Config:
    min_rejections: int = 1
    max_fitness: float = 10.0
    max_correction_translation_m: float = 2.0
    max_correction_yaw_deg: float = 2.0


class RejectedSeedReuseStrategy:
    name = "rejected_seed_reuse"
    design = "seed-reuse rule table"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: RecoverySample) -> RecoveryDecision:
        if sample.failure_kind != "rejected_measurement":
            return RecoveryDecision("advance_prediction_only", "not_rejected_measurement")
        if sample.consecutive_rejected_updates < self.config.min_rejections:
            return RecoveryDecision("advance_prediction_only", "insufficient_reject_streak")
        if sample.fitness_score > self.config.max_fitness:
            return RecoveryDecision("advance_prediction_only", "fitness_too_high")
        if sample.correction_translation_m > self.config.max_correction_translation_m:
            return RecoveryDecision("advance_prediction_only", "correction_translation_too_large")
        if sample.correction_yaw_deg > self.config.max_correction_yaw_deg:
            return RecoveryDecision("advance_prediction_only", "correction_yaw_too_large")
        return RecoveryDecision("reuse_rejected_seed", "within_seed_reuse_limits")
