from __future__ import annotations

from dataclasses import dataclass

from experiments.measurement_acceptance.interface import AcceptanceDecision
from experiments.measurement_acceptance.interface import AcceptanceSample


@dataclass(frozen=True)
class Config:
    max_degraded_correction_m: float = 3.0
    max_degraded_yaw_deg: float = 8.0
    max_degraded_gap_sec: float = 2.5
    degraded_streak_budget: int = 8
    jump_correction_m: float = 10.0
    jump_max_gap_sec: float = 1.0


class CorrectionConditionedAcceptance:
    """Accept degraded scores while the result still agrees with prediction.

    The Koide outdoor_hard_01a failure window shows the motivating case: the
    ground-truth pose itself scores far over the fitness threshold in degraded
    map regions, while the registration correction stays around one meter. A
    small correction against a fresh prediction is independent evidence that
    the pose is still right; a fresh prediction contradicted by a huge jump is
    evidence against an otherwise good score.
    """

    name = "correction_conditioned"
    design = "fitness threshold + correction/staleness cross-check"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: AcceptanceSample) -> AcceptanceDecision:
        config = self.config
        fresh = sample.accepted_gap_sec <= config.jump_max_gap_sec
        if (
            sample.fitness_score <= sample.effective_score_threshold
            and fresh
            and sample.correction_translation_m > config.jump_correction_m
        ):
            return AcceptanceDecision(True, "fresh_prediction_contradicted", sample.fitness_score)
        if sample.fitness_score <= sample.effective_score_threshold:
            return AcceptanceDecision(False, "score_within_threshold", sample.fitness_score)
        if (
            sample.correction_translation_m <= config.max_degraded_correction_m
            and abs(sample.correction_yaw_deg) <= config.max_degraded_yaw_deg
            and sample.accepted_gap_sec <= config.max_degraded_gap_sec
            and sample.consecutive_rejected_updates < config.degraded_streak_budget
        ):
            return AcceptanceDecision(False, "degraded_score_consistent_pose", sample.fitness_score)
        return AcceptanceDecision(True, "score_over_threshold_unsupported", sample.fitness_score)
