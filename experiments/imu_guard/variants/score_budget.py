from __future__ import annotations

from dataclasses import dataclass

from experiments.imu_guard.interface import GuardDecision
from experiments.imu_guard.interface import GuardSample


@dataclass(frozen=True)
class Config:
    threshold: float = 0.90
    translation_scale_m: float = 2.0
    yaw_scale_deg: float = 4.0
    seed_translation_scale_m: float = 2.0
    fitness_scale: float = 1.0


class ScoreBudgetGuard:
    name = "score_budget"
    design = "pipeline scorecard"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: GuardSample) -> GuardDecision:
        if not sample.imu_prediction_active:
            return GuardDecision(False, "imu_inactive", 0.0)

        score = 0.0
        score += 0.50 * (sample.correction_translation_m / self.config.translation_scale_m)
        score += 0.30 * (sample.correction_yaw_deg / self.config.yaw_scale_deg)
        score += 0.15 * (sample.seed_translation_since_accept_m / self.config.seed_translation_scale_m)
        score += 0.05 * (sample.fitness_score / self.config.fitness_scale)

        if score >= self.config.threshold:
            return GuardDecision(True, "score_budget_exceeded", score)
        return GuardDecision(False, "score_budget_safe", score)
