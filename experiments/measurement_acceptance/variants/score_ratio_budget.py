from __future__ import annotations

from dataclasses import dataclass

from experiments.measurement_acceptance.interface import AcceptanceDecision
from experiments.measurement_acceptance.interface import AcceptanceSample


@dataclass(frozen=True)
class Config:
    score_ratio_cap: float = 5.0
    max_degraded_gap_sec: float = 2.5
    degraded_streak_budget: int = 10


class ScoreRatioBudgetAcceptance:
    """Allow a bounded relative score overshoot while prediction stays fresh.

    Simpler comparator for the correction-conditioned variant: no correction
    cross-check, just a relative score cap with gap and streak budgets.
    """

    name = "score_ratio_budget"
    design = "relative score cap with gap/streak budget"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: AcceptanceSample) -> AcceptanceDecision:
        config = self.config
        if sample.fitness_score <= sample.effective_score_threshold:
            return AcceptanceDecision(False, "score_within_threshold", sample.fitness_score)
        if (
            sample.fitness_score <= sample.effective_score_threshold * config.score_ratio_cap
            and sample.accepted_gap_sec <= config.max_degraded_gap_sec
            and sample.consecutive_rejected_updates < config.degraded_streak_budget
        ):
            return AcceptanceDecision(False, "score_within_ratio_budget", sample.fitness_score)
        return AcceptanceDecision(True, "score_over_ratio_budget", sample.fitness_score)
