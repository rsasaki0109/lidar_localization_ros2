from __future__ import annotations

from dataclasses import dataclass

from experiments.measurement_acceptance.interface import AcceptanceDecision
from experiments.measurement_acceptance.interface import AcceptanceSample


@dataclass(frozen=True)
class Config:
    max_degraded_fitness: float = 12.0
    max_degraded_correction_m: float = 3.0
    max_degraded_yaw_deg: float = 8.0
    max_degraded_gap_sec: float = 2.5
    degraded_accept_budget: int = 3
    max_cumulative_degraded_correction_m: float = 5.0
    budget_reset_max_correction_m: float = 1.0


class BoundedDegradedAcceptance:
    """Correction-conditioned acceptance with a non-resetting degraded budget.

    The 2026-06-12 closed-loop replay showed why the first correction-
    conditioned variant fails end-to-end: every degraded acceptance re-anchors
    the prediction, so a drifting pose keeps confirming itself, and a budget
    that counts rejections resets on every acceptance. This variant counts
    consecutive *degraded acceptances* and the correction they absorb; only a
    genuinely below-threshold measurement resets either budget.
    """

    name = "bounded_degraded"
    design = "correction cross-check with non-resetting degraded-accept budget"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()
        self.reset()

    def reset(self) -> None:
        self.degraded_accept_streak = 0
        self.cumulative_degraded_correction_m = 0.0

    def step(self, sample: AcceptanceSample) -> AcceptanceDecision:
        config = self.config
        if sample.fitness_score <= sample.effective_score_threshold:
            # The 2026-06-12 bounded replay showed a below-threshold "ok" with a
            # 2.1 m jump refilling the budget mid-transient. Only a clean accept
            # whose correction is also small earns a budget reset.
            if sample.correction_translation_m <= config.budget_reset_max_correction_m:
                self.degraded_accept_streak = 0
                self.cumulative_degraded_correction_m = 0.0
            return AcceptanceDecision(False, "score_within_threshold", sample.fitness_score)
        if (
            sample.fitness_score <= config.max_degraded_fitness
            and sample.correction_translation_m <= config.max_degraded_correction_m
            and abs(sample.correction_yaw_deg) <= config.max_degraded_yaw_deg
            and sample.accepted_gap_sec <= config.max_degraded_gap_sec
            and self.degraded_accept_streak < config.degraded_accept_budget
            and self.cumulative_degraded_correction_m + sample.correction_translation_m
            <= config.max_cumulative_degraded_correction_m
        ):
            self.degraded_accept_streak += 1
            self.cumulative_degraded_correction_m += sample.correction_translation_m
            return AcceptanceDecision(False, "degraded_accept_within_budget", sample.fitness_score)
        return AcceptanceDecision(True, "degraded_budget_exhausted_or_unsupported", sample.fitness_score)
