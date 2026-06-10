from __future__ import annotations

from experiments.measurement_acceptance.interface import AcceptanceDecision
from experiments.measurement_acceptance.interface import AcceptanceSample


class FixedThresholdAcceptance:
    """Current runtime behavior: scalar fitness against the effective threshold."""

    name = "fixed_threshold"
    design = "scalar fitness threshold (runtime baseline)"

    def reset(self) -> None:
        return None

    def step(self, sample: AcceptanceSample) -> AcceptanceDecision:
        if sample.fitness_score > sample.effective_score_threshold:
            return AcceptanceDecision(True, "score_over_threshold", sample.fitness_score)
        return AcceptanceDecision(False, "score_within_threshold", sample.fitness_score)
