from __future__ import annotations

from dataclasses import dataclass

from experiments.borderline_gate.interface import GateDecision
from experiments.borderline_gate.interface import GateSample


@dataclass(frozen=True)
class Config:
    threshold: float = 5.5


class FixedThreshold55Gate:
    name = "fixed_threshold_55"
    design = "functional fixed threshold"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: GateSample) -> GateDecision:
        if sample.fitness_score > sample.effective_score_threshold:
            return GateDecision(True, "over_effective_threshold")
        if sample.fitness_score > self.config.threshold:
            return GateDecision(True, "over_fixed_5_5")
        return GateDecision(False, "within_thresholds")
