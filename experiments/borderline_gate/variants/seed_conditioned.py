from __future__ import annotations

from dataclasses import dataclass

from experiments.borderline_gate.interface import GateDecision
from experiments.borderline_gate.interface import GateSample


@dataclass(frozen=True)
class Config:
    borderline_threshold: float = 5.25
    min_seed_translation_m: float = 1.0


class SeedConditionedBorderlineGate:
    name = "seed_conditioned_borderline"
    design = "seed-aware rule table"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: GateSample) -> GateDecision:
        if sample.fitness_score > sample.effective_score_threshold:
            return GateDecision(True, "over_effective_threshold")
        if (
            sample.fitness_score > self.config.borderline_threshold and
            sample.seed_translation_since_accept_m >= self.config.min_seed_translation_m
        ):
            return GateDecision(True, "over_borderline_seed_gate")
        return GateDecision(False, "within_seed_conditioned_thresholds")
