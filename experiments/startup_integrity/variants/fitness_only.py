from __future__ import annotations

from dataclasses import dataclass

from experiments.startup_integrity.interface import StartupIntegrityDecision
from experiments.startup_integrity.interface import StartupIntegritySample


@dataclass(frozen=True)
class Config:
    score_threshold: float = 15.0


class FitnessOnlyMonitor:
    """Runtime baseline: trust the scalar NDT fitness threshold."""

    name = "fitness_only"
    design = "scalar NDT fitness threshold"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: StartupIntegritySample) -> StartupIntegrityDecision:
        trigger = sample.fitness_score > self.config.score_threshold
        reason = "fitness_over_threshold" if trigger else "fitness_within_threshold"
        return StartupIntegrityDecision(trigger, reason, sample.fitness_score)
