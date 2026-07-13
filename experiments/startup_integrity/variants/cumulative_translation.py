from __future__ import annotations

from dataclasses import dataclass

from experiments.startup_integrity.interface import StartupIntegrityDecision
from experiments.startup_integrity.interface import StartupIntegritySample


@dataclass(frozen=True)
class Config:
    startup_updates: int = 5
    translation_budget_m: float = 1.1


class CumulativeTranslationMonitor:
    """Bound total translation corrections before the predictor stabilizes."""

    name = "cumulative_translation"
    design = "startup cumulative translation-correction budget"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()
        self.reset()

    def reset(self) -> None:
        self.cumulative_translation_m = 0.0

    def step(self, sample: StartupIntegritySample) -> StartupIntegrityDecision:
        if sample.index >= self.config.startup_updates:
            return StartupIntegrityDecision(False, "startup_window_complete")
        self.cumulative_translation_m += sample.correction_translation_m
        trigger = self.cumulative_translation_m > self.config.translation_budget_m
        reason = "startup_translation_budget_exceeded" if trigger else "startup_translation_bounded"
        return StartupIntegrityDecision(trigger, reason, self.cumulative_translation_m)
