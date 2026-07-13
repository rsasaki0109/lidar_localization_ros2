from __future__ import annotations

from dataclasses import dataclass

from experiments.startup_integrity.interface import StartupIntegrityDecision
from experiments.startup_integrity.interface import StartupIntegritySample


@dataclass(frozen=True)
class Config:
    startup_updates: int = 5
    translation_scale_m: float = 0.4
    yaw_scale_deg: float = 4.0
    energy_budget: float = 4.0


class NormalizedInnovationEnergyMonitor:
    """Accumulate normalized squared translation and yaw corrections."""

    name = "normalized_innovation_energy"
    design = "startup normalized translation/yaw correction energy"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()
        self.reset()

    def reset(self) -> None:
        self.energy = 0.0

    def step(self, sample: StartupIntegritySample) -> StartupIntegrityDecision:
        if sample.index >= self.config.startup_updates:
            return StartupIntegrityDecision(False, "startup_window_complete")
        translation = sample.correction_translation_m / self.config.translation_scale_m
        yaw = abs(sample.correction_yaw_deg) / self.config.yaw_scale_deg
        self.energy += translation * translation + yaw * yaw
        trigger = self.energy > self.config.energy_budget
        reason = "startup_innovation_energy_exceeded" if trigger else "startup_innovation_energy_bounded"
        return StartupIntegrityDecision(trigger, reason, self.energy)
