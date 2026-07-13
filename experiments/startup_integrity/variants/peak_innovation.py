from __future__ import annotations

from dataclasses import dataclass

from experiments.startup_integrity.interface import StartupIntegrityDecision
from experiments.startup_integrity.interface import StartupIntegritySample


@dataclass(frozen=True)
class Config:
    startup_updates: int = 5
    max_translation_m: float = 0.75
    max_yaw_deg: float = 8.0


class PeakInnovationMonitor:
    """Reject one large registration correction during startup."""

    name = "peak_innovation"
    design = "single-update translation or yaw correction threshold"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: StartupIntegritySample) -> StartupIntegrityDecision:
        active = sample.index < self.config.startup_updates
        trigger = active and (
            sample.correction_translation_m > self.config.max_translation_m
            or abs(sample.correction_yaw_deg) > self.config.max_yaw_deg
        )
        reason = "startup_peak_innovation" if trigger else "startup_peak_bounded"
        return StartupIntegrityDecision(trigger, reason)
