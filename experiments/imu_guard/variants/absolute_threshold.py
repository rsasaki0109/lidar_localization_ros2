from __future__ import annotations

from dataclasses import dataclass

from experiments.imu_guard.interface import GuardDecision
from experiments.imu_guard.interface import GuardSample


@dataclass(frozen=True)
class Config:
    translation_m: float = 2.0
    yaw_deg: float = 4.0


class AbsoluteThresholdGuard:
    name = "absolute_threshold"
    design = "functional threshold rule"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: GuardSample) -> GuardDecision:
        if not sample.imu_prediction_active:
            return GuardDecision(False, "imu_inactive")
        if sample.correction_translation_m >= self.config.translation_m:
            return GuardDecision(True, "correction_translation_guard")
        if sample.correction_yaw_deg >= self.config.yaw_deg:
            return GuardDecision(True, "correction_yaw_guard")
        return GuardDecision(False, "within_absolute_thresholds")
