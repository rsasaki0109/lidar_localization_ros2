from __future__ import annotations

from dataclasses import dataclass

from experiments.imu_guard.interface import GuardDecision
from experiments.imu_guard.interface import GuardSample


@dataclass(frozen=True)
class Config:
    translation_m: float = 1.5
    yaw_deg: float = 3.5
    required_streak: int = 2


class StreakGuard:
    name = "streak_guard"
    design = "stateful OOP streak detector"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()
        self._hazard_streak = 0

    def reset(self) -> None:
        self._hazard_streak = 0

    def step(self, sample: GuardSample) -> GuardDecision:
        if not sample.imu_prediction_active:
            self._hazard_streak = 0
            return GuardDecision(False, "imu_inactive")

        hazard = (
            sample.correction_translation_m >= self.config.translation_m
            or sample.correction_yaw_deg >= self.config.yaw_deg
        )
        self._hazard_streak = self._hazard_streak + 1 if hazard else 0
        if self._hazard_streak >= self.config.required_streak:
            return GuardDecision(True, "hazard_streak_reached", float(self._hazard_streak))
        return GuardDecision(False, "waiting_for_streak", float(self._hazard_streak))
