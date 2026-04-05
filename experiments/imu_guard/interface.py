from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
from typing import Protocol


@dataclass(frozen=True)
class GuardSample:
    index: int
    status: str
    fitness_score: float
    alignment_time_sec: float
    imu_prediction_active: bool
    consecutive_rejected_updates: int
    seed_translation_since_accept_m: float
    seed_yaw_since_accept_deg: float
    accepted_gap_sec: float
    correction_translation_m: float
    correction_yaw_deg: float


@dataclass(frozen=True)
class GuardDecision:
    disable_imu_preintegration: bool
    reason: str
    score: Optional[float] = None


class GuardStrategy(Protocol):
    name: str
    design: str

    def reset(self) -> None:
        ...

    def step(self, sample: GuardSample) -> GuardDecision:
        ...
