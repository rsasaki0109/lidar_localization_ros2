from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
from typing import Protocol


@dataclass(frozen=True)
class StartupIntegritySample:
    index: int
    fitness_score: float
    correction_translation_m: float
    correction_yaw_deg: float


@dataclass(frozen=True)
class StartupIntegrityDecision:
    request_reinitialization: bool
    reason: str
    score: Optional[float] = None


class StartupIntegrityStrategy(Protocol):
    name: str
    design: str

    def reset(self) -> None:
        ...

    def step(self, sample: StartupIntegritySample) -> StartupIntegrityDecision:
        ...
