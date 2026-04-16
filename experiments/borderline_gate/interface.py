from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
from typing import Protocol


@dataclass(frozen=True)
class GateSample:
    index: int
    fitness_score: float
    effective_score_threshold: float
    seed_translation_since_accept_m: float
    consecutive_rejected_updates: int


@dataclass(frozen=True)
class GateDecision:
    reject_measurement: bool
    reason: str
    score: Optional[float] = None


class GateStrategy(Protocol):
    name: str
    design: str

    def reset(self) -> None:
        ...

    def step(self, sample: GateSample) -> GateDecision:
        ...
