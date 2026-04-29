from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
from typing import Protocol


@dataclass(frozen=True)
class ReinitSample:
    index: int
    failure_kind: str
    fitness_score: float
    seed_translation_since_accept_m: float
    accepted_gap_sec: float
    consecutive_rejected_updates: int


@dataclass(frozen=True)
class ReinitDecision:
    trigger_reinitialization: bool
    reason: str
    score: Optional[float] = None


class ReinitStrategy(Protocol):
    name: str
    design: str

    def reset(self) -> None:
        ...

    def step(self, sample: ReinitSample) -> ReinitDecision:
        ...
