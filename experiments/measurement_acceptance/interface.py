from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
from typing import Protocol


@dataclass(frozen=True)
class AcceptanceSample:
    """One registration result, as seen by the runtime acceptance decision.

    Fields mirror /alignment_status values so fixtures can be copied from
    recorded runs verbatim.
    """

    index: int
    fitness_score: float
    effective_score_threshold: float
    correction_translation_m: float
    correction_yaw_deg: float
    accepted_gap_sec: float
    consecutive_rejected_updates: int


@dataclass(frozen=True)
class AcceptanceDecision:
    reject_measurement: bool
    reason: str
    score: Optional[float] = None


class AcceptanceStrategy(Protocol):
    name: str
    design: str

    def reset(self) -> None:
        ...

    def step(self, sample: AcceptanceSample) -> AcceptanceDecision:
        ...
