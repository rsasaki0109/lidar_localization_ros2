from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class BiasFactor:
    """One scalar rotation factor distilled from a 3-D preintegration interval."""

    measured_delta_rad: float
    integration_bias_rad_s: float
    current_bias_rad_s: float
    duration_sec: float
    expected_delta_rad: float


class BiasCorrectionStrategy(Protocol):
    name: str
    design: str

    def correct(self, factors: list[BiasFactor]) -> list[float]:
        ...
