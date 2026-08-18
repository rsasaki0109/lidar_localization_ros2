from __future__ import annotations

from dataclasses import dataclass
from typing import List
from typing import Optional
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class StructureGateCandidate:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float


@dataclass(frozen=True)
class StructureGateResult:
    similarities: List[float]
    accepted: List[bool]
    reason: str


@dataclass(frozen=True)
class StructureGateInput:
    scan_xyz: np.ndarray
    candidates: List[StructureGateCandidate]
    map_points: Optional[np.ndarray] = None
    map_path: Optional[str] = None
    local_radius_m: float = 60.0


class StructureGate(Protocol):
    name: str
    design: str

    def apply(self, gate_input: StructureGateInput) -> StructureGateResult:
        ...
