"""Shared contracts for offline global-verifier experiments."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


def angle_error_rad(lhs: float, rhs: float) -> float:
    return abs(math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs)))


@dataclass(frozen=True)
class Candidate:
    attempt_id: str
    candidate_index: int
    x: float
    y: float
    yaw_rad: float
    oracle_recoverable: bool


@dataclass(frozen=True)
class GlobalEstimate:
    valid: bool
    x: float
    y: float
    yaw_rad: float
    final_inliers: int
    runtime_sec: float


@dataclass(frozen=True)
class Selection:
    candidate: Candidate | None
    accepted: bool
    reason: str
    translation_delta_m: float = math.inf
    yaw_delta_rad: float = math.inf


class CandidateSelector:
    name = "selector"

    def select(
        self, candidates: Iterable[Candidate], estimate: GlobalEstimate | None = None
    ) -> Selection:
        raise NotImplementedError
