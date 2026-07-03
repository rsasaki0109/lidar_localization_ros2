"""Pure policy for ranking G2 candidates by NDT registration fitness.

BBS occupancy score does not separate perceptually aliased poses from the true
location (see docs/g3_live_closed_loop.md). When NDT fitness is available, the
supervisor-facing score and candidate order should follow registration quality
(lower fitness is better).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class RankedG2Candidate:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    bbs_score: float
    registration_fitness: Optional[float]
    score: float
    hit_count: int = 0
    point_count: int = 0
    registration_converged: Optional[bool] = None


def registration_fitness_to_supervisor_score(
    fitness: Optional[float],
    score_gate: float = 6.0,
) -> float:
    """Map NDT fitness (lower is better) to a [0, 1] supervisor score."""
    if fitness is None or not math.isfinite(fitness) or score_gate <= 0.0:
        return 0.0
    if fitness <= 0.0:
        return 1.0
    return max(0.0, min(1.0, 1.0 - fitness / score_gate))


def _fitness_sort_key(fitness: Optional[float]) -> float:
    if fitness is None or not math.isfinite(fitness):
        return float("inf")
    return fitness


def rank_candidates_by_registration(
    candidates: Sequence[RankedG2Candidate],
) -> List[RankedG2Candidate]:
    """Sort by registration fitness ascending, then BBS score descending."""
    return sorted(
        candidates,
        key=lambda candidate: (
            _fitness_sort_key(candidate.registration_fitness),
            -candidate.bbs_score,
        ),
    )


def candidate_pose_from_registration(
    candidate: RankedG2Candidate,
    *,
    converged: bool,
    fitness: Optional[float],
    refined_x: float,
    refined_y: float,
    refined_z: float,
    refined_yaw: float,
) -> Tuple[float, float, float, float]:
    """Return supervisor-facing pose; use NDT refinement only when converged."""
    if converged and fitness is not None and math.isfinite(fitness):
        return refined_x, refined_y, refined_z, refined_yaw
    return candidate.x_m, candidate.y_m, candidate.z_m, candidate.yaw_rad


def apply_registration_ranking(
    candidates: Sequence[RankedG2Candidate],
    score_gate: float = 6.0,
) -> List[RankedG2Candidate]:
    """Recompute supervisor scores from fitness and return ranked candidates."""
    updated = []
    for candidate in candidates:
        fitness = candidate.registration_fitness
        updated.append(
            RankedG2Candidate(
                x_m=candidate.x_m,
                y_m=candidate.y_m,
                z_m=candidate.z_m,
                yaw_rad=candidate.yaw_rad,
                bbs_score=candidate.bbs_score,
                registration_fitness=fitness,
                score=registration_fitness_to_supervisor_score(fitness, score_gate),
                hit_count=candidate.hit_count,
                point_count=candidate.point_count,
                registration_converged=candidate.registration_converged,
            ))
    return rank_candidates_by_registration(updated)


def bbs_only_candidates(
    raw_candidates: Iterable[Tuple[float, float, float, float, float, int, int]],
) -> List[RankedG2Candidate]:
    """Wrap BBS-only candidates where score == bbs_score."""
    wrapped = []
    for x_m, y_m, z_m, yaw_rad, bbs_score, hit_count, point_count in raw_candidates:
        wrapped.append(
            RankedG2Candidate(
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
                yaw_rad=yaw_rad,
                bbs_score=bbs_score,
                registration_fitness=None,
                score=bbs_score,
                hit_count=hit_count,
                point_count=point_count,
                registration_converged=None,
            ))
    return list(wrapped)
