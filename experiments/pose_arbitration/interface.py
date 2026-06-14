"""Interface for the reference pose-arbitration consumer (issue #72 follow-up).

`docs/pose_covariance.md` says the calibrated `error_floor` covariance is
"usable for 2-sigma-level comparisons" by fusion / arbitration logic, but adds a
hard caveat: *never trust covariance alone in degenerate geometry* -- a pose can
be accepted with multi-metre error at *low* fitness, and in `error_floor` mode a
low fitness yields a *small* covariance, so a covariance-only consumer is
actively misled into high confidence exactly when it is most wrong. The doc tells
consumers to also watch `/alignment_status` (`failure_category`,
`consecutive_rejected_updates`, `reinitialization_requested`) -- but ships no
reference implementation of a consumer that does so.

This module is that reference consumer, built ROS-free in the Phase 3 fixture +
policy + regression-test style so the claim "covariance is consumable by
arbitration" is backed by a runnable, tested artifact instead of prose.

Each consumer maps a published pose plus its alignment diagnostics to a trust
verdict (TRUST / DOWNWEIGHT / REJECT) and a fusion weight. The fixtures carry the
ground-truth `abs_xy_error_m` so the demo can score whether a verdict was
*correct* (trusting a pose whose true error exceeds the trust radius is the
failure we are guarding against).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

# Failure categories, mirroring include/lidar_localization/alignment_failure_taxonomy.hpp
# so fixtures can be copied from recorded /alignment_status verbatim.
CATEGORY_HEALTHY = "healthy"
CATEGORY_MISSING_MAP = "missing_map"
CATEGORY_MISSING_INITIAL_POSE = "missing_initial_pose"
CATEGORY_WEAK_OVERLAP = "weak_overlap"
CATEGORY_BAD_MATCH = "bad_match"
CATEGORY_STALE_PREDICTION = "stale_prediction"
CATEGORY_OVERLOAD = "overload"

# Trust verdicts.
TRUST = "trust"
DOWNWEIGHT = "downweight"
REJECT = "reject"


@dataclass(frozen=True)
class PoseSample:
    """One published `/pcl_pose` as a downstream fusion consumer sees it.

    The covariance fields are the *published* standard deviations (the square
    roots of the diagonal terms), i.e. what `error_floor` mode emitted -- a
    consumer never sees the true error, only this covariance and the diagnostics.
    ``abs_xy_error_m`` is ground-truth-only and is used solely to *score* a
    verdict in tests / the demo; a real consumer must not have it.
    """

    index: int
    cov_xy_std_m: float
    cov_yaw_std_deg: float
    failure_category: str
    consecutive_rejected_updates: int
    reinitialization_requested: bool
    # Ground-truth label for scoring only (not an input to any consumer).
    abs_xy_error_m: float


@dataclass(frozen=True)
class ArbitrationVerdict:
    trust: str          # TRUST / DOWNWEIGHT / REJECT
    weight: float       # fusion weight in [0, 1]
    reason: str


class PoseConsumer(Protocol):
    name: str
    design: str

    def decide(self, sample: PoseSample) -> ArbitrationVerdict:
        ...
