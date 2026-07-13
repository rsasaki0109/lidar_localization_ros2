"""BBS baseline and KISS global-estimate candidate selectors."""

from __future__ import annotations

import math
from typing import Iterable

from .interface import Candidate
from .interface import CandidateSelector
from .interface import GlobalEstimate
from .interface import Selection
from .interface import angle_error_rad


class BbsFirstSelector(CandidateSelector):
    name = "bbs_first"

    def select(
        self, candidates: Iterable[Candidate], estimate: GlobalEstimate | None = None
    ) -> Selection:
        ordered = sorted(candidates, key=lambda candidate: candidate.candidate_index)
        if not ordered:
            return Selection(None, False, "no_candidates")
        return Selection(ordered[0], True, "bbs_first_no_gate", 0.0, 0.0)


class KissNearestSelector(CandidateSelector):
    name = "kiss_nearest"

    def __init__(
        self,
        min_final_inliers: int = 5,
        max_translation_delta_m: float = 2.0,
        max_yaw_delta_rad: float = math.radians(15.0),
        yaw_weight_m_per_rad: float = 1.0,
    ) -> None:
        self.min_final_inliers = min_final_inliers
        self.max_translation_delta_m = max_translation_delta_m
        self.max_yaw_delta_rad = max_yaw_delta_rad
        self.yaw_weight_m_per_rad = yaw_weight_m_per_rad

    def select(
        self, candidates: Iterable[Candidate], estimate: GlobalEstimate | None = None
    ) -> Selection:
        ordered = list(candidates)
        if not ordered:
            return Selection(None, False, "no_candidates")
        if estimate is None or not estimate.valid:
            return Selection(None, False, "invalid_global_estimate")
        if estimate.final_inliers < self.min_final_inliers:
            return Selection(None, False, "insufficient_final_inliers")

        def deltas(candidate: Candidate) -> tuple[float, float, float]:
            translation = math.hypot(candidate.x - estimate.x, candidate.y - estimate.y)
            yaw = angle_error_rad(candidate.yaw_rad, estimate.yaw_rad)
            return translation + self.yaw_weight_m_per_rad * yaw, translation, yaw

        selected = min(ordered, key=lambda candidate: deltas(candidate)[0])
        _score, translation, yaw = deltas(selected)
        if translation > self.max_translation_delta_m:
            return Selection(
                selected, False, "translation_delta_above_threshold", translation, yaw
            )
        if yaw > self.max_yaw_delta_rad:
            return Selection(selected, False, "yaw_delta_above_threshold", translation, yaw)
        return Selection(selected, True, "passed_reset_disabled", translation, yaw)
