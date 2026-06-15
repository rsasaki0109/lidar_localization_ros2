"""Reference pose-arbitration consumers for the calibrated covariance.

Two consumers are defined so the comparison is explicit:

* ``CovarianceOnlyConsumer`` -- the naive baseline the doc warns against: it
  trusts a pose purely on its published 2-sigma radius. On the calibrated
  ``error_floor`` model this is *actively* unsafe, because a degenerate-geometry
  pose has low fitness, and low fitness yields a *small* covariance, so this
  consumer assigns highest confidence to exactly the biased-tail poses that carry
  multi-metre error. It exists to demonstrate the failure, not to be used.

* ``CovariancePlusCategoryConsumer`` -- the reference consumer the doc's
  guidance describes: it still *consumes* the covariance (the 2-sigma radius is
  the trust gate and sets the fusion weight), but it first vetoes on the
  diagnostics that covariance cannot represent -- an active degeneracy/overload
  category (``weak_overlap`` is the tell for the bias tail), a reject streak, or
  an open reinitialization request. Covariance decides *how much* to trust a
  geometrically-sound pose; the category decides *whether the pose is
  geometrically sound at all*.

Both are pure and deterministic; ``test_pose_arbitration_policy.py`` pins that
the covariance-only consumer trusts the bias tail (the failure) and the reference
consumer rejects it (the fix), while both agree on the healthy and
covariance-loosened cases.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.pose_arbitration.interface import (
    ArbitrationVerdict,
    CATEGORY_HEALTHY,
    DOWNWEIGHT,
    PoseSample,
    REJECT,
    TRUST,
)

# Categories that mean "the geometry/timing behind this pose is not sound", so a
# tight covariance on it is not trustworthy regardless of its value. bad_match is
# included for completeness though an accepted, published pose is rarely flagged
# bad_match (its fitness cleared the gate); weak_overlap / stale_prediction /
# overload routinely co-occur with an accepted-but-degenerate pose.
_UNSOUND_CATEGORIES = frozenset({
    "weak_overlap",
    "bad_match",
    "stale_prediction",
    "overload",
    "missing_map",
    "missing_initial_pose",
})


@dataclass(frozen=True)
class ArbitrationParams:
    # A pose is trustworthy for tight fusion only if its 2-sigma xy radius is
    # within this bound (metres). Above it the pose is down-weighted, not vetoed.
    trust_xy_2sigma_m: float = 1.0
    # Down-weighted (rather than trusted) once the 2-sigma radius exceeds the
    # trust bound; rejected outright past this far looser bound.
    reject_xy_2sigma_m: float = 6.0
    # A reject streak at or above this length means tracking is degrading even if
    # the last accepted pose looks clean; stop trusting it for fusion.
    max_consecutive_rejects: int = 3


def _two_sigma_xy(sample: PoseSample) -> float:
    return 2.0 * sample.cov_xy_std_m


class CovarianceOnlyConsumer:
    """Trust purely on the published 2-sigma radius (the unsafe baseline)."""

    name = "covariance_only"
    design = (
        "Trust if 2-sigma xy radius <= trust bound; down-weight up to the reject "
        "bound; reject beyond it. Ignores failure_category -- so on the "
        "error_floor model it is fooled by tight covariance on low-fitness "
        "degenerate poses."
    )

    def __init__(self, params: ArbitrationParams | None = None) -> None:
        self.params = params or ArbitrationParams()

    def decide(self, sample: PoseSample) -> ArbitrationVerdict:
        two_sigma = _two_sigma_xy(sample)
        if two_sigma <= self.params.trust_xy_2sigma_m:
            return ArbitrationVerdict(
                TRUST, 1.0, "2sigma_within_trust_radius")
        if two_sigma <= self.params.reject_xy_2sigma_m:
            weight = self.params.trust_xy_2sigma_m / two_sigma
            return ArbitrationVerdict(DOWNWEIGHT, weight, "2sigma_loose")
        return ArbitrationVerdict(REJECT, 0.0, "2sigma_beyond_reject_radius")


class CovariancePlusCategoryConsumer:
    """Consume covariance, but veto on diagnostics it cannot represent."""

    name = "covariance_plus_category"
    design = (
        "Veto first on failure_category (degeneracy/overload), reject streak, or "
        "an open reinitialization request -- the bias-tail signals covariance "
        "cannot encode -- then use the 2-sigma radius to weight the surviving, "
        "geometrically-sound pose. This is the consumer docs/pose_covariance.md "
        "prescribes."
    )

    def __init__(self, params: ArbitrationParams | None = None) -> None:
        self.params = params or ArbitrationParams()

    def decide(self, sample: PoseSample) -> ArbitrationVerdict:
        # 1) Diagnostics veto -- these are the cases the calibrated covariance is
        #    documented as unable to bound (the systematically biased tail).
        if sample.reinitialization_requested:
            return ArbitrationVerdict(REJECT, 0.0, "reinitialization_requested")
        if sample.consecutive_rejected_updates >= self.params.max_consecutive_rejects:
            return ArbitrationVerdict(REJECT, 0.0, "reject_streak")
        if sample.failure_category in _UNSOUND_CATEGORIES:
            return ArbitrationVerdict(
                REJECT, 0.0, "unsound_category:%s" % sample.failure_category)
        if sample.failure_category != CATEGORY_HEALTHY:
            # Unknown / future category: be conservative, do not fully trust.
            return ArbitrationVerdict(
                DOWNWEIGHT, 0.5, "unknown_category:%s" % sample.failure_category)

        # 2) Geometrically sound -- now the covariance is meaningful, weight by it.
        two_sigma = _two_sigma_xy(sample)
        if two_sigma <= self.params.trust_xy_2sigma_m:
            return ArbitrationVerdict(TRUST, 1.0, "healthy_2sigma_within_trust")
        if two_sigma <= self.params.reject_xy_2sigma_m:
            weight = self.params.trust_xy_2sigma_m / two_sigma
            return ArbitrationVerdict(DOWNWEIGHT, weight, "healthy_2sigma_loose")
        return ArbitrationVerdict(REJECT, 0.0, "2sigma_beyond_reject_radius")


def verdict_is_correct(sample: PoseSample, verdict: ArbitrationVerdict,
                       trust_radius_m: float) -> bool:
    """Score a verdict against ground truth.

    A verdict is *wrong* only when it TRUSTs (weight 1.0) a pose whose true xy
    error exceeds the trust radius -- that is the dangerous outcome we guard
    against (feeding a confidently-wrong pose into fusion). DOWNWEIGHT and REJECT
    are conservative and counted correct regardless of the true error; trusting a
    genuinely accurate pose is likewise correct.
    """
    trusted = verdict.trust == TRUST
    accurate = sample.abs_xy_error_m <= trust_radius_m
    if trusted and not accurate:
        return False
    return True
