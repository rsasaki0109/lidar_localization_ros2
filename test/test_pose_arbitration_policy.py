#!/usr/bin/env python3
"""Regression tests for the reference pose-arbitration consumer (issue #72).

These pin the claim docs/pose_covariance.md makes -- the calibrated covariance is
consumable by arbitration logic *provided the consumer also watches
/alignment_status* -- as a runnable contract:

* the naive covariance-only consumer is fooled by the bias tail (it TRUSTs
  low-fitness degenerate poses, which carry multi-metre error but a tight
  covariance), and
* the reference covariance + category consumer rejects every bias-tail case while
  still trusting genuinely healthy poses and down-weighting honestly-loose ones.

Every fixture in experiments/pose_arbitration/fixtures declares the expected
verdict from each consumer, so the fixtures and the policy cannot drift apart.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.pose_arbitration.interface import PoseSample, TRUST  # noqa: E402
from experiments.pose_arbitration.policy import (  # noqa: E402
    CovarianceOnlyConsumer,
    CovariancePlusCategoryConsumer,
    verdict_is_correct,
)

FIXTURE_DIR = ROOT / "experiments" / "pose_arbitration" / "fixtures"
TRUST_RADIUS_M = 1.0


def _fixtures():
    out = []
    for path in sorted(FIXTURE_DIR.glob("*.json")):
        data = json.loads(path.read_text())
        out.append((data["name"], data["expectation"], PoseSample(**data["sample"])))
    return out


def test_fixtures_exist():
    assert _fixtures(), "no pose_arbitration fixtures found"


def test_each_fixture_matches_declared_expectation():
    consumers = {
        "covariance_only": CovarianceOnlyConsumer(),
        "covariance_plus_category": CovariancePlusCategoryConsumer(),
    }
    for name, expectation, sample in _fixtures():
        for consumer_name, expected_trust in expectation.items():
            got = consumers[consumer_name].decide(sample)
            assert got.trust == expected_trust, (
                f"{name}: {consumer_name} expected {expected_trust!r}, "
                f"got {got.trust!r} ({got.reason})")


def test_reference_consumer_makes_no_dangerous_trust():
    # The whole point: the reference consumer must never TRUST a pose whose true
    # error exceeds the trust radius. Covariance-only does, on the bias tail.
    ref = CovariancePlusCategoryConsumer()
    for name, _expectation, sample in _fixtures():
        verdict = ref.decide(sample)
        assert verdict_is_correct(sample, verdict, TRUST_RADIUS_M), (
            f"{name}: reference consumer dangerously TRUSTed a "
            f"{sample.abs_xy_error_m:.1f} m-error pose")


def test_covariance_only_is_fooled_by_the_bias_tail():
    # This is the failure that motivates the category signal -- if it ever stops
    # happening the demonstration (and the doc's caveat) would be stale. At least
    # one fixture must show covariance-only dangerously trusting a wrong pose.
    naive = CovarianceOnlyConsumer()
    dangerous = [
        name for name, _e, sample in _fixtures()
        if not verdict_is_correct(sample, naive.decide(sample), TRUST_RADIUS_M)
    ]
    assert dangerous, "expected covariance-only to be fooled by at least one fixture"


def test_bias_tail_fixture_is_the_decisive_split():
    # The degenerate low-fitness fixture is the crux: tight covariance, large
    # error. covariance-only TRUSTs it (wrong); the reference REJECTs it (right).
    name = "degenerate_low_fitness_bias_tail_should_reject"
    sample = next(s for n, _e, s in _fixtures() if n == name)
    assert CovarianceOnlyConsumer().decide(sample).trust == TRUST
    assert CovariancePlusCategoryConsumer().decide(sample).trust == "reject"
    # And it really is a tight covariance (the trap), not a loose one.
    assert 2.0 * sample.cov_xy_std_m < 1.0


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
