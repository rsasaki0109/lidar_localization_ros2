"""Exercise the search prior around a discontinuity in the odometry frame."""

import importlib.util
import math
from pathlib import Path

import pytest

EXPERIMENT = (
    Path(__file__).resolve().parents[1] / "experiments" / "localization_deployment"
)
spec = importlib.util.spec_from_file_location(
    "odom_window_prior", EXPERIMENT / "odom_window_prior.py"
)
prior_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior_module)
OdometryWindowPrior = prior_module.OdometryWindowPrior
PlanarPose = prior_module.PlanarPose

SECOND = 10**9


def test_gap_with_plausible_motion_keeps_the_startup_prior():
    prior = OdometryWindowPrior(PlanarPose(100.0, 100.0, 0.0))
    start = PlanarPose(140.0, 0.0, 0.0)
    end = PlanarPose(182.0, 0.0, 0.0)
    assert not prior.observe_odometry(18 * SECOND, start)
    prior.accept_map_match(18 * SECOND, start, start)
    assert not prior.observe_odometry(21 * SECOND, end)
    assert not prior.recovering
    assert prior.center(end) == (282.0, 100.0)


def test_restart_uses_last_match_and_then_new_map_to_odom_alignment():
    prior = OdometryWindowPrior(PlanarPose(100.0, 0.0, 0.0))
    old_odom = PlanarPose(140.0, 0.0, 1.0)
    previous_match = PlanarPose(239.0, 50.0, math.pi / 2)
    restarted = PlanarPose(0.0, 0.0, 0.0)
    moved = PlanarPose(10.0, 0.0, 0.0)
    prior.observe_odometry(18 * SECOND, old_odom)
    prior.accept_map_match(18 * SECOND, previous_match, old_odom)

    assert prior.observe_odometry(21 * SECOND, restarted)
    assert prior.recovering
    assert prior.center(restarted) == (239.0, 50.0)
    assert prior.center(moved) == pytest.approx((239.0, 60.0))

    assert not prior.observe_odometry(19 * SECOND, old_odom)
    prior.accept_map_match(19 * SECOND, previous_match, old_odom)
    assert prior.recovering
    assert prior.center(moved) == pytest.approx((239.0, 60.0))

    new_match = PlanarPose(250.0, 80.0, 0.0)
    prior.accept_map_match(22 * SECOND, new_match, moved)
    assert not prior.recovering
    assert prior.center(PlanarPose(11.0, 0.0, 0.0)) == (251.0, 80.0)


def test_restart_before_first_match_invalidates_the_prior():
    prior = OdometryWindowPrior(PlanarPose(100.0, 100.0, 0.0))
    prior.observe_odometry(SECOND, PlanarPose(120.0, 0.0, 0.0))
    assert prior.observe_odometry(2 * SECOND, PlanarPose(0.0, 0.0, 0.0))
    assert prior.center(PlanarPose(1.0, 0.0, 0.0)) is None
