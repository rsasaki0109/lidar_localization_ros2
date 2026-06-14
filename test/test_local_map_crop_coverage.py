#!/usr/bin/env python3
"""Unit tests for the local-map-crop coverage diagnostic core.

Exercises the pure, dependency-light functions (no PCD, no scipy required --
the brute-force path is used on small synthetic arrays) so the (A) vs (C) verdict
logic is pinned independently of any Boreas data.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import diagnose_local_map_crop_coverage as cov  # noqa: E402


def grid_map(x0, x1, y0, y1, step=1.0):
    xs = np.arange(x0, x1 + step, step)
    ys = np.arange(y0, y1 + step, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])


def test_count_within_radius_matches_brute_force_definition():
    map_xy = grid_map(0, 10, 0, 10, step=1.0)  # 121 points on a unit grid
    # A radius-1.0 disk around (5,5) covers (5,5) and its 4 axis neighbours.
    counts = cov.count_points_within_radius(map_xy, np.array([[5.0, 5.0]]), 1.0)
    assert counts[0] == 5


def test_empty_map_returns_zero_counts():
    counts = cov.count_points_within_radius(np.empty((0, 2)), np.array([[0.0, 0.0]]), 5.0)
    assert counts.tolist() == [0]


def test_coverage_cliff_detected_when_trajectory_leaves_map():
    # Map covers x in [0, 100]; vehicle drives straight out to x = 200.
    map_xy = grid_map(0, 100, -5, 5, step=1.0)
    t = np.arange(0, 40, 1.0)
    xy = np.column_stack([np.linspace(0, 200, len(t)), np.zeros(len(t))])
    result = cov.analyze_coverage(map_xy, xy, t, radius_m=10.0, min_points=20)
    assert result["cliff_index"] is not None
    # The cliff must land near where x passes ~100 + radius margin, i.e. well
    # into the second half of the run, never at the start.
    assert result["cliff_time"] > t[len(t) // 2]
    assert "COVERAGE-CLIFF" in cov.verdict(result, 20)


def test_no_cliff_when_trajectory_stays_inside_map():
    # Vehicle stays comfortably inside a large dense map for the whole run.
    map_xy = grid_map(-200, 200, -200, 200, step=2.0)
    t = np.arange(0, 30, 1.0)
    xy = np.column_stack([np.linspace(-50, 50, len(t)), np.zeros(len(t))])
    result = cov.analyze_coverage(map_xy, xy, t, radius_m=30.0, min_points=50)
    assert result["cliff_index"] is None
    v = cov.verdict(result, 50)
    assert "COVERAGE-OK" in v
    # The (A)/(C) split: OK means a real crop_too_small is prediction-driven.
    assert "prediction-driven" in v


def test_count_scale_compensates_for_subsampling():
    # Dense map: the full-map in-radius count is well above min_points, but a
    # stride-4 subsample alone undercounts ~4x and would trip a false cliff.
    map_xy = grid_map(-100, 100, -100, 100, step=1.0)
    t = np.arange(0, 10, 1.0)
    xy = np.column_stack([np.linspace(-20, 20, len(t)), np.zeros(len(t))])
    sub = map_xy[::4]
    raw = cov.analyze_coverage(sub, xy, t, radius_m=15.0, min_points=400)
    scaled = cov.analyze_coverage(sub, xy, t, radius_m=15.0, min_points=400, count_scale=4)
    # Unscaled strided counts trip the absolute threshold (the bug)...
    assert raw["cliff_index"] is not None
    # ...scaling back by the stride restores a true-negative (no false cliff).
    assert scaled["cliff_index"] is None


def test_out_of_bounds_flag_tracks_bbox_plus_radius():
    map_xy = grid_map(0, 100, 0, 100, step=2.0)
    t = np.array([0.0, 1.0])
    # First pose inside; second pose far outside bbox + radius.
    xy = np.array([[50.0, 50.0], [1000.0, 50.0]])
    result = cov.analyze_coverage(map_xy, xy, t, radius_m=10.0, min_points=20)
    assert not result["out_of_bounds"][0]
    assert result["out_of_bounds"][1]


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
