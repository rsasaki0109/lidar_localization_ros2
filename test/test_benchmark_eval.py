#!/usr/bin/env python3

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "benchmark"))

import benchmark_eval as bench


def _trajectory(stamps, x, y=0.0):
    rows = np.zeros((len(stamps), 8))
    rows[:, 0] = stamps
    rows[:, 1] = x
    rows[:, 2] = y
    rows[:, 7] = 1.0
    return rows


def test_tracking_is_scored_in_the_map_frame_without_alignment():
    truth = _trajectory(np.arange(100.0, 160.0, 0.01), np.arange(0.0, 60.0, 0.01))
    stamps = np.arange(103.0, 160.0, 0.1)
    estimate = _trajectory(stamps, stamps - 100.0 + 0.05)
    score = bench.score_run(estimate, truth, start_sec=100.0)
    assert score.initialized
    assert not score.wrong_init
    assert abs(score.time_to_first_pose_sec - 3.0) < 1e-6
    assert abs(score.median_xy_m - 0.05) < 1e-6
    assert score.frac_over_1_m == 0.0
    assert score.tracked_fraction > 0.95


def test_a_consistently_offset_track_is_a_wrong_initialization():
    # Shifted 1.8 m along the aisle and tracked: an aligned comparison would
    # report almost no error.
    truth = _trajectory(np.arange(0.0, 30.0, 0.01), np.linspace(0.0, 10.0, 3000))
    stamps = np.arange(2.0, 30.0, 0.1)
    estimate = _trajectory(stamps, np.interp(stamps, truth[:, 0], truth[:, 1]) + 1.8)
    score = bench.score_run(estimate, truth, start_sec=0.0)
    assert score.wrong_init
    assert abs(score.median_xy_m - 1.8) < 1e-6
    assert score.frac_over_1_m == 1.0


def test_runs_without_poses_or_overlap_are_reported():
    truth = _trajectory(np.arange(0.0, 10.0, 0.01), 0.0)
    assert not bench.score_run(np.empty((0, 8)), truth, 0.0).initialized
    late = bench.score_run(_trajectory(np.arange(50.0, 60.0, 0.1), 0.0), truth, 0.0)
    assert late.initialized
    assert late.matched == 0
    assert late.median_xy_m is None


def test_summary_counts_initializations_and_wrong_ones():
    truth = _trajectory(np.arange(0.0, 30.0, 0.01), 0.0)
    good = bench.score_run(_trajectory(np.arange(1.0, 30.0, 0.1), 0.1), truth, 0.0)
    wrong = bench.score_run(_trajectory(np.arange(1.0, 30.0, 0.1), 3.0), truth, 0.0)
    missing = bench.score_run(np.empty((0, 8)), truth, 0.0)
    row = bench.summary_table({"case": [good, wrong, missing]}).splitlines()[-1]
    assert row.startswith("| case | 3 | 2 | 1 |")


if __name__ == "__main__":
    test_tracking_is_scored_in_the_map_frame_without_alignment()
    test_a_consistently_offset_track_is_a_wrong_initialization()
    test_runs_without_poses_or_overlap_are_reported()
    test_summary_counts_initializations_and_wrong_ones()
