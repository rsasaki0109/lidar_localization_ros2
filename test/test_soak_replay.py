"""The scoring of tools/benchmark/soak_replay.py: laps, per-lap errors and the memory
trend, without ROS."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "benchmark"))

import soak_replay as soak

PERIOD = 11.0  # a 10 s lap plus the 1 s gap


def _reference():
    stamps = 100.0 + np.arange(0.0, 10.0, 0.1)
    rows = np.zeros((len(stamps), 8))
    rows[:, 0] = stamps
    rows[:, 1] = np.arange(len(stamps))
    rows[:, 7] = 1.0
    return rows


def test_lap_of_counts_shifted_stamps():
    assert soak.lap_of(100.0, 100.0, PERIOD) == 0
    assert soak.lap_of(109.9, 100.0, PERIOD) == 0
    assert soak.lap_of(111.0, 100.0, PERIOD) == 1
    assert soak.lap_of(100.0 + 3 * PERIOD + 5.0, 100.0, PERIOD) == 3
    # The initial pose's, published before /clock starts.
    assert soak.lap_of(0.0, 100.0, PERIOD) < 0


def test_laps_are_scored_against_the_unshifted_reference():
    reference = _reference()
    poses = []
    for lap in range(2):
        shifted = reference.copy()
        shifted[:, 0] += lap * PERIOD
        # Lap 1 is 0.5 m off in x.
        shifted[:, 1] += 0.5 * lap
        poses.extend(shifted.tolist())
    poses.append([0.0, 99.0, 0, 0, 0, 0, 0, 1])
    statuses = [
        (
            100.0 + lap * PERIOD,
            {"recovery_action": "accept_measurement", "alignment_time_sec": "0.010"},
        )
        for lap in range(2)
    ]
    laps = soak.score_laps(np.array(poses), reference, PERIOD, statuses, 2)
    assert [lap["lap"] for lap in laps] == [0, 1]
    assert laps[0]["matched"] == len(reference)
    assert laps[0]["rmse_m"] == 0.0
    assert laps[1]["rmse_m"] == 0.5
    assert laps[1]["accepted"] == 1
    assert laps[1]["align_ms_median"] == 10.0


def test_memory_trend_skips_the_warm_up():
    hours = np.arange(0.0, 1.0, 0.01)
    rss = 1000.0 + 20.0 * hours
    rss[:5] = 1500.0  # warm-up peak
    samples = np.column_stack([hours * 3600.0, rss])
    trend = soak.memory_trend(samples, skip_sec=0.05 * 3600.0)
    assert abs(trend["rss_mb_per_hour"] - 20.0) < 1e-6
    assert trend["rss_mb_max"] == 1500.0
    assert soak.memory_trend(samples[:1], skip_sec=0.0) == {}
