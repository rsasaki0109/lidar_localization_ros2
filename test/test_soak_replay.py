"""The scoring of tools/benchmark/soak_replay.py: laps, per-lap errors and the memory
trend, without ROS."""

import argparse
import sys
from pathlib import Path

import numpy as np
import pytest

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
    assert laps[0]["latency_ms_median"] is None
    assert laps[0]["late_poses"] == 0


def test_pose_latency_is_scored_per_lap():
    reference = _reference()
    # Clouds published at 0.1 s intervals; lap 1's poses arrive 0.3 s after theirs.
    stamps_ns = [
        round((100.0 + lap * PERIOD + 0.1 * i) * 1e9)
        for lap in range(2)
        for i in range(5)
    ]
    published = {stamp: 1000.0 + 0.1 * n for n, stamp in enumerate(stamps_ns)}
    received = {
        stamp: published[stamp] + (0.3 if n >= 5 else 0.02)
        for n, stamp in enumerate(stamps_ns)
    }
    # The pose after /initialpose has no cloud.
    received[round(99.5e9)] = 999.0
    latencies = soak.pose_latencies(published, received)
    assert len(latencies) == 10
    laps = soak.score_laps(np.empty((0, 8)), reference, PERIOD, [], 2, latencies)
    assert laps[0]["latency_ms_median"] == 20.0
    assert laps[0]["late_poses"] == 0
    assert laps[1]["latency_ms_p95"] == 300.0
    assert laps[1]["latency_ms_max"] == 300.0
    assert laps[1]["late_poses"] == 5


def test_memory_trend_skips_the_warm_up():
    hours = np.arange(0.0, 1.0, 0.01)
    rss = 1000.0 + 20.0 * hours
    rss[:5] = 1500.0  # warm-up peak
    samples = np.column_stack([hours * 3600.0, rss])
    trend = soak.memory_trend(samples, skip_sec=0.05 * 3600.0)
    assert abs(trend["rss_mb_per_hour"] - 20.0) < 1e-6
    assert trend["rss_mb_max"] == 1500.0
    assert soak.memory_trend(samples[:1], skip_sec=0.0) == {}


def test_parse_fault():
    assert soak.parse_fault("lidar@2:40+5") == {
        "kind": "lidar",
        "lap": 2,
        "start_sec": 40.0,
        "duration_sec": 5.0,
    }
    assert soak.parse_fault("reinit@3:60.5")["duration_sec"] == 0.0
    for text in ("drop@1:2+1", "lidar@x:1+1", "lidar@1", "imu@1:2+-1", "stall@-1:2+1"):
        with pytest.raises(argparse.ArgumentTypeError):
            soak.parse_fault(text)


def test_faults_are_scored_by_recovery(monkeypatch):
    monkeypatch.setattr(soak, "SETTLE_SEC", 2.0)
    reference = _reference()
    poses = []
    for lap in range(2):
        for row in reference:
            into_lap = row[0] - 100.0
            if lap == 1 and 3.0 <= into_lap < 5.0:
                continue  # no poses while the clouds are dropped
            pose = row.copy()
            pose[0] += lap * PERIOD
            if lap == 1 and 5.0 <= into_lap < 6.0:
                pose[1] += 2.0  # off for a second after the gap
            if lap == 0 and into_lap > 9.85:
                pose[1] += 1.0  # the last pose is off
            poses.append(pose)
    errors = soak.pose_errors(np.array(poses), reference, PERIOD, 2)
    faults = [
        soak.parse_fault("lidar@1:3+2"),
        soak.parse_fault("stall@0:9+0.5"),
    ]
    gap, stall = soak.score_faults(errors, 100.0, PERIOD, faults)
    assert gap["poses_during"] == 0
    assert gap["max_error_m"] == 2.0
    assert gap["recovered_after_sec"] == 1.0
    assert gap["rmse_settled_m"] == 0.0
    assert stall["poses_during"] == 5
    assert stall["max_error_m"] == 1.0
    assert stall["recovered_after_sec"] is None
    assert stall["rmse_settled_m"] is None


def test_a_republished_reset_pose_does_not_score_as_recovered(monkeypatch):
    # Boreas soak gl3_fast_a: after each 10 s stall no scan was accepted again,
    # but the localizer republished each supervisor reset, stamped at the scan G2
    # had fixed (and 0.02-0.25 m from the reference there), so the stall scored
    # as recovered 21-50 s later with a settled RMSE of 0.03-0.23 m.
    monkeypatch.setattr(soak, "SETTLE_SEC", 2.0)
    reference = _reference()
    period_ns = round(PERIOD * 1e9)
    poses, stamps_ns, published = [], [], set()
    for row in reference:
        stamp_ns = round(row[0] * 1e9) + period_ns
        published.add(stamp_ns)
        if row[0] - 100.0 < 3.0:
            pose = row.copy()
            pose[0] = stamp_ns * 1e-9
            poses.append(pose)
            stamps_ns.append(stamp_ns)
    # Two resets after the stall, echoed at their fix's stamp less a few ns, as
    # the supervisor's float stamp lands.
    for into_lap in (6.0, 8.0):
        row = reference[round(into_lap * 10)].copy()
        stamp_ns = round(row[0] * 1e9) + period_ns - 48
        row[0] = stamp_ns * 1e-9
        poses.append(row)
        stamps_ns.append(stamp_ns)
    poses = np.array(poses)
    fault = [soak.parse_fault("stall@1:3+2")]

    everything = soak.score_faults(
        soak.pose_errors(poses, reference, PERIOD, 2), 100.0, PERIOD, fault
    )[0]
    assert everything["recovered_after_sec"] is not None  # the reported bug

    scans = soak.scan_poses(poses, stamps_ns, published)
    assert len(scans) == 30
    (stall,) = soak.score_faults(
        soak.pose_errors(scans, reference, PERIOD, 2), 100.0, PERIOD, fault
    )
    assert stall["poses_during"] == 0
    assert stall["recovered_after_sec"] is None
    assert stall["rmse_settled_m"] is None
    assert stall["max_error_m"] is None


def test_scan_poses_keeps_every_pose_of_a_published_cloud():
    poses = np.array([[1.0, 0, 0, 0, 0, 0, 0, 1], [2.0, 0, 0, 0, 0, 0, 0, 1]])
    assert len(soak.scan_poses(poses, [10, 20], {10, 20})) == 2
    assert soak.scan_poses(poses, [10, 21], {10, 20}).tolist() == [poses[0].tolist()]
    assert soak.scan_poses(np.zeros((0, 8)), [], {10}).shape == (0, 8)
