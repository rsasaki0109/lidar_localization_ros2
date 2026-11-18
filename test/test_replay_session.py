#!/usr/bin/env python3
"""The replay-check helpers that need no ROS graph: parameter overrides, the
initial-pose parameters and stopping a child process group."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import replay_session


def test_overrides_take_yaml_values():
    parameters = {"scan_max_range": 60.0, "use_odom": False}
    replay_session.apply_overrides(
        parameters,
        ["scan_max_range=80", "use_odom=true", "registration_method=NDT_OMP"],
    )
    assert parameters == {
        "scan_max_range": 80,
        "use_odom": True,
        "registration_method": "NDT_OMP",
    }


def test_override_without_a_value_is_refused():
    with pytest.raises(ValueError, match="KEY=VALUE"):
        replay_session.apply_overrides({}, ["scan_max_range"])


def test_initial_pose_parameters():
    parameters = replay_session.initial_pose_parameters([1, 2, 3, 0, 0, 0.6, 0.8])
    assert parameters == {
        "set_initial_pose": True,
        "initial_pose_x": 1.0,
        "initial_pose_y": 2.0,
        "initial_pose_z": 3.0,
        "initial_pose_qx": 0.0,
        "initial_pose_qy": 0.0,
        "initial_pose_qz": 0.6,
        "initial_pose_qw": 0.8,
    }
    with pytest.raises(ValueError, match="got 3 values"):
        replay_session.initial_pose_parameters([1, 2, 3])


def test_stop_process_interrupts_the_process_group():
    process = subprocess.Popen(["sleep", "60"], start_new_session=True)
    replay_session.stop_process(process)
    assert process.returncode == -2  # SIGINT was enough
    replay_session.stop_process(process)  # an exited process is left alone


def test_read_process_reports_memory_and_cpu_time():
    rss, peak, cpu, anon = replay_session.read_process(os.getpid())
    assert 0 < rss <= peak
    assert cpu > 0
    assert 0 < anon <= rss


def test_find_process_falls_back_to_the_group():
    process = subprocess.Popen(["sleep", "60"], start_new_session=True)
    try:
        assert replay_session.find_process(process.pid, "sleep") == process.pid
        assert replay_session.find_process(process.pid, "no_such_node") == process.pid
    finally:
        replay_session.stop_process(process)


def test_resource_summary_rates_cpu_per_interval_and_per_scan():
    # (wall_sec, rss_mb, peak_rss_mb, cpu_sec): half a core, then one core.
    samples = [
        (0.0, 100.0, 300.0, 10.0),
        (1.0, 110.0, 300.0, 10.5),
        (2.0, 120.0, 300.0, 11.5),
        (3.0, 130.0, 300.0, 12.5),
        (4.0, 130.0, 300.0, 13.5),
    ]
    summary = replay_session.resource_summary(samples, data_sec=4.0, scans=40)
    assert summary["cpu_sec"] == 3.5
    assert summary["cpu_cores_mean"] == 0.875
    assert summary["cpu_cores_p95"] == 1.0
    assert summary["cpu_sec_per_data_sec"] == 0.875
    assert summary["cpu_ms_per_scan"] == 87.5
    assert summary["rss_mb_peak"] == 300.0
    assert summary["rss_mb_steady"] == 130.0
    assert summary["rss_mb_last"] == 130.0
    # A window leaves out the samples outside it, but not the peak.
    window = replay_session.resource_summary(samples, window=(1.0, 3.0))
    assert window["cpu_sec"] == 2.0
    assert window["cpu_cores_mean"] == 1.0
    assert window["rss_mb_peak"] == 300.0
    assert replay_session.resource_summary(samples[:1]) == {}
    assert "anon_rss_mb_peak" not in summary


def test_resource_summary_reports_anonymous_memory():
    # (wall_sec, rss_mb, peak_rss_mb, cpu_sec, anon_rss_mb)
    samples = [
        (0.0, 100.0, 120.0, 1.0, 40.0),
        (1.0, 110.0, 120.0, 1.1, 60.0),
        (2.0, 110.0, 120.0, 1.2, 50.0),
        (3.0, 110.0, 120.0, 1.3, 50.0),
    ]
    summary = replay_session.resource_summary(samples)
    assert summary["anon_rss_mb_peak"] == 60.0
    assert summary["anon_rss_mb_steady"] == 50.0


def test_resource_monitor_samples_into_a_csv(tmp_path):
    monitor = replay_session.ResourceMonitor(
        os.getpid(), tmp_path / "resources.csv", interval_sec=0.0
    )
    monitor.poll()
    sum(range(200000))
    monitor.poll()
    monitor.close()
    monitor.poll()  # closed: no more samples
    lines = (tmp_path / "resources.csv").read_text().splitlines()
    assert lines[0] == "wall_sec,rss_mb,peak_rss_mb,cpu_sec,anon_rss_mb"
    assert len(lines) == 3 == len(monitor.samples) + 1
    assert monitor.summary()["rss_mb_peak"] > 0
