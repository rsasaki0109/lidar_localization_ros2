#!/usr/bin/env python3
"""The replay-check helpers that need no ROS graph: parameter overrides, the
initial-pose parameters and stopping a child process group."""

import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import replay_session


def test_overrides_take_yaml_values():
    parameters = {"scan_max_range": 60.0, "use_imu": False}
    replay_session.apply_overrides(
        parameters,
        ["scan_max_range=80", "use_imu=true", "registration_method=NDT_OMP"],
    )
    assert parameters == {
        "scan_max_range": 80,
        "use_imu": True,
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
