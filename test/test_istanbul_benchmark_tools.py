#!/usr/bin/env python3
"""The Istanbul replay tools without ROS: reference cleanup, window setup,
scoring and the baseline/candidate comparison."""

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "benchmark"))

import istanbul_compare as compare
import istanbul_replay as replay

# Header stamps in the Istanbul bag run this far behind the receive time.
STAMP_LAG = 2371531.4


def _quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _poses(stamps, x, y=0.0, z=0.0, yaw=0.0):
    rows = np.zeros((len(stamps), 8))
    rows[:, 0] = stamps
    rows[:, 1] = x
    rows[:, 2] = y
    rows[:, 3] = z
    rows[:, 4:8] = _quaternion(yaw)
    return rows


def _write_reference(fixture, rows):
    lines = ["t,x,y,z,qx,qy,qz,qw"] + [
        ",".join(f"{v:.6f}" for v in row) for row in rows
    ]
    (fixture / "gnss_reference.csv").write_text("\n".join(lines) + "\n")


def test_reference_drops_the_stale_poses_before_the_first_jump(tmp_path):
    stamps = 100.0 + 0.05 * np.arange(40)
    rows = _poses(stamps, np.where(stamps < 100.12, 0.0, 2.0 + stamps - 100.15))
    _write_reference(tmp_path, rows)
    ref = replay.load_reference(tmp_path)
    assert len(ref) == 37
    assert abs(ref[0, 0] - 100.15) < 1e-6


def test_reference_keeps_a_jump_after_the_first_second(tmp_path):
    # A jump when GNSS comes back after an outage is part of the reference.
    stamps = 100.0 + 0.05 * np.arange(200)
    rows = _poses(stamps, np.where(stamps < 105.0, 0.0, 1.5))
    _write_reference(tmp_path, rows)
    assert len(replay.load_reference(tmp_path)) == 200


def test_window_start_is_mapped_to_the_header_stamps(tmp_path):
    received = 1.7e9 + 0.1 * np.arange(600)
    np.savetxt(
        tmp_path / "time_map.txt",
        np.column_stack([received, received - STAMP_LAG]),
        fmt="%.6f",
    )
    bag_start = received[0] - 0.5
    (tmp_path / "bag_start.txt").write_text(f"{bag_start:.9f}\n")
    t_start = replay.window_start_stamp(tmp_path, 20.0)
    assert abs(t_start - (bag_start + 20.0 - STAMP_LAG)) < 1e-3


def _preset(tmp_path):
    preset = tmp_path / "preset.yaml"
    preset.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "registration_method": "NDT_OMP",
                        "local_map_async_rebuild": True,
                        "ndt_resolution": 1.0,
                    }
                }
            }
        )
    )
    return preset


def test_window_params_start_at_the_reference_pose_and_apply_overrides(tmp_path):
    stamps = 50.0 + 0.05 * np.arange(100)
    ref = _poses(stamps, stamps - 50.0, y=3.0, z=12.0, yaw=0.5)
    params = replay.window_params(
        _preset(tmp_path),
        "/maps/istanbul.pcd",
        ref,
        51.02,
        [
            "local_map_async_rebuild=false",
            "ndt_resolution=2.0",
            "registration_method=SMALL_GICP",
        ],
    )
    p = params["/**"]["ros__parameters"]
    assert p["map_path"] == "/maps/istanbul.pcd"
    assert p["use_sim_time"] is True
    assert p["set_initial_pose"] is True
    # The first reference pose at or after the window start.
    assert abs(p["initial_pose_x"] - 1.05) < 1e-9
    assert p["initial_pose_y"] == 3.0
    assert p["initial_pose_z"] == 12.0
    assert abs(p["initial_pose_qz"] - math.sin(0.25)) < 1e-9
    assert abs(p["initial_pose_qw"] - math.cos(0.25)) < 1e-9
    assert p["local_map_async_rebuild"] is False
    assert p["ndt_resolution"] == 2.0
    assert p["registration_method"] == "SMALL_GICP"


def test_window_params_past_the_reference_use_its_last_pose(tmp_path):
    ref = _poses(np.arange(10.0), np.arange(10.0))
    params = replay.window_params(_preset(tmp_path), "map.pcd", ref, 99.0, [])
    assert params["/**"]["ros__parameters"]["initial_pose_x"] == 9.0


def _write_estimate(path, rows):
    np.savetxt(path, rows, fmt="%.6f")


def test_scores_cover_only_the_window_and_wrap_yaw(tmp_path):
    ref_stamps = np.arange(0.0, 20.0, 0.05)
    ref = _poses(ref_stamps, 10.0 * ref_stamps, yaw=math.radians(-179.0))
    stamps = np.arange(0.0, 20.0, 0.1) + 0.05
    est = _poses(stamps, 10.0 * stamps + 0.3, y=0.4, z=0.2, yaw=math.radians(179.0))
    est[stamps >= 15.0, 3] = 1.2
    _write_estimate(tmp_path / "est.tum", est)
    score = replay.evaluate(tmp_path / "est.tum", ref, 5.0, 15.0)
    assert score["poses"] == 100
    assert score["poses_per_sec"] == 10.0
    assert abs(score["horizontal_m"]["rmse"] - 0.5) < 1e-3
    assert abs(score["z_m"]["max"] - 0.2) < 1e-3
    assert abs(score["trans3d_m"]["median"] - math.hypot(0.5, 0.2)) < 1e-3
    assert abs(score["yaw_deg"]["rmse"] - 2.0) < 1e-3
    assert abs(score["final_trans3d_m"] - math.hypot(0.5, 0.2)) < 1e-3


def test_runs_without_poses_in_the_window_score_zero_poses(tmp_path):
    ref = _poses(np.arange(0.0, 20.0, 0.05), 0.0)
    assert replay.evaluate(tmp_path / "missing.tum", ref, 0.0, 10.0) == {"poses": 0}
    (tmp_path / "empty.tum").write_text("")
    assert replay.evaluate(tmp_path / "empty.tum", ref, 0.0, 10.0) == {"poses": 0}
    _write_estimate(tmp_path / "late.tum", _poses(np.arange(30.0, 40.0, 0.1), 0.0))
    assert replay.evaluate(tmp_path / "late.tum", ref, 0.0, 10.0) == {"poses": 0}


def _compare_args(out, **overrides):
    values = {
        "base_ws": "/ws/base",
        "cand_ws": "/ws/cand",
        "fixture": "/fixture",
        "bag": "/bag",
        "map": "/map.pcd",
        "out": str(out),
        "repeats": 1,
        "set": ["local_map_radius=100.0"],
        "cand_set": ["local_map_async_rebuild=true"],
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def _option(command, name):
    return command[command.index(name) + 1]


def test_compare_alternates_variants_and_skips_finished_runs(tmp_path, monkeypatch):
    done = tmp_path / "A_start_0" / "base_0"
    done.mkdir(parents=True)
    (done / "score.json").write_text("{}")
    commands = []
    monkeypatch.setattr(
        compare.subprocess, "run", lambda command, **_: commands.append(command)
    )
    compare.run(_compare_args(tmp_path))

    runs = [
        (Path(_option(c, "--out")).parent.name, Path(_option(c, "--out")).name)
        for c in commands
    ]
    assert runs == [
        ("A_start_0", "cand_0"),
        ("B_grade_2820", "base_0"),
        ("B_grade_2820", "cand_0"),
        ("C_climb_2300", "base_0"),
        ("C_climb_2300", "cand_0"),
    ]
    for command, (window, variant) in zip(commands, runs, strict=True):
        start, duration = compare.WINDOWS[window]
        assert float(_option(command, "--start")) == start
        assert float(_option(command, "--duration")) == duration
        overrides = [arg for arg in command if arg.startswith("--set=")]
        if variant.startswith("base"):
            assert _option(command, "--ws") == "/ws/base"
            assert overrides == ["--set=local_map_radius=100.0"]
        else:
            assert _option(command, "--ws") == "/ws/cand"
            assert overrides == [
                "--set=local_map_radius=100.0",
                "--set=local_map_async_rebuild=true",
            ]
    # Concurrent runs never share a ROS domain.
    domains = [_option(c, "--domain") for c in commands]
    assert len(set(domains)) == len(domains)


def _score(horizontal, z=0.1, yaw=0.5, p95=1.0):
    return {
        "horizontal_m": {"rmse": horizontal},
        "z_m": {"rmse": z},
        "yaw_deg": {"rmse": yaw},
        "trans3d_m": {"p95": p95},
    }


def test_summary_reports_the_median_and_range_per_variant(tmp_path, capsys):
    for repeat, horizontal in enumerate((0.6, 0.1, 0.2)):
        run = tmp_path / "B_grade_2820" / f"base_{repeat}"
        run.mkdir(parents=True)
        (run / "score.json").write_text(json.dumps(_score(horizontal)))
    run = tmp_path / "B_grade_2820" / "cand_0"
    run.mkdir(parents=True)
    (run / "score.json").write_text(json.dumps(_score(0.25, z=0.4)))
    compare.summarize(tmp_path)

    rows = capsys.readouterr().out.splitlines()[2:]
    assert rows == [
        "| B_grade_2820 | base | 3 | 0.200 (0.100-0.600) | 0.100 (0.100-0.100)"
        " | 0.500 (0.500-0.500) | 1.000 (1.000-1.000) |",
        "| B_grade_2820 | cand | 1 | 0.250 (0.250-0.250) | 0.400 (0.400-0.400)"
        " | 0.500 (0.500-0.500) | 1.000 (1.000-1.000) |",
    ]
