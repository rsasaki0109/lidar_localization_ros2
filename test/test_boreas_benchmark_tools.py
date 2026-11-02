#!/usr/bin/env python3
"""The Boreas fixture and replay tools without ROS or downloads: the ground-truth
rotation convention, voxel thinning, the PLY map and scoring."""

import http.client
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "benchmark"))

import boreas_prepare as prepare
import boreas_replay as replay


def _rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def test_heading_turns_clockwise_in_enu():
    # lidar_poses.csv rotations are (Rz Ry Rx)^T: a heading turns the other way round
    # from an ENU yaw.
    np.testing.assert_allclose(prepare.rotation(0.0, 0.0, 0.3), _rz(-0.3), atol=1e-12)


def test_quaternion_matches_the_rotation():
    for roll, pitch, heading in (
        (0.02, -0.03, 0.56),
        (0.1, 0.2, -2.9),
        (0.0, 0.0, math.pi),
    ):
        m = prepare.rotation(roll, pitch, heading)
        q = prepare.quaternion(m)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-9
        np.testing.assert_allclose(_matrix(q), m, atol=1e-9)


def test_voxel_first_keeps_the_first_point_of_each_voxel():
    points = np.array(
        [
            [0.01, 0.01, 0.0, 1.0],
            [0.05, 0.02, 0.0, 2.0],  # same 0.1 m voxel as the first
            [0.31, 0.0, 0.0, 3.0],
            [-0.01, 0.0, 0.0, 4.0],  # the voxel below zero
        ]
    )
    kept = prepare.voxel_first(points, 0.1)
    np.testing.assert_array_equal(kept[:, 3], [1.0, 3.0, 4.0])


def test_map_is_binary_little_endian_float32(tmp_path):
    points = np.array([[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]])
    path = tmp_path / "map.ply"
    prepare.write_ply(path, points)
    data = path.read_bytes()
    header, body = data.split(b"end_header\n", 1)
    assert b"format binary_little_endian 1.0" in header
    assert b"element vertex 2" in header
    np.testing.assert_array_equal(
        np.frombuffer(body, dtype="<f4").reshape(-1, 4), points
    )


def test_score_matches_stamps_and_ignores_unmatched_poses():
    reference = np.zeros((100, 8))
    reference[:, 0] = 1000.0 + 0.1 * np.arange(100)
    reference[:, 1] = np.arange(100)
    reference[:, 7] = 1.0
    poses = reference[::2].copy()
    poses[:, 2] += 0.3  # 0.3 m off in y
    stray = poses[0].copy()
    stray[0] += 50.0  # after the reference: not matched
    stray[1] += 50.0
    result = replay.score(
        [*poses, stray], reference, scans=100, diagnostics=set(range(50)), accepted=40
    )
    assert result["poses"] == 51
    assert result["matched_poses"] == 50
    assert abs(result["rmse_m"] - 0.3) < 1e-9
    assert abs(result["horizontal_rmse_m"] - 0.3) < 1e-9
    assert result["z_rmse_m"] == 0.0
    assert result["diagnostic_coverage"] == 0.5
    assert result["accepted_share"] == 0.4
    assert set(result["rmse_by_20_s"]) == {"0-20"}


def test_score_without_poses():
    reference = np.zeros((10, 8))
    reference[:, 0] = np.arange(10)
    result = replay.score([], reference, scans=10, diagnostics=set(), accepted=0)
    assert result["poses"] == 0
    assert "rmse_m" not in result


def test_fetch_retries_a_transfer_cut_off_mid_body(monkeypatch):
    calls = []

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *exc_info):
            return False

        def read(self):
            calls.append("read")
            if len(calls) == 1:
                raise http.client.IncompleteRead(b"partial", 100)
            return b"whole body"

    monkeypatch.setattr(
        prepare.urllib.request, "urlopen", lambda url, timeout: Response()
    )
    monkeypatch.setattr(prepare.time, "sleep", lambda seconds: None)
    assert prepare.fetch("https://example.invalid/key") == b"whole body"
    assert len(calls) == 2
