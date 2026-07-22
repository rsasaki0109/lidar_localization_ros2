#!/usr/bin/env python3

import csv
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import check_recovery_pose_gt as pose_gt  # noqa: E402


def _write_pose_csv(path: Path, rows):
    header = ["stamp_sec", "position_x", "position_y"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)


def _write_gt_txt(path: Path, rows):
    with path.open("w", encoding="utf-8") as handle:
        for stamp, x, y in rows:
            handle.write(f"{stamp} {x} {y} 0 0 0 0 1\n")


def _pose(stamp: float, x: float, y: float) -> pose_gt.PoseSample:
    return pose_gt.PoseSample(stamp_sec=stamp, x=x, y=y)


def _gt(stamp: float, x: float, y: float) -> pose_gt.GroundTruthSample:
    return pose_gt.GroundTruthSample(stamp_sec=stamp, x=x, y=y)


def test_never_lost():
    poses = [_pose(1000.0 + i, float(i), 0.0) for i in range(5)]
    gt = [_gt(1000.0 + i, float(i), 0.0) for i in range(5)]
    summary = pose_gt.summarize_recovery_pose_gt(poses, gt)
    assert summary.verdict == "never_lost"
    assert summary.first_loss_sec is None
    assert summary.ok


def test_loss_then_genuine_recovery_window():
    poses = [
        _pose(1000.0, 0.0, 0.0),
        _pose(1001.0, 10.0, 0.0),
    ]
    poses.extend(_pose(1002.0 + i, 0.0, 0.0) for i in range(20))
    gt = [_gt(1000.0 + i, 0.0, 0.0) for i in range(22)]
    summary = pose_gt.summarize_recovery_pose_gt(
        poses,
        gt,
        loss_threshold_m=5.0,
        recovered_threshold_m=3.0,
        min_recovered_window_sec=15.0,
        max_sample_gap_sec=5.0,
    )
    assert summary.verdict == "recovered_true"
    assert summary.ends_recovered is True
    assert summary.qualifying_window_count >= 1
    assert summary.longest_recovered_window_sec >= 15.0
    assert summary.ok


def test_loss_then_error_stays_high():
    poses = [_pose(1000.0, 0.0, 0.0)]
    poses.append(_pose(1001.0, 20.0, 0.0))
    poses.extend(_pose(1002.0 + i, 20.0, 0.0) for i in range(20))
    gt = [_gt(1000.0 + i, 0.0, 0.0) for i in range(22)]
    summary = pose_gt.summarize_recovery_pose_gt(poses, gt)
    assert summary.verdict == "recovered_false"
    assert summary.qualifying_window_count == 0
    assert not summary.ok


def test_early_window_then_terminal_loss():
    poses = [
        _pose(1000.0, 0.0, 0.0),
        _pose(1001.0, 10.0, 0.0),
    ]
    poses.extend(_pose(1002.0 + i, 0.0, 0.0) for i in range(20))
    poses.extend(_pose(1022.0 + i, 32.0, 0.0) for i in range(10))
    gt = [_gt(1000.0 + i, 0.0, 0.0) for i in range(32)]
    summary = pose_gt.summarize_recovery_pose_gt(
        poses,
        gt,
        loss_threshold_m=5.0,
        recovered_threshold_m=3.0,
        min_recovered_window_sec=15.0,
        max_sample_gap_sec=5.0,
    )
    assert summary.verdict == "recovered_false"
    assert summary.ends_recovered is False
    assert summary.qualifying_window_count >= 1
    assert summary.last_window_end_gap_sec is not None
    assert summary.last_window_end_gap_sec > 5.0
    assert not summary.ok


def test_gap_breaks_recovered_window():
    poses = [
        _pose(1000.0, 0.0, 0.0),
        _pose(1001.0, 10.0, 0.0),
    ]
    poses.extend(_pose(1002.0 + i, 0.0, 0.0) for i in range(8))
    poses.extend(_pose(1016.0 + i, 0.0, 0.0) for i in range(9))
    gt = [_gt(1000.0 + i, 0.0, 0.0) for i in range(30)]
    summary = pose_gt.summarize_recovery_pose_gt(
        poses,
        gt,
        min_recovered_window_sec=15.0,
        max_sample_gap_sec=5.0,
    )
    assert summary.verdict == "recovered_false"
    assert summary.qualifying_window_count == 0


def test_gt_nearest_stamp_rejection():
    poses = [_pose(1000.0, 0.0, 0.0), _pose(1002.0, 0.0, 0.0)]
    gt = [_gt(1000.0, 0.0, 0.0)]
    errors = pose_gt.compute_pose_errors(poses, gt, max_gt_stamp_delta_sec=0.5)
    assert errors[0].error_m == 0.0
    assert errors[1].error_m is None


def test_cli_json_output():
    with tempfile.TemporaryDirectory() as tmp:
        pose_csv = Path(tmp) / "pose_trace.csv"
        gt_txt = Path(tmp) / "gt.txt"
        out = Path(tmp) / "summary.json"
        _write_pose_csv(
            pose_csv,
            [
                {"stamp_sec": "1000000000.0", "position_x": "0.0", "position_y": "0.0"},
                {"stamp_sec": "0.0", "position_x": "99.0", "position_y": "0.0"},
            ],
        )
        _write_gt_txt(gt_txt, [(1000000000.0, 0.0, 0.0)])
        assert pose_gt.main([
            "--pose-trace", str(pose_csv),
            "--gt", str(gt_txt),
            "--output-json", str(out),
        ]) == 0
        payload = json.loads(out.read_text(encoding="utf-8"))
        assert payload["verdict"] == "never_lost"


def test_load_benchmark_csv_as_ground_truth():
    with tempfile.TemporaryDirectory() as tmp:
        gt_csv = Path(tmp) / "reference.csv"
        _write_pose_csv(
            gt_csv,
            [
                {"stamp_sec": "1000.0", "position_x": "1.0", "position_y": "2.0"},
                {"stamp_sec": "1001.0", "position_x": "3.0", "position_y": "4.0"},
            ],
        )
        samples = pose_gt.load_ground_truth(gt_csv)
        assert samples == [
            _gt(1000.0, 1.0, 2.0),
            _gt(1001.0, 3.0, 4.0),
        ]


if __name__ == "__main__":
    test_never_lost()
    test_loss_then_genuine_recovery_window()
    test_loss_then_error_stays_high()
    test_early_window_then_terminal_loss()
    test_gap_breaks_recovered_window()
    test_gt_nearest_stamp_rejection()
    test_cli_json_output()
    test_load_benchmark_csv_as_ground_truth()
    print("test_check_recovery_pose_gt: all tests passed")
