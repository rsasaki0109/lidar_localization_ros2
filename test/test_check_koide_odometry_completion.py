#!/usr/bin/env python3

import csv
import json
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import check_koide_odometry_completion as completion  # noqa: E402


FIELDS = [
    "stamp_sec", "position_x", "position_y", "position_z",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w",
]


def _yaw_quaternion(yaw_rad):
    return (0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad))


def _write_trajectory(path, duration=20.0, dt=0.1, scale=1.0, gap=None, nan_row=False):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        count = int(round(duration / dt)) + 1
        for index in range(count):
            stamp = index * dt
            if gap and gap[0] < stamp < gap[1]:
                continue
            x = scale * stamp
            quaternion = _yaw_quaternion(0.01 * scale * stamp)
            writer.writerow({
                "stamp_sec": stamp,
                "position_x": "nan" if nan_row and index == 2 else x,
                "position_y": 0.0,
                "position_z": 0.0,
                "orientation_x": quaternion[0],
                "orientation_y": quaternion[1],
                "orientation_z": quaternion[2],
                "orientation_w": quaternion[3],
            })


def _write_support_files(root, processing=0.05, scan_period=0.1, died=False):
    runtime = root / "runtime.json"
    summary = root / "summary.json"
    runtime.write_text(json.dumps({
        "processing_p95_sec": processing,
        "scan_period_sec": scan_period,
        "queue_growth_unbounded": False,
        "tf_jump_count": 0,
        "unauthorized_reset_count": 0,
    }), encoding="utf-8")
    summary.write_text(json.dumps({
        "target_process_died_during_run": died,
        "bag_stopped_by_runner": False,
        "return_codes": {"bag_play": 0},
    }), encoding="utf-8")
    return runtime, summary


def _evaluate(root, estimated_duration=20.0, scale=1.0, gap=None, nan_row=False,
              processing=0.05):
    estimated = root / "estimated.csv"
    reference = root / "reference.csv"
    _write_trajectory(reference, duration=20.0)
    _write_trajectory(
        estimated, duration=estimated_duration, scale=scale, gap=gap, nan_row=nan_row)
    runtime, summary = _write_support_files(root, processing=processing)
    thresholds = completion.CompletionThresholds(
        requested_duration_sec=20.0,
        max_translation_ate_rmse_m=2.0,
        max_translation_end_error_m=5.0,
    )
    return completion.evaluate_completion(
        estimated, reference, runtime, summary, thresholds)


def test_perfect_full_trajectory_passes_all_gates():
    with tempfile.TemporaryDirectory() as tmp:
        result = _evaluate(Path(tmp))
    assert result["ok"]
    assert result["metrics"]["coverage_ratio"] == 1.0
    assert result["metrics"]["translation_ate_rmse_m"] < 1e-9
    assert result["metrics"]["rpe_sample_count"] > 0


def test_short_accurate_trajectory_cannot_pass_completion():
    with tempfile.TemporaryDirectory() as tmp:
        result = _evaluate(Path(tmp), estimated_duration=5.0)
    assert not result["ok"]
    assert "coverage" in result["failed_gates"]
    assert "final_arrival" in result["failed_gates"]


def test_gap_non_finite_and_deadline_are_independent_failures():
    with tempfile.TemporaryDirectory() as tmp:
        result = _evaluate(
            Path(tmp), gap=(5.0, 7.0), nan_row=True, processing=0.2)
    assert not result["ok"]
    assert "pose_gap" in result["failed_gates"]
    assert "finite_poses" in result["failed_gates"]
    assert "runtime_deadline" in result["failed_gates"]


def test_scale_drift_fails_rpe_even_with_full_coverage():
    with tempfile.TemporaryDirectory() as tmp:
        result = _evaluate(Path(tmp), scale=1.05)
    assert result["metrics"]["coverage_ratio"] == 1.0
    assert not result["gates"]["rpe_translation"]
    assert not result["ok"]


def test_start_alignment_uses_reference_at_estimate_timestamp():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        estimated = root / "estimated.csv"
        reference = root / "reference.csv"
        _write_trajectory(reference, duration=20.0)
        with estimated.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            for index in range(51):
                stamp = 5.0 + index * 0.1
                quaternion = _yaw_quaternion(0.01 * stamp)
                writer.writerow({
                    "stamp_sec": stamp,
                    "position_x": stamp - 5.0,
                    "position_y": 0.0,
                    "position_z": 0.0,
                    "orientation_x": quaternion[0],
                    "orientation_y": quaternion[1],
                    "orientation_z": quaternion[2],
                    "orientation_w": quaternion[3],
                })
        loaded_estimate = completion.load_trajectory(estimated)
        loaded_reference = completion.load_trajectory(reference)
        metrics = completion._trajectory_metrics(
            loaded_estimate,
            loaded_reference,
            completion.CompletionThresholds(requested_duration_sec=20.0),
        )
    assert metrics["translation_ate_rmse_m"] < 1e-9


def test_cli_writes_machine_readable_failure():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        estimated = root / "estimated.csv"
        reference = root / "reference.csv"
        output = root / "completion.json"
        _write_trajectory(reference, duration=20.0)
        _write_trajectory(estimated, duration=5.0)
        runtime, summary = _write_support_files(root)
        return_code = completion.main([
            "--estimated-csv", str(estimated),
            "--reference-csv", str(reference),
            "--runtime-json", str(runtime),
            "--run-summary-json", str(summary),
            "--output-json", str(output),
            "--requested-duration-sec", "20",
        ])
        payload = json.loads(output.read_text(encoding="utf-8"))
    assert return_code == 1
    assert payload["ok"] is False
    assert "coverage" in payload["failed_gates"]


if __name__ == "__main__":
    test_perfect_full_trajectory_passes_all_gates()
    test_short_accurate_trajectory_cannot_pass_completion()
    test_gap_non_finite_and_deadline_are_independent_failures()
    test_scale_drift_fails_rpe_even_with_full_coverage()
    test_start_alignment_uses_reference_at_estimate_timestamp()
    test_cli_writes_machine_readable_failure()
    print("test_check_koide_odometry_completion: all tests passed")
