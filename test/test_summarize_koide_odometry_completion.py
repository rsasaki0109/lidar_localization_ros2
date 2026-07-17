#!/usr/bin/env python3

import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import summarize_koide_odometry_completion as repeat  # noqa: E402


def _write(path, ok=True, estimated="run.csv", thresholds=None):
    path.write_text(json.dumps({
        "ok": ok,
        "estimated_csv": estimated,
        "thresholds": thresholds or {"requested_duration_sec": 380.0},
        "failed_gates": [] if ok else ["coverage"],
        "metrics": {"coverage_ratio": 1.0 if ok else 0.5},
    }), encoding="utf-8")


def test_three_distinct_passes_are_required_and_sufficient():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        paths = []
        for index in range(3):
            path = root / f"run{index}.json"
            _write(path, estimated=f"run{index}.csv")
            paths.append(path)
        result = repeat.summarize(paths)
    assert result["ok"]
    assert result["passing_run_count"] == 3


def test_one_failure_rejects_the_repeat_gate():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        paths = []
        for index in range(3):
            path = root / f"run{index}.json"
            _write(path, ok=index != 2, estimated=f"run{index}.csv")
            paths.append(path)
        result = repeat.summarize(paths)
    assert not result["ok"]
    assert "all_repeats_pass" in result["failed_gates"]


def test_duplicate_artifact_cannot_fake_repeats():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        paths = []
        for index in range(3):
            path = root / f"run{index}.json"
            _write(path, estimated="same.csv")
            paths.append(path)
        result = repeat.summarize(paths)
    assert not result["ok"]
    assert "distinct_run_artifacts" in result["failed_gates"]


def test_threshold_changes_reject_the_comparison():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        paths = []
        for index, duration in enumerate((380.0, 380.0, 120.0)):
            path = root / f"run{index}.json"
            _write(
                path, estimated=f"run{index}.csv",
                thresholds={"requested_duration_sec": duration})
            paths.append(path)
        result = repeat.summarize(paths)
    assert not result["ok"]
    assert "identical_thresholds" in result["failed_gates"]


if __name__ == "__main__":
    test_three_distinct_passes_are_required_and_sufficient()
    test_one_failure_rejects_the_repeat_gate()
    test_duplicate_artifact_cannot_fake_repeats()
    test_threshold_changes_reject_the_comparison()
    print("test_summarize_koide_odometry_completion: all tests passed")
