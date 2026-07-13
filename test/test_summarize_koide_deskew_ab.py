#!/usr/bin/env python3

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "summarize_koide_deskew_ab.py"
spec = importlib.util.spec_from_file_location("summarize_koide_deskew_ab", SCRIPT)
tool = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(tool)


def run(mode, rmse, end_error, latency):
    return {
        "backend_hint": mode,
        "run_dir": f"/tmp/{mode}",
        "trajectory_eval": {
            "translation_rmse_m": rmse,
            "translation_error_last_m": end_error,
            "translation_error_max_m": end_error,
            "rotation_rmse_deg": 1.0,
            "rotation_error_last_deg": 1.0,
            "matched_sample_count": 100,
        },
        "pose_trace": {"requested_duration_ratio": 1.0},
        "alignment": {
            "rejected_streak_max": 2,
            "reinitialization_requested_rows": 0,
            "alignment_time_p95_sec": latency,
        },
    }


class TestSummarizeKoideDeskewAb(unittest.TestCase):
    def _write_repeat(self, root: Path, index: int) -> None:
        repeat = root / f"repeat_{index:02d}"
        (repeat / "deskew").mkdir(parents=True)
        (repeat / "imu_pose_history").mkdir(parents=True)
        (repeat / "comparison.json").write_text(
            json.dumps({
                "runs": [
                    run("lidar_only", 2.0, 3.0, 0.10),
                    run("deskew", 0.2, 0.1, 0.11),
                    run("imu_pose_history", 0.3, 0.2, 0.11),
                    run("lidar_constant_velocity", 0.4, 0.3, 0.11),
                    run("localizability_guard", 0.5, 0.4, 0.11),
                ]
            }),
            encoding="utf-8",
        )
        (repeat / "imu_pose_history/imu_validation.json").write_text(
            json.dumps({
                "checks": [{"level": "OK", "message": "pose history active"}],
                "summary": {
                    "continuous_time_deskew_applied_ratio": 0.9,
                    "continuous_time_deskew_pose_history_coverage_median": 1.0,
                },
            }),
            encoding="utf-8",
        )
        (repeat / "deskew/imu_validation.json").write_text(
            json.dumps({
                "checks": [{"level": "OK", "message": "deskew active"}],
                "summary": {
                    "continuous_time_deskew_applied_ratio": 0.9,
                    "scan_time_duration_median_sec": 0.2,
                    "scan_time_duration_p95_sec": 0.3,
                    "scan_time_duration_max_sec": 0.3,
                    "continuous_time_deskew_skipped_invalid_time_count_max": 0,
                    "continuous_time_deskew_clamped_time_count_max": 0,
                },
            }),
            encoding="utf-8",
        )

    def test_three_good_deskew_repeats_pass_gate(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            for index in range(1, 4):
                self._write_repeat(root, index)

            code = tool.main(["--output-dir", str(root), "--expected-repeats", "3"])
            summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))

        self.assertEqual(code, 0)
        self.assertTrue(summary["overall_pass"])
        self.assertEqual(summary["modes"]["deskew"]["success_count"], 3)
        self.assertAlmostEqual(summary["latency_ratios"]["deskew"], 1.1)

    def test_missing_repeat_fails_gate(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._write_repeat(root, 1)

            code = tool.main(["--output-dir", str(root), "--expected-repeats", "3"])
            summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))

        self.assertEqual(code, 1)
        self.assertFalse(summary["gates"]["repeat_matrix_complete"])


if __name__ == "__main__":
    sys.exit(unittest.main())
