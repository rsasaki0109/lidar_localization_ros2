#!/usr/bin/env python3

import csv
import importlib.machinery
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "benchmark_compare_runs"
loader = importlib.machinery.SourceFileLoader("benchmark_compare_runs", str(SCRIPT_PATH))
spec = importlib.util.spec_from_loader(loader.name, loader)
compare_runs = importlib.util.module_from_spec(spec)
loader.exec_module(compare_runs)


class TestBenchmarkCompareRuns(unittest.TestCase):
    def test_alignment_summary_reports_interpolated_p95(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "alignment_status.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=["stamp_sec", "level", "values_json"],
                )
                writer.writeheader()
                for index, duration in enumerate([0.01, 0.02, 0.03]):
                    writer.writerow({
                        "stamp_sec": str(index),
                        "level": "0",
                        "values_json": '{"alignment_time_sec": "' + str(duration) + '"}',
                    })

            summary = compare_runs.summarize_alignment(path)

        self.assertAlmostEqual(summary["alignment_time_median_sec"], 0.02)
        self.assertAlmostEqual(summary["alignment_time_p95_sec"], 0.029)

    def test_pose_trace_summary_reports_last_elapsed_and_ratio(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            pose_path = Path(tmp_dir) / "pose_trace.csv"
            with pose_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["stamp_sec", "position_x"])
                writer.writeheader()
                writer.writerow({"stamp_sec": "100.0", "position_x": "1.0"})
                writer.writerow({"stamp_sec": "122.5", "position_x": "2.0"})

            summary = compare_runs.summarize_pose_trace(
                pose_path,
                run_start_stamp_sec=95.0,
                requested_duration_sec=100.0,
            )

        self.assertEqual(summary["sample_count"], 2)
        self.assertEqual(summary["span_sec"], 22.5)
        self.assertEqual(summary["last_elapsed_sec"], 27.5)
        self.assertEqual(summary["requested_duration_ratio"], 0.275)


if __name__ == "__main__":
    sys.exit(unittest.main())
