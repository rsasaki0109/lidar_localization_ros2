#!/usr/bin/env python3

import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "benchmark_eval_trajectory"


def write_poses(path: Path, xs):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "stamp_sec", "position_x", "position_y", "position_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w",
        ])
        writer.writeheader()
        for index, x in enumerate(xs):
            writer.writerow({
                "stamp_sec": str(index), "position_x": str(x),
                "position_y": "0", "position_z": "0", "orientation_x": "0",
                "orientation_y": "0", "orientation_z": "0", "orientation_w": "1",
            })


class TestBenchmarkEvalTrajectory(unittest.TestCase):
    def test_reports_last_and_max_errors(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            estimated = root / "estimated.csv"
            reference = root / "reference.csv"
            output = root / "result.json"
            write_poses(estimated, [0.0, 2.0, 1.0])
            write_poses(reference, [0.0, 0.0, 0.0])

            result = subprocess.run(
                [str(SCRIPT), "--estimated-csv", str(estimated),
                 "--reference-csv", str(reference), "--output-json", str(output)],
                cwd=REPO_ROOT, check=False, text=True, capture_output=True,
            )
            report = json.loads(output.read_text(encoding="utf-8"))

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["translation_error_last_m"], 1.0)
        self.assertEqual(report["translation_error_max_m"], 2.0)
        self.assertEqual(report["rotation_error_last_deg"], 0.0)


if __name__ == "__main__":
    sys.exit(unittest.main())
