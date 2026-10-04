#!/usr/bin/env python3

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "tum_trajectory_to_pose_reference_csv.py"


class TestTumTrajectoryToPoseReferenceCsv(unittest.TestCase):
    def test_csv_only_run_does_not_write_an_initial_pose(self):
        # Without --output-initial-pose-yaml the converter used to try to write
        # the YAML to "." (Path("")) and fail after writing the CSV.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            tum = root / "route.txt"
            tum.write_text("1.0 0 0 0 0 0 0 1\n2.0 1 0 0 0 0 0 1\n", encoding="utf-8")
            csv_path = root / "route.csv"
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--input",
                    str(tum),
                    "--output-csv",
                    str(csv_path),
                ],
                cwd=root,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("rows_written: 2", result.stdout)
            self.assertNotIn("output_initial_pose_yaml", result.stdout)
            self.assertEqual(len(csv_path.read_text(encoding="utf-8").splitlines()), 3)


if __name__ == "__main__":
    unittest.main()
