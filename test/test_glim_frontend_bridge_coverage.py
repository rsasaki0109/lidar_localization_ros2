import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "experiments"
    / "koide_glim_frontend_localization"
    / "summarize_bridge_coverage.py"
)
SPEC = importlib.util.spec_from_file_location("summarize_bridge_coverage", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write_alignment(path):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["stamp_sec", "message", "values_json"]
        )
        writer.writeheader()
        for stamp, accepted, seed in (
            (1.0, True, "odom_tf_prediction"),
            (2.0, False, "odom_tf_prediction"),
            (3.0, False, "imu_preintegration"),
            (4.0, True, "odom_tf_prediction"),
        ):
            writer.writerow(
                {
                    "stamp_sec": stamp,
                    "message": "ok" if accepted else "registration_not_converged",
                    "values_json": json.dumps(
                        {
                            "recovery_action": (
                                "accept_measurement" if accepted else "reject_measurement"
                            ),
                            "registration_seed_source": seed,
                        }
                    ),
                }
            )


def _write_poses(path):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["stamp_sec"])
        writer.writeheader()
        for stamp in (0.0, 0.5, 1.001, 2.0, 4.0):
            writer.writerow({"stamp_sec": stamp})


class GlimFrontendBridgeCoverageTest(unittest.TestCase):
    def _fixture(self, directory):
        alignment = directory / "alignment.csv"
        poses = directory / "poses.csv"
        _write_alignment(alignment)
        _write_poses(poses)
        return MODULE.summarize(alignment, poses, tolerance_sec=0.01)

    def test_separates_matched_bridge_and_output_coverage(self):
        with tempfile.TemporaryDirectory() as raw_directory:
            summary = self._fixture(Path(raw_directory))

        self.assertEqual(summary["alignment_scan_count"], 4)
        self.assertEqual(summary["accepted_measurement_count"], 2)
        self.assertEqual(summary["rejected_measurement_count"], 2)
        self.assertEqual(summary["scan_output_count"], 3)
        self.assertEqual(summary["accepted_output_count"], 2)
        self.assertEqual(summary["bridge_output_count"], 1)
        self.assertEqual(summary["temporally_covered_scan_count"], 4)
        self.assertEqual(summary["missing_output_count"], 0)
        self.assertEqual(summary["unmatched_pose_count"], 1)
        self.assertEqual(summary["output_coverage_percent"], 100.0)
        self.assertEqual(summary["matched_coverage_percent"], 50.0)
        self.assertEqual(summary["max_output_gap_sec"], 2.0)
        self.assertEqual(
            summary["seed_source_counts"],
            {"imu_preintegration": 1, "odom_tf_prediction": 3},
        )

    def test_gate_keeps_output_and_match_thresholds_independent(self):
        with tempfile.TemporaryDirectory() as raw_directory:
            summary = self._fixture(Path(raw_directory))

        failures = MODULE._gate(
            summary,
            min_output_coverage=70.0,
            min_matched_coverage=95.0,
            max_output_gap=1.0,
        )

        self.assertFalse(any("output_coverage" in failure for failure in failures))
        self.assertTrue(any("matched_coverage" in failure for failure in failures))
        self.assertTrue(any("max_output_gap" in failure for failure in failures))

    def test_timer_outputs_cover_scans_without_exact_stamp_matches(self):
        with tempfile.TemporaryDirectory() as raw_directory:
            directory = Path(raw_directory)
            alignment = directory / "alignment.csv"
            poses = directory / "poses.csv"
            _write_alignment(alignment)
            with poses.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["stamp_sec"])
                writer.writeheader()
                for index in range(33):
                    writer.writerow({"stamp_sec": 0.95 + 0.1 * index})

            summary = MODULE.summarize(alignment, poses, tolerance_sec=0.01)

        self.assertEqual(summary["scan_output_count"], 0)
        self.assertEqual(summary["temporally_covered_scan_count"], 4)
        self.assertEqual(summary["output_coverage_percent"], 100.0)
        self.assertAlmostEqual(summary["max_output_gap_sec"], 0.1)


if __name__ == "__main__":
    unittest.main()
