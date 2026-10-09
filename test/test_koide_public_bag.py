"""Check dataset preparation and failure-sensitive scoring, without ROS nodes."""

import importlib.util
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "koide_public_bag", ROOT / "scripts/koide_public_bag.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TestKoidePublicBag(unittest.TestCase):
    def test_acceleration_and_known_covariance_use_si_units(self):
        message = SimpleNamespace(
            linear_acceleration=SimpleNamespace(x=1.0, y=-2.0, z=0.5),
            linear_acceleration_covariance=[0.25] * 9,
        )
        MODULE.scale_acceleration(message)
        self.assertAlmostEqual(message.linear_acceleration.y, -2 * 9.80665)
        self.assertAlmostEqual(
            message.linear_acceleration_covariance[4], 0.25 * 9.80665**2
        )

    def test_unknown_covariance_remains_unknown(self):
        covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        message = SimpleNamespace(
            linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=1.0),
            linear_acceleration_covariance=covariance.copy(),
        )
        MODULE.scale_acceleration(message)
        self.assertEqual(message.linear_acceleration_covariance, covariance)
        self.assertAlmostEqual(message.linear_acceleration.z, 9.80665)

    def test_archive_cannot_write_outside_the_data_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "data.zip"
            with zipfile.ZipFile(archive, "w") as zipped:
                zipped.writestr("../outside.txt", "outside")
            with self.assertRaisesRegex(ValueError, "outside the data directory"):
                MODULE.extract_archive(archive, root / "data")
            self.assertFalse((root / "outside.txt").exists())

    def test_corrupt_cached_asset_is_not_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "outdoor_hard_02b.zip").write_bytes(b"corrupt")
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                MODULE.prepare_assets(root, False)

    def test_missing_data_points_to_download_option(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            self.assertRaisesRegex(ValueError, "--download"),
        ):
            MODULE.prepare_assets(Path(directory), False)

    def test_scoring_does_not_align_away_wrong_map_location(self):
        gt = np.array([[float(t), 0, 0, 0, 0, 0, 0, 1] for t in range(10)])
        poses = gt.copy()
        poses[:, 1] = 10
        result = MODULE.evaluate(poses, gt, range(10), range(10))
        self.assertAlmostEqual(result["rmse_m"], 10.0)
        self.assertFalse(result["passed"])

    def test_sparse_output_fails_even_if_observed_poses_are_accurate(self):
        gt = np.array([[float(t), 0, 0, 0, 0, 0, 0, 1] for t in range(10)])
        result = MODULE.evaluate(gt[:2], gt, range(10), range(10))
        self.assertEqual(result["rmse_m"], 0.0)
        self.assertFalse(result["passed"])
        with self.assertRaisesRegex(ValueError, "within 0.15 s"):
            MODULE.evaluate(
                gt + np.array([100, 0, 0, 0, 0, 0, 0, 0]), gt, range(10), range(10)
            )

    def test_diagnostic_coverage_counts_only_expected_clouds(self):
        gt = np.array([[float(t), 0, 0, 0, 0, 0, 0, 1] for t in range(10)])
        result = MODULE.evaluate(gt, gt, range(10), [0, 1, 999, 999])
        self.assertEqual(result["diagnostic_clouds"], 2)
        self.assertEqual(result["diagnostic_coverage"], 0.2)
        self.assertFalse(result["passed"])

    def test_duplicate_or_nonfinite_poses_cannot_pass(self):
        gt = np.array([[float(t), 0, 0, 0, 0, 0, 0, 1] for t in range(10)])
        poses = np.repeat(gt[:2], 5, axis=0)
        result = MODULE.evaluate(poses, gt, range(10), range(10))
        self.assertEqual(result["unique_matched_pose_stamps"], 2)
        self.assertFalse(result["passed"])
        poses[0, 1] = np.nan
        with self.assertRaisesRegex(ValueError, "finite TUM"):
            MODULE.evaluate(poses, gt, range(10), range(10))


if __name__ == "__main__":
    unittest.main()
