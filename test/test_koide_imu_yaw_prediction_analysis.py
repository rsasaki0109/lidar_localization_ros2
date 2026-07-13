#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import unittest


SCRIPT = (Path(__file__).resolve().parents[1] / "experiments" /
          "imu_yaw_prediction" / "run_dataset_analysis.py")
SPEC = importlib.util.spec_from_file_location("imu_dataset_analysis", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class KoideImuDatasetAnalysisTest(unittest.TestCase):
    def test_aggregate_keeps_families_and_variant_scores_separate(self):
        template = {
            "identity_rate_rmse_rad_s": 1.0,
            "signed_axis_rate_rmse_rad_s": 0.2,
            "wahba_rate_rmse_rad_s": 0.1,
            "imu_minus_reference_time_offset_sec": 0.01,
            "accel_units": "m_s2",
            "gravity_world_sign": "+Z",
            "static_extrinsic_difference_deg": 0.5,
        }
        results = [
            {**template, "family": "indoor"},
            {**template, "family": "indoor", "wahba_rate_rmse_rad_s": 0.3},
            {**template, "family": "outdoor", "accel_units": "g",
             "static_extrinsic_difference_deg": None},
        ]
        aggregate = MODULE.aggregate_results(results)
        self.assertEqual(aggregate["indoor"]["sequence_count"], 2)
        self.assertAlmostEqual(
            aggregate["indoor"]["variants"]["wahba"]["median_rate_rmse_rad_s"],
            0.2)
        self.assertEqual(aggregate["outdoor"]["accel_units"], ["g"])

    def test_manifest_covers_all_eleven_sequences(self):
        self.assertEqual(len(MODULE.SEQUENCES), 11)
        self.assertEqual(
            sum(family == "indoor" for _, _, family in MODULE.SEQUENCES.values()), 5)

    def test_recorded_result_rejects_runtime_promotion(self):
        result_path = SCRIPT.parent / "results.json"
        result = json.loads(result_path.read_text(encoding="utf-8"))
        self.assertEqual(result["dataset_sequence_count"], 11)
        self.assertEqual(result["promotion_decision"], "rejected")
        indoor = next(
            run for run in result["runtime_ab"]
            if run["sequence"] == "indoor_easy_01")
        self.assertGreater(
            indoor["imu_rotation"]["translation_rmse_m"],
            indoor["lidar_only"]["translation_rmse_m"])


if __name__ == "__main__":
    unittest.main()
