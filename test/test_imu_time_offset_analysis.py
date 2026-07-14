#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import unittest


SCRIPT = (Path(__file__).resolve().parents[1] / "experiments" /
          "imu_time_offset" / "run_dataset_analysis.py")
SPEC = importlib.util.spec_from_file_location("imu_time_offset_analysis", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ImuTimeOffsetAnalysisTest(unittest.TestCase):
    def test_manifest_covers_all_eleven_sequences(self):
        self.assertEqual(len(MODULE.SEQUENCES), 11)
        self.assertEqual(
            sum(value[2] == "outdoor" for value in MODULE.SEQUENCES.values()), 6)

    def test_stable_narrow_offset_becomes_runtime_candidate(self):
        metrics = {
            "window_count": 5,
            "search_boundary_window_count": 0,
            "relative_rmse_improvement_over_zero": 0.08,
            "one_percent_best_width_sec": 0.02,
            "window_offset_mad_sec": 0.004,
            "window_offset_median_sec": 0.04,
            "refined_offset_sec": 0.043,
        }
        self.assertEqual(
            MODULE.classify_offset_candidate(metrics)["decision"],
            "runtime_ab_candidate")

    def test_flat_or_unstable_offset_is_rejected(self):
        metrics = {
            "window_count": 4,
            "search_boundary_window_count": 0,
            "relative_rmse_improvement_over_zero": 0.005,
            "one_percent_best_width_sec": 0.08,
            "window_offset_mad_sec": 0.03,
            "window_offset_median_sec": -0.02,
            "refined_offset_sec": 0.04,
        }
        result = MODULE.classify_offset_candidate(metrics)
        self.assertEqual(result["decision"], "reject")
        self.assertIn("broad_minimum", result["reasons"])
        self.assertIn("window_offset_unstable", result["reasons"])

    def test_recorded_runtime_gate_rejects_promotion(self):
        result = json.loads(
            (SCRIPT.parent / "results.json").read_text(encoding="utf-8"))
        self.assertEqual(result["dataset_sequence_count"], 11)
        self.assertEqual(result["promotion_decision"], "rejected")
        self.assertFalse(result["production_runtime_changed"])
        runtime = result["runtime_ab"]
        self.assertGreater(
            runtime["candidate"]["translation_rmse_m"],
            runtime["baseline"]["translation_rmse_m"])
        self.assertGreater(
            runtime["candidate"]["rotation_rmse_deg"],
            runtime["baseline"]["rotation_rmse_deg"])


if __name__ == "__main__":
    unittest.main()
