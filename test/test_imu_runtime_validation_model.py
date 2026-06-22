#!/usr/bin/env python3

import contextlib
import csv
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from lidar_localization_mid360.validation_model import AlignmentDiagnosticSample
from lidar_localization_mid360.validation_model import RuntimeValidationConfig
from lidar_localization_mid360.validation_model import evaluate_runtime_summary
from lidar_localization_mid360.validation_model import load_alignment_csv
from lidar_localization_mid360.validation_model import render_runtime_report
from lidar_localization_mid360.validation_model import summarize_runtime
from lidar_localization_mid360.validation_model import validation_exit_code

VALIDATOR_PATH = REPO_ROOT / "scripts" / "validate_lidar_localization_imu.py"
validator_spec = importlib.util.spec_from_file_location(
    "validate_lidar_localization_imu",
    VALIDATOR_PATH,
)
validator_tool = importlib.util.module_from_spec(validator_spec)
assert validator_spec.loader is not None
validator_spec.loader.exec_module(validator_tool)


def sample(status="imu_preintegration_prediction_active", **values):
    merged = {
        "imu_preintegration_status": status,
        "imu_integrated_sample_count": "12",
        "imu_received_sample_count": "14",
        "imu_skipped_sample_count": "2",
        "registration_seed_source": "imu_preintegration",
        "scan_time_status": "scan_time_range_ready",
        "deskew_readiness_status": "deskew_ready",
    }
    merged.update({key: str(value) for key, value in values.items()})
    return AlignmentDiagnosticSample(
        stamp_sec=1.0,
        level=0,
        message="ok",
        values=merged,
    )


class TestImuRuntimeValidationModel(unittest.TestCase):
    def test_active_imu_preintegration_passes_default_thresholds(self):
        summary = summarize_runtime([sample(), sample(), sample()])
        checks = evaluate_runtime_summary(summary, RuntimeValidationConfig())

        self.assertEqual(validation_exit_code(checks), 0)
        self.assertEqual(summary["imu_active_count"], 3)
        self.assertEqual(summary["imu_integrated_sample_count_max"], 12)
        self.assertEqual(summary["registration_seed_source_counts"]["imu_preintegration"], 3)

    def test_required_imu_seed_source_passes_when_selected_enough(self):
        summary = summarize_runtime([
            sample(),
            sample(),
            sample(registration_seed_source="previous_delta"),
        ])
        checks = evaluate_runtime_summary(
            summary,
            RuntimeValidationConfig(
                require_imu_seed_source=True,
                min_imu_seed_source_ratio=0.5,
            ),
        )

        self.assertEqual(validation_exit_code(checks), 0)
        self.assertTrue(any("seed source ratio" in check.message for check in checks))

    def test_required_imu_seed_source_fails_when_missing(self):
        missing_source = sample()
        missing_source.values.pop("registration_seed_source")
        summary = summarize_runtime([missing_source, missing_source, missing_source])
        checks = evaluate_runtime_summary(
            summary,
            RuntimeValidationConfig(require_imu_seed_source=True),
        )

        self.assertEqual(validation_exit_code(checks), 1)
        self.assertTrue(any("registration_seed_source is missing" in check.message for check in checks))

    def test_required_imu_seed_source_fails_when_other_seed_is_used(self):
        summary = summarize_runtime([
            sample(registration_seed_source="previous_delta"),
            sample(registration_seed_source="twist_prediction"),
            sample(registration_seed_source="current_pose"),
        ])
        checks = evaluate_runtime_summary(
            summary,
            RuntimeValidationConfig(require_imu_seed_source=True),
        )

        self.assertEqual(validation_exit_code(checks), 1)
        self.assertTrue(any("seed source ratio" in check.message for check in checks))

    def test_inactive_imu_ratio_fails(self):
        summary = summarize_runtime([
            sample(),
            sample(status="imu_preintegration_waiting_for_imu", imu_integrated_sample_count=0),
            sample(status="imu_preintegration_waiting_for_imu", imu_integrated_sample_count=0),
        ])
        checks = evaluate_runtime_summary(
            summary,
            RuntimeValidationConfig(min_imu_active_ratio=0.8),
        )

        self.assertEqual(validation_exit_code(checks), 1)
        self.assertTrue(any("active ratio" in check.message for check in checks))

    def test_fallback_fails_by_default(self):
        summary = summarize_runtime([
            sample(),
            sample(status="imu_preintegration_fallback_mode", imu_preintegration_fallback_mode="true"),
            sample(),
        ])
        checks = evaluate_runtime_summary(summary, RuntimeValidationConfig())

        self.assertEqual(validation_exit_code(checks), 1)
        self.assertEqual(summary["imu_fallback_count"], 1)
        self.assertTrue(any("fallback count" in check.message for check in checks))

    def test_deskew_required_passes_when_applied_enough(self):
        summary = summarize_runtime([
            sample(
                continuous_time_deskew_status="continuous_time_deskew_applied",
                continuous_time_deskew_applied="true",
                continuous_time_deskew_point_count="100",
            ),
            sample(
                continuous_time_deskew_status="continuous_time_deskew_applied",
                continuous_time_deskew_applied="true",
                continuous_time_deskew_point_count="120",
            ),
            sample(continuous_time_deskew_status="continuous_time_deskew_waiting_for_new_imu"),
        ])
        checks = evaluate_runtime_summary(
            summary,
            RuntimeValidationConfig(require_deskew_applied=True, min_deskew_applied_ratio=0.5),
        )

        self.assertEqual(validation_exit_code(checks), 0)
        self.assertEqual(summary["continuous_time_deskew_applied_count"], 2)
        self.assertEqual(summary["continuous_time_deskew_point_count_max"], 120)

    def test_deskew_required_fails_when_never_applied(self):
        summary = summarize_runtime([
            sample(continuous_time_deskew_status="continuous_time_deskew_scan_time_not_ready"),
            sample(continuous_time_deskew_status="continuous_time_deskew_scan_time_not_ready"),
            sample(continuous_time_deskew_status="continuous_time_deskew_scan_time_not_ready"),
        ])
        checks = evaluate_runtime_summary(
            summary,
            RuntimeValidationConfig(require_deskew_applied=True),
        )

        self.assertEqual(validation_exit_code(checks), 1)
        self.assertTrue(any("deskew applied ratio" in check.message for check in checks))

    def test_load_alignment_csv_reads_values_json(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "alignment_status.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=[
                        "message_index",
                        "stamp_sec",
                        "status_index",
                        "level",
                        "name",
                        "message",
                        "hardware_id",
                        "values_json",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "message_index": "0",
                        "stamp_sec": "10.25",
                        "status_index": "0",
                        "level": "0",
                        "name": "alignment",
                        "message": "ok",
                        "hardware_id": "",
                        "values_json": json.dumps(
                            {"imu_preintegration_status": "imu_preintegration_prediction_active"}
                        ),
                    }
                )

            samples = load_alignment_csv(str(path))

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].stamp_sec, 10.25)
        self.assertEqual(
            samples[0].values["imu_preintegration_status"],
            "imu_preintegration_prediction_active",
        )

    def test_report_includes_key_counts_and_checks(self):
        summary = summarize_runtime([sample(), sample(), sample()])
        checks = evaluate_runtime_summary(summary, RuntimeValidationConfig())
        report = "\n".join(render_runtime_report(summary, checks))

        self.assertIn("imu status counts", report)
        self.assertIn("registration seed sources", report)
        self.assertIn("imu preintegration seed source", report)
        self.assertIn("[OK] IMU preintegration active ratio", report)

    def test_validator_reports_missing_alignment_csv_without_traceback(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            code = validator_tool.main([
                "--alignment-csv",
                "/tmp/missing_alignment_status.csv",
            ])

        self.assertEqual(code, 2)
        self.assertIn("input error:", stderr.getvalue())
        self.assertIn("missing_alignment_status.csv", stderr.getvalue())


if __name__ == "__main__":
    sys.exit(unittest.main())
