#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from lidar_localization_mid360.bringup_model import BringupCheckConfig
from lidar_localization_mid360.bringup_model import BringupSnapshot
from lidar_localization_mid360.bringup_model import FAIL
from lidar_localization_mid360.bringup_model import OK
from lidar_localization_mid360.bringup_model import WARN
from lidar_localization_mid360.bringup_model import TopicStats
from lidar_localization_mid360.bringup_model import build_tf_checks
from lidar_localization_mid360.bringup_model import evaluate_snapshot
from lidar_localization_mid360.bringup_model import exit_code
from lidar_localization_mid360.bringup_model import report_lines
from lidar_localization_mid360.bringup_model import topic_summary
from lidar_localization_mid360 import bringup_cli
import check_lidar_localization_bringup as generic_bringup_cli


def marked_topic(frame_id="base_link", count=1):
    topic = TopicStats()
    for index in range(count):
        topic.mark(frame_id, float(index), float(index))
    return topic


def marked_cloud(frame_id="livox_frame", fields=("x", "y", "z")):
    cloud = marked_topic(frame_id)
    cloud.has_xyz_fields = {"x", "y", "z"}.issubset(fields)
    cloud.field_names = tuple(fields)
    return cloud


def marked_status(message="ok", level=0, values=None):
    status = marked_topic("base_link")
    status.last_status_level = level
    status.last_status_message = message
    status.status_values = values or {}
    return status


def snapshot(config, cloud, imu=None, pose=None, status=None, tf_available=None, topic_types=None):
    return BringupSnapshot(
        cloud=cloud,
        imu=imu if imu is not None else TopicStats(),
        pose=pose if pose is not None else TopicStats(),
        status=status if status is not None else TopicStats(),
        tf_checks=build_tf_checks(config, tf_available or {}),
        topic_types=topic_types or {},
    )


class TestMid360BringupModel(unittest.TestCase):
    def test_missing_cloud_is_hard_failure(self):
        config = BringupCheckConfig()
        results = evaluate_snapshot(config, snapshot(config, TopicStats()))

        self.assertEqual(results[0].level, FAIL)
        self.assertIn("/livox/points", results[0].message)
        self.assertIn("cloud_topic", results[0].hint)
        self.assertEqual(exit_code(results), 1)

    def test_missing_cloud_suggests_observed_pointcloud_topic(self):
        config = BringupCheckConfig(cloud_topic="/velodyne_points")
        results = evaluate_snapshot(
            config,
            snapshot(
                config,
                TopicStats(),
                topic_types={
                    "/ouster/points": ["sensor_msgs/msg/PointCloud2"],
                    "/camera/image": ["sensor_msgs/msg/Image"],
                },
            ),
        )

        self.assertEqual(results[0].level, FAIL)
        self.assertIn("Observed PointCloud2 topic(s): /ouster/points", results[0].hint)
        self.assertIn("cloud_topic:=/ouster/points", results[0].hint)

    def test_missing_cloud_reports_expected_topic_is_advertised_but_silent(self):
        config = BringupCheckConfig(cloud_topic="/velodyne_points")
        results = evaluate_snapshot(
            config,
            snapshot(
                config,
                TopicStats(),
                topic_types={"/velodyne_points": ["sensor_msgs/msg/PointCloud2"]},
            ),
        )

        self.assertIn("/velodyne_points is advertised as PointCloud2", results[0].hint)
        self.assertIn("ros2 topic hz /velodyne_points", results[0].hint)

    def test_missing_cloud_reports_expected_topic_wrong_type(self):
        config = BringupCheckConfig(cloud_topic="/velodyne_points")
        results = evaluate_snapshot(
            config,
            snapshot(
                config,
                TopicStats(),
                topic_types={"/velodyne_points": ["sensor_msgs/msg/LaserScan"]},
            ),
        )

        self.assertIn(
            "/velodyne_points exists, but its type is sensor_msgs/msg/LaserScan",
            results[0].hint,
        )
        self.assertIn("sensor_msgs/msg/PointCloud2", results[0].hint)

    def test_imu_is_optional_by_default(self):
        config = BringupCheckConfig()
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        results = evaluate_snapshot(config, snapshot(config, cloud))

        self.assertEqual(results[0].level, OK)
        self.assertEqual(results[1].level, WARN)

    def test_imu_can_be_required_for_strict_runs(self):
        config = BringupCheckConfig(require_imu=True)
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        results = evaluate_snapshot(config, snapshot(config, cloud))

        self.assertEqual(results[1].level, FAIL)

    def test_missing_required_imu_suggests_observed_imu_topic(self):
        config = BringupCheckConfig(require_imu=True, imu_topic="/imu")
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        results = evaluate_snapshot(
            config,
            snapshot(
                config,
                cloud,
                topic_types={"/imu/data": ["sensor_msgs/msg/Imu"]},
            ),
        )

        self.assertEqual(results[1].level, FAIL)
        self.assertIn("Observed IMU topic(s): /imu/data", results[1].hint)
        self.assertIn("imu_topic:=/imu/data", results[1].hint)

    def test_missing_required_imu_reports_expected_topic_is_advertised_but_silent(self):
        config = BringupCheckConfig(require_imu=True, imu_topic="/imu")
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        results = evaluate_snapshot(
            config,
            snapshot(
                config,
                cloud,
                topic_types={"/imu": ["sensor_msgs/msg/Imu"]},
            ),
        )

        self.assertEqual(results[1].level, FAIL)
        self.assertIn("/imu is advertised as Imu", results[1].hint)
        self.assertIn("ros2 topic hz /imu", results[1].hint)

    def test_missing_required_imu_reports_expected_topic_wrong_type(self):
        config = BringupCheckConfig(require_imu=True, imu_topic="/imu")
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        results = evaluate_snapshot(
            config,
            snapshot(
                config,
                cloud,
                topic_types={"/imu": ["std_msgs/msg/String"]},
            ),
        )

        self.assertEqual(results[1].level, FAIL)
        self.assertIn(
            "/imu exists, but its type is std_msgs/msg/String",
            results[1].hint,
        )
        self.assertIn("sensor_msgs/msg/Imu", results[1].hint)

    def test_cloud_frame_mismatch_gets_launch_hint(self):
        config = BringupCheckConfig(
            cloud_topic="/rslidar_points",
            lidar_frame="velodyne",
        )
        cloud = marked_cloud("rslidar")
        results = evaluate_snapshot(config, snapshot(config, cloud))

        mismatch_results = [
            result for result in results
            if result.message == "pointcloud frame is rslidar, but lidar_frame is velodyne"
        ]

        self.assertEqual(mismatch_results[0].level, WARN)
        self.assertIn("lidar_frame_id:=rslidar", mismatch_results[0].hint)

    def test_cloud_time_field_can_be_required_for_continuous_time_readiness(self):
        config = BringupCheckConfig(require_cloud_time_field=True)
        cloud = marked_cloud(fields=("x", "y", "z", "intensity"))

        results = evaluate_snapshot(config, snapshot(config, cloud))

        missing_time_results = [
            result for result in results
            if result.message == "/livox/points has no per-point time field"
        ]
        self.assertEqual(missing_time_results[0].level, FAIL)
        self.assertIn("Continuous-time deskew", missing_time_results[0].hint)
        self.assertIn("offset_time", missing_time_results[0].hint)

    def test_cloud_time_field_accepts_common_names(self):
        config = BringupCheckConfig(require_cloud_time_field=True)
        cloud = marked_cloud(fields=("x", "y", "z", "timestamp"))

        results = evaluate_snapshot(config, snapshot(config, cloud))

        self.assertFalse(
            any(result.message == "/livox/points has no per-point time field"
                for result in results)
        )

    def test_map_to_odom_tf_can_be_warning_or_failure(self):
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        availability = {
            "base_link <- livox_frame": True,
            "odom <- base_link": True,
            "map <- odom": False,
        }

        loose_config = BringupCheckConfig()
        loose_results = evaluate_snapshot(
            loose_config,
            snapshot(loose_config, cloud, tf_available=availability),
        )
        self.assertIn(WARN, [result.level for result in loose_results])

        strict_config = BringupCheckConfig(require_map_odom_tf=True)
        strict_results = evaluate_snapshot(
            strict_config,
            snapshot(strict_config, cloud, tf_available=availability),
        )
        self.assertIn(FAIL, [result.level for result in strict_results])

    def test_odom_base_tf_can_be_optional_for_standalone_mode(self):
        config = BringupCheckConfig(require_odom_base_tf=False)
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        availability = {
            "base_link <- livox_frame": True,
            "odom <- base_link": False,
            "map <- odom": False,
        }

        results = evaluate_snapshot(config, snapshot(config, cloud, tf_available=availability))

        odom_base_results = [
            result for result in results if result.message == "TF missing: odom <- base_link"
        ]
        self.assertEqual(odom_base_results[0].level, WARN)
        self.assertIn("standalone", odom_base_results[0].hint)

    def test_imu_base_tf_is_checked_only_when_required(self):
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        availability = {
            "base_link <- livox_frame": True,
            "odom <- base_link": True,
            "map <- odom": True,
            "base_link <- livox_imu_frame": False,
        }

        loose_config = BringupCheckConfig(require_imu_base_tf=False)
        loose_results = evaluate_snapshot(
            loose_config,
            snapshot(loose_config, cloud, tf_available=availability),
        )
        self.assertFalse(
            any(result.message == "TF missing: base_link <- livox_imu_frame"
                for result in loose_results)
        )

        strict_config = BringupCheckConfig(require_imu_base_tf=True)
        strict_results = evaluate_snapshot(
            strict_config,
            snapshot(strict_config, cloud, tf_available=availability),
        )
        imu_tf_results = [
            result for result in strict_results
            if result.message == "TF missing: base_link <- livox_imu_frame"
        ]
        self.assertEqual(imu_tf_results[0].level, FAIL)
        self.assertIn("imu_frame_id:=livox_imu_frame", imu_tf_results[0].hint)

    def test_legacy_mid360_cli_accepts_imu_base_tf_arguments(self):
        parser = bringup_cli.build_arg_parser()
        args = parser.parse_args([
            "--imu-frame",
            "imu_link",
            "--require-cloud-time-field",
            "--require-imu-base-tf",
        ])

        config = bringup_cli.config_from_args(args)

        self.assertEqual(config.imu_frame, "imu_link")
        self.assertTrue(config.require_cloud_time_field)
        self.assertTrue(config.require_imu_base_tf)

    def test_generic_cli_accepts_cloud_time_field_requirement(self):
        parser = generic_bringup_cli.build_arg_parser()
        args = parser.parse_args([
            "--profile",
            "mid360",
            "--require-cloud-time-field",
        ])

        config = generic_bringup_cli.config_from_args(args)

        self.assertEqual(config.cloud_topic, "/livox/points")
        self.assertTrue(config.require_cloud_time_field)

    def test_report_lines_include_hints_after_failures(self):
        config = BringupCheckConfig()
        results = evaluate_snapshot(config, snapshot(config, TopicStats()))

        lines = report_lines(config, snapshot(config, TopicStats()), results)

        self.assertIn("[FAIL] no pointcloud received on /livox/points", lines)
        self.assertTrue(any(line.startswith("  hint: ") for line in lines))

    def test_alignment_status_error_is_failure(self):
        config = BringupCheckConfig()
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        status = marked_status("registration_not_converged", level=2)

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        self.assertIn(
            "registration_not_converged",
            [result.message for result in results if result.level == FAIL][-1],
        )

    def test_alignment_status_reports_active_imu_preintegration(self):
        config = BringupCheckConfig(require_imu=True)
        cloud = marked_cloud("livox_frame")
        status = marked_status(
            values={
                "imu_preintegration_status": "imu_preintegration_prediction_active",
                "imu_received_sample_count": "20",
                "imu_integrated_sample_count": "19",
                "imu_skipped_sample_count": "1",
                "registration_seed_source": "imu_preintegration",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        active_results = [
            result for result in results
            if result.message.startswith("IMU preintegration active")
        ]
        self.assertEqual(active_results[0].level, OK)
        self.assertIn("received=20 integrated=19 skipped=1", active_results[0].message)
        self.assertIn("seed_source=imu_preintegration", active_results[0].message)

    def test_alignment_status_reports_imu_tf_failure_with_hint(self):
        config = BringupCheckConfig(require_imu=True, require_imu_base_tf=True)
        cloud = marked_cloud("livox_frame")
        status = marked_status(
            values={
                "imu_preintegration_status": "imu_preintegration_transform_unavailable",
                "imu_received_sample_count": "5",
                "imu_integrated_sample_count": "0",
                "imu_skipped_sample_count": "5",
                "imu_transform_failure_count": "5",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        imu_results = [
            result for result in results
            if result.message.startswith("IMU preintegration cannot transform samples")
        ]
        self.assertEqual(imu_results[0].level, FAIL)
        self.assertIn("base_link <- livox_imu_frame", imu_results[0].hint)
        self.assertIn("imu_frame_id:=livox_imu_frame", imu_results[0].hint)

    def test_alignment_status_reports_invalid_imu_dt(self):
        config = BringupCheckConfig(require_imu=True)
        cloud = marked_cloud("livox_frame")
        status = marked_status(
            values={
                "imu_preintegration_status": "imu_preintegration_invalid_delta_time",
                "imu_received_sample_count": "3",
                "imu_integrated_sample_count": "0",
                "imu_skipped_sample_count": "3",
                "imu_invalid_dt_count": "3",
                "imu_last_dt_sec": "-0.010000",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        imu_results = [
            result for result in results
            if result.message.startswith("IMU preintegration skipped samples")
        ]
        self.assertEqual(imu_results[0].level, FAIL)
        self.assertIn("Last IMU dt was -0.010000 sec", imu_results[0].hint)
        self.assertIn("use_sim_time", imu_results[0].hint)

    def test_alignment_status_reports_disabled_preintegration_when_strict(self):
        config = BringupCheckConfig(require_imu=True)
        cloud = marked_cloud("livox_frame")
        status = marked_status(
            values={"imu_preintegration_status": "imu_preintegration_disabled"}
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        disabled_results = [
            result for result in results
            if result.message == "IMU preintegration is disabled"
        ]
        self.assertEqual(disabled_results[0].level, FAIL)

    def test_alignment_status_reports_scan_time_ready(self):
        config = BringupCheckConfig(require_cloud_time_field=True)
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "offset_time"))
        status = marked_status(
            values={
                "scan_time_status": "scan_time_range_ready",
                "scan_time_field": "offset_time",
                "scan_time_duration_sec": "0.095000",
                "scan_time_valid_point_count": "120",
                "scan_time_invalid_point_count": "3",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        scan_time_results = [
            result for result in results
            if result.message.startswith("per-point scan time ready")
        ]
        self.assertEqual(scan_time_results[0].level, OK)
        self.assertIn("field=offset_time", scan_time_results[0].message)
        self.assertIn("valid_points=120 invalid_points=3", scan_time_results[0].message)

    def test_alignment_status_reports_missing_scan_time_as_strict_failure(self):
        config = BringupCheckConfig(require_cloud_time_field=True)
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "offset_time"))
        status = marked_status(values={"scan_time_status": "scan_time_field_missing"})

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        scan_time_results = [
            result for result in results
            if result.message == "per-point scan time field is missing"
        ]
        self.assertEqual(scan_time_results[0].level, FAIL)
        self.assertIn("offset_time", scan_time_results[0].hint)
        self.assertIn("--require-cloud-time-field", scan_time_results[0].hint)

    def test_alignment_status_reports_invalid_scan_time_as_optional_warning(self):
        config = BringupCheckConfig(require_cloud_time_field=False)
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "time"))
        status = marked_status(
            values={
                "scan_time_status": "scan_time_field_invalid",
                "scan_time_field": "time",
                "scan_time_valid_point_count": "0",
                "scan_time_invalid_point_count": "50",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        scan_time_results = [
            result for result in results
            if result.message.startswith("per-point scan time field is invalid")
        ]
        self.assertEqual(scan_time_results[0].level, WARN)
        self.assertIn("field=time", scan_time_results[0].message)
        self.assertIn("finite per-point times", scan_time_results[0].hint)

    def test_alignment_status_reports_scan_time_range_too_large(self):
        config = BringupCheckConfig(require_cloud_time_field=True)
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "timestamp"))
        status = marked_status(
            values={
                "scan_time_status": "scan_time_range_too_large",
                "scan_time_field": "timestamp",
                "scan_time_duration_sec": "100000000.000000",
                "scan_time_valid_point_count": "80",
                "scan_time_invalid_point_count": "0",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        scan_time_results = [
            result for result in results
            if result.message.startswith("per-point scan time range is too large")
        ]
        self.assertEqual(scan_time_results[0].level, FAIL)
        self.assertIn("time units", scan_time_results[0].hint)
        self.assertIn("nanoseconds", scan_time_results[0].hint)

    def test_alignment_status_reports_deskew_readiness(self):
        config = BringupCheckConfig(require_cloud_time_field=True, require_imu=True)
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "offset_time"))
        status = marked_status(values={"deskew_readiness_status": "deskew_ready"})

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        deskew_results = [
            result for result in results
            if result.message == "continuous-time deskew inputs are ready"
        ]
        self.assertEqual(deskew_results[0].level, OK)

    def test_alignment_status_reports_deskew_blocker_as_strict_failure(self):
        config = BringupCheckConfig(
            require_cloud_time_field=True,
            require_imu=True,
            require_imu_base_tf=True,
        )
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "offset_time"))
        status = marked_status(
            values={"deskew_readiness_status": "deskew_imu_transform_unavailable"}
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        deskew_results = [
            result for result in results
            if result.message == (
                "continuous-time deskew inputs are not ready: "
                "deskew_imu_transform_unavailable"
            )
        ]
        self.assertEqual(deskew_results[0].level, FAIL)
        self.assertIn("base_link <- livox_imu_frame", deskew_results[0].hint)

    def test_alignment_status_reports_optional_deskew_blocker_as_warning(self):
        config = BringupCheckConfig(require_cloud_time_field=False, require_imu=False)
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z"))
        status = marked_status(
            values={"deskew_readiness_status": "deskew_scan_time_field_missing"}
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        deskew_results = [
            result for result in results
            if result.message == (
                "continuous-time deskew inputs are not ready: "
                "deskew_scan_time_field_missing"
            )
        ]
        self.assertEqual(deskew_results[0].level, WARN)
        self.assertIn("per-point timing", deskew_results[0].hint)

    def test_alignment_status_reports_continuous_time_deskew_applied(self):
        config = BringupCheckConfig(require_cloud_time_field=True, require_imu=True)
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "offset_time"))
        status = marked_status(
            values={
                "continuous_time_deskew_status": "continuous_time_deskew_applied",
                "continuous_time_deskew_applied": "true",
                "continuous_time_deskew_point_count": "120",
                "continuous_time_deskew_skipped_invalid_time_count": "1",
                "continuous_time_deskew_clamped_time_count": "2",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        deskew_results = [
            result for result in results
            if result.message.startswith("continuous-time deskew applied")
        ]
        self.assertEqual(deskew_results[0].level, OK)
        self.assertIn("points=120 skipped_invalid_time=1 clamped_time=2", deskew_results[0].message)

    def test_alignment_status_reports_continuous_time_deskew_skip_as_strict_failure(self):
        config = BringupCheckConfig(
            require_cloud_time_field=True,
            require_imu=True,
            require_imu_base_tf=True,
        )
        cloud = marked_cloud("livox_frame", fields=("x", "y", "z", "offset_time"))
        status = marked_status(
            values={
                "continuous_time_deskew_status": "continuous_time_deskew_waiting_for_new_imu",
                "continuous_time_deskew_applied": "false",
            }
        )

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        deskew_results = [
            result for result in results
            if result.message == (
                "continuous-time deskew was not applied: "
                "continuous_time_deskew_waiting_for_new_imu"
            )
        ]
        self.assertEqual(deskew_results[0].level, FAIL)
        self.assertIn("imu_preintegration_status", deskew_results[0].hint)

    def test_topic_rate_is_deterministic(self):
        stats = marked_topic("livox_frame", count=3)

        self.assertAlmostEqual(stats.hz(), 1.0)
        self.assertIn("1.0 Hz", topic_summary("/livox/points", stats))

    def test_cloud_topic_summary_includes_point_fields(self):
        stats = marked_cloud(fields=("x", "y", "z", "offset_time"))

        self.assertIn("fields=x,y,z,offset_time", topic_summary("/livox/points", stats))

    def test_status_topic_summary_includes_imu_preintegration_status(self):
        stats = marked_status(
            values={
                "imu_preintegration_status": "imu_preintegration_prediction_active",
                "registration_seed_source": "imu_preintegration",
                "scan_time_status": "scan_time_range_ready",
                "deskew_readiness_status": "deskew_ready",
                "continuous_time_deskew_status": "continuous_time_deskew_applied",
            }
        )

        self.assertIn(
            "imu_preintegration_status=imu_preintegration_prediction_active",
            topic_summary("/alignment_status", stats),
        )
        self.assertIn(
            "registration_seed_source=imu_preintegration",
            topic_summary("/alignment_status", stats),
        )
        self.assertIn(
            "scan_time_status=scan_time_range_ready",
            topic_summary("/alignment_status", stats),
        )
        self.assertIn(
            "deskew_readiness_status=deskew_ready",
            topic_summary("/alignment_status", stats),
        )
        self.assertIn(
            "continuous_time_deskew_status=continuous_time_deskew_applied",
            topic_summary("/alignment_status", stats),
        )


if __name__ == "__main__":
    unittest.main()
