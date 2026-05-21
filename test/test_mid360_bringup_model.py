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
from lidar_localization_mid360.bringup_model import topic_summary


def marked_topic(frame_id="base_link", count=1):
    topic = TopicStats()
    for index in range(count):
        topic.mark(frame_id, float(index), float(index))
    return topic


def snapshot(config, cloud, imu=None, pose=None, status=None, tf_available=None):
    return BringupSnapshot(
        cloud=cloud,
        imu=imu if imu is not None else TopicStats(),
        pose=pose if pose is not None else TopicStats(),
        status=status if status is not None else TopicStats(),
        tf_checks=build_tf_checks(config, tf_available or {}),
    )


class TestMid360BringupModel(unittest.TestCase):
    def test_missing_cloud_is_hard_failure(self):
        config = BringupCheckConfig()
        results = evaluate_snapshot(config, snapshot(config, TopicStats()))

        self.assertEqual(results[0].level, FAIL)
        self.assertIn("/livox/points", results[0].message)
        self.assertEqual(exit_code(results), 1)

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

    def test_alignment_status_error_is_failure(self):
        config = BringupCheckConfig()
        cloud = marked_topic("livox_frame")
        cloud.has_xyz_fields = True
        status = marked_topic("base_link")
        status.last_status_level = 2
        status.last_status_message = "registration_not_converged"

        results = evaluate_snapshot(config, snapshot(config, cloud, status=status))

        self.assertIn(
            "registration_not_converged",
            [result.message for result in results if result.level == FAIL][-1],
        )

    def test_topic_rate_is_deterministic(self):
        stats = marked_topic("livox_frame", count=3)

        self.assertAlmostEqual(stats.hz(), 1.0)
        self.assertIn("1.0 Hz", topic_summary("/livox/points", stats))


if __name__ == "__main__":
    unittest.main()
