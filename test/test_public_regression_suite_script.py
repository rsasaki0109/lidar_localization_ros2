#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


class TestPublicRegressionSuiteScript(unittest.TestCase):
    def test_rosbag_replay_launches_use_sim_time(self):
        source = (
            REPO_ROOT / "scripts" / "run_public_regression_suite.sh"
        ).read_text(encoding="utf-8")
        launch_commands = re.findall(
            r'--system-command "(ros2 launch lidar_localization_ros2 '
            r'lidar_localization\.launch\.py [^"]+)"',
            source,
        )

        self.assertEqual(len(launch_commands), 3)
        for command in launch_commands:
            self.assertIn("use_sim_time:=true", command)
        self.assertNotIn("use_imu_preintegration:=", launch_commands[0])
        self.assertIn("use_imu_preintegration:=false", launch_commands[1])
        self.assertIn("use_imu_preintegration:=true", launch_commands[2])


if __name__ == "__main__":
    unittest.main()
