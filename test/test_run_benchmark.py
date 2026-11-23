#!/usr/bin/env python3

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "benchmark"))

import run_benchmark as rb


class FakeProcess:
    def __init__(self, polls_until_exit=None, returncode=0):
        self.polls = 0
        self.polls_until_exit = polls_until_exit
        self.returncode = returncode

    def poll(self):
        self.polls += 1
        if self.polls_until_exit is not None and self.polls > self.polls_until_exit:
            return self.returncode
        return None


CONFIG = """lidar_localization:
  ros__parameters:
    base_frame_id: "livox_frame"
    use_odom_tf_prediction: {}
"""


class TestRunBenchmark(unittest.TestCase):
    def test_odometry_is_expected_when_the_suite_launches_one(self):
        self.assertTrue(rb.expects_odometry({"odometry_launch": "pkg odom.launch.py"}))
        self.assertFalse(rb.expects_odometry({}))
        self.assertFalse(
            rb.expects_odometry(
                {"odometry_launch": "pkg odom.launch.py", "expect_odometry": False}
            )
        )

    def test_odometry_settings_are_passed_unless_the_case_sets_them(self):
        self.assertEqual(
            rb.odometry_quickstart_args(["--global-z-min=0.0"], "livox_frame"),
            [
                "--global-z-min=0.0",
                "--odom-tf-prediction",
                "--base-frame=livox_frame",
            ],
        )
        self.assertEqual(
            rb.odometry_quickstart_args(
                ["--no-odom-tf-prediction", "--base-frame", "base_link"], "livox_frame"
            ),
            ["--no-odom-tf-prediction", "--base-frame", "base_link"],
        )

    def test_the_pose_topic_follows_the_quickstart_profile(self):
        self.assertEqual(rb.pose_topic({"quickstart_args": []}), "/pcl_pose")
        self.assertEqual(
            rb.pose_topic({"quickstart_args": ["--profile=mid360"]}),
            "/localization/pose_with_covariance",
        )
        self.assertEqual(
            rb.pose_topic({"quickstart_args": ["--profile", "nav2"]}),
            "/localization/pose_with_covariance",
        )
        self.assertEqual(
            rb.pose_topic({"quickstart_args": ["--profile", "standalone"]}),
            "/pcl_pose",
        )
        self.assertEqual(
            rb.pose_topic(
                {"quickstart_args": ["--profile=mid360"], "pose_topic": "/pose"}
            ),
            "/pose",
        )

    def test_quickstart_configuration_reports_odometry(self):
        self.assertTrue(rb.quickstart_uses_odometry(CONFIG.format("true")))
        self.assertFalse(rb.quickstart_uses_odometry(CONFIG.format("false")))
        self.assertIsNone(rb.quickstart_uses_odometry("a: {b: 1}\n"))
        self.assertIsNone(rb.quickstart_uses_odometry("a: [unclosed\n"))

    def _supervise(self, directory, config_text, quickstart, expect=True):
        config = Path(directory) / "quickstart.yaml"
        if config_text is not None:
            config.write_text(config_text, encoding="utf-8")
        with patch.object(rb.time, "sleep"):
            return rb.supervise(
                FakeProcess(polls_until_exit=3), quickstart, config, expect, 60.0
            )

    def test_a_run_without_the_expected_odometry_is_invalid(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertIsNone(
                self._supervise(directory, CONFIG.format("true"), FakeProcess())
            )
            self.assertIn(
                "use_odom_tf_prediction: false",
                self._supervise(directory, CONFIG.format("false"), FakeProcess()),
            )
        with tempfile.TemporaryDirectory() as directory:
            self.assertIn(
                "exited with 2",
                self._supervise(directory, None, FakeProcess(0, returncode=2)),
            )
            self.assertIn(
                "no configuration", self._supervise(directory, None, FakeProcess())
            )
            # Without odometry expected, nothing is checked.
            self.assertIsNone(
                self._supervise(directory, None, FakeProcess(), expect=False)
            )

    def test_odometry_wait_returns_the_child_frame_or_the_reason(self):
        done = subprocess.CompletedProcess([], 0, stdout="livox_frame\n", stderr="")
        with patch.object(rb.subprocess, "run", return_value=done) as run:
            self.assertEqual(
                rb.wait_for_odometry_tf({"odometry_tf_timeout_sec": 5}, {}),
                ("livox_frame", ""),
            )
        self.assertEqual(run.call_args.args[0][-1], "5.0")
        timeout = subprocess.CompletedProcess(
            [], 1, stdout="", stderr="no odom TF on /tf within 30 s"
        )
        with patch.object(rb.subprocess, "run", return_value=timeout):
            self.assertEqual(
                rb.wait_for_odometry_tf({}, {}),
                (None, "no odom TF on /tf within 30 s"),
            )

    def test_invalid_runs_are_left_out_and_fail_the_benchmark(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            suite = root / "suite.yaml"
            suite.write_text(
                "cases:\n  - {name: c, bag: b, map: m, gt: g, topics: []}\n",
                encoding="utf-8",
            )
            run_dir = root / "out" / "c" / "run1"
            run_dir.mkdir(parents=True)
            (run_dir / rb.INVALID_FILE).write_text("no odometry: test\n")
            (run_dir / "est.tum").write_text("")
            with patch.object(rb, "score") as score:
                code = rb.main(
                    [str(suite), "--out", str(root / "out"), "--evaluate-only"]
                )
            score.assert_not_called()
            self.assertEqual(code, 1)
            summary = (root / "out" / "summary.md").read_text()
            self.assertIn("c run 1: no odometry: test", summary)


if __name__ == "__main__":
    unittest.main()
