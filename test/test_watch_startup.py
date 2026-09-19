#!/usr/bin/env python3
"""Unit tests for watch_startup pure formatting (no ROS required)."""

import importlib.util
import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "watch_startup", SCRIPTS_DIR / "watch_startup.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


watch = load_module()


class TestWatchStartup(unittest.TestCase):
    def test_active_is_green(self):
        line, color = watch.format_startup_line(
            {
                "state": "active",
                "source": "saved",
                "global_attempts": 0,
                "reason": "saved_pose_verified",
            }
        )
        self.assertEqual(color, "32")
        self.assertIn("[active]", line)
        self.assertIn("startup complete", line)

    def test_needs_operator_no_source_points_to_rviz(self):
        line, color = watch.format_startup_line(
            {
                "state": "needs_operator",
                "source": "",
                "global_attempts": 6,
                "reason": "no_safe_automatic_source",
            }
        )
        self.assertEqual(color, "31")
        self.assertIn("2D Pose Estimate", line)

    def test_querying_is_yellow_with_stationary_hint(self):
        line, color = watch.format_startup_line(
            {
                "state": "querying_global",
                "source": "global",
                "global_attempts": 1,
                "reason": "query",
            }
        )
        self.assertEqual(color, "33")
        self.assertIn("stationary", line)

    def test_invalid_json_returns_empty(self):
        self.assertEqual(watch.parse_status_json("not json"), {})
        self.assertEqual(watch.parse_status_json(""), {})

    def test_unknown_state_has_no_color(self):
        _, color = watch.format_startup_line({})
        self.assertEqual(color, "0")


if __name__ == "__main__":
    sys.exit(unittest.main())
