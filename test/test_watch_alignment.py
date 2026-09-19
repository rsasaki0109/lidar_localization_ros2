#!/usr/bin/env python3
"""Unit tests for watch_alignment pure formatting (no ROS required)."""

import importlib.util
import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "watch_alignment", SCRIPTS_DIR / "watch_alignment.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


watch = load_module()


class TestWatchAlignment(unittest.TestCase):
    def test_healthy_line_is_green(self):
        line, color = watch.format_alignment_line(
            "ok",
            {
                "failure_category": "healthy",
                "fitness_score": "0.4",
                "effective_score_threshold": "6.0",
                "consecutive_rejected_updates": "0",
                "seed_translation_since_accept_m": "0.1",
                "seed_yaw_since_accept_deg": "0.5",
                "reinitialization_requested": "false",
            },
        )
        self.assertEqual(color, "32")
        self.assertIn("[healthy]", line)
        self.assertIn("fitness=0.4/6.0", line)
        self.assertIn("tracking OK", line)

    def test_stale_is_red_with_reinit_hint(self):
        line, color = watch.format_alignment_line(
            "fitness_score_over_threshold_rejected",
            {
                "failure_category": "stale_prediction",
                "reinitialization_requested": "true",
            },
        )
        self.assertEqual(color, "31")
        self.assertIn("reinit_requested=true", line)

    def test_unknown_category_has_no_color(self):
        _, color = watch.format_alignment_line("ok", {})
        self.assertEqual(color, "0")

    def test_next_action_table_covers_taxonomy(self):
        for category in (
            "missing_map",
            "missing_initial_pose",
            "weak_overlap",
            "bad_match",
            "overload",
        ):
            with self.subTest(category=category):
                action = watch.suggest_next_action({"failure_category": category}, "x")
                self.assertTrue(action and action != "message=x")

    def test_colorize_respects_no_color(self):
        self.assertEqual(watch.colorize("abc", "31", False), "abc")
        self.assertIn("\033[31m", watch.colorize("abc", "31", True))


if __name__ == "__main__":
    sys.exit(unittest.main())
