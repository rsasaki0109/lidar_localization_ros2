#!/usr/bin/env python3
"""Unit tests for the plain-language localization status (no ROS required)."""

import importlib.util
import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "localization_status_model", SCRIPTS_DIR / "localization_status_model.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


status = load_module()

HEALTHY = {"failure_category": "healthy", "reinitialization_requested": "false"}
BAD_MATCH = {
    "failure_category": "bad_match",
    "fitness_score": "4.500000",
    "effective_score_threshold": "2.000000",
}
STALE = {"failure_category": "stale_prediction", "accepted_gap_sec": "7.2"}


class TestAlignmentStatus(unittest.TestCase):
    def test_healthy_is_tracking(self):
        result = status.alignment_status(HEALTHY, "set 2D Pose Estimate in RViz")
        self.assertEqual(result.phase, status.PHASE_TRACKING)
        self.assertEqual(result.severity, status.SEVERITY_INFO)

    def test_bad_match_names_the_numbers_and_the_fix(self):
        result = status.alignment_status(BAD_MATCH, "set 2D Pose Estimate in RViz")
        self.assertEqual(result.phase, status.PHASE_DEGRADED)
        self.assertIn("fitness 4.50 > 2.00", result.line)
        self.assertIn("set 2D Pose Estimate in RViz", result.line)

    def test_stale_prediction_is_lost(self):
        result = status.alignment_status(STALE, "publish /initialpose (RViz is off)")
        self.assertEqual(result.phase, status.PHASE_LOST)
        self.assertEqual(result.severity, status.SEVERITY_ERROR)
        self.assertIn("for 7 s", result.line)
        self.assertIn("publish /initialpose", result.line)
        self.assertNotIn("Automatic recovery", result.line)

    def test_lost_mentions_automatic_recovery_when_enabled(self):
        result = status.alignment_status(STALE, "set 2D Pose Estimate in RViz", True)
        self.assertIn("Automatic recovery is searching the map", result.line)

    def test_long_rejection_streak_is_lost_even_as_bad_match(self):
        short = dict(BAD_MATCH, accepted_gap_sec="3.0")
        self.assertEqual(
            status.alignment_status(short, "x").phase, status.PHASE_DEGRADED
        )
        long = dict(BAD_MATCH, accepted_gap_sec="6.0")
        result = status.alignment_status(long, "x")
        self.assertEqual(result.phase, status.PHASE_LOST)
        self.assertIn("for 6 s", result.line)

    def test_empty_scan_names_the_covered_sensor(self):
        values = {"failure_category": "bad_match", "message": "filtered_scan_empty"}
        result = status.alignment_status(values, "x")
        self.assertEqual(result.phase, status.PHASE_DEGRADED)
        self.assertIn("No usable scan points", result.line)
        self.assertIn("covered", result.line)

    def test_empty_scans_without_gap_turn_lost_from_last_good_sample(self):
        tracker = status.StatusTracker(debounce_sec=2.0, lost_after_sec=5.0)
        tracker.update_startup({"state": "active"}, 0.0)
        tracker.update_alignment(HEALTHY, 0.0)
        empty = {"failure_category": "bad_match", "message": "filtered_scan_empty"}
        changes = []
        for t in range(1, 12):
            changes += tracker.update_alignment(empty, float(t))
        self.assertEqual(
            [c.phase for c in changes], [status.PHASE_DEGRADED, status.PHASE_LOST]
        )
        self.assertIn("for 7 s", changes[1].line)

    def test_reinitialization_request_is_lost(self):
        values = dict(HEALTHY, reinitialization_requested="true")
        result = status.alignment_status(values, "x")
        self.assertEqual(result.phase, status.PHASE_LOST)

    def test_weak_overlap_reports_point_count(self):
        values = {"failure_category": "weak_overlap", "filtered_point_count": "42"}
        result = status.alignment_status(values, "x")
        self.assertEqual(result.phase, status.PHASE_DEGRADED)
        self.assertIn("only 42 scan points", result.line)

    def test_overload_points_at_cpu_settings(self):
        values = {"failure_category": "overload", "alignment_time_sec": "0.45"}
        result = status.alignment_status(values, "x")
        self.assertIn("0.45 s per scan", result.line)
        self.assertIn("voxel_leaf_size", result.line)

    def test_missing_initial_pose_asks_for_pose(self):
        values = {"failure_category": "missing_initial_pose"}
        result = status.alignment_status(values, "set 2D Pose Estimate in RViz")
        self.assertEqual(result.phase, status.PHASE_NEEDS_POSE)
        self.assertIn("set 2D Pose Estimate in RViz", result.line)

    def test_unparsable_numbers_do_not_crash(self):
        values = {"failure_category": "bad_match", "fitness_score": "nan?"}
        result = status.alignment_status(values, "x")
        self.assertNotIn("fitness", result.line.split(".")[0])


class TestStartupStatus(unittest.TestCase):
    def test_active_is_handed_over(self):
        self.assertIsNone(status.startup_status({"state": "active"}))

    def test_search_label_counts_tries(self):
        result = status.startup_status(
            {"state": "querying_global", "global_attempts": 2}
        )
        self.assertEqual(result.phase, status.PHASE_STARTING)
        self.assertEqual(result.label, "Searching the map (try 2)")

    def test_needs_operator_asks_for_pose(self):
        result = status.startup_status({"state": "needs_operator"})
        self.assertEqual(result.phase, status.PHASE_NEEDS_POSE)
        self.assertIn("2D Pose Estimate", result.label)


class TestStatusTracker(unittest.TestCase):
    def make(self):
        tracker = status.StatusTracker(debounce_sec=2.0, no_scan_timeout_sec=5.0)
        tracker.update_startup({"state": "active"}, 0.0)
        return tracker

    def test_silent_while_startup_runs(self):
        tracker = status.StatusTracker()
        tracker.update_startup({"state": "querying_global"}, 0.0)
        self.assertEqual(tracker.update_alignment(BAD_MATCH, 1.0), [])
        self.assertEqual(tracker.tick(30.0), [])
        self.assertEqual(tracker.display().label, "Searching the map")

    def test_first_status_is_reported_at_once(self):
        tracker = self.make()
        changes = tracker.update_alignment(HEALTHY, 0.1)
        self.assertEqual([c.line for c in changes], ["Tracking."])
        self.assertEqual(tracker.update_alignment(HEALTHY, 0.2), [])

    def test_single_rejected_scan_is_not_reported(self):
        tracker = self.make()
        tracker.update_alignment(HEALTHY, 0.0)
        self.assertEqual(tracker.update_alignment(BAD_MATCH, 1.0), [])
        self.assertEqual(tracker.update_alignment(HEALTHY, 1.1), [])
        self.assertEqual(tracker.update_alignment(BAD_MATCH, 2.5), [])
        self.assertEqual(tracker.display().phase, status.PHASE_TRACKING)

    def test_lasting_problem_is_reported_then_recovery(self):
        tracker = self.make()
        tracker.update_alignment(HEALTHY, 0.0)
        tracker.update_alignment(STALE, 1.0)
        changes = tracker.update_alignment(STALE, 3.0)
        self.assertEqual([c.phase for c in changes], [status.PHASE_LOST])
        self.assertEqual(tracker.update_alignment(STALE, 4.0), [])
        tracker.update_alignment(HEALTHY, 10.0)
        changes = tracker.update_alignment(HEALTHY, 12.0)
        self.assertEqual(changes[0].line, "Tracking again after 9 s.")

    def test_pending_change_commits_on_tick(self):
        tracker = self.make()
        tracker.update_alignment(HEALTHY, 0.0)
        tracker.update_alignment(BAD_MATCH, 1.0)
        changes = tracker.tick(3.5)
        self.assertEqual([c.phase for c in changes], [status.PHASE_DEGRADED])

    def test_no_scans_reported_once(self):
        tracker = self.make()
        tracker.update_alignment(HEALTHY, 0.0)
        self.assertEqual(tracker.tick(4.0), [])
        changes = tracker.tick(6.0)
        self.assertEqual([c.phase for c in changes], [status.PHASE_NO_SCANS])
        self.assertIn("No scan processed for 6 s", changes[0].line)
        self.assertEqual(tracker.tick(20.0), [])

    def test_no_scans_counts_from_startup_when_none_ever_arrived(self):
        tracker = self.make()
        changes = tracker.tick(5.5)
        self.assertEqual([c.phase for c in changes], [status.PHASE_NO_SCANS])

    def test_works_without_startup_node(self):
        tracker = status.StatusTracker()
        changes = tracker.update_alignment(HEALTHY, 0.0)
        self.assertEqual([c.phase for c in changes], [status.PHASE_TRACKING])


if __name__ == "__main__":
    unittest.main()
