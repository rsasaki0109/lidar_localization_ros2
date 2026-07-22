#!/usr/bin/env python3

import importlib.util
import sys
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "quickstart_model", ROOT / "scripts" / "quickstart_model.py")
MODEL = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODEL
SPEC.loader.exec_module(MODEL)


class TestPoseStore(unittest.TestCase):
    def _pose(self, identity):
        return MODEL.StoredPose(
            schema_version=MODEL.POSE_STATE_SCHEMA,
            map_identity=identity,
            frame_id="map",
            stamp_sec=12.5,
            saved_at_sec=100.0,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            covariance=tuple([0.0] * 36),
        )

    def test_map_identity_uses_contents_not_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "first.pcd"
            second = root / "second.pcd"
            first.write_bytes(b"same map")
            second.write_bytes(b"same map")
            self.assertEqual(
                MODEL.compute_map_identity(first), MODEL.compute_map_identity(second))
            second.write_bytes(b"different map")
            self.assertNotEqual(
                MODEL.compute_map_identity(first), MODEL.compute_map_identity(second))

    def test_round_trip_and_map_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "map.pcd"
            state_path = root / "state" / "pose.json"
            map_path.write_bytes(b"map")
            identity = MODEL.compute_map_identity(map_path)
            MODEL.save_stored_pose(state_path, self._pose(identity))

            loaded = MODEL.load_stored_pose(
                state_path, identity, "map", now_sec=120.0, max_age_sec=30.0)
            self.assertEqual(loaded.reason, "ok")
            self.assertEqual(loaded.pose.position, (1.0, 2.0, 3.0))

            mismatch = MODEL.load_stored_pose(
                state_path,
                MODEL.MapIdentity(3, "f" * 64),
                "map",
                now_sec=120.0,
            )
            self.assertEqual(mismatch.reason, "map_mismatch")

    def test_expired_and_malformed_pose_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            identity = MODEL.MapIdentity(3, "a" * 64)
            state_path = root / "pose.json"
            MODEL.save_stored_pose(state_path, self._pose(identity))
            expired = MODEL.load_stored_pose(
                state_path, identity, "map", now_sec=200.0, max_age_sec=20.0)
            self.assertEqual(expired.reason, "expired")
            state_path.write_text("not json", encoding="utf-8")
            malformed = MODEL.load_stored_pose(
                state_path, identity, "map", now_sec=time.time())
            self.assertEqual(malformed.reason, "invalid_file")

    def test_saved_pose_score_must_converge_and_be_finite(self):
        self.assertTrue(MODEL.saved_pose_score_acceptable(True, 0.5, 1.0))
        self.assertFalse(MODEL.saved_pose_score_acceptable(False, 0.5, 1.0))
        self.assertFalse(MODEL.saved_pose_score_acceptable(True, float("nan"), 1.0))
        self.assertFalse(MODEL.saved_pose_score_acceptable(True, 2.0, 1.0))

    def test_global_registration_scoring_requirement_fails_closed(self):
        self.assertTrue(MODEL.global_registration_scoring_acceptable(True, True))
        self.assertFalse(MODEL.global_registration_scoring_acceptable(False, True))
        self.assertFalse(MODEL.global_registration_scoring_acceptable(None, True))
        self.assertTrue(MODEL.global_registration_scoring_acceptable(False, False))


class TestDiscovery(unittest.TestCase):
    def test_single_topic_is_detected(self):
        selected, reason = MODEL.select_discovered_topic(
            [("/livox/points", "sensor_msgs/msg/PointCloud2")],
            "sensor_msgs/msg/PointCloud2",
            "/velodyne_points",
        )
        self.assertEqual((selected, reason), ("/livox/points", "single_detected"))

    def test_ambiguous_topics_do_not_guess(self):
        selected, reason = MODEL.select_discovered_topic(
            [
                ("/front", "sensor_msgs/msg/PointCloud2"),
                ("/rear", "sensor_msgs/msg/PointCloud2"),
            ],
            "sensor_msgs/msg/PointCloud2",
            "/velodyne_points",
        )
        self.assertEqual((selected, reason), ("/velodyne_points", "ambiguous"))


class TestStartupPolicy(unittest.TestCase):
    def setUp(self):
        self.params = MODEL.StartupParams(
            verification_samples=2,
            max_global_attempts=6,
            min_score_margin=0.05,
        )

    def obs(self, now, **overrides):
        values = {
            "now_sec": now,
            "scan_ready": True,
            "saved_pose_available": False,
            "global_available": True,
        }
        values.update(overrides)
        return MODEL.StartupObservation(**values)

    def test_invalid_safety_parameters_are_rejected(self):
        self.assertIsNone(MODEL.validate_startup_params(self.params))
        self.assertIn(
            "between 0 and 1",
            MODEL.validate_startup_params(MODEL.StartupParams(min_candidate_score=2.0)),
        )
        self.assertIn(
            "positive",
            MODEL.validate_startup_params(MODEL.StartupParams(global_consensus_samples=0)),
        )

    def test_saved_pose_is_verified_before_active(self):
        state = MODEL.StartupState()
        decision = MODEL.decide_startup(
            self.params, state, self.obs(0.0, saved_pose_available=True))
        self.assertEqual(decision.action, MODEL.ACTION_PUBLISH_SAVED)
        state = decision.state
        decision = MODEL.decide_startup(
            self.params,
            state,
            self.obs(1.0, diagnostic_fresh=True, tracking_good=True, fitness=0.5),
        )
        self.assertEqual(decision.action, MODEL.ACTION_WAIT)
        decision = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(2.0, diagnostic_fresh=True, tracking_good=True, fitness=0.4),
        )
        self.assertEqual(decision.action, MODEL.ACTION_ACTIVE)
        self.assertEqual(decision.reason, "saved_pose_verified")

    def test_explicit_pose_is_monitored_without_republication_or_operator_error(self):
        decision = MODEL.decide_startup(
            self.params,
            MODEL.StartupState(),
            self.obs(0.0, preconfigured_pose_available=True),
        )
        self.assertEqual(decision.action, MODEL.ACTION_WAIT)
        self.assertEqual(decision.reason, "verifying_explicit_pose")
        self.assertEqual(decision.state.source, "explicit")

    def test_failed_saved_pose_falls_back_to_global(self):
        decision = MODEL.decide_startup(
            self.params,
            MODEL.StartupState(),
            self.obs(0.0, saved_pose_available=True),
        )
        decision = MODEL.decide_startup(
            self.params, decision.state, self.obs(9.0))
        self.assertEqual(decision.action, MODEL.ACTION_QUERY_GLOBAL)

    def test_global_candidate_needs_score_margin_freshness_and_confirmation(self):
        decision = MODEL.decide_startup(
            self.params, MODEL.StartupState(), self.obs(0.0))
        self.assertEqual(decision.action, MODEL.ACTION_QUERY_GLOBAL)

        ambiguous = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(
                1.0,
                query_candidate_scores=(0.80, 0.78),
                query_candidate_age_sec=0.2,
            ),
        )
        self.assertEqual(ambiguous.action, MODEL.ACTION_QUERY_GLOBAL)
        accepted = MODEL.decide_startup(
            self.params,
            ambiguous.state,
            self.obs(
                2.0,
                query_candidate_scores=(0.90, 0.70),
                query_candidate_age_sec=0.2,
                query_top_pose=(1.0, 2.0, 0.1),
                query_scan_stamp_sec=10.0,
            ),
        )
        self.assertEqual(accepted.action, MODEL.ACTION_QUERY_GLOBAL)
        self.assertEqual(accepted.reason, "global_consensus_primed")
        accepted = MODEL.decide_startup(
            self.params,
            accepted.state,
            self.obs(
                3.0,
                query_candidate_scores=(0.91, 0.68),
                query_candidate_age_sec=0.1,
                query_top_pose=(1.2, 2.1, 0.12),
                query_scan_stamp_sec=10.1,
            ),
        )
        self.assertEqual(accepted.action, MODEL.ACTION_PUBLISH_GLOBAL)

    def test_no_source_never_falls_back_to_identity(self):
        decision = MODEL.decide_startup(
            self.params,
            MODEL.StartupState(),
            self.obs(0.0, global_available=False),
        )
        self.assertEqual(decision.action, MODEL.ACTION_NEEDS_OPERATOR)
        self.assertEqual(decision.reason, "no_safe_automatic_source")

        decision = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(
                1.0,
                global_available=False,
                diagnostic_fresh=True,
                tracking_good=True,
                fitness=0.4,
            ),
        )
        decision = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(
                2.0,
                global_available=False,
                diagnostic_fresh=True,
                tracking_good=True,
                fitness=0.4,
            ),
        )
        self.assertEqual(decision.action, MODEL.ACTION_ACTIVE)
        self.assertEqual(decision.reason, "manual_pose_verified")

    def test_weak_candidates_exhaust_to_operator(self):
        params = MODEL.StartupParams(max_global_attempts=2)
        decision = MODEL.decide_startup(
            params, MODEL.StartupState(), self.obs(0.0))
        decision = MODEL.decide_startup(
            params,
            decision.state,
            self.obs(1.0, query_candidate_scores=(0.2,), query_candidate_age_sec=0.1),
        )
        self.assertEqual(decision.action, MODEL.ACTION_QUERY_GLOBAL)
        decision = MODEL.decide_startup(
            params,
            decision.state,
            self.obs(2.0, query_candidate_scores=(0.2,), query_candidate_age_sec=0.1),
        )
        self.assertEqual(decision.action, MODEL.ACTION_NEEDS_OPERATOR)

    def test_consensus_rejects_different_scan_pose(self):
        decision = MODEL.decide_startup(
            self.params, MODEL.StartupState(), self.obs(0.0))
        primed = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(
                1.0,
                query_candidate_scores=(0.9, 0.7),
                query_candidate_age_sec=0.1,
                query_top_pose=(0.0, 0.0, 0.0),
                query_scan_stamp_sec=10.0,
            ),
        )
        mismatch = MODEL.decide_startup(
            self.params,
            primed.state,
            self.obs(
                2.0,
                query_candidate_scores=(0.9, 0.7),
                query_candidate_age_sec=0.1,
                query_top_pose=(20.0, 0.0, 0.0),
                query_scan_stamp_sec=10.1,
            ),
        )
        self.assertEqual(mismatch.action, MODEL.ACTION_QUERY_GLOBAL)
        self.assertEqual(mismatch.reason, "global_consensus_mismatch_retry")

    def test_in_flight_query_timeout_falls_back_without_duplicate_attempt(self):
        decision = MODEL.decide_startup(
            self.params, MODEL.StartupState(), self.obs(0.0))
        timed_out = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(31.0, query_in_flight=True),
        )
        self.assertEqual(timed_out.action, MODEL.ACTION_NEEDS_OPERATOR)
        self.assertEqual(timed_out.reason, "global_query_timeout")
        self.assertEqual(timed_out.state.global_attempts, 1)


if __name__ == "__main__":
    unittest.main()
