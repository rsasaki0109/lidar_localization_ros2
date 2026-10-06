#!/usr/bin/env python3

import importlib.util
import math
import sys
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "quickstart_model", ROOT / "scripts" / "quickstart_model.py"
)
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
                MODEL.compute_map_identity(first), MODEL.compute_map_identity(second)
            )
            second.write_bytes(b"different map")
            self.assertNotEqual(
                MODEL.compute_map_identity(first), MODEL.compute_map_identity(second)
            )

    def test_round_trip_and_map_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "map.pcd"
            state_path = root / "state" / "pose.json"
            map_path.write_bytes(b"map")
            identity = MODEL.compute_map_identity(map_path)
            MODEL.save_stored_pose(state_path, self._pose(identity))

            loaded = MODEL.load_stored_pose(
                state_path, identity, "map", now_sec=120.0, max_age_sec=30.0
            )
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
                state_path, identity, "map", now_sec=200.0, max_age_sec=20.0
            )
            self.assertEqual(expired.reason, "expired")
            state_path.write_text("not json", encoding="utf-8")
            malformed = MODEL.load_stored_pose(
                state_path, identity, "map", now_sec=time.time()
            )
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

    def test_quaternion_from_rpy_matches_zyx_euler(self):
        for roll, pitch, yaw in (
            (math.pi, 0.0, 0.5),
            (0.1, -0.2, 2.0),
            (0.0, 0.0, -1.0),
        ):
            x, y, z, w = MODEL.quaternion_from_rpy(roll, pitch, yaw)
            # Rotate the x axis and compare with the ZYX composition.
            rx = 1 - 2 * (y * y + z * z)
            ry = 2 * (x * y + z * w)
            rz = 2 * (x * z - y * w)
            self.assertAlmostEqual(rx, math.cos(yaw) * math.cos(pitch))
            self.assertAlmostEqual(ry, math.sin(yaw) * math.cos(pitch))
            self.assertAlmostEqual(rz, -math.sin(pitch))
            self.assertAlmostEqual(x * x + y * y + z * z + w * w, 1.0)

    def test_startup_is_described_in_plain_words(self):
        describe = MODEL.describe_startup
        self.assertEqual(
            describe(MODEL.STATE_WAITING_FOR_SCAN, "waiting_for_scan"),
            "Waiting for the first LiDAR scan.",
        )
        self.assertEqual(
            describe(
                MODEL.STATE_QUERYING_GLOBAL, "ambiguous_candidate_retry", "global", 2, 6
            ),
            "Searching the map for the robot (try 2 of 6): the view matches more than "
            "one place; trying again.",
        )
        self.assertEqual(
            describe(MODEL.STATE_VERIFYING, "verifying_pose", "global"),
            "Checking the found pose against the next scans.",
        )
        self.assertEqual(
            describe(MODEL.STATE_ACTIVE, "tracking", "saved"),
            "Localized from the saved pose.",
        )
        self.assertIn(
            "did not match the next scans",
            describe(
                MODEL.STATE_QUERYING_GLOBAL,
                "global_verification_failed",
                "global",
                3,
                6,
            ),
        )
        needs_pose = describe(MODEL.STATE_NEEDS_OPERATOR, "global_attempts_exhausted")
        self.assertIn("no unambiguous match was found", needs_pose)
        self.assertIn("2D Pose Estimate", needs_pose)

    def test_odometry_frame_follows_the_odom_tf_tree(self):
        select = MODEL.select_odometry_frame
        rko = {("odom", "livox_frame")}
        # RKO-LIO's single odom child becomes the base frame.
        self.assertEqual(select(rko, "odom", None), ("livox_frame", True))
        # An explicit base frame is kept, and must hang below odom.
        self.assertEqual(select(rko, "odom", "base_link"), ("base_link", False))
        chain = {
            ("map", "odom"),
            ("odom", "base_footprint"),
            ("base_footprint", "base_link"),
            ("base_link", "velodyne"),
        }
        self.assertEqual(select(chain, "odom", None), ("base_link", True))
        self.assertEqual(select(chain, "odom", "velodyne"), ("velodyne", True))
        # Several odom children are ambiguous; no TF at all means no odometry.
        two = {("odom", "robot_a"), ("odom", "robot_b")}
        self.assertEqual(select(two, "odom", None), ("base_link", False))
        self.assertEqual(select(set(), "odom", None), ("base_link", False))

    def test_lidar_tf_is_published_only_when_nothing_links_the_frames(self):
        needed = MODEL.lidar_tf_needed
        robot = {("odom", "base_link"), ("base_link", "velodyne")}
        self.assertFalse(needed(robot, "base_link", "velodyne"))
        self.assertFalse(needed(robot, "velodyne", "base_link"))
        self.assertTrue(needed(robot, "base_link", "livox_frame"))
        self.assertTrue(needed(set(), "base_link", "velodyne"))
        # RKO-LIO tracks the LiDAR frame itself.
        self.assertFalse(
            needed({("odom", "livox_frame")}, "livox_frame", "livox_frame")
        )


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
            MODEL.validate_startup_params(
                MODEL.StartupParams(global_consensus_samples=0)
            ),
        )

    def test_saved_pose_is_verified_before_active(self):
        state = MODEL.StartupState()
        decision = MODEL.decide_startup(
            self.params, state, self.obs(0.0, saved_pose_available=True)
        )
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
        decision = MODEL.decide_startup(self.params, decision.state, self.obs(9.0))
        self.assertEqual(decision.action, MODEL.ACTION_QUERY_GLOBAL)

    def test_ndt_distinctiveness_replaces_the_bbs_score_margin(self):
        # Go2 in the JEPLO capture room: BBS scores saturate (0.99 vs 0.98) but
        # the top fix registers 18 times better than the best fix elsewhere.
        start = MODEL.decide_startup(self.params, MODEL.StartupState(), self.obs(0.0))
        common = {
            "query_candidate_scores": (0.99, 0.98),
            "query_candidate_age_sec": 0.2,
            "query_top_pose": (1.0, 2.0, 0.1),
            "query_scan_stamp_sec": 10.0,
        }
        judged = MODEL.decide_startup(
            self.params,
            start.state,
            self.obs(
                1.0,
                query_top_registration_fitness=0.017,
                query_alternative_registration_fitness=0.30,
                **common,
            ),
        )
        self.assertEqual(judged.reason, "global_consensus_primed")
        # Without NDT scores to compare, the margin still decides.
        unscored = MODEL.decide_startup(
            self.params, start.state, self.obs(1.0, **common)
        )
        self.assertEqual(unscored.reason, "ambiguous_candidate_retry")
        # And an NDT tie is still ambiguous.
        tie = MODEL.decide_startup(
            self.params,
            start.state,
            self.obs(
                1.0,
                query_top_registration_fitness=0.20,
                query_alternative_registration_fitness=0.30,
                **common,
            ),
        )
        self.assertEqual(tie.reason, "ambiguous_registration_retry")

    def test_global_candidate_needs_score_margin_freshness_and_confirmation(self):
        decision = MODEL.decide_startup(
            self.params, MODEL.StartupState(), self.obs(0.0)
        )
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

    def test_high_confidence_registration_bypasses_ambiguous_margin_and_consensus(self):
        params = MODEL.StartupParams(
            verification_samples=2,
            registration_fitness_high_confidence_threshold=0.5,
        )
        decision = MODEL.decide_startup(params, MODEL.StartupState(), self.obs(0.0))
        decision = MODEL.decide_startup(
            params,
            decision.state,
            self.obs(
                1.0,
                query_candidate_scores=(0.9685, 0.9684),
                query_candidate_age_sec=10.0,
                query_top_pose=(-86.0, -8.0, -1.4),
                query_scan_stamp_sec=100.0,
                query_top_registration_fitness=0.19,
            ),
        )
        self.assertEqual(decision.action, MODEL.ACTION_PUBLISH_GLOBAL)
        self.assertEqual(decision.reason, "global_registration_high_confidence")

    def test_alternative_fitness_ignores_nearby_and_unscored_candidates(self):
        candidates = [
            {"x": 103.9, "y": -2.6, "registration_fitness": 0.05},
            {"x": 106.0, "y": -2.6, "registration_fitness": 0.06},
            {"x": -54.0, "y": -70.0, "registration_fitness": float("inf")},
            {"x": -62.0, "y": -74.0, "registration_fitness": 0.93},
            {"x": -50.0, "y": -70.0},
            {"x": -47.0, "y": -69.0, "registration_fitness": 1.02},
        ]
        self.assertEqual(MODEL.alternative_registration_fitness(candidates, 5.0), 0.93)
        self.assertEqual(MODEL.alternative_registration_fitness(candidates, 0.0), 0.06)
        self.assertIsNone(MODEL.alternative_registration_fitness(candidates[:1], 5.0))

    def test_aliased_registration_is_retried_and_distinct_one_proceeds(self):
        # Koide outdoor: an aliased area scored many similar mediocre poses, while
        # the true pose registered an order of magnitude better than anywhere else.
        start = MODEL.decide_startup(self.params, MODEL.StartupState(), self.obs(0.0))
        query = {
            "query_candidate_age_sec": 0.1,
            "query_top_pose": (-52.7, -82.0, -1.57),
            "query_scan_stamp_sec": 10.0,
        }
        aliased = MODEL.decide_startup(
            self.params,
            start.state,
            self.obs(
                1.0,
                query_candidate_scores=(0.77, 0.70),
                query_top_registration_fitness=1.361,
                query_alternative_registration_fitness=1.806,
                **query,
            ),
        )
        self.assertEqual(aliased.action, MODEL.ACTION_QUERY_GLOBAL)
        self.assertEqual(aliased.reason, "ambiguous_registration_retry")

        distinct = MODEL.decide_startup(
            self.params,
            aliased.state,
            self.obs(
                2.0,
                query_candidate_scores=(0.99, 0.86),
                query_top_registration_fitness=0.0441,
                query_alternative_registration_fitness=0.8228,
                **query,
            ),
        )
        self.assertEqual(distinct.reason, "global_consensus_primed")

        disabled = MODEL.decide_startup(
            replace(self.params, max_registration_fitness_ratio=0.0),
            start.state,
            self.obs(
                1.0,
                query_candidate_scores=(0.77, 0.70),
                query_top_registration_fitness=1.361,
                query_alternative_registration_fitness=1.806,
                **query,
            ),
        )
        self.assertEqual(disabled.reason, "global_consensus_primed")
        self.assertIn(
            "non-negative",
            MODEL.validate_startup_params(
                MODEL.StartupParams(max_registration_fitness_ratio=-1.0)
            ),
        )

    def test_planar_motion_round_trips(self):
        start = (10.0, -2.0, math.radians(30.0))
        end = (14.0, 5.0, math.radians(-120.0))
        motion = MODEL.planar_motion_between(start, end)
        moved = MODEL.apply_planar_motion(start, motion)
        for actual, expected in zip(moved, end, strict=True):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(motion[2], math.radians(-150.0))

    def test_consensus_follows_odometry_while_moving(self):
        # Koide 02b at 1x: correct G2 fixes 45 s apart while the robot walked 50 m,
        # with a wrong fix from an aliased area in between.
        first_gt = (103.6, -1.7, math.radians(4.5))
        last_gt = (103.6, -51.3, math.radians(9.0))
        odom_first = (0.0, 0.0, 0.0)
        odom_last = MODEL.planar_motion_between(first_gt, last_gt)
        odom_middle = MODEL.planar_motion_between(
            first_gt, (104.3, -25.0, math.radians(-40.0))
        )
        query = {
            "query_candidate_scores": (0.99, 0.80),
            "query_candidate_age_sec": 11.0,
        }
        fixes = [
            ((103.1, -2.4, math.radians(5.0)), 4.8, odom_first),
            ((-57.1, -75.6, math.radians(-140.0)), 28.1, odom_middle),
            ((103.1, -51.6, math.radians(10.0)), 50.1, odom_last),
        ]

        def run(with_odometry):
            decision = MODEL.decide_startup(
                self.params, MODEL.StartupState(), self.obs(0.0)
            )
            reasons = []
            for index, (pose, stamp, odom) in enumerate(fixes, start=1):
                decision = MODEL.decide_startup(
                    self.params,
                    decision.state,
                    self.obs(
                        float(index),
                        query_top_pose=pose,
                        query_scan_stamp_sec=stamp,
                        query_odom_pose=odom if with_odometry else None,
                        **query,
                    ),
                )
                reasons.append(decision.reason)
            return decision, reasons

        _, unaided = run(with_odometry=False)
        self.assertEqual(unaided[-1], "global_consensus_mismatch_retry")
        aided, reasons = run(with_odometry=True)
        self.assertEqual(
            reasons,
            [
                "global_consensus_primed",
                "global_consensus_mismatch_retry",
                "global_candidate_accepted",
            ],
        )
        self.assertEqual(aided.action, MODEL.ACTION_PUBLISH_GLOBAL)

    def test_odometry_agreement_confirms_a_weak_later_fix(self):
        # Koide 02b run: the later correct fix scored weak (G2 fitness 5.9) on its own.
        first_gt = (103.6, -1.7, math.radians(4.5))
        later_gt = (103.7, -37.6, math.radians(55.0))
        decision = MODEL.decide_startup(
            self.params, MODEL.StartupState(), self.obs(0.0)
        )
        decision = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(
                1.0,
                query_candidate_scores=(0.99, 0.80),
                query_candidate_age_sec=8.0,
                query_top_pose=(102.9, -2.2, math.radians(5.0)),
                query_scan_stamp_sec=6.3,
                query_odom_pose=(0.0, 0.0, 0.0),
            ),
        )
        self.assertEqual(decision.reason, "global_consensus_primed")
        weak = {
            "query_candidate_scores": (0.30, 0.28),
            "query_candidate_age_sec": 7.0,
            "query_scan_stamp_sec": 37.8,
            "query_odom_pose": MODEL.planar_motion_between(first_gt, later_gt),
        }
        confirmed = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(2.0, query_top_pose=later_gt, **weak),
        )
        self.assertEqual(confirmed.action, MODEL.ACTION_PUBLISH_GLOBAL)
        elsewhere = MODEL.decide_startup(
            self.params,
            decision.state,
            self.obs(2.0, query_top_pose=(-50.3, -79.0, math.radians(-125.0)), **weak),
        )
        self.assertEqual(elsewhere.reason, "weak_candidate_retry")

    def test_odometry_pairs_need_one_gated_fix_from_another_scan(self):
        # Koide 02b run: the fix at 17 s was right but failed the distinctiveness
        # gate; the gated fix at 28 s is confirmed by it once moved by odometry.
        gt_17 = (102.3, -12.4, math.radians(10.0))
        gt_28 = (103.9, -25.4, math.radians(-35.0))
        odom_17 = (0.0, 0.0, 0.0)
        odom_28 = MODEL.planar_motion_between(gt_17, gt_28)
        ungated = {
            "query_candidate_scores": (0.90, 0.80),
            "query_candidate_age_sec": 11.0,
            "query_top_registration_fitness": 1.8,
            "query_alternative_registration_fitness": 2.0,
        }
        gated = {
            "query_candidate_scores": (0.95, 0.70),
            "query_candidate_age_sec": 11.0,
        }
        start = MODEL.decide_startup(self.params, MODEL.StartupState(), self.obs(0.0))
        first = MODEL.decide_startup(
            self.params,
            start.state,
            self.obs(
                1.0,
                query_top_pose=gt_17,
                query_scan_stamp_sec=17.3,
                query_odom_pose=odom_17,
                **ungated,
            ),
        )
        self.assertEqual(first.reason, "ambiguous_registration_retry")
        self.assertFalse(first.state.consensus_odom_anchors[0].gated)

        later = {"query_scan_stamp_sec": 28.5, "query_odom_pose": odom_28}
        confirmed = MODEL.decide_startup(
            self.params,
            first.state,
            self.obs(2.0, query_top_pose=gt_28, **later, **gated),
        )
        self.assertEqual(confirmed.action, MODEL.ACTION_PUBLISH_GLOBAL)
        both_ungated = MODEL.decide_startup(
            self.params,
            first.state,
            self.obs(2.0, query_top_pose=gt_28, **later, **ungated),
        )
        self.assertEqual(both_ungated.reason, "ambiguous_registration_retry")
        same_scan = MODEL.decide_startup(
            self.params,
            first.state,
            self.obs(
                2.0,
                query_top_pose=gt_17,
                query_scan_stamp_sec=17.3,
                query_odom_pose=odom_17,
                **gated,
            ),
        )
        self.assertEqual(same_scan.reason, "global_consensus_mismatch_retry")

    def test_odometry_match_widens_with_distance_travelled(self):
        # Koide 02b run: fixes 47 s and 56 m apart disagreed by 2.36 m after
        # odometry because of the first fix's 5 degree heading quantization.
        first_odom = (0.0, 0.0, 0.0)
        later_odom = (-2.73, -56.02, math.radians(14.0))
        gated = {
            "query_candidate_scores": (0.95, 0.70),
            "query_candidate_age_sec": 10.0,
        }
        start = MODEL.decide_startup(self.params, MODEL.StartupState(), self.obs(0.0))
        primed = MODEL.decide_startup(
            self.params,
            start.state,
            self.obs(
                1.0,
                query_top_pose=(102.9, -2.4, math.radians(5.0)),
                query_scan_stamp_sec=7.3,
                query_odom_pose=first_odom,
                **gated,
            ),
        )
        later = {
            "query_top_pose": (103.9, -56.4, math.radians(15.0)),
            "query_scan_stamp_sec": 54.3,
            "query_odom_pose": later_odom,
            **gated,
        }
        widened = MODEL.decide_startup(
            self.params, primed.state, self.obs(2.0, **later)
        )
        self.assertEqual(widened.action, MODEL.ACTION_PUBLISH_GLOBAL)
        fixed = MODEL.decide_startup(
            replace(self.params, global_consensus_translation_per_odom_m=0.0),
            primed.state,
            self.obs(2.0, **later),
        )
        self.assertEqual(fixed.reason, "global_consensus_mismatch_retry")

    def test_retry_waits_for_a_new_view_with_odometry(self):
        params = MODEL.StartupParams()
        sees = MODEL.retry_sees_new_view
        # Without odometry every retry goes ahead, as before.
        self.assertTrue(sees(params, 0.1, None))
        # Still (or standing up): wait until 2 s have passed.
        self.assertFalse(sees(params, 0.5, (0.05, 0.0, 0.02)))
        self.assertTrue(sees(params, 2.0, (0.05, 0.0, 0.02)))
        # Walking 0.5 m or turning 15 degrees gives a new view sooner.
        self.assertTrue(sees(params, 0.5, (0.4, 0.3, 0.0)))
        self.assertTrue(sees(params, 0.5, (0.0, 0.0, math.radians(-16.0))))
        self.assertIsNone(MODEL.validate_startup_params(params))
        self.assertIsNotNone(
            MODEL.validate_startup_params(
                replace(params, global_retry_max_wait_sec=-1.0)
            )
        )

    def test_travel_refunds_attempts_until_the_query_cap(self):
        params = replace(self.params, max_global_attempts=6, max_global_queries=30)

        def run(step_m):
            decision = MODEL.decide_startup(params, MODEL.StartupState(), self.obs(0.0))
            for n in range(1, 100):
                decision = MODEL.decide_startup(
                    params,
                    decision.state,
                    self.obs(
                        float(n),
                        query_candidate_scores=(0.30, 0.20),
                        query_candidate_age_sec=10.0,
                        query_top_pose=(float(n), 0.0, 0.0),
                        query_scan_stamp_sec=10.0 * n,
                        query_odom_pose=(step_m * n, 0.0, 0.0),
                    ),
                )
                if decision.action == MODEL.ACTION_NEEDS_OPERATOR:
                    return decision
            self.fail("startup never asked for the operator")

        stationary = run(step_m=0.0)
        self.assertEqual(stationary.reason, "global_attempts_exhausted")
        self.assertEqual(stationary.state.global_queries, 6)
        walking = run(step_m=8.0)
        self.assertEqual(walking.reason, "global_queries_exhausted")
        self.assertEqual(walking.state.global_queries, 30)

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
        decision = MODEL.decide_startup(params, MODEL.StartupState(), self.obs(0.0))
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
            self.params, MODEL.StartupState(), self.obs(0.0)
        )
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
            self.params, MODEL.StartupState(), self.obs(0.0)
        )
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
