#!/usr/bin/env python3
"""ROS integration smoke for the G3 reinitialization supervisor node.

Validates the ROS glue that the pure-policy unit tests cannot: that the node
actually receives /reinitialization_requested + /alignment_status, calls the G2
~/query service, parses the candidate JSON, and publishes /initialpose behind the
guards. Requires a sourced ROS 2 env; skipped otherwise so the pure-Python test
runs are unaffected.

Run: source scripts/setup_local_env.sh && python3 -m pytest \
        test/test_reinitialization_supervisor_node_ros.py -q
"""

import json
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import reinitialization_supervisor_node as rsn
from diagnostic_msgs.msg import (
    DiagnosticArray,
    DiagnosticStatus,
    KeyValue,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

_REL = QoSProfile(depth=1)
_REL.reliability = ReliabilityPolicy.RELIABLE
_POSE_QOS = QoSProfile(depth=10)
_POSE_QOS.reliability = ReliabilityPolicy.RELIABLE
_POSE_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


class _Harness(Node):
    """Fakes the localizer + G2 service and watches /initialpose.

    The G2 reply carries a ranked ``candidates`` list (top is aliased-wrong). If
    ``recover_on_second`` is set, fitness drops to recovered only after the node has
    walked to the second candidate -- exercising the ranked-candidate walk glue.
    """

    def __init__(
        self,
        recover_on_second=False,
        pose_z=None,
        top_candidate=None,
        second_candidate=None,
    ):
        super().__init__("g3_test_harness")
        self.create_service(Trigger, "/global_localization_node/query", self._on_query)
        self._reinit = self.create_publisher(Bool, "/reinitialization_requested", _REL)
        self._status = self.create_publisher(DiagnosticArray, "/alignment_status", 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_pose, _REL
        )
        # Optionally fake the localizer pose output so the supervisor can carry z.
        self._pose_z = pose_z
        self._top_candidate = top_candidate or {
            "x": 12.0,
            "y": 34.0,
            "yaw_deg": 45.0,
            "score": 0.99,
        }
        self._second_candidate = second_candidate or {
            "x": 99.0,
            "y": 88.0,
            "yaw_deg": -90.0,
            "score": 0.98,
        }
        self._pcl_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/pcl_pose", _POSE_QOS
        )
        self.query_calls = 0
        self.recover_on_second = recover_on_second
        self.poses = []
        self.create_timer(0.1, self._drive)

    @property
    def initialpose(self):
        return self.poses[-1] if self.poses else None

    def _on_query(self, request, response):
        self.query_calls += 1
        response.success = True
        response.message = json.dumps(
            {
                "candidate_count": 2,
                "candidates": [
                    self._top_candidate,
                    self._second_candidate,
                ],
            }
        )
        return response

    def _on_pose(self, msg):
        self.poses.append(msg)

    def _drive(self):
        self._reinit.publish(Bool(data=True))
        # Recover only once the node has walked to the 2nd candidate, else stay bad.
        recovered = self.recover_on_second and len(self.poses) >= 2

        def kv(key, value):
            item = KeyValue()
            item.key, item.value = key, value
            return item

        status = DiagnosticStatus()
        status.message = "ok" if recovered else "fitness_score_over_threshold_rejected"
        status.values = [
            kv("fitness_score", "0.2" if recovered else "9.0"),
            kv("reinitialization_requested", "false" if recovered else "true"),
            kv(
                "recovery_state",
                "tracking" if recovered else "reinitialization_requested",
            ),
            kv(
                "recovery_action",
                "accept_measurement" if recovered else "request_reinitialization",
            ),
        ]
        array = DiagnosticArray()
        array.status = [status]
        self._status.publish(array)
        if self._pose_z is not None:
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.pose.pose.position.z = self._pose_z
            pose.pose.pose.orientation.w = 1.0
            self._pcl_pose.publish(pose)


def test_supervisor_node_closes_the_loop():
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    harness = _Harness()
    executor = SingleThreadedExecutor()
    executor.add_node(sup)
    executor.add_node(harness)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and harness.initialpose is None:
            time.sleep(0.1)

        assert harness.query_calls >= 1, "supervisor never queried the G2 service"
        assert harness.initialpose is not None, (
            "supervisor never published /initialpose"
        )
        pose = harness.initialpose.pose.pose
        assert abs(pose.position.x - 12.0) < 1e-3
        assert abs(pose.position.y - 34.0) < 1e-3
        # position covariance reflects the default reset_position_std (0.5 m)^2.
        assert abs(harness.initialpose.pose.covariance[0] - 0.25) < 1e-3
        # Having published a reset, the policy is now awaiting recovery evidence.
        assert sup.state.name == rsn.rsp.STATE_SETTLING
    finally:
        executor.shutdown()
        sup.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_supervisor_node_walks_to_second_candidate_and_recovers():
    # The top candidate is aliased-wrong (fitness stays bad); the node must walk to
    # the second candidate from the same query, at which point the localizer locks.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    # Walk fast and within a single query (max_attempts=1 proves walking does not
    # spend attempts -- a fresh query would have given up).
    sup.params = replace(
        sup.params,
        settle_timeout_sec=2.0,
        request_debounce_sec=0.5,
        min_seconds_between_attempts=1.0,
        max_attempts=1,
        enable_confirm_cross_check=False,
    )
    harness = _Harness(recover_on_second=True)
    executor = SingleThreadedExecutor()
    executor.add_node(sup)
    executor.add_node(harness)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and sup.state.name != rsn.rsp.STATE_STANDDOWN:
            time.sleep(0.1)

        assert len(harness.poses) >= 2, "node never walked to the second candidate"
        first, second = harness.poses[0].pose.pose, harness.poses[1].pose.pose
        assert (
            abs(first.position.x - 12.0) < 1e-3 and abs(first.position.y - 34.0) < 1e-3
        )
        assert (
            abs(second.position.x - 99.0) < 1e-3
            and abs(second.position.y - 88.0) < 1e-3
        )
        # Recovered on the walked candidate, within one query, without giving up.
        assert sup.state.name == rsn.rsp.STATE_STANDDOWN
        assert harness.query_calls == 1
    finally:
        executor.shutdown()
        sup.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_reset_carries_z_from_localizer_pose():
    # A 2D candidate (z=0) on a map whose true z is far from zero seeds the reset
    # outside the registration z-basin. The supervisor must carry z from the last
    # /pcl_pose so the published reset uses the real height, not 0.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    sup.params = replace(sup.params, request_debounce_sec=0.5)
    harness = _Harness(pose_z=-11.05)
    executor = SingleThreadedExecutor()
    executor.add_node(sup)
    executor.add_node(harness)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and harness.initialpose is None:
            time.sleep(0.1)
        assert harness.initialpose is not None, (
            "supervisor never published /initialpose"
        )
        # Candidate carried x/y from the query but z from the localizer pose.
        pose = harness.initialpose.pose.pose
        assert abs(pose.position.x - 12.0) < 1e-3
        assert abs(pose.position.z - (-11.05)) < 1e-2, pose.position.z
    finally:
        executor.shutdown()
        sup.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_reset_uses_registration_verified_candidate_height():
    # The localizer pose may have been bridged on drifting odometry for minutes
    # (Koide outdoor_kidnap_b: +12.5 m); a height G2 verified against the map wins.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    sup.params = replace(sup.params, request_debounce_sec=0.5)
    harness = _Harness(
        pose_z=1.2,
        top_candidate={
            "x": 12.0,
            "y": 34.0,
            "z": -11.3,
            "yaw_deg": 45.0,
            "score": 0.99,
            "registration_fitness": 0.3,
            "registration_converged": True,
        },
    )
    executor = SingleThreadedExecutor()
    executor.add_node(sup)
    executor.add_node(harness)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and harness.initialpose is None:
            time.sleep(0.1)
        assert harness.initialpose is not None, (
            "supervisor never published /initialpose"
        )
        assert abs(harness.initialpose.pose.pose.position.z - (-11.3)) < 1e-3
    finally:
        executor.shutdown()
        sup.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_aliased_answer_is_not_published_as_a_reset():
    # Koide 02b quickstart: while odometry bridged a correct pose, G2 answered from
    # an aliased area (fitness 1.19 vs 1.78 elsewhere) and the walk reset onto it.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    sup.params = replace(sup.params, request_debounce_sec=0.5)
    harness = _Harness(
        top_candidate={
            "x": -59.1,
            "y": -79.8,
            "yaw_deg": 25.0,
            "score": 0.80,
            "registration_fitness": 1.189,
        },
        second_candidate={
            "x": -57.1,
            "y": -70.8,
            "yaw_deg": 100.0,
            "score": 0.79,
            "registration_fitness": 1.784,
        },
    )
    executor = SingleThreadedExecutor()
    executor.add_node(sup)
    executor.add_node(harness)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
        assert harness.query_calls >= 1, "supervisor never queried G2"
        assert harness.initialpose is None, "aliased G2 answer was published"
    finally:
        executor.shutdown()
        sup.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_aliased_verify_reply_does_not_reseed():
    # Koide outdoor_kidnap_b: a correct reset (1.0 m) was verified against an aliased
    # answer (fitness 4.05 vs 4.72 elsewhere); the mismatch reseeded 46 m off.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup.state = replace(sup.state, name=rsn.rsp.STATE_VERIFYING, attempts=1)
        reply = {
            "scan_stamp_sec": 100.0,
            "candidates": [
                {
                    "x": -63.3,
                    "y": -75.6,
                    "yaw_deg": 90.0,
                    "score": 0.32,
                    "registration_fitness": 4.054,
                },
                {
                    "x": -63.5,
                    "y": -73.2,
                    "yaw_deg": 90.0,
                    "score": 0.27,
                    "registration_fitness": 4.377,
                },
                {
                    "x": -57.3,
                    "y": -78.0,
                    "yaw_deg": 100.0,
                    "score": 0.21,
                    "registration_fitness": 4.720,
                },
            ],
        }

        class _Future:
            def result(self):
                return Trigger.Response(success=True, message=json.dumps(reply))

        sup._on_query_response(_Future())

        assert sup._pending_reply == (0.0, 0.0, 0.0), sup._pending_reply
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_odometry_confirmation_needs_a_second_agreeing_answer():
    # Koide 02b quickstart: a single wrong answer (65 m off, ratio 0.30) passed
    # the distinctiveness gate while the odometry bridge held the right pose.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        for stamp in range(0, 40):
            sup._odom_bridge_history.append((float(stamp), 1.0 * stamp, 0.0, 0.0))
        sup._odometry_stamps.extend(0.1 * step for step in range(0, 400))

        def answer(stamp, x, y, yaw_deg=0.0):
            summary = {"scan_stamp_sec": float(stamp)}
            candidates = [{"x": x, "y": y, "yaw_deg": yaw_deg, "score": 0.9}]
            return sup._withhold_unconfirmed_answer(summary, candidates, (0.9,))

        # The bridge drifted: the robot is really 30 m north of the bridged pose.
        assert answer(10, 10.0, 30.0) == (0.0,)
        assert answer(20, -40.0, -70.0) == (0.0,), "unrelated answer accepted"
        assert answer(30, 30.5, 30.4) == (0.9,), "agreeing answer withheld"
        assert answer(35, 35.0, 30.0) == (0.0,), "confirmation was not consumed"
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_odometry_dropout_waives_confirmation():
    # Koide outdoor_kidnap_b: the LIO front end stops for 14-34 s while the sensor is
    # covered, so bridged motion across the gap cannot confirm the next answer.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        for stamp in range(0, 40):
            sup._odom_bridge_history.append((float(stamp), 1.0 * stamp, 0.0, 0.0))
        sup._odometry_stamps.extend(0.1 * step for step in range(0, 100))
        sup._odometry_stamps.extend(30.0 + 0.1 * step for step in range(0, 100))

        summary = {"scan_stamp_sec": 35.0}
        candidates = [{"x": 35.0, "y": 30.0, "yaw_deg": 0.0, "score": 0.9}]
        scores = sup._withhold_unconfirmed_answer(summary, candidates, (0.9,))

        assert scores == (0.9,), "answer after an odometry dropout was withheld"
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_walk_skips_lower_candidates_from_other_places():
    # Koide outdoor_kidnap_b: after the correct top (1.4 m) did not settle, the walk
    # reset onto lower-ranked candidates 35 m and 136 m away.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        candidates = [
            {"x": 50.0, "y": -70.0, "yaw_deg": -80.0, "registration_fitness": 0.20},
            {"x": 52.0, "y": -71.0, "yaw_deg": -80.0, "registration_fitness": 0.35},
            {"x": 85.0, "y": -70.0, "yaw_deg": 10.0, "registration_fitness": 1.20},
            {
                "x": -80.0,
                "y": -60.0,
                "yaw_deg": 90.0,
                "registration_fitness": float("inf"),
            },
            {"x": 20.0, "y": 10.0, "yaw_deg": 0.0},
        ]
        scores = sup._restrict_walk_to_distinct_candidates(
            candidates, (0.9, 0.85, 0.8, 0.7, 0.6)
        )
        assert scores == (0.9, 0.85, 0.0, 0.0, 0.6), scores
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_seed_motion_history_uses_one_fix_per_query(monkeypatch):
    # Candidate walking inside one query must not overwrite the previous-query
    # motion sample. Otherwise a wrong walked candidate poisons the next query's
    # velocity estimate and compensation never fires.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup.enable_seed_motion = True
        sup.seed_motion_wall_fallback = True
        sup.max_seed_speed = 30.0
        sup.max_seed_latency = 30.0
        sup._prev_fix = (0.0, 0.0, 100.0)
        sup._query_issue_time = 110.0
        sup._current_query_issue_time = 110.0

        monkeypatch.setattr(rsn.time, "monotonic", lambda: 115.0)
        first = sup._compensate_seed(10.0, 0.0, candidate_index=0)
        walked = sup._compensate_seed(99.0, 0.0, candidate_index=1)

        assert first == (15.0, 0.0)
        assert walked == (104.0, 0.0)
        assert sup._prev_fix == (10.0, 0.0, 110.0)

        sup._query_issue_time = 120.0
        sup._current_query_issue_time = 120.0
        monkeypatch.setattr(rsn.time, "monotonic", lambda: 122.0)
        next_query = sup._compensate_seed(20.0, 0.0, candidate_index=0)

        assert next_query == (22.0, 0.0)
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_seed_motion_uses_candidate_age_from_query_reply(monkeypatch):
    # New G2 replies include candidate_age_sec: the seed should be compensated
    # from the scan time, not from the query issue time. This matters when the
    # latest scan was already a little old or a publish happens after response.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup.enable_seed_motion = True
        sup.seed_motion_wall_fallback = True
        sup.max_seed_speed = 30.0
        sup.max_seed_latency = 30.0
        sup._prev_fix = (0.0, 0.0, 100.0)
        sup._query_issue_time = 110.0
        sup._current_query_issue_time = 110.0
        sup._current_query_candidate_age_sec = 3.0
        sup._current_query_response_sec = 113.0

        monkeypatch.setattr(rsn.time, "monotonic", lambda: 115.0)
        compensated = sup._compensate_seed(10.0, 0.0, candidate_index=0)

        assert compensated == (15.0, 0.0)
        # The stored fix timestamp is response_time - candidate_age, not the
        # query issue timestamp.
        assert sup._prev_fix == (10.0, 0.0, 110.0)
        assert "candidate age" in sup._last_seed_motion_status
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_bbs_reset_preserves_source_scan_stamp():
    """GLIL uses this stamp to compose the delayed fix with matching odometry."""
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    published = []

    class Publisher:
        def publish(self, message):
            published.append(message)

    try:
        sup.initialpose_pub = Publisher()
        sup._candidates = [{"x": -110.0, "y": 46.0, "yaw_deg": -170.0, "score": 0.7}]
        sup._current_query_scan_stamp_sec = 1693922514.4998405
        sup._publish_reset()

        assert len(published) == 1
        stamp = published[0].header.stamp
        assert stamp.sec == 1693922514
        assert abs(stamp.nanosec - 499840498) <= 1
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_verified_glil_request_clear_confirms_recovery(monkeypatch):
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup.request_clear_confirms_recovery = True
        sup.params = replace(
            sup.params,
            recovery_confirmation_samples=1,
            enable_confirm_cross_check=False,
        )
        sup._requested = False
        sup.state = replace(
            sup.state,
            name=rsn.rsp.STATE_SETTLING,
            attempts=1,
            last_reset_sec=100.0,
            candidate_scores=(0.9,),
            candidate_index=0,
        )
        monkeypatch.setattr(rsn.time, "monotonic", lambda: 101.0)

        sup._tick()

        assert sup.state.name == rsn.rsp.STATE_STANDDOWN
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_no_scan_reply_is_marked_retryable_without_spending_budget():
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()

    class Response:
        success = False
        message = json.dumps({"error": "no_scan_received", "scan_point_count": 0})

    class Future:
        @staticmethod
        def result():
            return Response()

    try:
        sup._query_in_flight = True
        sup._on_query_response(Future())

        assert sup._pending_reply == ()
        assert sup._pending_reply_retryable is True
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_first_query_seed_motion_uses_local_pose_delta(monkeypatch):
    # The first query has no previous BBS fix, but the localizer still publishes
    # /pcl_pose. Use that local pose delta to compensate query latency.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup.enable_seed_motion = True
        sup.seed_motion_wall_fallback = True
        sup.max_seed_speed = 3.0
        sup.max_seed_latency = 30.0
        sup._query_issue_time = 100.0
        sup._current_query_issue_time = 100.0
        sup._query_issue_pose = (10.0, 20.0, 100.0)
        sup._query_issue_pose_trusted = True
        sup._stable_tracking = True
        sup._last_pose_x = 11.5
        sup._last_pose_y = 24.0
        sup._last_pose_observed_sec = 102.0

        monkeypatch.setattr(rsn.time, "monotonic", lambda: 102.5)
        compensated = sup._compensate_seed(-109.0, 14.0, candidate_index=0)

        assert compensated == (-107.5, 18.0)
        assert sup._prev_fix == (-109.0, 14.0, 100.0)
        assert sup._current_query_pose_delta == (1.5, 4.0)
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_first_query_seed_motion_falls_back_to_last_pose_velocity(monkeypatch):
    # If tracking is already lost, /pcl_pose may not update during the slow query.
    # Use the last accepted local pose velocity to keep the first seed fresh.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup.enable_seed_motion = True
        sup.seed_motion_wall_fallback = True
        sup.max_seed_speed = 3.0
        sup.max_seed_latency = 30.0
        sup._query_issue_time = 100.0
        sup._current_query_issue_time = 100.0
        sup._query_issue_pose = None
        sup._query_issue_velocity = rsn.rsp.SeedVelocity(0.0, 1.2, True)

        monkeypatch.setattr(rsn.time, "monotonic", lambda: 110.0)
        compensated = sup._compensate_seed(-109.0, 14.0, candidate_index=0)

        assert compensated == (-109.0, 26.0)
        assert sup._current_query_pose_delta == (0.0, 12.0)
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_first_query_velocity_fallback_uses_candidate_age(monkeypatch):
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup.enable_seed_motion = True
        sup.seed_motion_wall_fallback = True
        sup.max_seed_speed = 3.0
        sup.max_seed_latency = 30.0
        sup._query_issue_time = 100.0
        sup._current_query_issue_time = 100.0
        sup._query_issue_pose = None
        sup._query_issue_velocity = rsn.rsp.SeedVelocity(0.0, 1.2, True)
        sup._current_query_candidate_age_sec = 3.0
        sup._current_query_response_sec = 104.0

        monkeypatch.setattr(rsn.time, "monotonic", lambda: 106.0)
        compensated = sup._compensate_seed(-109.0, 14.0, candidate_index=0)

        assert compensated == (-109.0, 20.0)
        assert sup._current_query_pose_delta == (0.0, 6.0)
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_seed_motion_history_survives_brief_request_drop(monkeypatch):
    # The C++ request line can briefly de-assert after a reset even when the whole
    # episode has not actually recovered. Keep the previous top fix through that
    # transient so the next fresh query can estimate velocity.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup._requested = True
        sup._prev_fix = (10.0, 0.0, 100.0)
        sup.state = replace(
            sup.state,
            name=rsn.rsp.STATE_COOLDOWN,
            attempts=1,
            cooldown_since_sec=200.0,
        )

        sup._on_reinit(Bool(data=False))
        assert sup._prev_fix == (10.0, 0.0, 100.0)

        monkeypatch.setattr(rsn.time, "monotonic", lambda: 206.0)
        sup._tick()
        assert sup.state.name == rsn.rsp.STATE_IDLE
        assert sup._prev_fix is None
    finally:
        sup.destroy_node()
        rclpy.shutdown()


def test_recovery_confirmation_requires_post_reset_fitness(monkeypatch):
    # A low fitness sample observed before /initialpose publication is not recovery
    # evidence for that reset. The node must wait for a fresh alignment_status row.
    rclpy.init()
    sup = rsn.ReinitializationSupervisorNode()
    try:
        sup._requested = True
        sup._fitness = 0.1
        sup._fitness_observed_sec = 99.0
        sup.state = replace(
            sup.state,
            name=rsn.rsp.STATE_SETTLING,
            attempts=1,
            last_reset_sec=100.0,
            candidate_scores=(0.9,),
            candidate_index=0,
        )
        sup.params = replace(sup.params, enable_confirm_cross_check=False)

        monkeypatch.setattr(rsn.time, "monotonic", lambda: 101.0)
        sup._tick()
        assert sup.state.name == rsn.rsp.STATE_SETTLING

        sup._fitness_observed_sec = 101.5
        monkeypatch.setattr(rsn.time, "monotonic", lambda: 102.0)
        sup._tick()
        assert sup.state.name == rsn.rsp.STATE_SETTLING

        sup._fitness_observed_sec = 102.5
        monkeypatch.setattr(rsn.time, "monotonic", lambda: 103.0)
        sup._tick()
        assert sup.state.name == rsn.rsp.STATE_SETTLING

        sup._fitness_observed_sec = 103.5
        monkeypatch.setattr(rsn.time, "monotonic", lambda: 104.0)
        sup._tick()
        assert sup.state.name == rsn.rsp.STATE_STANDDOWN
    finally:
        sup.destroy_node()
        rclpy.shutdown()
