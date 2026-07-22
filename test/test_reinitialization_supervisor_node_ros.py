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

from rclpy.executors import SingleThreadedExecutor          # noqa: E402
from rclpy.node import Node                                  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool                                # noqa: E402
from std_srvs.srv import Trigger                             # noqa: E402
from diagnostic_msgs.msg import (                            # noqa: E402
    DiagnosticArray, DiagnosticStatus, KeyValue)
from geometry_msgs.msg import PoseWithCovarianceStamped      # noqa: E402

import reinitialization_supervisor_node as rsn               # noqa: E402

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

    def __init__(self, recover_on_second=False, pose_z=None):
        super().__init__("g3_test_harness")
        self.create_service(Trigger, "/global_localization_node/query", self._on_query)
        self._reinit = self.create_publisher(Bool, "/reinitialization_requested", _REL)
        self._status = self.create_publisher(DiagnosticArray, "/alignment_status", 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_pose, _REL)
        # Optionally fake the localizer pose output so the supervisor can carry z.
        self._pose_z = pose_z
        self._pcl_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/pcl_pose", _POSE_QOS)
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
        response.message = json.dumps({
            "candidate_count": 2,
            "candidates": [
                {"x": 12.0, "y": 34.0, "yaw_deg": 45.0, "score": 0.99},
                {"x": 99.0, "y": 88.0, "yaw_deg": -90.0, "score": 0.98},
            ],
        })
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
            kv("recovery_state", "tracking" if recovered else "reinitialization_requested"),
            kv("recovery_action", "accept_measurement" if recovered else "request_reinitialization"),
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
        assert harness.initialpose is not None, "supervisor never published /initialpose"
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
        sup.params, settle_timeout_sec=2.0, request_debounce_sec=0.5,
        min_seconds_between_attempts=1.0, max_attempts=1,
        enable_confirm_cross_check=False)
    harness = _Harness(recover_on_second=True)
    executor = SingleThreadedExecutor()
    executor.add_node(sup)
    executor.add_node(harness)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        deadline = time.monotonic() + 15.0
        while (time.monotonic() < deadline
               and sup.state.name != rsn.rsp.STATE_STANDDOWN):
            time.sleep(0.1)

        assert len(harness.poses) >= 2, "node never walked to the second candidate"
        first, second = harness.poses[0].pose.pose, harness.poses[1].pose.pose
        assert abs(first.position.x - 12.0) < 1e-3 and abs(first.position.y - 34.0) < 1e-3
        assert abs(second.position.x - 99.0) < 1e-3 and abs(second.position.y - 88.0) < 1e-3
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
        assert harness.initialpose is not None, "supervisor never published /initialpose"
        # Candidate carried x/y from the query but z from the localizer pose.
        pose = harness.initialpose.pose.pose
        assert abs(pose.position.x - 12.0) < 1e-3
        assert abs(pose.position.z - (-11.05)) < 1e-2, pose.position.z
    finally:
        executor.shutdown()
        sup.destroy_node()
        harness.destroy_node()
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
        sup._candidates = [
            {"x": -110.0, "y": 46.0, "yaw_deg": -170.0, "score": 0.7}]
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
            sup.params, recovery_confirmation_samples=1,
            enable_confirm_cross_check=False)
        sup._requested = False
        sup.state = replace(
            sup.state, name=rsn.rsp.STATE_SETTLING, attempts=1,
            last_reset_sec=100.0, candidate_scores=(0.9,), candidate_index=0)
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
