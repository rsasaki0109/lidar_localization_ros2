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
from rclpy.qos import QoSProfile, ReliabilityPolicy          # noqa: E402
from std_msgs.msg import Bool                                # noqa: E402
from std_srvs.srv import Trigger                             # noqa: E402
from diagnostic_msgs.msg import (                            # noqa: E402
    DiagnosticArray, DiagnosticStatus, KeyValue)
from geometry_msgs.msg import PoseWithCovarianceStamped      # noqa: E402

import reinitialization_supervisor_node as rsn               # noqa: E402

_REL = QoSProfile(depth=1)
_REL.reliability = ReliabilityPolicy.RELIABLE


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
            PoseWithCovarianceStamped, "/pcl_pose", 10)
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
        kv = KeyValue()
        kv.key, kv.value = "fitness_score", ("0.2" if recovered else "9.0")
        status = DiagnosticStatus()
        status.values = [kv]
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
        min_seconds_between_attempts=1.0, max_attempts=1)
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
