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
    """Fakes the localizer + G2 service and watches /initialpose."""

    def __init__(self):
        super().__init__("g3_test_harness")
        self.create_service(Trigger, "/global_localization_node/query", self._on_query)
        self._reinit = self.create_publisher(Bool, "/reinitialization_requested", _REL)
        self._status = self.create_publisher(DiagnosticArray, "/alignment_status", 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_pose, _REL)
        self.query_calls = 0
        self.initialpose = None
        self.create_timer(0.1, self._drive)

    def _on_query(self, request, response):
        self.query_calls += 1
        response.success = True
        response.message = json.dumps(
            {"candidate_count": 1,
             "top": {"x": 12.0, "y": 34.0, "yaw_deg": 45.0, "score": 0.99}})
        return response

    def _on_pose(self, msg):
        self.initialpose = msg

    def _drive(self):
        # Sustained reinit request + persistently-bad fitness (no premature recovery).
        self._reinit.publish(Bool(data=True))
        status = DiagnosticStatus()
        kv = KeyValue()
        kv.key, kv.value = "fitness_score", "9.0"
        status.values = [kv]
        array = DiagnosticArray()
        array.status = [status]
        self._status.publish(array)


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
