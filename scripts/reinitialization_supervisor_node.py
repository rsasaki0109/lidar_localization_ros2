#!/usr/bin/env python3
"""G3 guarded automatic reinitialization node.

Closes the global-localization recovery loop: it watches the localization
component's `/reinitialization_requested` signal (raised by the C++ recovery
supervisor when tracking is judged lost) and, behind the safety guards in
``reinitialization_supervisor_policy``, queries the G2 global-localization
service and republishes `/initialpose` to reseed tracking.

All of the *decision* logic -- when to query, when it is safe to publish a reset,
when to give up -- lives in the ROS-free policy module and is regression-tested in
``test/test_reinitialization_supervisor_policy.py``. This node is only the I/O
shell: subscribe, call the service, publish, and feed observations to the policy.

Opt-in only, exactly like the G2 node: it is never part of the default launch and
publishes nothing until `/reinitialization_requested` is asserted and every guard
in the policy is satisfied.

Wiring::

    /reinitialization_requested (std_msgs/Bool)      --in-->  supervisor
    /alignment_status (diagnostic_msgs/DiagnosticArray) --in--> supervisor (fitness)
    <query_service> (std_srvs/Trigger)               <--call-- supervisor
    /initialpose (geometry_msgs/PoseWithCovarianceStamped) <-pub- supervisor
"""

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import reinitialization_supervisor_policy as rsp


class ReinitializationSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("reinitialization_supervisor_node")

        self.declare_parameter("reinitialization_topic", "/reinitialization_requested")
        self.declare_parameter("alignment_status_topic", "/alignment_status")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("query_service", "/global_localization_node/query")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("tick_period_sec", 0.5)
        # Initial-pose covariance to advertise on a reset (diagonal x, y, yaw).
        self.declare_parameter("reset_position_std_m", 0.5)
        self.declare_parameter("reset_yaw_std_rad", 0.25)
        # Guard policy parameters (see reinitialization_supervisor_policy).
        self.declare_parameter("request_debounce_sec", 1.0)
        self.declare_parameter("min_seconds_between_attempts", 5.0)
        self.declare_parameter("max_attempts", 3)
        self.declare_parameter("min_candidate_score", 0.6)
        self.declare_parameter("query_timeout_sec", 10.0)
        self.declare_parameter("settle_timeout_sec", 8.0)
        self.declare_parameter("recovery_fitness_threshold", 1.5)

        self.params = rsp.SupervisorParams(
            request_debounce_sec=float(self.get_parameter("request_debounce_sec").value),
            min_seconds_between_attempts=float(
                self.get_parameter("min_seconds_between_attempts").value),
            max_attempts=int(self.get_parameter("max_attempts").value),
            min_candidate_score=float(self.get_parameter("min_candidate_score").value),
            query_timeout_sec=float(self.get_parameter("query_timeout_sec").value),
            settle_timeout_sec=float(self.get_parameter("settle_timeout_sec").value),
            recovery_fitness_threshold=float(
                self.get_parameter("recovery_fitness_threshold").value),
        )
        self.state = rsp.initial_state()

        self.global_frame_id = self.get_parameter("global_frame_id").value
        self.position_std = float(self.get_parameter("reset_position_std_m").value)
        self.yaw_std = float(self.get_parameter("reset_yaw_std_rad").value)

        # Latest observed signals.
        self._requested = False
        self._fitness = None
        # One-shot service reply: (score, pose-dict) delivered to the policy once.
        self._pending_reply = None
        self._query_in_flight = False
        # The candidate most recently approved for publication.
        self._last_top = None

        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            Bool, self.get_parameter("reinitialization_topic").value,
            self._on_reinit, reliable)
        self.create_subscription(
            DiagnosticArray, self.get_parameter("alignment_status_topic").value,
            self._on_alignment_status, 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("initialpose_topic").value,
            reliable)
        self.query_client = self.create_client(
            Trigger, self.get_parameter("query_service").value)

        self.create_timer(
            float(self.get_parameter("tick_period_sec").value), self._tick)
        self.get_logger().info(
            "reinitialization supervisor ready (opt-in); guards=%s" % (self.params,))

    # --- subscriptions -------------------------------------------------------

    def _on_reinit(self, msg: Bool) -> None:
        self._requested = bool(msg.data)

    def _on_alignment_status(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            for kv in status.values:
                if kv.key == "fitness_score":
                    try:
                        self._fitness = float(kv.value)
                    except ValueError:
                        pass
                    return

    # --- main loop -----------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        score = None
        if self._pending_reply is not None:
            score, self._last_top = self._pending_reply
            self._pending_reply = None

        obs = rsp.SupervisorObservation(
            now_sec=now,
            reinitialization_requested=self._requested,
            best_candidate_score=score,
            best_fitness=self._fitness,
        )
        decision = rsp.decide(self.params, self.state, obs)
        self.state = decision.state

        if decision.action == rsp.ACTION_QUERY:
            self._issue_query()
        elif decision.action == rsp.ACTION_PUBLISH_RESET:
            self._publish_reset()
        elif decision.action == rsp.ACTION_GIVE_UP:
            self.get_logger().error(
                "reinitialization gave up (%s) after %d attempt(s); operator "
                "intervention needed" % (decision.reason, self.state.attempts))

        if decision.reason not in ("tracking", "settling", "cooldown", "standdown",
                                   "exhausted", "awaiting_candidates"):
            self.get_logger().info(
                "supervisor: %s -> %s (%s)"
                % (decision.action, self.state.name, decision.reason))

    def _issue_query(self) -> None:
        if not self.query_client.service_is_ready():
            self.get_logger().warn("query service not available; will retry")
            return
        future = self.query_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_query_response)
        self._query_in_flight = True

    def _on_query_response(self, future) -> None:
        self._query_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - log and let the policy time out
            self.get_logger().warn("query call failed: %s" % exc)
            return
        if not response.success:
            self.get_logger().info("query returned no candidate: %s" % response.message)
            # Deliver a sub-threshold score so the policy counts a failed attempt.
            self._pending_reply = (float("-inf"), None)
            return
        try:
            summary = json.loads(response.message)
            top = summary["top"]
            score = float(top["score"])
        except (ValueError, KeyError) as exc:
            self.get_logger().warn("could not parse query reply: %s" % exc)
            self._pending_reply = (float("-inf"), None)
            return
        self._pending_reply = (score, top)

    def _publish_reset(self) -> None:
        if self._last_top is None:
            self.get_logger().error("publish_reset requested with no candidate; skipping")
            return
        top = self._last_top
        yaw = math.radians(top["yaw_deg"])
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame_id
        msg.pose.pose.position.x = float(top["x"])
        msg.pose.pose.position.y = float(top["y"])
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = self.position_std ** 2          # x
        cov[7] = self.position_std ** 2          # y
        cov[35] = self.yaw_std ** 2              # yaw
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        self.get_logger().warn(
            "published /initialpose reset to (%.2f, %.2f, %.1f deg) score=%s"
            % (top["x"], top["y"], top["yaw_deg"], top["score"]))


def main() -> None:
    rclpy.init()
    node = ReinitializationSupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Normal shutdown paths: Ctrl-C, or an external SIGTERM / context
        # shutdown (the latter raises ExternalShutdownException from spin).
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
