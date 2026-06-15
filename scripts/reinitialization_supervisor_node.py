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


def _roll_pitch_from_quat(x: float, y: float, z: float, w: float):
    """Extract (roll, pitch) from a quaternion (yaw is taken from the candidate)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    """Quaternion (x, y, z, w) from roll/pitch/yaw (ZYX convention)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class ReinitializationSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("reinitialization_supervisor_node")

        self.declare_parameter("reinitialization_topic", "/reinitialization_requested")
        self.declare_parameter("alignment_status_topic", "/alignment_status")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("query_service", "/global_localization_node/query")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("tick_period_sec", 0.5)
        # Localizer pose output, used to carry z / roll / pitch onto a 2D candidate.
        self.declare_parameter("pose_topic", "/pcl_pose")
        # Fallback z if no pose has been observed yet (e.g. a cold kidnapped start).
        self.declare_parameter("reset_default_z_m", 0.0)
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
        self.declare_parameter("max_walk_candidates", 4)
        # Seed motion compensation: forward-extrapolate a candidate by the measured
        # query->publish latency so it lands where the moving vehicle is *now*, not
        # where it was when the (slow) BBS query was issued. Off by default; the
        # velocity is inferred from successive query fixes (see the policy module's
        # seed-motion section). max_seed_speed_mps rejects an implausible estimate
        # (a wrong perceptual-aliasing candidate); max_seed_latency_sec clamps how
        # far the first-order extrapolation is trusted.
        self.declare_parameter("enable_seed_motion_compensation", False)
        self.declare_parameter("max_seed_speed_mps", 30.0)
        self.declare_parameter("max_seed_latency_sec", 30.0)

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
            max_walk_candidates=int(self.get_parameter("max_walk_candidates").value),
        )
        self.state = rsp.initial_state()

        self.global_frame_id = self.get_parameter("global_frame_id").value
        self.position_std = float(self.get_parameter("reset_position_std_m").value)
        self.yaw_std = float(self.get_parameter("reset_yaw_std_rad").value)
        self.reset_default_z = float(self.get_parameter("reset_default_z_m").value)
        self.enable_seed_motion = bool(
            self.get_parameter("enable_seed_motion_compensation").value)
        self.max_seed_speed = float(self.get_parameter("max_seed_speed_mps").value)
        self.max_seed_latency = float(self.get_parameter("max_seed_latency_sec").value)

        # Latest observed signals.
        self._requested = False
        self._fitness = None
        # Latest z / roll / pitch from the localizer pose. A G2 candidate is 2D
        # (x, y, yaw, z=0); on an outdoor map the true z can be tens of metres from
        # zero, which seeds the reset outside the registration z-basin and the lock
        # never takes. The vehicle's height and attitude drift slowly even while xy
        # tracking is lost, so the most recent pose's z / roll / pitch are a good
        # carry-over -- we override only x / y / yaw from the candidate.
        self._last_pose_z = None
        self._last_pose_roll = 0.0
        self._last_pose_pitch = 0.0
        # One-shot service reply delivered to the policy once: a tuple of ranked
        # candidate scores (high-to-low), or () for a reply that carried no usable
        # candidate. None means no reply is pending this tick.
        self._pending_reply = None
        self._query_in_flight = False
        # Ranked candidate poses from the most recent query reply, indexed by the
        # policy's candidate_index when it asks for a publish (the ranked-candidate
        # walk -- see reinitialization_supervisor_policy).
        self._candidates = []
        # Seed motion compensation bookkeeping: the monotonic time the in-flight
        # query was issued (a proxy for its fix's scan time), and the previously
        # published candidate fix (x, y, issue_time) used to infer map velocity.
        self._query_issue_time = None
        self._prev_fix = None

        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            Bool, self.get_parameter("reinitialization_topic").value,
            self._on_reinit, reliable)
        self.create_subscription(
            DiagnosticArray, self.get_parameter("alignment_status_topic").value,
            self._on_alignment_status, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value,
            self._on_pose, 10)
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
        requested = bool(msg.data)
        if self._requested and not requested:
            # Problem cleared: the next recovery episode is a fresh trajectory, so
            # drop the previous fix rather than differencing across the gap.
            self._prev_fix = None
        self._requested = requested

    def _on_alignment_status(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            for kv in status.values:
                if kv.key == "fitness_score":
                    try:
                        self._fitness = float(kv.value)
                    except ValueError:
                        pass
                    return

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._last_pose_z = float(p.position.z)
        self._last_pose_roll, self._last_pose_pitch = _roll_pitch_from_quat(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    # --- main loop -----------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        candidate_scores = None
        if self._pending_reply is not None:
            candidate_scores = self._pending_reply
            self._pending_reply = None

        obs = rsp.SupervisorObservation(
            now_sec=now,
            reinitialization_requested=self._requested,
            candidate_scores=candidate_scores,
            best_fitness=self._fitness,
        )
        decision = rsp.decide(self.params, self.state, obs)
        self.state = decision.state

        if decision.action == rsp.ACTION_QUERY:
            self._issue_query()
        elif decision.action == rsp.ACTION_PUBLISH_RESET:
            self._publish_reset(decision.candidate_index)
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
        # The candidate this query returns is a fix of the scan available now, so
        # stamp the issue time for the query->publish latency used in compensation.
        self._query_issue_time = time.monotonic()

    def _on_query_response(self, future) -> None:
        self._query_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - log and let the policy time out
            self.get_logger().warn("query call failed: %s" % exc)
            return
        if not response.success:
            self.get_logger().info("query returned no candidate: %s" % response.message)
            # Empty reply -> the policy counts a failed attempt.
            self._candidates = []
            self._pending_reply = ()
            return
        try:
            summary = json.loads(response.message)
            candidates = summary.get("candidates")
            if not candidates:
                # Back-compat with a G2 that only reports the single "top" candidate.
                candidates = [summary["top"]]
            scores = tuple(float(c["score"]) for c in candidates)
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn("could not parse query reply: %s" % exc)
            self._candidates = []
            self._pending_reply = ()
            return
        self._candidates = candidates
        self._pending_reply = scores

    def _publish_reset(self, candidate_index: int = 0) -> None:
        if not self._candidates or candidate_index >= len(self._candidates):
            self.get_logger().error("publish_reset requested with no candidate; skipping")
            return
        top = self._candidates[candidate_index]
        yaw = math.radians(top["yaw_deg"])
        raw_x, raw_y = float(top["x"]), float(top["y"])
        seed_x, seed_y = self._compensate_seed(raw_x, raw_y)
        # The candidate is 2D (x, y, yaw); carry z / roll / pitch from the last
        # localizer pose so the seed lands in the registration z-basin on a non-flat
        # map. Fall back to the configured default z if no pose seen yet.
        z = self._last_pose_z if self._last_pose_z is not None else self.reset_default_z
        qx, qy, qz, qw = _quat_from_rpy(
            self._last_pose_roll, self._last_pose_pitch, yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame_id
        msg.pose.pose.position.x = seed_x
        msg.pose.pose.position.y = seed_y
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = self.position_std ** 2          # x
        cov[7] = self.position_std ** 2          # y
        cov[35] = self.yaw_std ** 2              # yaw
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        comp_note = ""
        if (seed_x, seed_y) != (raw_x, raw_y):
            comp_note = " (motion-compensated from %.2f, %.2f)" % (raw_x, raw_y)
        self.get_logger().warn(
            "published /initialpose reset to (%.2f, %.2f, z=%.2f, %.1f deg) score=%s "
            "[candidate %d/%d]%s"
            % (seed_x, seed_y, z, top["yaw_deg"], top["score"],
               candidate_index + 1, len(self._candidates), comp_note))

    def _compensate_seed(self, raw_x: float, raw_y: float):
        """Forward-extrapolate (raw_x, raw_y) by the query->publish latency.

        Records the raw fix (the true measured position) for the next call's
        velocity estimate, then -- if enabled and a prior fix from an earlier query
        exists -- returns the candidate pushed ahead along the inferred map-frame
        velocity. Disabled, first-query, within-query-walk, or implausible-velocity
        cases all return the raw position unchanged (see the policy module).
        """
        issue = self._query_issue_time
        if not self.enable_seed_motion or issue is None:
            return raw_x, raw_y
        seed_x, seed_y = raw_x, raw_y
        if self._prev_fix is not None:
            prev_x, prev_y, prev_issue = self._prev_fix
            velocity = rsp.estimate_seed_velocity(
                (prev_x, prev_y), prev_issue, (raw_x, raw_y), issue,
                max_speed_mps=self.max_seed_speed)
            latency = time.monotonic() - issue
            seed_x, seed_y = rsp.forward_compensate_xy(
                (raw_x, raw_y), velocity, latency, max_latency_sec=self.max_seed_latency)
        # Store the raw (un-compensated) fix: it is the actual measurement the next
        # query's velocity estimate must difference against.
        self._prev_fix = (raw_x, raw_y, issue)
        return seed_x, seed_y


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
