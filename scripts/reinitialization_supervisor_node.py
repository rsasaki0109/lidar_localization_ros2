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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("recovery_confirmation_samples", 3)
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
            recovery_confirmation_samples=int(
                self.get_parameter("recovery_confirmation_samples").value),
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
        self._fitness_observed_sec = None
        self._stable_tracking = None
        # Latest z / roll / pitch from the localizer pose. A G2 candidate is 2D
        # (x, y, yaw, z=0); on an outdoor map the true z can be tens of metres from
        # zero, which seeds the reset outside the registration z-basin and the lock
        # never takes. The vehicle's height and attitude drift slowly even while xy
        # tracking is lost, so the most recent pose's z / roll / pitch are a good
        # carry-over -- we override only x / y / yaw from the candidate.
        self._last_pose_z = None
        self._last_pose_x = None
        self._last_pose_y = None
        self._last_pose_observed_sec = None
        self._last_pose_velocity = rsp.SeedVelocity()
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
        # published query's top fix (x, y, issue_time) used to infer map velocity.
        # Candidate walking inside one query must not clobber this history: every
        # candidate in that list came from the same scan time, so the first
        # candidate of each new query is the representative motion sample.
        self._query_issue_time = None
        self._query_issue_pose = None
        self._query_issue_velocity = rsp.SeedVelocity()
        self._prev_fix = None
        self._current_query_velocity = rsp.SeedVelocity()
        self._current_query_pose_delta = None
        self._current_query_issue_time = None
        self._last_seed_motion_status = "disabled"

        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Bool, self.get_parameter("reinitialization_topic").value,
            self._on_reinit, reliable)
        self.create_subscription(
            DiagnosticArray, self.get_parameter("alignment_status_topic").value,
            self._on_alignment_status, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value,
            self._on_pose, pose_qos)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("initialpose_topic").value,
            reliable)
        self.query_client = self.create_client(
            Trigger, self.get_parameter("query_service").value)

        self.create_timer(
            float(self.get_parameter("tick_period_sec").value), self._tick)
        self.get_logger().info(
            "reinitialization supervisor ready (opt-in); guards=%s" % (self.params,))
        self.get_logger().info(
            "seed motion compensation: enabled=%s max_speed_mps=%.2f max_latency_sec=%.2f"
            % (self.enable_seed_motion, self.max_seed_speed, self.max_seed_latency))

    # --- subscriptions -------------------------------------------------------

    def _on_reinit(self, msg: Bool) -> None:
        self._requested = bool(msg.data)

    def _on_alignment_status(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            if "fitness_score" not in values:
                continue
            try:
                self._fitness = float(values["fitness_score"])
                self._fitness_observed_sec = time.monotonic()
                reinit_requested = (
                    str(values.get("reinitialization_requested", "")).lower()
                    == "true")
                recovery_state = str(values.get("recovery_state", ""))
                recovery_action = str(values.get("recovery_action", ""))
                if recovery_state or recovery_action or "reinitialization_requested" in values:
                    self._stable_tracking = (
                        status.message == "ok"
                        and recovery_state == "tracking"
                        and recovery_action == "accept_measurement"
                        and not reinit_requested)
                else:
                    self._stable_tracking = (status.message == "ok")
            except ValueError:
                pass
            return

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        observed_sec = time.monotonic()
        if (self._last_pose_x is not None and self._last_pose_y is not None
                and self._last_pose_observed_sec is not None):
            self._last_pose_velocity = rsp.estimate_seed_velocity(
                (self._last_pose_x, self._last_pose_y),
                self._last_pose_observed_sec,
                (float(p.position.x), float(p.position.y)),
                observed_sec,
                max_speed_mps=self.max_seed_speed)
        self._last_pose_x = float(p.position.x)
        self._last_pose_y = float(p.position.y)
        self._last_pose_z = float(p.position.z)
        self._last_pose_observed_sec = observed_sec
        self._last_pose_roll, self._last_pose_pitch = _roll_pitch_from_quat(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    # --- main loop -----------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        candidate_scores = None
        if self._pending_reply is not None:
            candidate_scores = self._pending_reply
            self._pending_reply = None
        best_fitness = self._fitness
        stable_tracking = self._stable_tracking
        if (self.state.name == rsp.STATE_SETTLING
                and self.state.last_reset_sec is not None
                and (self._fitness_observed_sec is None
                     or self._fitness_observed_sec <= self.state.last_reset_sec)):
            # Recovery evidence must be observed after the reset was published.
            # Otherwise a low fitness sample from before the reset can produce a
            # false recovery_confirmed while the localizer has not consumed the
            # /initialpose yet.
            best_fitness = None
            stable_tracking = None

        obs = rsp.SupervisorObservation(
            now_sec=now,
            reinitialization_requested=self._requested,
            candidate_scores=candidate_scores,
            best_fitness=best_fitness,
            fitness_observed_sec=self._fitness_observed_sec,
            stable_tracking=stable_tracking,
        )
        decision = rsp.decide(self.params, self.state, obs)
        self.state = decision.state

        if decision.reason in (
            "recovery_confirmed",
            "request_cleared",
            "standdown_cleared",
            "exhausted_cleared",
        ):
            self._clear_seed_motion_history()

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

    def _clear_seed_motion_history(self) -> None:
        self._prev_fix = None
        self._current_query_velocity = rsp.SeedVelocity()
        self._current_query_pose_delta = None
        self._current_query_issue_time = None

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
        if (self._last_pose_x is not None and self._last_pose_y is not None
                and self._last_pose_observed_sec is not None
                and self._query_issue_time - self._last_pose_observed_sec
                <= self.max_seed_latency):
            self._query_issue_pose = (
                self._last_pose_x, self._last_pose_y, self._last_pose_observed_sec)
        else:
            self._query_issue_pose = None
        self._query_issue_velocity = self._last_pose_velocity

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
        self._current_query_velocity = rsp.SeedVelocity()
        self._current_query_issue_time = self._query_issue_time

    def _publish_reset(self, candidate_index: int = 0) -> None:
        if not self._candidates or candidate_index >= len(self._candidates):
            self.get_logger().error("publish_reset requested with no candidate; skipping")
            return
        top = self._candidates[candidate_index]
        yaw = math.radians(top["yaw_deg"])
        raw_x, raw_y = float(top["x"]), float(top["y"])
        seed_x, seed_y = self._compensate_seed(raw_x, raw_y, candidate_index)
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
        elif self.enable_seed_motion:
            comp_note = " (motion-compensation skipped: %s)" % (
                self._last_seed_motion_status,)
        self.get_logger().warn(
            "published /initialpose reset to (%.2f, %.2f, z=%.2f, %.1f deg) score=%s "
            "[candidate %d/%d]%s"
            % (seed_x, seed_y, z, top["yaw_deg"], top["score"],
               candidate_index + 1, len(self._candidates), comp_note))

    def _compensate_seed(self, raw_x: float, raw_y: float, candidate_index: int = 0):
        """Forward-extrapolate (raw_x, raw_y) by the query->publish latency.

        The first candidate of a new query is the representative fix used to
        estimate velocity against the previous query. Within-query candidate
        walking reuses that same velocity but does not update the history, because
        all candidates in one reply correspond to the same scan time.

        Disabled, first-query, or implausible-velocity cases return the raw
        position unchanged (see the policy module).
        """
        issue = self._query_issue_time
        self._last_seed_motion_status = "disabled"
        if not self.enable_seed_motion:
            return raw_x, raw_y
        if issue is None:
            self._last_seed_motion_status = "no query issue time"
            return raw_x, raw_y
        if self._current_query_issue_time != issue:
            self._current_query_velocity = rsp.SeedVelocity()
            self._current_query_pose_delta = None
            self._current_query_issue_time = issue

        if candidate_index == 0:
            velocity = rsp.SeedVelocity()
            if self._prev_fix is not None:
                prev_x, prev_y, prev_issue = self._prev_fix
                velocity = rsp.estimate_seed_velocity(
                    (prev_x, prev_y), prev_issue, (raw_x, raw_y), issue,
                    max_speed_mps=self.max_seed_speed)
                if not velocity.valid:
                    dt = issue - prev_issue
                    if dt > 0.0:
                        speed = math.hypot(raw_x - prev_x, raw_y - prev_y) / dt
                        self._last_seed_motion_status = (
                            "invalid velocity estimate %.2fm/s over %.2fs"
                            % (speed, dt))
                    else:
                        self._last_seed_motion_status = "invalid velocity estimate"
            else:
                pose_delta = self._estimate_pose_delta_since_query_issue()
                if pose_delta is not None:
                    dx, dy, speed, dt = pose_delta
                    self._current_query_pose_delta = (dx, dy)
                    self._last_seed_motion_status = (
                        "pose delta %.2fm/s over %.2fs" % (speed, dt))
                else:
                    if self._last_seed_motion_status == "disabled":
                        self._last_seed_motion_status = "no previous query fix"
            self._current_query_velocity = velocity
            # Store the raw, top-ranked fix for the next query's velocity
            # estimate. Do not store walked candidates from the same query.
            self._prev_fix = (raw_x, raw_y, issue)
        else:
            if self._current_query_pose_delta is not None:
                dx, dy = self._current_query_pose_delta
                self._last_seed_motion_status = "applied query pose delta"
                return raw_x + dx, raw_y + dy
            velocity = self._current_query_velocity
            if not velocity.valid:
                self._last_seed_motion_status = "no valid velocity for current query"

        if self._current_query_pose_delta is not None:
            dx, dy = self._current_query_pose_delta
            return raw_x + dx, raw_y + dy
        if velocity.valid:
            latency = time.monotonic() - issue
            self._last_seed_motion_status = "applied latency %.2fs" % (latency,)
            return rsp.forward_compensate_xy(
                (raw_x, raw_y), velocity, latency, max_latency_sec=self.max_seed_latency)
        return raw_x, raw_y

    def _estimate_pose_delta_since_query_issue(self):
        if self._query_issue_pose is None:
            return self._estimate_pose_delta_from_query_velocity()
        if (self._last_pose_x is None or self._last_pose_y is None
                or self._last_pose_observed_sec is None):
            return self._estimate_pose_delta_from_query_velocity()
        issue_x, issue_y, issue_pose_sec = self._query_issue_pose
        dt = self._last_pose_observed_sec - issue_pose_sec
        if dt <= 0.0 or dt > self.max_seed_latency:
            return self._estimate_pose_delta_from_query_velocity()
        dx = self._last_pose_x - issue_x
        dy = self._last_pose_y - issue_y
        speed = math.hypot(dx, dy) / dt
        if speed > self.max_seed_speed:
            self._last_seed_motion_status = (
                "invalid pose-delta velocity %.2fm/s over %.2fs" % (speed, dt))
            return None
        return dx, dy, speed, dt

    def _estimate_pose_delta_from_query_velocity(self):
        if not self._query_issue_velocity.valid or self._query_issue_time is None:
            return None
        latency = time.monotonic() - self._query_issue_time
        if latency <= 0.0 or latency > self.max_seed_latency:
            return None
        speed = math.hypot(self._query_issue_velocity.vx, self._query_issue_velocity.vy)
        if speed > self.max_seed_speed:
            return None
        return (
            self._query_issue_velocity.vx * latency,
            self._query_issue_velocity.vy * latency,
            speed,
            latency)


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
