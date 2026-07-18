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
    odom_bridge_pose (geometry_msgs/PoseWithCovarianceStamped) --in--> supervisor (opt-in)
    <query_service> (std_srvs/Trigger)               <--call-- supervisor
    /initialpose (geometry_msgs/PoseWithCovarianceStamped) <-pub- supervisor
"""

import json
import math
import time
import csv
from collections import deque
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import reinitialization_supervisor_policy as rsp

_ACCEPT_MEASUREMENT_ACTIONS = frozenset({
    "accept_measurement",
    "accept_measurement_with_warning",
    "accept_measurement_after_recovery",
})
_ACCEPT_TRACKING_STATES = frozenset({
    "tracking",
    "degraded",
    "recovering",
})


def _roll_pitch_from_quat(x: float, y: float, z: float, w: float):
    """Extract (roll, pitch) from a quaternion (yaw is taken from the candidate)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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
        self.declare_parameter("prefer_reset_default_z_m", False)
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
        self.declare_parameter("confirm_cross_check", True)
        self.declare_parameter("cross_check_mismatch_m", 5.0)
        # Before publishing any BBS reset, require successive BBS fixes to
        # describe the same relative SE(2) motion as the odom bridge. This is a
        # shadow gate: withheld fixes never touch /initialpose or public TF.
        self.declare_parameter("enable_bbs_shadow_motion_gate", False)
        self.declare_parameter("bbs_shadow_required_samples", 2)
        self.declare_parameter("bbs_shadow_max_translation_mismatch_m", 5.0)
        self.declare_parameter("bbs_shadow_max_yaw_mismatch_deg", 20.0)
        self.declare_parameter("bbs_shadow_bridge_stamp_tolerance_sec", 2.0)
        # Odom bridge (see reinitialization_supervisor_policy): a candidate built
        # from the C++ recovery supervisor's odom_bridge_pose topic, which -- with
        # enable_map_odom_tf's frozen map -> odom rebroadcast and an external LIO
        # front end's continuous odom -> base_link (e.g. GLIM) -- carries
        # map -> odom(last accepted) x odom -> base_link(now). Published on a
        # plain topic rather than read back off /tf: a separate process's TF
        # listener was found not to reliably receive this node's /tf in testing.
        # Opt-in; off by default because it needs that external front end.
        self.declare_parameter("use_odom_bridge_candidate", False)
        self.declare_parameter("odom_bridge_pose_topic", "/odom_bridge_pose")
        self.declare_parameter("odom_bridge_max_attempts", 1)
        self.declare_parameter("odom_bridge_max_age_sec", 2.0)
        self.declare_parameter("odom_bridge_position_std_m", 0.3)
        self.declare_parameter("odom_bridge_yaw_std_rad", 0.1)
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
        self.declare_parameter("seed_velocity_max_age_sec", 60.0)
        self.declare_parameter("seed_motion_wall_fallback", False)
        self.declare_parameter("seed_motion_skip_registration_fitness_threshold", 1.0)
        self.declare_parameter("event_log_csv", "")

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
            enable_confirm_cross_check=bool(
                self.get_parameter("confirm_cross_check").value),
            cross_check_mismatch_m=float(
                self.get_parameter("cross_check_mismatch_m").value),
            use_odom_bridge_candidate=bool(
                self.get_parameter("use_odom_bridge_candidate").value),
            odom_bridge_max_attempts=int(
                self.get_parameter("odom_bridge_max_attempts").value),
        )
        self.state = rsp.initial_state()

        self.global_frame_id = self.get_parameter("global_frame_id").value
        self.use_odom_bridge_candidate = self.params.use_odom_bridge_candidate
        self.odom_bridge_max_age_sec = float(
            self.get_parameter("odom_bridge_max_age_sec").value)
        self.odom_bridge_position_std = float(
            self.get_parameter("odom_bridge_position_std_m").value)
        self.odom_bridge_yaw_std = float(
            self.get_parameter("odom_bridge_yaw_std_rad").value)
        self.enable_bbs_shadow_motion_gate = bool(
            self.get_parameter("enable_bbs_shadow_motion_gate").value)
        self.bbs_shadow_required_samples = max(
            2, int(self.get_parameter("bbs_shadow_required_samples").value))
        self.bbs_shadow_max_translation_mismatch = float(
            self.get_parameter("bbs_shadow_max_translation_mismatch_m").value)
        self.bbs_shadow_max_yaw_mismatch_deg = float(
            self.get_parameter("bbs_shadow_max_yaw_mismatch_deg").value)
        self.bbs_shadow_bridge_stamp_tolerance_sec = float(
            self.get_parameter("bbs_shadow_bridge_stamp_tolerance_sec").value)
        # Latest odom_bridge_pose message and the wall time it was received, for
        # the freshness check in _odom_bridge_status. None while none has arrived
        # yet (or use_odom_bridge_candidate is off).
        self._odom_bridge_pose_msg = None
        self._odom_bridge_pose_received_sec = None
        self._odom_bridge_history = deque()
        self._bbs_shadow_gate = rsp.BbsShadowMotionGate(
            self.bbs_shadow_required_samples,
            self.bbs_shadow_max_translation_mismatch,
            self.bbs_shadow_max_yaw_mismatch_deg)
        self.position_std = float(self.get_parameter("reset_position_std_m").value)
        self.yaw_std = float(self.get_parameter("reset_yaw_std_rad").value)
        self.reset_default_z = float(self.get_parameter("reset_default_z_m").value)
        self.prefer_reset_default_z = bool(
            self.get_parameter("prefer_reset_default_z_m").value)
        self.enable_seed_motion = bool(
            self.get_parameter("enable_seed_motion_compensation").value)
        self.max_seed_speed = float(self.get_parameter("max_seed_speed_mps").value)
        self.max_seed_latency = float(self.get_parameter("max_seed_latency_sec").value)
        self.seed_velocity_max_age = float(
            self.get_parameter("seed_velocity_max_age_sec").value)
        self.seed_motion_wall_fallback = bool(
            self.get_parameter("seed_motion_wall_fallback").value)
        self.seed_motion_skip_registration_fitness = float(
            self.get_parameter("seed_motion_skip_registration_fitness_threshold").value)
        event_log_csv = (
            self.get_parameter("event_log_csv").get_parameter_value().string_value)
        self._event_log_csv = Path(event_log_csv) if event_log_csv else None
        self._stable_window_count = 0
        self._awaiting_stable_window = False
        self._stable_window_logged = False
        if self._event_log_csv is not None:
            self._event_log_csv.parent.mkdir(parents=True, exist_ok=True)
            with self._event_log_csv.open("w", newline="", encoding="utf-8") as handle:
                csv.writer(handle).writerow(["stamp_sec", "event"])

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
        self._seed_velocity_tracker = rsp.TrustedSeedVelocityTracker(self.max_seed_speed)
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
        self._query_issue_pose_trusted = False
        self._query_issue_velocity = rsp.SeedVelocity()
        self._prev_fix = None
        self._last_sim_stamp_sec = None
        self._prev_sim_fix = None
        self._sim_fix_velocity = rsp.SeedVelocity()
        self._current_query_velocity = rsp.SeedVelocity()
        self._current_query_pose_delta = None
        self._current_query_issue_time = None
        self._current_query_candidate_age_sec = None
        self._current_query_response_sec = None
        self._last_seed_motion_status = "disabled"
        # Bag-clock pose history for post-confirm cross-check mismatch (pruned in
        # _on_pose). Sim fix-to-fix velocity history is separate and preserved
        # across episode boundaries by _clear_wall_seed_motion_history.
        self._pose_history = deque()
        self._pending_cross_check_mismatch = None

        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        if self.use_odom_bridge_candidate or self.enable_bbs_shadow_motion_gate:
            # Subscribe only when the bridge is consumed either as a candidate
            # or as the independent relative-motion reference for shadow BBS.
            self.create_subscription(
                PoseWithCovarianceStamped,
                self.get_parameter("odom_bridge_pose_topic").value,
                self._on_odom_bridge_pose, pose_qos)

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
            "seed motion compensation: enabled=%s max_speed_mps=%.2f max_latency_sec=%.2f "
            "velocity_max_age_sec=%.2f wall_fallback=%s skip_registration_fitness<=%.2f "
            "(sim fix-to-fix on bag clock preferred when available)"
            % (self.enable_seed_motion, self.max_seed_speed, self.max_seed_latency,
               self.seed_velocity_max_age, self.seed_motion_wall_fallback,
               self.seed_motion_skip_registration_fitness))
        if self.prefer_reset_default_z:
            self.get_logger().info(
                "seed z: using reset_default_z_m=%.3f (prefer_reset_default_z_m=true)"
                % self.reset_default_z)
        if self.use_odom_bridge_candidate:
            self.get_logger().info(
                "odom bridge: enabled, tried before every BBS query; "
                "topic=%s max_attempts=%d max_age_sec=%.2f "
                "position_std_m=%.2f yaw_std_rad=%.2f"
                % (self.get_parameter("odom_bridge_pose_topic").value,
                   self.params.odom_bridge_max_attempts, self.odom_bridge_max_age_sec,
                   self.odom_bridge_position_std, self.odom_bridge_yaw_std))
        if self.enable_bbs_shadow_motion_gate:
            self.get_logger().info(
                "BBS shadow motion gate: required_samples=%d translation<=%.2fm "
                "yaw<=%.1fdeg bridge_stamp_tolerance<=%.2fs"
                % (self.bbs_shadow_required_samples,
                   self.bbs_shadow_max_translation_mismatch,
                   self.bbs_shadow_max_yaw_mismatch_deg,
                   self.bbs_shadow_bridge_stamp_tolerance_sec))

    # --- subscriptions -------------------------------------------------------

    def _on_reinit(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested and not self._requested:
            self._bbs_shadow_gate.reset()
        self._requested = requested

    def _wall_now_sec(self) -> float:
        """Wall clock for supervisor policy timing and G2 query latency."""
        return time.monotonic()

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _update_last_sim_stamp(self, stamp) -> None:
        sec = self._stamp_to_sec(stamp)
        if self._last_sim_stamp_sec is None or sec > self._last_sim_stamp_sec:
            self._last_sim_stamp_sec = sec

    def _on_alignment_status(self, msg: DiagnosticArray) -> None:
        self._update_last_sim_stamp(msg.header.stamp)
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            if "fitness_score" not in values:
                continue
            try:
                self._fitness = float(values["fitness_score"])
                self._fitness_observed_sec = self._wall_now_sec()
                reinit_requested = (
                    str(values.get("reinitialization_requested", "")).lower()
                    == "true")
                recovery_state = str(values.get("recovery_state", ""))
                recovery_action = str(values.get("recovery_action", ""))
                if recovery_state or recovery_action or "reinitialization_requested" in values:
                    self._stable_tracking = (
                        status.message == "ok"
                        and recovery_state in _ACCEPT_TRACKING_STATES
                        and recovery_action in _ACCEPT_MEASUREMENT_ACTIONS
                        and not reinit_requested)
                else:
                    self._stable_tracking = (status.message == "ok")
            except ValueError:
                pass
            return

    def _pose_velocity_trusted(self) -> bool:
        return bool(self._stable_tracking) and self.state.name in (
            rsp.STATE_IDLE, rsp.STATE_STANDDOWN)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        sim_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self._update_last_sim_stamp(msg.header.stamp)
        p = msg.pose.pose
        observed_sec = self._wall_now_sec()
        self._seed_velocity_tracker.observe(
            float(p.position.x), float(p.position.y), observed_sec,
            self._pose_velocity_trusted())
        self._last_pose_x = float(p.position.x)
        self._last_pose_y = float(p.position.y)
        self._last_pose_z = float(p.position.z)
        self._last_pose_observed_sec = observed_sec
        self._last_pose_roll, self._last_pose_pitch = _roll_pitch_from_quat(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        self._pose_history.append(
            (sim_stamp_sec, float(p.position.x), float(p.position.y)))
        while (len(self._pose_history) > 1
               and self._pose_history[-1][0] - self._pose_history[0][0] > 180.0):
            self._pose_history.popleft()

    def _on_odom_bridge_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Latest map -> odom(last accepted) x odom -> base_link(now) pose.

        Published by the C++ recovery supervisor (publishOdomBridgePose) only
        while both halves are available -- a stale/missing chain (external
        front end down, or no accepted match has ever happened) simply stops
        this topic, which _odom_bridge_status's freshness check below catches.
        """
        if self._odom_bridge_pose_msg is None:
            self.get_logger().info(
                "odom bridge: first odom_bridge_pose received (stamp=%d.%09d)"
                % (msg.header.stamp.sec, msg.header.stamp.nanosec))
        self._odom_bridge_pose_msg = msg
        self._odom_bridge_pose_received_sec = self._wall_now_sec()
        self._update_last_sim_stamp(msg.header.stamp)
        p = msg.pose.pose
        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self._odom_bridge_history.append((
            stamp_sec,
            float(p.position.x),
            float(p.position.y),
            _yaw_from_quat(
                p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w),
        ))
        while (len(self._odom_bridge_history) > 1
               and self._odom_bridge_history[-1][0]
               - self._odom_bridge_history[0][0] > 300.0):
            self._odom_bridge_history.popleft()

    def _odom_bridge_status(self):
        """Return the current odom-bridge pose, or None if unavailable/stale."""
        if not self.use_odom_bridge_candidate or self._odom_bridge_pose_msg is None:
            return None
        stamp_sec = self._stamp_to_sec(self._odom_bridge_pose_msg.header.stamp)
        if (self._last_sim_stamp_sec is not None
                and self._last_sim_stamp_sec - stamp_sec > self.odom_bridge_max_age_sec):
            # Newer sim-clock traffic has arrived (e.g. /alignment_status) since
            # this pose, but no fresher odom_bridge_pose -- the front end stalled.
            self.get_logger().warn(
                "odom bridge: pose stale (age %.2fs > %.2fs)"
                % (self._last_sim_stamp_sec - stamp_sec, self.odom_bridge_max_age_sec),
                throttle_duration_sec=5.0)
            return None
        return self._odom_bridge_pose_msg

    # --- main loop -----------------------------------------------------------

    def _tick(self) -> None:
        now = self._wall_now_sec()
        candidate_scores = None
        cross_check_mismatch_m = None
        if self._pending_reply is not None:
            candidate_scores = self._pending_reply
            cross_check_mismatch_m = self._pending_cross_check_mismatch
            self._pending_reply = None
            self._pending_cross_check_mismatch = None
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

        odom_bridge_pose = self._odom_bridge_status()

        obs = rsp.SupervisorObservation(
            now_sec=now,
            reinitialization_requested=self._requested,
            candidate_scores=candidate_scores,
            best_fitness=best_fitness,
            fitness_observed_sec=self._fitness_observed_sec,
            stable_tracking=stable_tracking,
            cross_check_mismatch_m=cross_check_mismatch_m,
            odom_bridge_available=odom_bridge_pose is not None,
        )
        decision = rsp.decide(self.params, self.state, obs)
        self.state = decision.state

        if decision.reason in (
            "recovery_confirmed",
            "cross_check_timeout",
            "request_cleared",
            "standdown_cleared",
            "exhausted_cleared",
        ):
            self._clear_wall_seed_motion_history()

        if decision.reason in ("recovery_confirmed", "cross_check_timeout"):
            self._awaiting_stable_window = True
            self._stable_window_count = 0
            self._stable_window_logged = False
            self._log_event(decision.reason)
        elif decision.reason in (
            "reset_published",
            "next_candidate",
            "recovery_failed_exhausted",
            "attempts_exhausted",
            "query_timeout",
            "weak_candidate_rejected",
            "recovery_unconfirmed",
            "standdown_cleared",
            "confirm_cross_check_query",
            "self_clear_cross_check_query",
            "cross_check_mismatch",
            "cross_check_reseed",
            "odom_bridge_reset_published",
            "odom_bridge_unconfirmed",
        ):
            self._log_event(decision.reason)

        self._update_stable_window_tracking()

        if decision.action == rsp.ACTION_QUERY:
            self._issue_query()
        elif decision.action == rsp.ACTION_PUBLISH_RESET:
            if decision.candidate_source == "odom_bridge":
                self._publish_odom_bridge_reset()
            else:
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

    def _log_event(self, event_name: str) -> None:
        if self._event_log_csv is None:
            return
        with self._event_log_csv.open("a", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerow([f"{time.time():.6f}", event_name])

    def _update_stable_window_tracking(self) -> None:
        if not self._awaiting_stable_window or self._stable_window_logged:
            return
        if self._requested or self._stable_tracking is not True:
            self._stable_window_count = 0
            return
        self._stable_window_count += 1
        if self._stable_window_count >= self.params.recovery_confirmation_samples:
            self._log_event("stable_recovered_request_window")
            self._stable_window_logged = True
            self._awaiting_stable_window = False

    def _clear_wall_seed_motion_history(self) -> None:
        # The sim fix-to-fix history is grounded in bag-clock scan stamps from G2
        # fixes and is independent of localizer state, so it must survive
        # request_cleared/episode boundaries; estimate_sim_fix_velocity's max_dt
        # bound and estimate_pose_delta_from_sim_fix's staleness clamp already
        # guard against stale reuse.
        self._prev_fix = None
        self._current_query_velocity = rsp.SeedVelocity()
        self._current_query_pose_delta = None
        self._current_query_issue_time = None
        self._current_query_candidate_age_sec = None
        self._current_query_response_sec = None

    def _reset_failed_query_state(self) -> None:
        """Shared reset for an empty or unparseable query reply.

        The empty tuple in _pending_reply still reaches the policy so the
        failed attempt is counted. _prev_sim_fix is kept: the next good reply
        can still pair with the fix from before the failure.
        """
        self._candidates = []
        self._current_query_candidate_age_sec = None
        self._current_query_response_sec = None
        self._sim_fix_velocity = rsp.SeedVelocity()
        self._pending_reply = ()

    def _issue_query(self) -> None:
        if not self.query_client.service_is_ready():
            self.get_logger().warn("query service not available; will retry")
            return
        future = self.query_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_query_response)
        self._query_in_flight = True
        # The candidate this query returns is a fix of the scan available now, so
        # stamp the issue time for the query->publish latency used in compensation.
        self._query_issue_time = self._wall_now_sec()
        self._query_issue_pose_trusted = self._pose_velocity_trusted()
        if (self._last_pose_x is not None and self._last_pose_y is not None
                and self._last_pose_observed_sec is not None
                and self._query_issue_time - self._last_pose_observed_sec
                <= self.max_seed_latency):
            self._query_issue_pose = (
                self._last_pose_x, self._last_pose_y, self._last_pose_observed_sec)
        else:
            self._query_issue_pose = None
        self._query_issue_velocity = self._seed_velocity_tracker.velocity_at(
            self._query_issue_time, self.seed_velocity_max_age)

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
            self._reset_failed_query_state()
            return
        try:
            summary = json.loads(response.message)
            candidates = summary.get("candidates")
            if not candidates:
                # Back-compat with a G2 that only reports the single "top" candidate.
                candidates = [summary["top"]]
            scores = tuple(float(c["score"]) for c in candidates)
            candidate_age_sec = self._parse_nonnegative_float(
                summary.get("candidate_age_sec"))
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn("could not parse query reply: %s" % exc)
            self._reset_failed_query_state()
            return
        if self.enable_bbs_shadow_motion_gate:
            scores = self._apply_bbs_shadow_motion_gate(
                summary, candidates, scores)
        self._candidates = candidates
        self._pending_reply = scores
        self._current_query_velocity = rsp.SeedVelocity()
        self._current_query_issue_time = self._query_issue_time
        self._current_query_candidate_age_sec = candidate_age_sec
        self._current_query_response_sec = self._wall_now_sec()
        self._update_sim_fix_velocity_from_query(candidates, summary)
        if self.state.name == rsp.STATE_VERIFYING:
            # The verify fix above also becomes the second sim fix-to-fix sample,
            # so a later reset after cross_check_mismatch gets motion compensation.
            self._pending_cross_check_mismatch = self._cross_check_mismatch_m(
                summary, candidates[0] if candidates else None)

    def _apply_bbs_shadow_motion_gate(self, summary, candidates, scores):
        """Withhold BBS candidates until their temporal motion matches odometry."""
        withheld = tuple(0.0 for _ in scores)
        if not candidates or not scores or scores[0] < self.params.min_candidate_score:
            self.get_logger().info(
                "BBS shadow gate: weak/empty top candidate withheld")
            return withheld
        scan_stamp_sec = self._parse_nonnegative_float(summary.get("scan_stamp_sec"))
        if scan_stamp_sec is None:
            self.get_logger().warn("BBS shadow gate: missing scan stamp; withholding")
            return withheld
        bridge = self._nearest_history_entry(
            self._odom_bridge_history,
            scan_stamp_sec,
            self.bbs_shadow_bridge_stamp_tolerance_sec)
        if bridge is None:
            self.get_logger().warn(
                "BBS shadow gate: no odom bridge sample near %.3f; withholding"
                % scan_stamp_sec)
            return withheld
        try:
            top = candidates[0]
            candidate_pose = (
                float(top["x"]), float(top["y"]),
                math.radians(float(top["yaw_deg"])))
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn("BBS shadow gate: malformed top pose; withholding")
            return withheld
        bridge_pose = (bridge[1], bridge[2], bridge[3])
        result = self._bbs_shadow_gate.observe(candidate_pose, bridge_pose)
        if result.mismatch is None:
            self._log_event("bbs_shadow_primed")
            self.get_logger().info(
                "BBS shadow gate: primed 1/%d; candidate not published"
                % self.bbs_shadow_required_samples)
            return withheld
        mismatch = result.mismatch
        if not result.publish_allowed:
            self._log_event("bbs_shadow_withheld")
            self.get_logger().warn(
                "BBS shadow gate: candidate withheld; relative mismatch=%.2fm/%.1fdeg "
                "consistent_samples=%d/%d"
                % (mismatch.translation_m, mismatch.yaw_deg,
                   result.consistent_samples,
                   self.bbs_shadow_required_samples))
            return withheld

        # Only rank 1 participated in the temporal validation. Keep the rest at
        # zero so the policy cannot walk to an unvalidated candidate from this query.
        self._log_event("bbs_shadow_validated")
        self.get_logger().warn(
            "BBS shadow gate: rank-1 candidate validated before publication; "
            "relative mismatch=%.2fm/%.1fdeg samples=%d"
            % (mismatch.translation_m, mismatch.yaw_deg,
               result.consistent_samples))
        return (scores[0],) + tuple(0.0 for _ in scores[1:])

    def _cross_check_mismatch_m(self, summary, top_candidate):
        # Every "insufficient evidence" branch below returns math.inf, not
        # None: a verify query that comes back empty, weak, or unparseable is
        # not "cannot tell, so assume the reset is good" -- during a genuine
        # recovery a fresh BBS query centered near the (correct) current pose
        # should usually find at least one reasonable-scoring candidate, so a
        # nothing/weak reply is itself mild evidence *against* confirming.
        # (2026-07-18: a fail-open None here let a self-cleared episode
        # confirm recovery off an empty/weak reply -- see g3_live_closed_loop
        # postmortem -- while the localizer was still clearly lost, standing
        # the supervisor down for the rest of the run instead of retrying.)
        # The one deliberate exception is the query timeout path in the
        # policy (STATE_VERIFYING's own fail-open when no reply arrives at
        # all), which this function has no say over.
        if top_candidate is None:
            return math.inf
        try:
            score = float(top_candidate["score"])
        except (KeyError, TypeError, ValueError):
            return math.inf
        if score < self.params.min_candidate_score:
            return math.inf
        scan_stamp_sec = self._parse_nonnegative_float(summary.get("scan_stamp_sec"))
        if scan_stamp_sec is None:
            return math.inf
        try:
            fix_x = float(top_candidate["x"])
            fix_y = float(top_candidate["y"])
        except (KeyError, TypeError, ValueError):
            return math.inf
        nearest = self._nearest_pose_history_entry(scan_stamp_sec, max_delta_sec=2.0)
        if nearest is None:
            # A trustworthy fix with NO localizer pose near its scan stamp is not
            # "cannot tell" -- it means the localizer was rejecting scans (not
            # tracking) while G2 could still fix the scan. Treat it as an
            # unbounded mismatch so the cross-check keeps the episode alive
            # instead of fail-open confirming a dead track.
            return math.inf
        _stamp, pose_x, pose_y = nearest
        return math.hypot(fix_x - pose_x, fix_y - pose_y)

    def _nearest_pose_history_entry(self, stamp_sec, max_delta_sec):
        return self._nearest_history_entry(
            self._pose_history, stamp_sec, max_delta_sec)

    @staticmethod
    def _nearest_history_entry(history, stamp_sec, max_delta_sec):
        if not history:
            return None
        best = None
        best_delta = None
        for entry in history:
            delta = abs(entry[0] - stamp_sec)
            if best_delta is None or delta < best_delta:
                best = entry
                best_delta = delta
        if best is None or best_delta is None or best_delta > max_delta_sec:
            return None
        return best

    def _update_sim_fix_velocity_from_query(self, candidates, summary) -> None:
        if not candidates:
            self._sim_fix_velocity = rsp.SeedVelocity()
            return
        scan_stamp_sec = self._parse_nonnegative_float(summary.get("scan_stamp_sec"))
        if scan_stamp_sec is None:
            self._sim_fix_velocity = rsp.SeedVelocity()
            return
        top = candidates[0]
        raw_x, raw_y = float(top["x"]), float(top["y"])
        velocity = rsp.SeedVelocity()
        if self._prev_sim_fix is not None:
            prev_x, prev_y, prev_scan_stamp_sec = self._prev_sim_fix
            velocity = rsp.estimate_sim_fix_velocity(
                (prev_x, prev_y), prev_scan_stamp_sec,
                (raw_x, raw_y), scan_stamp_sec,
                max_speed_mps=self.max_seed_speed)
        self._sim_fix_velocity = velocity
        self._prev_sim_fix = (raw_x, raw_y, scan_stamp_sec)

    @staticmethod
    def _parse_nonnegative_float(value):
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number) or number < 0.0:
            return None
        return number

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
        if self.prefer_reset_default_z:
            z = self.reset_default_z
        else:
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
            comp_note = " (motion-compensated from %.2f, %.2f; %s)" % (
                raw_x, raw_y, self._last_seed_motion_status)
        elif self.enable_seed_motion:
            comp_note = " (motion-compensation skipped: %s)" % (
                self._last_seed_motion_status,)
        self.get_logger().warn(
            "published /initialpose reset to (%.2f, %.2f, z=%.2f, %.1f deg) score=%s "
            "[candidate %d/%d]%s"
            % (seed_x, seed_y, z, top["yaw_deg"], top["score"],
               candidate_index + 1, len(self._candidates), comp_note))

    def _publish_odom_bridge_reset(self) -> None:
        """Publish /initialpose from the odom bridge's latest pose.

        Unlike a BBS candidate, this pose already carries full 3D position and
        orientation straight from the C++ node's TF composition
        (map -> odom(last accepted) x odom -> base_link(now)) -- no z/roll/pitch
        carry-over and no seed-motion compensation are needed, since it has zero
        query latency by construction (see reinitialization_supervisor_policy's
        odom-bridge section). A tighter covariance than the BBS reset reflects
        the front end's low relative odom drift over a short dropout.
        """
        bridge_pose = self._odom_bridge_status()
        if bridge_pose is None:
            self.get_logger().error(
                "publish_odom_bridge_reset requested with no bridge pose; "
                "skipping")
            return
        p = bridge_pose.pose.pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame_id
        msg.pose.pose.position.x = p.position.x
        msg.pose.pose.position.y = p.position.y
        msg.pose.pose.position.z = p.position.z
        msg.pose.pose.orientation = p.orientation
        cov = [0.0] * 36
        cov[0] = self.odom_bridge_position_std ** 2   # x
        cov[7] = self.odom_bridge_position_std ** 2   # y
        cov[35] = self.odom_bridge_yaw_std ** 2       # yaw
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        self.get_logger().warn(
            "published /initialpose reset from odom bridge to (%.2f, %.2f, z=%.2f) "
            "[map -> odom(last accepted) x odom -> base_link(now), stamp=%d.%09d]"
            % (p.position.x, p.position.y, p.position.z,
               bridge_pose.header.stamp.sec, bridge_pose.header.stamp.nanosec))

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
        sim_delta = None
        if self.enable_seed_motion:
            sim_delta = self._estimate_pose_delta_sim_fix()
            if sim_delta is not None:
                dx, dy, speed, staleness = sim_delta
                self._last_seed_motion_status = (
                    "sim fix-to-fix %.2fm/s, staleness %.2fs" % (speed, staleness))
                return raw_x + dx, raw_y + dy
        # Staleness is real regardless of how well the candidate registered at its
        # scan stamp; the skip gate below applies only to wall-clock paths whose
        # velocity is less trustworthy.
        if candidate_index < len(self._candidates):
            reg_fit = self._candidates[candidate_index].get("registration_fitness")
            if (reg_fit is not None
                    and float(reg_fit) <= self.seed_motion_skip_registration_fitness):
                self._last_seed_motion_status = (
                    "skipped: registration_fitness %.3f <= %.3f"
                    % (float(reg_fit), self.seed_motion_skip_registration_fitness))
                return raw_x, raw_y
        if not self.enable_seed_motion:
            return raw_x, raw_y
        # sim_delta is always None here: the sim fix-to-fix branch above returns
        # when it produced a delta.
        if not self.seed_motion_wall_fallback:
            self._last_seed_motion_status = (
                "sim velocity unavailable; wall fallback disabled")
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
            fix_time = self._current_query_fix_time(issue)
            if self._prev_fix is not None:
                prev_x, prev_y, prev_fix_time = self._prev_fix
                velocity = rsp.estimate_seed_velocity(
                    (prev_x, prev_y), prev_fix_time, (raw_x, raw_y), fix_time,
                    max_speed_mps=self.max_seed_speed)
                if not velocity.valid:
                    dt = fix_time - prev_fix_time
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
            self._prev_fix = (raw_x, raw_y, fix_time)
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
            latency, latency_source = self._seed_latency_sec(issue)
            self._last_seed_motion_status = "applied %s %.2fs" % (
                latency_source, latency)
            return rsp.forward_compensate_xy(
                (raw_x, raw_y), velocity, latency, max_latency_sec=self.max_seed_latency)
        return raw_x, raw_y

    def _seed_latency_sec(self, issue):
        if (self._current_query_candidate_age_sec is not None
                and self._current_query_response_sec is not None):
            response_age = max(
                0.0, self._wall_now_sec() - self._current_query_response_sec)
            return self._current_query_candidate_age_sec + response_age, "candidate age"
        return self._wall_now_sec() - issue, "latency"

    def _current_query_fix_time(self, fallback_issue):
        if (self._current_query_candidate_age_sec is not None
                and self._current_query_response_sec is not None):
            return self._current_query_response_sec - self._current_query_candidate_age_sec
        return fallback_issue

    def _estimate_pose_delta_since_query_issue(self):
        if (not self._query_issue_pose_trusted
                or not self._pose_velocity_trusted()):
            self._last_seed_motion_status = "pose history untrusted during episode"
            return self._estimate_pose_delta_from_query_velocity()
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

    def _estimate_pose_delta_sim_fix(self):
        if (not self._sim_fix_velocity.valid
                or self._last_sim_stamp_sec is None
                or self._prev_sim_fix is None):
            return None
        # A valid velocity is always set together with _prev_sim_fix, whose
        # stamp is the fix the velocity ends at.
        return rsp.estimate_pose_delta_from_sim_fix(
            self._sim_fix_velocity,
            self._prev_sim_fix[2],
            self._last_sim_stamp_sec,
            self.max_seed_latency)

    def _estimate_pose_delta_from_query_velocity(self):
        if self._query_issue_time is None:
            return None
        if not self._query_issue_velocity.valid:
            reason = self._seed_velocity_tracker.velocity_rejection_reason(
                self._query_issue_time, self.seed_velocity_max_age)
            if reason is not None:
                self._last_seed_motion_status = reason
            return None
        latency, _latency_source = self._seed_latency_sec(self._query_issue_time)
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
