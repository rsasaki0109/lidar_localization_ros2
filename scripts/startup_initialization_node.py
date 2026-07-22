#!/usr/bin/env python3
"""Safe startup pose restoration, global initialization, and pose persistence."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))

import quickstart_model as model  # noqa: E402


_ACCEPT_ACTIONS = frozenset({
    "accept_measurement",
    "accept_measurement_with_warning",
    "accept_measurement_after_recovery",
})
_ACCEPT_STATES = frozenset({"tracking", "degraded", "recovering"})


class StartupInitializationNode(Node):
    def __init__(self) -> None:
        super().__init__("startup_initialization")
        self.declare_parameter("map_path", "")
        self.declare_parameter("pose_state_path", "")
        self.declare_parameter("restore_saved_pose", True)
        self.declare_parameter("initial_pose_preconfigured", False)
        self.declare_parameter("saved_pose_max_age_sec", 0.0)
        self.declare_parameter("preverify_saved_pose", True)
        self.declare_parameter("saved_pose_score_threshold", 6.0)
        self.declare_parameter("saved_pose_preverify_attempts", 3)
        self.declare_parameter("enable_global_initialization", False)
        self.declare_parameter("require_global_registration_scoring", True)
        self.declare_parameter("cloud_topic", "/velodyne_points")
        self.declare_parameter("pose_topic", "/pcl_pose")
        self.declare_parameter("alignment_status_topic", "/alignment_status")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("query_service", "/global_localization_node/query")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("default_z_m", 0.0)
        self.declare_parameter("min_candidate_score", 0.6)
        self.declare_parameter("min_score_margin", 0.05)
        self.declare_parameter("max_candidate_age_sec", 30.0)
        self.declare_parameter("query_timeout_sec", 30.0)
        self.declare_parameter("verification_timeout_sec", 8.0)
        self.declare_parameter("verification_fitness_threshold", 1.5)
        self.declare_parameter("verification_samples", 3)
        self.declare_parameter("max_global_attempts", 6)
        self.declare_parameter("global_consensus_samples", 2)
        self.declare_parameter("global_consensus_translation_m", 2.0)
        self.declare_parameter("global_consensus_yaw_deg", 20.0)
        self.declare_parameter("save_interval_sec", 2.0)
        self.declare_parameter("position_std_m", 0.5)
        self.declare_parameter("yaw_std_rad", 0.25)

        self.global_frame = str(self.get_parameter("global_frame_id").value)
        self.map_path = Path(str(self.get_parameter("map_path").value)).expanduser()
        state_value = str(self.get_parameter("pose_state_path").value)
        self.state_path = Path(state_value).expanduser() if state_value else None
        self.restore_saved_pose = bool(self.get_parameter("restore_saved_pose").value)
        self.initial_pose_preconfigured = bool(
            self.get_parameter("initial_pose_preconfigured").value)
        self.enable_global = bool(
            self.get_parameter("enable_global_initialization").value)
        self.require_global_registration_scoring = bool(
            self.get_parameter("require_global_registration_scoring").value)
        self.default_z = float(self.get_parameter("default_z_m").value)
        self.save_interval_sec = float(self.get_parameter("save_interval_sec").value)
        self.position_std = float(self.get_parameter("position_std_m").value)
        self.yaw_std = float(self.get_parameter("yaw_std_rad").value)
        self.params = model.StartupParams(
            min_candidate_score=float(self.get_parameter("min_candidate_score").value),
            min_score_margin=float(self.get_parameter("min_score_margin").value),
            max_candidate_age_sec=float(
                self.get_parameter("max_candidate_age_sec").value),
            query_timeout_sec=float(self.get_parameter("query_timeout_sec").value),
            verification_timeout_sec=float(
                self.get_parameter("verification_timeout_sec").value),
            verification_fitness_threshold=float(
                self.get_parameter("verification_fitness_threshold").value),
            verification_samples=max(
                1, int(self.get_parameter("verification_samples").value)),
            max_global_attempts=max(
                1, int(self.get_parameter("max_global_attempts").value)),
            global_consensus_samples=max(
                1, int(self.get_parameter("global_consensus_samples").value)),
            global_consensus_translation_m=float(
                self.get_parameter("global_consensus_translation_m").value),
            global_consensus_yaw_deg=float(
                self.get_parameter("global_consensus_yaw_deg").value),
        )
        parameter_error = model.validate_startup_params(self.params)
        if parameter_error is not None:
            raise ValueError(parameter_error)
        self.state = model.StartupState()
        self.map_identity = model.compute_map_identity(self.map_path)
        self.saved_pose = None
        self.saved_pose_reason = "disabled"
        if self.restore_saved_pose and self.state_path is not None:
            loaded = model.load_stored_pose(
                self.state_path,
                self.map_identity,
                self.global_frame,
                now_sec=time.time(),
                max_age_sec=float(
                    self.get_parameter("saved_pose_max_age_sec").value),
            )
            self.saved_pose = loaded.pose
            self.saved_pose_reason = loaded.reason
        self.saved_pose_preverified = self.saved_pose is not None and not bool(
            self.get_parameter("preverify_saved_pose").value)
        self.saved_pose_score_threshold = float(
            self.get_parameter("saved_pose_score_threshold").value)
        self.saved_pose_preverify_attempts = max(
            1, int(self.get_parameter("saved_pose_preverify_attempts").value))
        self.saved_pose_preverify_count = 0
        self.saved_pose_scorer = None
        self._pointcloud2_xyz_array = None
        if self.saved_pose is not None and not self.saved_pose_preverified:
            self._prepare_saved_pose_scorer()

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cloud_qos = QoSProfile(depth=1)
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            reliable,
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), transient)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            cloud_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            transient,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("alignment_status_topic").value),
            self._on_alignment,
            transient,
        )
        self.query_client = self.create_client(
            Trigger, str(self.get_parameter("query_service").value))
        self.create_timer(0.25, self._tick)

        self.scan_ready = False
        self.latest_pose = None
        self.latest_fitness = None
        self.latest_tracking_good = False
        self.diagnostic_fresh = False
        self.stable_samples = 0
        self.last_save_sec = None
        self.pending_scores = None
        self.pending_candidate_age = None
        self.pending_top_pose = None
        self.pending_scan_stamp_sec = None
        self.latest_cloud_stamp_sec = None
        self.last_query_scan_stamp_sec = None
        self.candidates = []
        self.query_in_flight = False
        self.last_report = None
        self._report(
            "starting",
            extra={"saved_pose": self.saved_pose_reason, "map_sha256": self.map_identity.sha256},
        )

    def _on_cloud(self, _msg: PointCloud2) -> None:
        self.latest_cloud_stamp_sec = (
            float(_msg.header.stamp.sec) + float(_msg.header.stamp.nanosec) * 1e-9)
        if self.saved_pose is not None and not self.saved_pose_preverified:
            if self.saved_pose_scorer is not None:
                self._preverify_saved_pose(_msg)
                if (
                    not self.saved_pose_preverified
                    and self.saved_pose_preverify_count < self.saved_pose_preverify_attempts
                ):
                    return
            else:
                self.saved_pose_reason = "preverification_unavailable"
        self.scan_ready = True

    def _prepare_saved_pose_scorer(self) -> None:
        try:
            import global_localization_query as global_query
            import make_bbs_relocalization_attempts as bbs_engine

            global_query._append_module_dirs("g2_ndt_score")
            import g2_ndt_score

            self.saved_pose_scorer = g2_ndt_score.MapNdtScorer(str(self.map_path))
            self._pointcloud2_xyz_array = bbs_engine.pointcloud2_xyz_array
            self.saved_pose_reason = "awaiting_scan_preverification"
        except (ImportError, RuntimeError, OSError) as exc:
            self.saved_pose_scorer = None
            self.saved_pose_reason = "preverification_unavailable"
            self.get_logger().warning(
                "saved pose will not be auto-published because NDT preverification "
                f"is unavailable: {exc}")

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        x, y, z, w = orientation
        siny_cosp = 2.0 * (
            w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _preverify_saved_pose(self, cloud: PointCloud2) -> None:
        self.saved_pose_preverify_count += 1
        try:
            points = self._pointcloud2_xyz_array(cloud)
            if points.shape[0] == 0:
                self.saved_pose_reason = "empty_preverification_scan"
                return
            position = self.saved_pose.position
            orientation = self.saved_pose.orientation
            result = self.saved_pose_scorer.score_candidate(
                points,
                position[0],
                position[1],
                position[2],
                self._yaw_from_quaternion(orientation),
            )
            if model.saved_pose_score_acceptable(
                result.converged, result.fitness, self.saved_pose_score_threshold):
                self.saved_pose_preverified = True
                self.saved_pose_reason = "scan_preverified"
                self.get_logger().info(
                    "saved pose passed pre-publication NDT verification: "
                    f"fitness={result.fitness:.3f}")
            else:
                self.saved_pose_reason = "scan_preverification_failed"
        except (RuntimeError, TypeError, ValueError) as exc:
            self.saved_pose_reason = "scan_preverification_error"
            self.get_logger().warning(f"saved pose preverification failed: {exc}")
        finally:
            if (
                self.saved_pose_preverified
                or self.saved_pose_preverify_count >= self.saved_pose_preverify_attempts
            ):
                self.saved_pose_scorer = None
                self._pointcloud2_xyz_array = None

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg
        self._maybe_save_pose()

    def _on_alignment(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            values = {entry.key: entry.value for entry in status.values}
            if "fitness_score" not in values:
                continue
            try:
                fitness = float(values["fitness_score"])
            except ValueError:
                continue
            reinit = str(values.get("reinitialization_requested", "false")).lower() == "true"
            recovery_state = str(values.get("recovery_state", ""))
            recovery_action = str(values.get("recovery_action", ""))
            if recovery_state or recovery_action:
                good = (
                    status.message == "ok"
                    and recovery_state in _ACCEPT_STATES
                    and recovery_action in _ACCEPT_ACTIONS
                    and not reinit
                )
            else:
                good = status.message == "ok" and not reinit
            self.latest_fitness = fitness
            self.latest_tracking_good = good
            self.diagnostic_fresh = True
            if (
                good
                and math.isfinite(fitness)
                and fitness <= self.params.verification_fitness_threshold
            ):
                self.stable_samples += 1
            else:
                self.stable_samples = 0
            self._maybe_save_pose()
            return

    def _maybe_save_pose(self) -> None:
        if (
            self.state_path is None
            or self.latest_pose is None
            or self.stable_samples < self.params.verification_samples
        ):
            return
        now = time.time()
        if self.last_save_sec is not None and now - self.last_save_sec < self.save_interval_sec:
            return
        msg = self.latest_pose
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().error(
                "refusing to save pose in frame "
                f"{msg.header.frame_id!r}; expected {self.global_frame!r}")
            return
        pose = msg.pose.pose
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        record = model.StoredPose(
            schema_version=model.POSE_STATE_SCHEMA,
            map_identity=self.map_identity,
            frame_id=msg.header.frame_id or self.global_frame,
            stamp_sec=stamp_sec,
            saved_at_sec=now,
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation=(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            covariance=tuple(float(value) for value in msg.pose.covariance),
        )
        try:
            model.save_stored_pose(self.state_path, record)
            self.last_save_sec = now
        except (OSError, ValueError) as exc:
            self.get_logger().error(f"could not save verified pose: {exc}")

    def _tick(self) -> None:
        # The G2 service can take longer than the localizer to load a large map.
        # Keep the configured fallback alive while its service is starting.
        global_available = self.enable_global
        observation = model.StartupObservation(
            now_sec=time.monotonic(),
            scan_ready=self.scan_ready,
            saved_pose_available=(
                self.saved_pose is not None and self.saved_pose_preverified),
            global_available=global_available,
            preconfigured_pose_available=self.initial_pose_preconfigured,
            query_in_flight=self.query_in_flight,
            query_candidate_scores=self.pending_scores,
            query_candidate_age_sec=self.pending_candidate_age,
            query_top_pose=self.pending_top_pose,
            query_scan_stamp_sec=self.pending_scan_stamp_sec,
            diagnostic_fresh=self.diagnostic_fresh,
            tracking_good=self.latest_tracking_good,
            fitness=self.latest_fitness,
        )
        self.pending_scores = None
        self.pending_candidate_age = None
        self.pending_top_pose = None
        self.pending_scan_stamp_sec = None
        self.diagnostic_fresh = False
        decision = model.decide_startup(self.params, self.state, observation)
        self.state = decision.state
        if decision.action == model.ACTION_PUBLISH_SAVED:
            self._publish_saved_pose()
        elif decision.action == model.ACTION_QUERY_GLOBAL:
            self._issue_query()
        elif decision.action == model.ACTION_PUBLISH_GLOBAL:
            self._publish_global_candidate(decision.candidate_index)
        elif (
            self.state.name == model.STATE_QUERYING_GLOBAL
            and not self.query_in_flight
            and self.pending_scores is None
            and self.query_client.service_is_ready()
        ):
            self._issue_query()
        if decision.action == model.ACTION_NEEDS_OPERATOR:
            self._report(
                decision.reason,
                level="error",
                extra={"action": "Set 2D Pose Estimate in RViz on /initialpose"},
            )
        else:
            self._report(decision.reason)

    def _issue_query(self) -> None:
        if self.query_in_flight:
            return
        if not self.query_client.service_is_ready():
            self.get_logger().warning("global localization service is not ready")
            return
        if (
            self.last_query_scan_stamp_sec is not None
            and (
                self.latest_cloud_stamp_sec is None
                or self.latest_cloud_stamp_sec <= self.last_query_scan_stamp_sec + 1.0e-9
            )
        ):
            return
        self.query_in_flight = True
        future = self.query_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_query_response)

    def _on_query_response(self, future) -> None:
        self.query_in_flight = False
        try:
            response = future.result()
            payload = json.loads(response.message)
            if not response.success:
                raise RuntimeError("G2 query returned no candidate")
            if not model.global_registration_scoring_acceptable(
                payload.get("registration_scoring_enabled"),
                self.require_global_registration_scoring,
            ):
                detail = payload.get(
                    "registration_scoring_error", "3D registration scorer unavailable")
                raise RuntimeError(
                    "refusing unverified 2D-only global candidate: " + str(detail))
            candidates = payload.get("candidates") or (
                [payload["top"]] if "top" in payload else [])
            self.candidates = candidates
            self.pending_scores = tuple(float(item["score"]) for item in candidates)
            age = payload.get("candidate_age_sec")
            self.pending_candidate_age = float(age) if age is not None else None
            stamp = payload.get("scan_stamp_sec")
            self.pending_scan_stamp_sec = float(stamp) if stamp is not None else None
            self.last_query_scan_stamp_sec = self.pending_scan_stamp_sec
            top = candidates[0]
            self.pending_top_pose = (
                float(top["x"]),
                float(top["y"]),
                math.radians(float(top["yaw_deg"])),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"global localization query failed: {exc}")
            self.candidates = []
            self.pending_scores = ()
            self.pending_candidate_age = None
            self.pending_top_pose = None
            self.pending_scan_stamp_sec = None

    def _new_initialpose(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.pose.covariance[0] = self.position_std ** 2
        msg.pose.covariance[7] = self.position_std ** 2
        msg.pose.covariance[35] = self.yaw_std ** 2
        return msg

    def _publish_saved_pose(self) -> None:
        if self.saved_pose is None:
            return
        msg = self._new_initialpose()
        position = self.saved_pose.position
        orientation = self.saved_pose.orientation
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = orientation
        msg.pose.covariance = list(self.saved_pose.covariance)
        self.initialpose_pub.publish(msg)
        self.get_logger().info("published map-matched saved pose for verification")

    def _publish_global_candidate(self, index: int) -> None:
        if index >= len(self.candidates):
            return
        candidate = self.candidates[index]
        yaw = math.radians(float(candidate["yaw_deg"]))
        msg = self._new_initialpose()
        msg.pose.pose.position.x = float(candidate["x"])
        msg.pose.pose.position.y = float(candidate["y"])
        if candidate.get("z") is not None:
            msg.pose.pose.position.z = float(candidate["z"])
        elif self.saved_pose is not None:
            msg.pose.pose.position.z = self.saved_pose.position[2]
        elif self.latest_pose is not None:
            msg.pose.pose.position.z = self.latest_pose.pose.pose.position.z
        else:
            msg.pose.pose.position.z = self.default_z
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            "published globally searched pose for verification: "
            f"x={candidate['x']} y={candidate['y']} yaw={candidate['yaw_deg']}"
        )

    def _report(self, reason: str, level: str = "info", extra=None) -> None:
        payload = {
            "state": self.state.name,
            "source": self.state.source,
            "reason": reason,
            "global_attempts": self.state.global_attempts,
        }
        if extra:
            payload.update(extra)
        text = json.dumps(payload, sort_keys=True)
        if text == self.last_report:
            return
        self.last_report = text
        self.status_pub.publish(String(data=text))
        if level == "error":
            self.get_logger().error(f"quickstart: {text}")
        elif level == "warning":
            self.get_logger().warning(f"quickstart: {text}")
        else:
            self.get_logger().info(f"quickstart: {text}")


def main() -> None:
    rclpy.init()
    node = StartupInitializationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
