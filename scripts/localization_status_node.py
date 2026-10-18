#!/usr/bin/env python3
"""One plain line per change of tracking health, plus a status label in RViz.

Diagnostics only: listens to /startup_initialization/status and /alignment_status,
prints when tracking starts, weakens, is lost, or comes back, and publishes the
current status as a text marker above the robot and as JSON on
~/status. Never changes acceptance, startup or recovery behavior.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parent))

import localization_status_model as model


class LocalizationStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_status")
        self.declare_parameter("alignment_status_topic", "/alignment_status")
        self.declare_parameter("startup_status_topic", "/startup_initialization/status")
        self.declare_parameter("pose_topic", "/pcl_pose")
        self.declare_parameter("marker_topic", "~/marker")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("manual_pose_step", "set 2D Pose Estimate in RViz")
        self.declare_parameter("automatic_recovery", False)
        self.declare_parameter("debounce_sec", 2.0)
        self.declare_parameter("no_scan_timeout_sec", 5.0)
        self.declare_parameter("marker_height_m", 2.0)

        self.tracker = model.StatusTracker(
            manual_pose_step=str(self.get_parameter("manual_pose_step").value),
            debounce_sec=float(self.get_parameter("debounce_sec").value),
            no_scan_timeout_sec=float(self.get_parameter("no_scan_timeout_sec").value),
            automatic_recovery=bool(self.get_parameter("automatic_recovery").value),
        )
        self.global_frame = str(self.get_parameter("global_frame_id").value)
        self.marker_height = float(self.get_parameter("marker_height_m").value)
        self.position = (0.0, 0.0, 0.0)
        self.last_status_text = ""

        transient = QoSProfile(depth=1)
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient.reliability = ReliabilityPolicy.RELIABLE
        self.marker_pub = self.create_publisher(
            Marker, str(self.get_parameter("marker_topic").value), 1
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), transient
        )
        self.create_subscription(
            String,
            str(self.get_parameter("startup_status_topic").value),
            self._on_startup,
            transient,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("alignment_status_topic").value),
            self._on_alignment,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            10,
        )
        self.create_timer(0.5, self._on_timer)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_startup(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(payload, dict):
            self.tracker.update_startup(payload, self._now())
            self._publish()

    def _on_alignment(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            if "failure_category" not in values:
                continue
            self._report(self.tracker.update_alignment(values, self._now()))
            return

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def _on_timer(self) -> None:
        self._report(self.tracker.tick(self._now()))
        self._publish()

    def _report(self, changes) -> None:
        for status in changes:
            if status.severity == model.SEVERITY_ERROR:
                self.get_logger().error(status.line)
            elif status.severity == model.SEVERITY_WARNING:
                self.get_logger().warning(status.line)
            else:
                self.get_logger().info(status.line)
        if changes:
            self._publish()

    def _publish(self) -> None:
        status = self.tracker.display()
        text = json.dumps(
            {"phase": status.phase, "category": status.category, "label": status.label},
            sort_keys=True,
        )
        if text != self.last_status_text:
            self.last_status_text = text
            self.status_pub.publish(String(data=text))

        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "localization_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2] + self.marker_height
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.8
        r, g, b, a = model.PHASE_COLORS.get(status.phase, (1.0, 1.0, 1.0, 1.0))
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, a
        marker.text = status.label or status.phase
        self.marker_pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = LocalizationStatusNode()
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
