#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool


class ReinitializationSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("reinitialization_supervisor")

        self.declare_parameter("request_topic", "/reinitialization_requested")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("use_latest_pose", False)
        self.declare_parameter("publish_count", 1)
        self.declare_parameter("publish_interval_sec", 0.2)
        self.declare_parameter("republish_cooldown_sec", 15.0)
        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_z", 0.0)
        self.declare_parameter("initial_pose_qx", 0.0)
        self.declare_parameter("initial_pose_qy", 0.0)
        self.declare_parameter("initial_pose_qz", 0.0)
        self.declare_parameter("initial_pose_qw", 1.0)

        request_topic = self.get_parameter("request_topic").get_parameter_value().string_value
        initialpose_topic = self.get_parameter("initialpose_topic").get_parameter_value().string_value
        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.global_frame_id = self.get_parameter("global_frame_id").get_parameter_value().string_value
        self.use_latest_pose = self.get_parameter("use_latest_pose").get_parameter_value().bool_value
        self.publish_count = max(1, self.get_parameter("publish_count").get_parameter_value().integer_value)
        self.publish_interval = Duration(
            seconds=max(
                0.05,
                self.get_parameter("publish_interval_sec").get_parameter_value().double_value,
            )
        )
        self.republish_cooldown = Duration(
            seconds=max(
                self.publish_interval.nanoseconds * 1e-9,
                self.get_parameter("republish_cooldown_sec").get_parameter_value().double_value,
            )
        )

        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = self.global_frame_id
        self.initial_pose.pose.pose.position.x = (
            self.get_parameter("initial_pose_x").get_parameter_value().double_value
        )
        self.initial_pose.pose.pose.position.y = (
            self.get_parameter("initial_pose_y").get_parameter_value().double_value
        )
        self.initial_pose.pose.pose.position.z = (
            self.get_parameter("initial_pose_z").get_parameter_value().double_value
        )
        self.initial_pose.pose.pose.orientation.x = (
            self.get_parameter("initial_pose_qx").get_parameter_value().double_value
        )
        self.initial_pose.pose.pose.orientation.y = (
            self.get_parameter("initial_pose_qy").get_parameter_value().double_value
        )
        self.initial_pose.pose.pose.orientation.z = (
            self.get_parameter("initial_pose_qz").get_parameter_value().double_value
        )
        self.initial_pose.pose.pose.orientation.w = (
            self.get_parameter("initial_pose_qw").get_parameter_value().double_value
        )

        self.latest_pose: Optional[PoseWithCovarianceStamped] = None
        self.request_active = False
        self.pending_publish_count = 0
        self.last_publish_time = None
        self.next_publish_time = None

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, initialpose_topic, 10)
        request_qos = QoSProfile(depth=1)
        request_qos.reliability = ReliabilityPolicy.RELIABLE
        request_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.request_sub = self.create_subscription(Bool, request_topic, self._on_request, request_qos)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._on_pose, 10
        )
        timer_period = max(0.05, min(self.publish_interval.nanoseconds * 1e-9, 0.25))
        self.timer = self.create_timer(timer_period, self._tick)

        self.get_logger().info(
            "Watching "
            f"{request_topic} and republishing {initialpose_topic} with "
            f"{'latest localization pose' if self.use_latest_pose else 'configured initial pose'}; "
            f"burst={self.publish_count}, interval={self.publish_interval.nanoseconds * 1e-9:.2f}s, "
            f"cooldown={self.republish_cooldown.nanoseconds * 1e-9:.2f}s"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg

    def _on_request(self, msg: Bool) -> None:
        if msg.data:
            if not self.request_active:
                self.get_logger().warn("Received reinitialization request")
            self.request_active = True
            self._start_burst_if_due("request_active")
            return

        if self.request_active or self.pending_publish_count > 0:
            self.get_logger().info("Reinitialization request cleared")
        self.request_active = False
        self.pending_publish_count = 0
        self.next_publish_time = None

    def _tick(self) -> None:
        if not self.request_active:
            return

        now = self.get_clock().now()
        if self.pending_publish_count > 0:
            if self.next_publish_time is not None and now >= self.next_publish_time:
                self._publish_once()
            return

        self._start_burst_if_due("request_still_active")

    def _start_burst_if_due(self, reason: str) -> None:
        now = self.get_clock().now()
        if self.pending_publish_count > 0:
            return
        if self.last_publish_time is not None and now - self.last_publish_time < self.republish_cooldown:
            return
        self.pending_publish_count = self.publish_count
        self.next_publish_time = now
        self.get_logger().warn(f"Starting reinitialization publish burst: {reason}")

    def _make_pose_message(self) -> PoseWithCovarianceStamped:
        if self.use_latest_pose and self.latest_pose is not None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.global_frame_id
            msg.pose = self.latest_pose.pose
            return msg

        if self.use_latest_pose and self.latest_pose is None:
            self.get_logger().warn(
                "Reinitialization requested before any localization pose arrived; using configured initial pose"
            )

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.global_frame_id
        msg.pose = self.initial_pose.pose
        return msg

    def _publish_once(self) -> None:
        now = self.get_clock().now()
        msg = self._make_pose_message()
        msg.header.stamp = now.to_msg()
        self.initialpose_pub.publish(msg)
        self.last_publish_time = now
        self.pending_publish_count -= 1
        if self.pending_publish_count > 0:
            self.next_publish_time = now + self.publish_interval
        else:
            self.next_publish_time = None


def main() -> None:
    rclpy.init()
    node = ReinitializationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
