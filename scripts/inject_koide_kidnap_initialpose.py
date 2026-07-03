#!/usr/bin/env python3
"""Publish a wrong /initialpose once sim time crosses a trigger threshold."""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


class KidnapInitialPoseInjector(Node):
    def __init__(self) -> None:
        super().__init__("kidnap_initialpose_injector")
        self.declare_parameter("trigger_sim_sec", 22.0)
        self.declare_parameter("trigger_after_first_clock_sec", -1.0)
        self.declare_parameter("x", -36.0)
        self.declare_parameter("y", -58.8)
        self.declare_parameter("z", -11.0)
        self.declare_parameter("yaw_deg", 90.0)
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.trigger_sim_sec = float(self.get_parameter("trigger_sim_sec").value)
        self.trigger_after_first_clock_sec = float(
            self.get_parameter("trigger_after_first_clock_sec").value)
        self.first_clock_sec = None
        self.published = False
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topic = self.get_parameter("initialpose_topic").value
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, qos)
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)

    def _on_clock(self, msg: Clock) -> None:
        if self.published:
            return
        sim_sec = msg.clock.sec + msg.clock.nanosec * 1.0e-9
        if self.first_clock_sec is None:
            self.first_clock_sec = sim_sec
        if self.trigger_after_first_clock_sec >= 0.0:
            elapsed = sim_sec - self.first_clock_sec
            if elapsed < self.trigger_after_first_clock_sec:
                return
        elif sim_sec < self.trigger_sim_sec:
            return
        yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.clock
        pose.header.frame_id = self.get_parameter("global_frame_id").value
        pose.pose.pose.position.x = float(self.get_parameter("x").value)
        pose.pose.pose.position.y = float(self.get_parameter("y").value)
        pose.pose.pose.position.z = float(self.get_parameter("z").value)
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(pose)
        self.published = True
        self.get_logger().warn(
            "kidnap injected at sim %.3f -> (%.2f, %.2f, z=%.2f, yaw=%.1f deg)"
            % (sim_sec, pose.pose.pose.position.x, pose.pose.pose.position.y,
               pose.pose.pose.position.z, math.degrees(yaw)))


def main() -> None:
    rclpy.init(args=sys.argv)
    node = KidnapInitialPoseInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
