#!/usr/bin/env python3
"""Delay GLIM's point input so same-scan IMU callbacks are consumed first.

The offline ``glim_rosbag`` path inserts messages in rosbag receive order. DDS
does not preserve that order across the IMU and point topics, even though each
topic is individually reliable. Holding only GLIM's point copy briefly makes
the live node see the corresponding IMU prefix before each scan, while leaving
message stamps and the localizer's original point topic unchanged.
"""

from collections import deque
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class GlimPointDelayRelay(Node):
    def __init__(self):
        super().__init__("glim_point_delay_relay")
        self.declare_parameter("input_topic", "/livox/points")
        self.declare_parameter("output_topic", "/glim_frontend/points")
        self.declare_parameter("delay_sec", 0.25)
        self.delay_sec = float(self.get_parameter("delay_sec").value)
        if self.delay_sec < 0.0:
            raise ValueError("delay_sec must be nonnegative")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.queue = deque()
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, qos)
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value, self._on_point, qos)
        self.timer = self.create_timer(0.005, self._flush_ready)
        self.get_logger().info(
            "delaying GLIM point input by %.3f s: %s -> %s"
            % (self.delay_sec, self.get_parameter("input_topic").value,
               self.get_parameter("output_topic").value))

    def _on_point(self, msg):
        self.queue.append((time.monotonic(), msg))

    def _flush_ready(self):
        cutoff = time.monotonic() - self.delay_sec
        while self.queue and self.queue[0][0] <= cutoff:
            _, msg = self.queue.popleft()
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = GlimPointDelayRelay()
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
