#!/usr/bin/env python3
"""Replay a validated external-odometry pose CSV as a clock-synchronous TF."""

import bisect
import csv
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


def _normalized_lerp_quaternion(a, b, ratio):
    if sum(x * y for x, y in zip(a, b)) < 0.0:
        b = tuple(-x for x in b)
    q = tuple((1.0 - ratio) * x + ratio * y for x, y in zip(a, b))
    norm = math.sqrt(sum(x * x for x in q))
    return tuple(x / norm for x in q) if norm > 1e-12 else a


class PoseCsvTfPublisher(Node):
    def __init__(self):
        super().__init__("pose_csv_tf_publisher")
        self.declare_parameter("pose_csv", "")
        self.declare_parameter("parent_frame", "glim_odom")
        self.declare_parameter("child_frame", "livox_frame")
        path = str(self.get_parameter("pose_csv").value)
        if not path:
            raise ValueError("pose_csv is required")
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.samples = []
        with open(path, newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                stamp = float(row["stamp_sec"])
                if stamp <= 0.0:
                    continue
                self.samples.append((
                    stamp,
                    (float(row["position_x"]), float(row["position_y"]),
                     float(row["position_z"])),
                    (float(row["orientation_x"]), float(row["orientation_y"]),
                     float(row["orientation_z"]), float(row["orientation_w"])),
                ))
        if len(self.samples) < 2:
            raise ValueError("pose_csv must contain at least two positive-stamp rows")
        self.stamps = [sample[0] for sample in self.samples]
        self.pub = self.create_publisher(TFMessage, "/tf", 100)
        self.sub = self.create_subscription(
            Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.get_logger().info(
            "replaying %d external-odometry poses: %s -> %s"
            % (len(self.samples), self.parent_frame, self.child_frame))

    def _on_clock(self, msg):
        stamp = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if stamp < self.stamps[0] or stamp > self.stamps[-1]:
            return
        right = bisect.bisect_left(self.stamps, stamp)
        if right == 0:
            left = right
            ratio = 0.0
        elif right == len(self.stamps):
            left = right - 1
            right = left
            ratio = 0.0
        else:
            left = right - 1
            dt = self.stamps[right] - self.stamps[left]
            ratio = 0.0 if dt <= 0.0 else (stamp - self.stamps[left]) / dt
        _, p0, q0 = self.samples[left]
        _, p1, q1 = self.samples[right]
        position = tuple((1.0 - ratio) * a + ratio * b for a, b in zip(p0, p1))
        quaternion = _normalized_lerp_quaternion(q0, q1, ratio)
        transform = TransformStamped()
        transform.header.stamp = msg.clock
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.pub.publish(TFMessage(transforms=[transform]))


def main():
    rclpy.init()
    node = PoseCsvTfPublisher()
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
