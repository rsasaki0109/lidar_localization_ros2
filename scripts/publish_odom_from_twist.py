#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomFromTwistPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_from_twist_publisher")

        self.declare_parameter("twist_topic", "/twist")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("max_dt_sec", 0.5)
        self.declare_parameter("stamp_with_current_time", True)

        twist_topic = self.get_parameter("twist_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.max_dt_sec = self.get_parameter("max_dt_sec").get_parameter_value().double_value
        self.stamp_with_current_time = (
            self.get_parameter("stamp_with_current_time").get_parameter_value().bool_value
        )

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if publish_tf else None
        self.subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            twist_topic,
            self._twist_callback,
            50,
        )

        self.last_stamp: Optional[Time] = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.get_logger().info(
            f"Publishing odom on {odom_topic} by integrating {twist_topic} with "
            f"{self.odom_frame_id}->{self.base_frame_id}"
        )

    def _twist_callback(self, msg: TwistWithCovarianceStamped) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        twist = msg.twist.twist

        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1e-9
            if dt < 0.0:
                self.get_logger().warn("Received twist message with non-monotonic timestamp, skipping integration.")
                dt = 0.0
            elif dt > self.max_dt_sec:
                self.get_logger().warn(
                    f"Twist dt {dt:.3f}s exceeds max_dt_sec {self.max_dt_sec:.3f}s, clamping integration step."
                )
                dt = self.max_dt_sec

            if dt > 0.0:
                cos_yaw = math.cos(self.yaw)
                sin_yaw = math.sin(self.yaw)
                world_vx = twist.linear.x * cos_yaw - twist.linear.y * sin_yaw
                world_vy = twist.linear.x * sin_yaw + twist.linear.y * cos_yaw
                self.x += world_vx * dt
                self.y += world_vy * dt
                self.z += twist.linear.z * dt
                self.roll = normalize_angle(self.roll + twist.angular.x * dt)
                self.pitch = normalize_angle(self.pitch + twist.angular.y * dt)
                self.yaw = normalize_angle(self.yaw + twist.angular.z * dt)

        self.last_stamp = stamp
        self._publish(msg)

    def _publish(self, msg: TwistWithCovarianceStamped) -> None:
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        stamp = self.get_clock().now().to_msg() if self.stamp_with_current_time else msg.header.stamp

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist = msg.twist
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = 0.10
        odom.pose.covariance[21] = 0.10
        odom.pose.covariance[28] = 0.10
        odom.pose.covariance[35] = 0.10
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomFromTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
