#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class CmdVelOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_odom_publisher")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("publish_tf", True)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.cmd_vel_timeout_sec = (
            self.get_parameter("cmd_vel_timeout_sec").get_parameter_value().double_value
        )
        publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if publish_tf else None
        self.subscription = self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 50)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_cmd_time: Optional[rclpy.time.Time] = None
        self.last_tick_time = self.get_clock().now()

        period_sec = 1.0 / max(rate_hz, 1e-3)
        self.timer = self.create_timer(period_sec, self._tick)

        self.get_logger().info(
            f"Publishing simulated odom on {odom_topic} from {cmd_vel_topic} at {rate_hz:.1f} Hz"
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_tick_time).nanoseconds * 1e-9
        self.last_tick_time = now
        if dt <= 0.0:
            return

        if self.last_cmd_time is None:
            linear_x = 0.0
            angular_z = 0.0
        else:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_vel_timeout_sec:
                linear_x = 0.0
                angular_z = 0.0
            else:
                linear_x = self.linear_x
                angular_z = self.angular_z

        self.x += linear_x * math.cos(self.yaw) * dt
        self.y += linear_x * math.sin(self.yaw) * dt
        self.yaw += angular_z * dt

        stamp = now.to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = CmdVelOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
