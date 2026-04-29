#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class IdentityOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("identity_odom_publisher")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("publish_tf", True)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if publish_tf else None

        period_sec = 1.0 / max(rate_hz, 1e-3)
        self.timer = self.create_timer(period_sec, self._tick)

        self.get_logger().info(
            f"Publishing identity odom on {odom_topic} with {self.odom_frame_id}->{self.base_frame_id} at {rate_hz:.1f} Hz"
        )

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = IdentityOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
