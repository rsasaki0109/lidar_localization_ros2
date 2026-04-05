#!/usr/bin/env python3

import math
from typing import Optional
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_multiply(lhs, rhs):
    return (
        lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3] + lhs[2] * rhs[0],
        lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[2] * rhs[3],
        lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
    )


def quaternion_inverse(quaternion):
    norm_sq = sum(component * component for component in quaternion)
    if norm_sq <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (
        -quaternion[0] / norm_sq,
        -quaternion[1] / norm_sq,
        -quaternion[2] / norm_sq,
        quaternion[3] / norm_sq,
    )


def rotate_vector(quaternion, vector):
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, vector_quaternion),
        quaternion_inverse(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
    cosy_cosp = 1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
    return math.atan2(siny_cosp, cosy_cosp)


class OdomFromLocalizationPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_from_localization_publisher")

        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)

        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if publish_tf else None
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, pose_topic, self._on_pose, 10)

        self.anchor_position = None
        self.anchor_orientation = None
        self.previous_odom_pose: Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]] = None
        self.previous_stamp_sec: Optional[float] = None

        self.get_logger().info(
            f"Publishing odom from localization pose {pose_topic} on {odom_topic} with "
            f"{self.odom_frame_id}->{self.base_frame_id}"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        if self.anchor_position is None:
            self.anchor_position = position
            self.anchor_orientation = orientation
            self.get_logger().info(
                f"Anchored odom frame from localization at "
                f"x={position[0]:.3f} y={position[1]:.3f} z={position[2]:.3f}"
            )

        inverse_anchor = quaternion_inverse(self.anchor_orientation)
        relative_translation_map = (
            position[0] - self.anchor_position[0],
            position[1] - self.anchor_position[1],
            position[2] - self.anchor_position[2],
        )
        relative_position = rotate_vector(inverse_anchor, relative_translation_map)
        relative_orientation = quaternion_multiply(inverse_anchor, orientation)

        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = relative_position[0]
        odom.pose.pose.position.y = relative_position[1]
        odom.pose.pose.position.z = relative_position[2]
        odom.pose.pose.orientation.x = relative_orientation[0]
        odom.pose.pose.orientation.y = relative_orientation[1]
        odom.pose.pose.orientation.z = relative_orientation[2]
        odom.pose.pose.orientation.w = relative_orientation[3]
        odom.pose.covariance = msg.pose.covariance

        if self.previous_odom_pose is not None and self.previous_stamp_sec is not None:
            dt = stamp_sec - self.previous_stamp_sec
            if dt > 1e-6:
                prev_position, prev_orientation = self.previous_odom_pose
                odom.twist.twist.linear.x = (relative_position[0] - prev_position[0]) / dt
                odom.twist.twist.linear.y = (relative_position[1] - prev_position[1]) / dt
                odom.twist.twist.linear.z = (relative_position[2] - prev_position[2]) / dt
                yaw = quaternion_to_yaw(relative_orientation)
                prev_yaw = quaternion_to_yaw(prev_orientation)
                yaw_delta = math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
                odom.twist.twist.angular.z = yaw_delta / dt

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = relative_position[0]
            transform.transform.translation.y = relative_position[1]
            transform.transform.translation.z = relative_position[2]
            transform.transform.rotation.x = relative_orientation[0]
            transform.transform.rotation.y = relative_orientation[1]
            transform.transform.rotation.z = relative_orientation[2]
            transform.transform.rotation.w = relative_orientation[3]
            self.tf_broadcaster.sendTransform(transform)

        self.previous_odom_pose = (relative_position, relative_orientation)
        self.previous_stamp_sec = stamp_sec


def main() -> None:
    rclpy.init()
    node = OdomFromLocalizationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
