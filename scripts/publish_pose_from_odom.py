#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def inverse_planar_transform(x: float, y: float, yaw: float) -> Tuple[float, float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    inv_x = -(c * x + s * y)
    inv_y = -(-s * x + c * y)
    return inv_x, inv_y, -yaw


def compose_planar_transforms(
    ax: float,
    ay: float,
    ayaw: float,
    bx: float,
    by: float,
    byaw: float,
) -> Tuple[float, float, float]:
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    x = ax + c * bx - s * by
    y = ay + s * bx + c * by
    yaw = ayaw + byaw
    return x, y, yaw


class PoseFromOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("pose_from_odom_publisher")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_z", 0.0)
        self.declare_parameter("initial_pose_qx", 0.0)
        self.declare_parameter("initial_pose_qy", 0.0)
        self.declare_parameter("initial_pose_qz", 0.0)
        self.declare_parameter("initial_pose_qw", 1.0)
        self.declare_parameter("xy_covariance", 0.05)
        self.declare_parameter("z_covariance", 0.10)
        self.declare_parameter("roll_pitch_covariance", 0.20)
        self.declare_parameter("yaw_covariance", 0.05)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.global_frame_id = self.get_parameter("global_frame_id").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        self.initial_x = self.get_parameter("initial_pose_x").get_parameter_value().double_value
        self.initial_y = self.get_parameter("initial_pose_y").get_parameter_value().double_value
        self.initial_z = self.get_parameter("initial_pose_z").get_parameter_value().double_value
        self.initial_qx = self.get_parameter("initial_pose_qx").get_parameter_value().double_value
        self.initial_qy = self.get_parameter("initial_pose_qy").get_parameter_value().double_value
        self.initial_qz = self.get_parameter("initial_pose_qz").get_parameter_value().double_value
        self.initial_qw = self.get_parameter("initial_pose_qw").get_parameter_value().double_value
        self.initial_yaw = yaw_from_quaternion(
            self.initial_qx,
            self.initial_qy,
            self.initial_qz,
            self.initial_qw,
        )

        self.xy_covariance = self.get_parameter("xy_covariance").get_parameter_value().double_value
        self.z_covariance = self.get_parameter("z_covariance").get_parameter_value().double_value
        self.roll_pitch_covariance = (
            self.get_parameter("roll_pitch_covariance").get_parameter_value().double_value
        )
        self.yaw_covariance = self.get_parameter("yaw_covariance").get_parameter_value().double_value

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.subscription = self.create_subscription(Odometry, odom_topic, self._odom_callback, 50)

        self._map_to_odom: Optional[Tuple[float, float, float]] = None

        self.get_logger().info(
            "Publishing map pose from odom on "
            f"{pose_topic} with {self.global_frame_id}->{self.odom_frame_id}"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        if self._map_to_odom is None:
            inv_x, inv_y, inv_yaw = inverse_planar_transform(odom_x, odom_y, odom_yaw)
            self._map_to_odom = compose_planar_transforms(
                self.initial_x,
                self.initial_y,
                self.initial_yaw,
                inv_x,
                inv_y,
                inv_yaw,
            )

        map_to_odom_x, map_to_odom_y, map_to_odom_yaw = self._map_to_odom
        map_x, map_y, map_yaw = compose_planar_transforms(
            map_to_odom_x,
            map_to_odom_y,
            map_to_odom_yaw,
            odom_x,
            odom_y,
            odom_yaw,
        )

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.global_frame_id
        pose_msg.pose.pose.position.x = map_x
        pose_msg.pose.pose.position.y = map_y
        pose_msg.pose.pose.position.z = self.initial_z + msg.pose.pose.position.z
        qx, qy, qz, qw = quaternion_from_yaw(map_yaw)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        pose_msg.pose.covariance[0] = self.xy_covariance
        pose_msg.pose.covariance[7] = self.xy_covariance
        pose_msg.pose.covariance[14] = self.z_covariance
        pose_msg.pose.covariance[21] = self.roll_pitch_covariance
        pose_msg.pose.covariance[28] = self.roll_pitch_covariance
        pose_msg.pose.covariance[35] = self.yaw_covariance
        self.pose_pub.publish(pose_msg)

        if self.tf_broadcaster is None:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.global_frame_id
        tf_msg.child_frame_id = self.odom_frame_id
        tf_msg.transform.translation.x = map_to_odom_x
        tf_msg.transform.translation.y = map_to_odom_y
        tf_msg.transform.translation.z = self.initial_z
        qx, qy, qz, qw = quaternion_from_yaw(map_to_odom_yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = PoseFromOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
