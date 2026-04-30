#!/usr/bin/env python3

import copy

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class ReplayStampRelay(Node):
    def __init__(self) -> None:
        super().__init__("relay_localization_inputs_with_current_stamp")

        self.declare_parameter("input_pointcloud_topic", "/localization/util/downsample/pointcloud")
        self.declare_parameter("output_pointcloud_topic", "/localization/replay/downsample/pointcloud")
        self.declare_parameter("input_twist_topic", "/localization/twist_estimator/twist_with_covariance")
        self.declare_parameter("output_twist_topic", "/localization/replay/twist_with_covariance")

        input_pointcloud_topic = (
            self.get_parameter("input_pointcloud_topic").get_parameter_value().string_value
        )
        output_pointcloud_topic = (
            self.get_parameter("output_pointcloud_topic").get_parameter_value().string_value
        )
        input_twist_topic = self.get_parameter("input_twist_topic").get_parameter_value().string_value
        output_twist_topic = (
            self.get_parameter("output_twist_topic").get_parameter_value().string_value
        )

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            output_pointcloud_topic,
            qos_profile_sensor_data,
        )
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, output_twist_topic, 50)

        self.create_subscription(
            PointCloud2,
            input_pointcloud_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            input_twist_topic,
            self._on_twist,
            50,
        )

        self.get_logger().info(
            "Relaying localization inputs with current ROS clock stamp: "
            f"{input_pointcloud_topic} -> {output_pointcloud_topic}, "
            f"{input_twist_topic} -> {output_twist_topic}"
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        relay_msg = copy.deepcopy(msg)
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        self.pointcloud_pub.publish(relay_msg)

    def _on_twist(self, msg: TwistWithCovarianceStamped) -> None:
        relay_msg = copy.deepcopy(msg)
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(relay_msg)


def main() -> None:
    rclpy.init()
    node = ReplayStampRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
