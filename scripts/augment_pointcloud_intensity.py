#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


class IntensityAugmenter(Node):
    def __init__(self) -> None:
        super().__init__("augment_pointcloud_intensity")

        self.declare_parameter("input_topic", "/points_in")
        self.declare_parameter("output_topic", "/points_xyzi")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self._callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Augmenting pointcloud intensity from {input_topic} to {output_topic}"
        )

    def _callback(self, msg: PointCloud2) -> None:
        field_names = {field.name for field in msg.fields}
        if "intensity" in field_names:
            self.publisher.publish(msg)
            return

        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
            points.append((point[0], point[1], point[2], 0.0))

        augmented = point_cloud2.create_cloud(
            msg.header,
            [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            points,
        )
        self.publisher.publish(augmented)


def main() -> None:
    rclpy.init()
    node = IntensityAugmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
