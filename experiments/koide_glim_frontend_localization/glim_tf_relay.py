#!/usr/bin/env python3
"""Relay GLIM's local odometry onto /tf under a conflict-free parent frame.

glim_rosnode (rviz_viewer module) unconditionally publishes BOTH
odom -> livox_frame (the LIO odometry edge we want) and
<map_frame_id> -> odom (its own SLAM correction). The latter gives frame
`odom` a second parent and corrupts the localizer's map -> odom -> base_link
chain (TF2 single-parent rule). There is no GLIM config flag to disable it,
so the container remaps /tf to /glim/tf_raw. This host-side relay renames
odom -> livox_frame as glim_odom -> livox_frame and deliberately discards the
global mapping correction. NDT owns map -> glim_odom, while GLIM supplies only
continuous local motion.
"""
import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def rename_parent(odom_to_child, output_parent):
    """Copy a local odometry edge under a conflict-free parent name."""
    out = copy.deepcopy(odom_to_child)
    out.header.frame_id = output_parent
    return out


class GlimTfRelay(Node):
    def __init__(self):
        super().__init__("glim_tf_relay")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("output_parent_frame", "glim_odom")
        self.declare_parameter("child_frame", "livox_frame")
        self.odom_frame = self.get_parameter("odom_frame").value
        self.output_parent = self.get_parameter("output_parent_frame").value
        self.child = self.get_parameter("child_frame").value
        self.pub = self.create_publisher(TFMessage, "/tf", 100)
        self.sub = self.create_subscription(TFMessage, "/glim/tf_raw", self.cb, 100)
        self.get_logger().info(
            "relaying local GLIM odometry: %s -> %s as %s -> %s"
            % (self.odom_frame, self.child, self.output_parent, self.child))

    def cb(self, msg):
        relayed = [rename_parent(transform, self.output_parent)
                   for transform in msg.transforms
                   if transform.header.frame_id == self.odom_frame
                   and transform.child_frame_id == self.child]
        if relayed:
            self.pub.publish(TFMessage(transforms=relayed))


def main():
    rclpy.init()
    node = GlimTfRelay()
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
