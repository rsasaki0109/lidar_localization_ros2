#!/usr/bin/env python3
"""Experimental G2 that centres its occupancy window using relative odometry."""

import contextlib
import json
import math
from collections import deque

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped

# Both the Python G2 node and compiled BBS/NDT modules live in the overlay.
from global_localization_node import GlobalLocalizationNode
from moving_window import crop_map
from odom_window_prior import OdometryWindowPrior, PlanarPose
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_ros import TransformException

RECOVERY_WINDOW_SCALE = 1.5


def planar_pose(position, rotation):
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
    )
    return PlanarPose(position.x, position.y, yaw)


class OdometryWindowG2(GlobalLocalizationNode):
    def __init__(self):
        super().__init__()
        names = ("prior_odom_origin_x_m", "prior_odom_origin_y_m", "prior_odom_yaw_rad")
        for name in names:
            self.declare_parameter(name, float("nan"))
        self.declare_parameter("search_radius_m", 50.0)
        self.declare_parameter("refresh_window", True)
        self.declare_parameter("enable_restart_recovery", False)
        self.refresh_window = self.get_parameter("refresh_window").value
        self.restart_recovery = (
            self.refresh_window and self.get_parameter("enable_restart_recovery").value
        )
        x, y, yaw = (self.get_parameter(name).value for name in names)
        self.radius = self.get_parameter("search_radius_m").value
        if (
            not all(math.isfinite(v) for v in (x, y, yaw, self.radius))
            or self.radius <= 0
        ):
            raise ValueError(
                "A finite map pose for the odom origin and positive radius are required"
            )
        if (
            self.engine.occupancy_map is None
            or not self.odom_frame_id
            or not self.base_frame_id
            or self.engine.backend != "cpp"
        ):
            raise ValueError(
                "The experiment needs a C++ BBS occupancy map, odom and base frames"
            )
        self.origin = np.array([x, y])
        self.prior = OdometryWindowPrior(PlanarPose(x, y, yaw))
        self.grid = self.engine.occupancy_map
        self.matching = self.engine.matching_grid
        if self.restart_recovery:
            self.map_poses = deque(maxlen=32)
            self.healthy_stamps = deque(maxlen=32)
            self.pending_scan_stamp = None
            self.create_subscription(
                PoseWithCovarianceStamped, "pcl_pose", self.map_pose_received, 10
            )
            self.create_subscription(
                DiagnosticArray, "alignment_status", self.alignment_received, 10
            )

    def odometry_at(self, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame_id, self.base_frame_id, stamp
            ).transform
        except TransformException:
            return None
        return planar_pose(tf.translation, tf.rotation)

    def observe_odometry(self, stamp, pose):
        if self.prior.observe_odometry(stamp.nanoseconds, pose):
            self.get_logger().warn(
                "Odometry jumped; re-anchoring the search to the last healthy map match"
                if self.prior.map_anchor is not None
                else "Odometry jumped before any healthy map match; searching the full map"
            )

    def cloud_received(self, msg):
        super().cloud_received(msg)
        if self.restart_recovery:
            if self.pending_scan_stamp is not None:
                pose = self.odometry_at(self.pending_scan_stamp)
                if pose is not None:
                    self.observe_odometry(self.pending_scan_stamp, pose)
            stamp = Time.from_msg(msg.header.stamp)
            pose = self.odometry_at(stamp)
            self.pending_scan_stamp = stamp if pose is None else None
            if pose is not None:
                self.observe_odometry(stamp, pose)

    def map_pose_received(self, msg):
        if msg.header.frame_id != self.global_frame_id:
            return
        stamp = Time.from_msg(msg.header.stamp)
        pose = planar_pose(msg.pose.pose.position, msg.pose.pose.orientation)
        self.map_poses.append((stamp, pose))
        if stamp in self.healthy_stamps:
            self.accept_match(stamp, pose)

    def alignment_received(self, msg):
        for status in msg.status:
            values = {entry.key: entry.value for entry in status.values}
            if status.message == "ok" and values.get("failure_category") == "healthy":
                stamp = Time.from_msg(msg.header.stamp)
                self.healthy_stamps.append(stamp)
                for pose_stamp, pose in reversed(self.map_poses):
                    if pose_stamp == stamp:
                        self.accept_match(stamp, pose)
                        break
                return

    def accept_match(self, stamp, pose):
        odometry = self.odometry_at(stamp)
        if odometry is not None:
            self.prior.accept_map_match(stamp.nanoseconds, pose, odometry)

    def handle_query(self, request, response):
        cloud = self.latest_cloud
        if cloud is None:
            return super().handle_query(request, response)
        stamp = Time.from_msg(cloud.header.stamp)
        if self.restart_recovery:
            pose = self.odometry_at(stamp)
            if pose is not None:
                self.observe_odometry(stamp, pose)
        else:
            tf = self._lookup(
                self.odom_frame_id, self.base_frame_id, cloud.header.stamp
            )
            pose = (
                planar_pose(tf.transform.translation, tf.transform.rotation)
                if tf is not None
                else None
            )
        if pose is None:
            response.success = False
            response.message = json.dumps({"error": "no_odometry_at_scan_stamp"})
            return response
        center = self.prior.center(pose) if self.refresh_window else self.origin
        if center is None:
            return super().handle_query(request, response)
        radius = self.radius
        if self.prior.recovering:
            radius *= RECOVERY_WINDOW_SCALE
            self.get_logger().info(
                f"Recovery search: center=({center[0]:.2f}, {center[1]:.2f}), "
                f"radius={radius:.1f} m"
            )
        try:
            grid, matching = crop_map(
                self.grid, self.matching, np.asarray(center), radius
            )
        except ValueError:
            response.success = False
            response.message = json.dumps({"error": "prior_outside_map"})
            return response
        self.engine.occupancy_map, self.engine.matching_grid = grid, matching
        try:
            return super().handle_query(request, response)
        finally:
            self.engine.occupancy_map, self.engine.matching_grid = (
                self.grid,
                self.matching,
            )


def main():
    rclpy.init()
    node = OdometryWindowG2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with contextlib.suppress(KeyboardInterrupt):
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
