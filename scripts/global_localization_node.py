#!/usr/bin/env python3
"""On-demand global localization service node (G2).

Loads a 2D occupancy map, keeps the latest scan from the localization cloud
topic, and answers std_srvs/Trigger queries on `~/query` by running the BBS_2D
branch-and-bound search map-wide. Ranked candidates are published as a
geometry_msgs/PoseArray on `~/candidates` and summarized as JSON in the
service response. Opt-in only: nothing is published or changed unless the
service is called, and the node is never part of the default launch files.
"""

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))

import make_bbs_relocalization_attempts as bbs_engine  # noqa: E402
from global_localization_query import GlobalLocalizationConfig  # noqa: E402
from global_localization_query import GlobalLocalizationEngine  # noqa: E402


class GlobalLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("global_localization_node")

        self.declare_parameter("occupancy_yaml", "")
        self.declare_parameter("cloud_topic", "cloud")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("z_min_m", 0.5)
        self.declare_parameter("z_max_m", 5.0)
        self.declare_parameter("min_range_m", 1.0)
        self.declare_parameter("max_scan_points", 512)
        self.declare_parameter("angular_resolution_deg", 5.0)
        self.declare_parameter("pyramid_depth", 4)
        self.declare_parameter("max_candidates", 16)
        self.declare_parameter("nms_radius_m", 2.0)
        self.declare_parameter("dilate_cells", 1)
        self.declare_parameter("seed_z_m", 0.0)
        # Opt-in C++ backend (bbs_cpp). Default False -> pure Python; if the
        # compiled module is unavailable the engine falls back to Python.
        self.declare_parameter("use_cpp_backend", False)

        occupancy_yaml = (
            self.get_parameter("occupancy_yaml").get_parameter_value().string_value)
        if not occupancy_yaml:
            raise RuntimeError(
                "occupancy_yaml parameter is required (output of "
                "generate_occupancy_map_from_pcd.py)")

        config = GlobalLocalizationConfig(
            z_min_m=self.get_parameter("z_min_m").value,
            z_max_m=self.get_parameter("z_max_m").value,
            min_range_m=self.get_parameter("min_range_m").value,
            max_scan_points=int(self.get_parameter("max_scan_points").value),
            angular_resolution_rad=math.radians(
                self.get_parameter("angular_resolution_deg").value),
            pyramid_depth=int(self.get_parameter("pyramid_depth").value),
            max_candidates=int(self.get_parameter("max_candidates").value),
            nms_radius_m=self.get_parameter("nms_radius_m").value,
            dilate_cells=int(self.get_parameter("dilate_cells").value),
            seed_z_m=self.get_parameter("seed_z_m").value,
            use_cpp_backend=bool(self.get_parameter("use_cpp_backend").value),
        )
        self.engine = GlobalLocalizationEngine(Path(occupancy_yaml), config)
        if config.use_cpp_backend and self.engine.backend != "cpp":
            self.get_logger().warn(
                "use_cpp_backend requested but bbs_cpp is unavailable (%s); "
                "falling back to the Python search"
                % self.engine.backend_error)
        self.global_frame_id = (
            self.get_parameter("global_frame_id").get_parameter_value().string_value)
        self.latest_cloud = None

        cloud_topic = (
            self.get_parameter("cloud_topic").get_parameter_value().string_value)
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_received, qos_profile_sensor_data)
        candidate_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.candidates_pub = self.create_publisher(
            PoseArray, "~/candidates", candidate_qos)
        self.query_srv = self.create_service(Trigger, "~/query", self.handle_query)
        self.get_logger().info(
            "global localization service ready: map=%s scan=%s backend=%s"
            % (occupancy_yaml, cloud_topic, self.engine.backend))

    def cloud_received(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg

    def handle_query(self, request, response):
        if self.latest_cloud is None:
            response.success = False
            response.message = json.dumps({"error": "no_scan_received"})
            return response

        cloud = self.latest_cloud
        points_xyz = bbs_engine.pointcloud2_xyz_array(cloud)
        started = time.monotonic()
        result = self.engine.query(points_xyz)
        runtime_sec = time.monotonic() - started

        pose_array = PoseArray()
        pose_array.header.stamp = cloud.header.stamp
        pose_array.header.frame_id = self.global_frame_id
        for candidate in result.candidates:
            pose = Pose()
            pose.position.x = candidate.x_m
            pose.position.y = candidate.y_m
            pose.position.z = candidate.z_m
            pose.orientation.z = math.sin(candidate.yaw_rad / 2.0)
            pose.orientation.w = math.cos(candidate.yaw_rad / 2.0)
            pose_array.poses.append(pose)
        self.candidates_pub.publish(pose_array)

        ranked = [
            {
                "x": round(c.x_m, 3),
                "y": round(c.y_m, 3),
                "yaw_deg": round(math.degrees(c.yaw_rad), 1),
                "score": round(c.score, 4),
            }
            for c in result.candidates
        ]
        summary = {
            "candidate_count": len(result.candidates),
            "scan_point_count": result.scan_point_count,
            "runtime_sec": round(runtime_sec, 3),
            "backend": self.engine.backend,
            # Full ranked list (high-to-low) so a consumer can walk past an
            # aliased top candidate; "top" kept for back-compat.
            "candidates": ranked,
        }
        if ranked:
            summary["top"] = ranked[0]
        response.success = bool(result.candidates)
        response.message = json.dumps(summary)
        self.get_logger().info("query answered: %s" % response.message)
        return response


def main() -> None:
    rclpy.init()
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
