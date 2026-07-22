#!/usr/bin/env python3

import importlib.util
import json
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location(
    "startup_initialization_node", SCRIPTS / "startup_initialization_node.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def spin_until(executor, predicate, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        map_path = root / "map.pcd"
        state_path = root / "pose.json"
        map_path.write_bytes(b"quickstart integration map")
        rclpy.init(args=[
            "--ros-args",
            "-p", f"map_path:={map_path}",
            "-p", f"pose_state_path:={state_path}",
            "-p", "restore_saved_pose:=false",
            "-p", "enable_global_initialization:=true",
            "-p", "verification_samples:=2",
            "-p", "min_score_margin:=0.05",
        ])
        startup = MODULE.StartupInitializationNode()
        harness = Node("quickstart_test_harness")
        received_poses = []
        received_status = []
        query_count = {"value": 0}

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cloud_qos = QoSProfile(depth=1)
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        harness.create_service(
            Trigger,
            "/global_localization_node/query",
            lambda _request, response: _query_response(response, query_count),
        )
        cloud_pub = harness.create_publisher(PointCloud2, "/velodyne_points", cloud_qos)
        pose_pub = harness.create_publisher(
            PoseWithCovarianceStamped, "/pcl_pose", transient)
        diagnostics_pub = harness.create_publisher(
            DiagnosticArray, "/alignment_status", transient)
        harness.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            lambda message: received_poses.append(message),
            10,
        )
        harness.create_subscription(
            String,
            "/startup_initialization/status",
            lambda message: received_status.append(json.loads(message.data)),
            transient,
        )

        executor = SingleThreadedExecutor()
        executor.add_node(startup)
        executor.add_node(harness)
        try:
            first_cloud = PointCloud2()
            first_cloud.header.stamp.sec = 1
            for _ in range(3):
                cloud_pub.publish(first_cloud)
                executor.spin_once(timeout_sec=0.1)
            assert spin_until(
                executor, lambda: startup.state.consensus_samples == 1), (
                "first global candidate did not prime consensus")
            second_cloud = PointCloud2()
            second_cloud.header.stamp.sec = 2
            cloud_pub.publish(second_cloud)
            assert spin_until(executor, lambda: bool(received_poses)), (
                "startup node did not publish a global candidate")
            seed = received_poses[-1]
            assert abs(seed.pose.pose.position.x - 4.0) < 1.0e-9
            assert abs(seed.pose.pose.position.y - 5.0) < 1.0e-9
            assert abs(seed.pose.pose.position.z - 1.25) < 1.0e-9

            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.pose.pose.position.x = 4.1
            pose.pose.pose.position.y = 5.1
            pose.pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            for expected in (1, 2):
                diagnostics_pub.publish(_healthy_diagnostic())
                assert spin_until(
                    executor,
                    lambda: (
                        startup.state.name == MODULE.model.STATE_ACTIVE
                        or startup.state.confirmation_samples >= expected),
                    timeout_sec=2.0,
                ), f"diagnostic confirmation {expected} was not consumed"
            assert spin_until(
                executor,
                lambda: startup.state.name == MODULE.model.STATE_ACTIVE,
            ), "startup candidate never reached verified active state"
            assert spin_until(executor, state_path.exists), "verified pose was not saved"
            saved = json.loads(state_path.read_text(encoding="utf-8"))
            assert saved["map_identity"]["sha256"] == startup.map_identity.sha256
            assert saved["position"][:2] == [4.1, 5.1]
            assert any(item["reason"] == "global_pose_verified" for item in received_status)
            # rclpy rejects changing severity at one dynamic logger callsite.
            # The manual fallback must remain callable after informational reports.
            startup._report("manual_fallback_smoke", level="error")
        finally:
            executor.remove_node(harness)
            executor.remove_node(startup)
            harness.destroy_node()
            startup.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


def _query_response(response, query_count):
    query_count["value"] += 1
    response.success = True
    response.message = json.dumps({
        "registration_scoring_enabled": True,
        "candidate_age_sec": 0.1,
        "scan_stamp_sec": float(query_count["value"]),
        "candidates": [
            {"x": 4.0, "y": 5.0, "z": 1.25, "yaw_deg": 30.0, "score": 0.9},
            {"x": -4.0, "y": -5.0, "z": 1.25, "yaw_deg": 210.0, "score": 0.7},
        ],
    })
    return response


def _healthy_diagnostic():
    message = DiagnosticArray()
    status = DiagnosticStatus()
    status.message = "ok"
    status.values = [KeyValue(key="fitness_score", value="0.4")]
    message.status = [status]
    return message


if __name__ == "__main__":
    main()
