#!/usr/bin/env python3

import argparse
import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class NavigateToPoseClient(Node):
    def __init__(self, args):
        super().__init__("nav2_goal_smoke_client")
        self._args = args
        self._client = ActionClient(self, NavigateToPose, args.action_name)
        self._goal_event = threading.Event()
        self._result_event = threading.Event()
        self._accepted = False
        self._status = GoalStatus.STATUS_UNKNOWN
        self._latest_pose = None
        self._first_pose = None
        self._pose_event = threading.Event()
        self._pose_sub = None
        self._tf_buffer = None
        self._tf_listener = None
        self._lifecycle_clients = {}
        if args.use_current_pose:
            self._pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                args.pose_topic,
                self._on_pose,
                10,
            )
        if args.wait_for_transform_target_frame and args.wait_for_transform_source_frame:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        for node_name in args.wait_for_lifecycle_node:
            service_name = f"/{node_name}/get_state"
            self._lifecycle_clients[node_name] = self.create_client(GetState, service_name)

    def wait_for_server(self) -> bool:
        return self._client.wait_for_server(timeout_sec=self._args.server_timeout_sec)

    def wait_for_pose(self) -> bool:
        if not self._args.use_current_pose:
            return True

        deadline = time.monotonic() + self._args.pose_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._pose_event.is_set():
                return True
        return False

    def wait_for_pose_motion(self) -> bool:
        if not self._args.use_current_pose or self._args.wait_for_pose_motion_m <= 0.0:
            return True

        if self._first_pose is None and self._latest_pose is not None:
            self._first_pose = self._latest_pose

        deadline = time.monotonic() + self._args.pose_motion_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._first_pose is None or self._latest_pose is None:
                continue

            dx = self._latest_pose.pose.pose.position.x - self._first_pose.pose.pose.position.x
            dy = self._latest_pose.pose.pose.position.y - self._first_pose.pose.pose.position.y
            distance = math.hypot(dx, dy)
            if distance >= self._args.wait_for_pose_motion_m:
                self.get_logger().info(
                    f"observed pose motion {distance:.3f} m; sending relative goal"
                )
                return True
        return False

    def wait_for_transform(self) -> bool:
        if self._tf_buffer is None:
            return True

        deadline = time.monotonic() + self._args.transform_timeout_sec
        target = self._args.wait_for_transform_target_frame
        source = self._args.wait_for_transform_source_frame
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._tf_buffer.can_transform(target, source, rclpy.time.Time()):
                self.get_logger().info(
                    f"observed transform {target} <- {source}; continuing"
                )
                return True
        return False

    def wait_for_lifecycle_nodes(self) -> bool:
        if not self._lifecycle_clients:
            return True

        remaining = set(self._lifecycle_clients.keys())
        deadline = time.monotonic() + self._args.lifecycle_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            for node_name, client in self._lifecycle_clients.items():
                if node_name in remaining and client.wait_for_service(timeout_sec=0.1):
                    future = client.call_async(GetState.Request())
                    end = time.monotonic() + 1.0
                    while rclpy.ok() and not future.done() and time.monotonic() < end:
                        rclpy.spin_once(self, timeout_sec=0.1)
                    if not future.done():
                        continue
                    result = future.result()
                    if result is None:
                        continue
                    if result.current_state.id == 3:
                        self.get_logger().info(f"lifecycle node active: {node_name}")
                        remaining.discard(node_name)
            if not remaining:
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def send_goal(self) -> None:
        goal_x, goal_y, goal_z, goal_yaw = self._resolve_goal()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._args.frame_id
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = goal_z
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(goal_yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(goal_yaw * 0.5)

        self.get_logger().info(
            f"sending goal x={goal_x:.3f} y={goal_y:.3f} z={goal_z:.3f} yaw={goal_yaw:.3f}"
        )

        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _resolve_goal(self):
        if not self._args.use_current_pose:
            return self._args.goal_x, self._args.goal_y, self._args.goal_z, self._args.goal_yaw

        pose = self._latest_pose
        current_x = pose.pose.pose.position.x
        current_y = pose.pose.pose.position.y
        current_z = pose.pose.pose.position.z
        current_yaw = quaternion_to_yaw(pose.pose.pose.orientation)
        dx = (
            self._args.forward_m * math.cos(current_yaw)
            - self._args.left_m * math.sin(current_yaw)
        )
        dy = (
            self._args.forward_m * math.sin(current_yaw)
            + self._args.left_m * math.cos(current_yaw)
        )
        goal_x = current_x + dx
        goal_y = current_y + dy
        goal_z = current_z + self._args.z_offset_m
        goal_yaw = current_yaw + self._args.yaw_delta_rad
        return goal_x, goal_y, goal_z, goal_yaw

    def _on_pose(self, msg) -> None:
        self._latest_pose = msg
        if self._first_pose is None:
            self._first_pose = msg
        self._pose_event.set()

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"feedback distance_remaining={feedback.distance_remaining:.3f} "
            f"estimated_time_remaining={feedback.estimated_time_remaining.sec}.{feedback.estimated_time_remaining.nanosec:09d}"
        )

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            self._accepted = False
            self._goal_event.set()
            self._result_event.set()
            return

        self._accepted = True
        self.get_logger().info("goal accepted")
        self._goal_event.set()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error("goal result missing")
            self._status = GoalStatus.STATUS_UNKNOWN
            self._result_event.set()
            return
        self._status = result.status
        self.get_logger().info(f"goal finished with status={STATUS_NAMES.get(result.status, result.status)}")
        self._result_event.set()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-z", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--use-current-pose", action="store_true")
    parser.add_argument("--pose-topic", default="/localization/pose_with_covariance")
    parser.add_argument("--pose-timeout-sec", type=float, default=30.0)
    parser.add_argument("--wait-for-pose-motion-m", type=float, default=0.0)
    parser.add_argument("--pose-motion-timeout-sec", type=float, default=60.0)
    parser.add_argument("--forward-m", type=float, default=0.0)
    parser.add_argument("--left-m", type=float, default=0.0)
    parser.add_argument("--yaw-delta-rad", type=float, default=0.0)
    parser.add_argument("--z-offset-m", type=float, default=0.0)
    parser.add_argument("--server-timeout-sec", type=float, default=60.0)
    parser.add_argument("--wait-for-transform-target-frame", default="")
    parser.add_argument("--wait-for-transform-source-frame", default="")
    parser.add_argument("--transform-timeout-sec", type=float, default=30.0)
    parser.add_argument("--wait-for-lifecycle-node", action="append", default=[])
    parser.add_argument("--lifecycle-timeout-sec", type=float, default=30.0)
    parser.add_argument("--post-ready-delay-sec", type=float, default=0.0)
    parser.add_argument("--result-timeout-sec", type=float, default=120.0)
    args = parser.parse_args()
    if not args.use_current_pose and (args.goal_x is None or args.goal_y is None):
        parser.error("--goal-x and --goal-y are required unless --use-current-pose is set")
    has_wait_target = bool(args.wait_for_transform_target_frame)
    has_wait_source = bool(args.wait_for_transform_source_frame)
    if has_wait_target != has_wait_source:
        parser.error(
            "--wait-for-transform-target-frame and --wait-for-transform-source-frame "
            "must be provided together"
        )
    return args


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    args = parse_args()
    rclpy.init()
    node = NavigateToPoseClient(args)

    if args.use_current_pose and not node.wait_for_pose():
        node.get_logger().error(f"pose topic did not publish in time: {args.pose_topic}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not node.wait_for_transform():
        node.get_logger().error(
            "required transform did not become available before timeout "
            f"({args.wait_for_transform_target_frame} <- {args.wait_for_transform_source_frame})"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not node.wait_for_lifecycle_nodes():
        node.get_logger().error(
            "required lifecycle nodes did not become active before timeout: "
            + ", ".join(args.wait_for_lifecycle_node)
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not node.wait_for_server():
        node.get_logger().error("navigate_to_pose action server not available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if args.use_current_pose and not node.wait_for_pose_motion():
        node.get_logger().error(
            "pose topic did not move enough before timeout "
            f"({args.wait_for_pose_motion_m:.3f} m on {args.pose_topic})"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if args.post_ready_delay_sec > 0.0:
        node.get_logger().info(
            f"waiting an additional {args.post_ready_delay_sec:.1f} s before sending goal"
        )
        deadline = time.monotonic() + args.post_ready_delay_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

    node.send_goal()

    deadline = time.monotonic() + args.result_timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node._result_event.is_set():
            break

    if not node._accepted:
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not node._result_event.is_set():
        node.get_logger().error("timed out waiting for goal result")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    status_name = STATUS_NAMES.get(node._status, str(node._status))
    print(f"Goal finished with status: {status_name}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node._status == GoalStatus.STATUS_SUCCEEDED else 1


if __name__ == "__main__":
    sys.exit(main())
