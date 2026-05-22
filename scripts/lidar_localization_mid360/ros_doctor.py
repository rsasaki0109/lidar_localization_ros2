import time
from typing import Dict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer
from tf2_ros import TransformListener

from lidar_localization_mid360.bringup_model import BringupCheckConfig
from lidar_localization_mid360.bringup_model import BringupSnapshot
from lidar_localization_mid360.bringup_model import TopicStats
from lidar_localization_mid360.bringup_model import build_tf_checks


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Mid360BringupDoctor(Node):
    def __init__(self, config: BringupCheckConfig):
        super().__init__("mid360_legged_bringup_doctor")
        self.config = config
        self.cloud = TopicStats()
        self.imu = TopicStats()
        self.pose = TopicStats()
        self.status = TopicStats()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        latched_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            PointCloud2,
            config.cloud_topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            config.imu_topic,
            self.on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            config.pose_topic,
            self.on_pose,
            latched_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            config.alignment_status_topic,
            self.on_status,
            latched_qos,
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        self.cloud.mark(msg.header.frame_id, stamp_to_sec(msg.header.stamp), time.monotonic())
        self.cloud.last_point_count = int(msg.width) * int(msg.height)
        field_names = {field.name for field in msg.fields}
        self.cloud.has_xyz_fields = {"x", "y", "z"}.issubset(field_names)

    def on_imu(self, msg: Imu) -> None:
        self.imu.mark(msg.header.frame_id, stamp_to_sec(msg.header.stamp), time.monotonic())

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose.mark(msg.header.frame_id, stamp_to_sec(msg.header.stamp), time.monotonic())

    def on_status(self, msg: DiagnosticArray) -> None:
        self.status.mark(msg.header.frame_id, stamp_to_sec(msg.header.stamp), time.monotonic())
        if msg.status:
            latest = msg.status[0]
            self.status.last_status_level = int(latest.level)
            self.status.last_status_message = latest.message

    def have_tf(self, target: str, source: str) -> bool:
        try:
            return self.tf_buffer.can_transform(target, source, rclpy.time.Time())
        except Exception:
            return False

    def snapshot(self) -> BringupSnapshot:
        availability: Dict[str, bool] = {
            f"{self.config.base_frame} <- {self.config.lidar_frame}": self.have_tf(
                self.config.base_frame, self.config.lidar_frame
            ),
            f"{self.config.odom_frame} <- {self.config.base_frame}": self.have_tf(
                self.config.odom_frame, self.config.base_frame
            ),
            f"{self.config.global_frame} <- {self.config.odom_frame}": self.have_tf(
                self.config.global_frame, self.config.odom_frame
            ),
        }
        return BringupSnapshot(
            cloud=self.cloud,
            imu=self.imu,
            pose=self.pose,
            status=self.status,
            tf_checks=build_tf_checks(self.config, availability),
        )
