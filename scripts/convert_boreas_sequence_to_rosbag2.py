#!/usr/bin/env python3

import argparse
import csv
import math
import shutil
from pathlib import Path
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence

import numpy as np
import yaml
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores
from rosbags.typesys import get_typestore


TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)

Time = TYPESTORE.types["builtin_interfaces/msg/Time"]
Header = TYPESTORE.types["std_msgs/msg/Header"]
PointField = TYPESTORE.types["sensor_msgs/msg/PointField"]
PointCloud2 = TYPESTORE.types["sensor_msgs/msg/PointCloud2"]
Point = TYPESTORE.types["geometry_msgs/msg/Point"]
Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]
Pose = TYPESTORE.types["geometry_msgs/msg/Pose"]
PoseWithCovariance = TYPESTORE.types["geometry_msgs/msg/PoseWithCovariance"]
PoseWithCovarianceStamped = TYPESTORE.types["geometry_msgs/msg/PoseWithCovarianceStamped"]
Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
Twist = TYPESTORE.types["geometry_msgs/msg/Twist"]
TwistWithCovariance = TYPESTORE.types["geometry_msgs/msg/TwistWithCovariance"]
TwistWithCovarianceStamped = TYPESTORE.types["geometry_msgs/msg/TwistWithCovarianceStamped"]
Imu = TYPESTORE.types["sensor_msgs/msg/Imu"]


POINT_STEP = 24
POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("ring", "<u2"),
        ("pad", "<u2"),
        ("time", "<f4"),
    ]
)


class PoseRecord:
    def __init__(
        self,
        stamp_us: int,
        x: float,
        y: float,
        z: float,
        vx_enu: float,
        vy_enu: float,
        vz_enu: float,
        roll: float,
        pitch: float,
        yaw: float,
        wx: float,
        wy: float,
        wz: float,
    ) -> None:
        self.stamp_us = stamp_us
        self.x = x
        self.y = y
        self.z = z
        self.vx_enu = vx_enu
        self.vy_enu = vy_enu
        self.vz_enu = vz_enu
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.wx = wx
        self.wy = wy
        self.wz = wz


class ImuRecord:
    def __init__(self, stamp_us: int, wx: float, wy: float, wz: float, ax: float, ay: float, az: float) -> None:
        self.stamp_us = stamp_us
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.ax = ax
        self.ay = ay
        self.az = az


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a raw Boreas sequence into rosbag2 topics usable by lidar_localization_ros2."
    )
    parser.add_argument("--sequence-dir", required=True, help="Boreas sequence directory")
    parser.add_argument("--bag-dir", required=True, help="Output rosbag2 directory")
    parser.add_argument(
        "--pose-csv",
        default="applanix/lidar_poses.csv",
        help="Pose CSV relative to the sequence dir",
    )
    parser.add_argument(
        "--imu-csv",
        default="applanix/imu.csv",
        help="IMU CSV relative to the sequence dir",
    )
    parser.add_argument("--cloud-topic", default="/velodyne_points", help="PointCloud2 topic")
    parser.add_argument("--imu-topic", default="/imu/data", help="Imu topic")
    parser.add_argument(
        "--pose-topic",
        default="/ground_truth/pose_with_covariance",
        help="Ground-truth pose topic",
    )
    parser.add_argument(
        "--twist-topic",
        default="/vehicle/twist",
        help="TwistWithCovarianceStamped topic",
    )
    parser.add_argument("--cloud-frame", default="velodyne", help="Frame id for pointclouds")
    parser.add_argument("--imu-frame", default="applanix", help="Frame id for IMU messages")
    parser.add_argument("--map-frame", default="map", help="Frame id for GT poses")
    parser.add_argument(
        "--start-offset-sec",
        type=float,
        default=0.0,
        help="Seconds to skip from the first lidar timestamp",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="Duration to export. 0 means until the end",
    )
    parser.add_argument(
        "--lidar-stride",
        type=int,
        default=1,
        help="Keep every Nth lidar scan",
    )
    parser.add_argument(
        "--point-stride",
        type=int,
        default=1,
        help="Keep every Nth point within each scan",
    )
    parser.add_argument(
        "--output-reference-csv",
        default="",
        help="Optional output reference CSV path",
    )
    parser.add_argument(
        "--output-initial-pose-yaml",
        default="",
        help="Optional output initial pose YAML path",
    )
    parser.add_argument(
        "--initial-pose-skip-sec",
        type=float,
        default=0.05,
        help="Skip this much time before selecting the exported initial pose",
    )
    parser.add_argument("--force", action="store_true", help="Overwrite outputs")
    return parser.parse_args()


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def prepare_output_path(path: Path, force: bool) -> None:
    if path.exists():
        if not force:
            raise FileExistsError(f"{path} already exists, pass --force to overwrite it")
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()


def numeric_rows(path: Path) -> Iterable[List[float]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        for row in reader:
            if not row:
                continue
            stripped = [cell.strip() for cell in row]
            if not stripped or stripped[0].startswith("#"):
                continue
            try:
                yield [float(value) for value in stripped]
            except ValueError:
                continue


def load_pose_records(path: Path, start_us: int, end_us: Optional[int]) -> List[PoseRecord]:
    records: List[PoseRecord] = []
    for row in numeric_rows(path):
        if len(row) < 13:
            continue
        stamp_us = int(round(row[0]))
        if stamp_us < start_us:
            continue
        if end_us is not None and stamp_us > end_us:
            continue
        records.append(
            PoseRecord(
                stamp_us=stamp_us,
                x=row[1],
                y=row[2],
                z=row[3],
                vx_enu=row[4],
                vy_enu=row[5],
                vz_enu=row[6],
                roll=row[7],
                pitch=row[8],
                yaw=row[9],
                wx=row[12],
                wy=row[11],
                wz=row[10],
            )
        )
    if not records:
        raise RuntimeError(f"No pose rows selected from {path}")
    return records


def load_imu_records(path: Path, start_us: int, end_us: Optional[int]) -> List[ImuRecord]:
    records: List[ImuRecord] = []
    for row in numeric_rows(path):
        if len(row) < 7:
            continue
        stamp_us = int(round(row[0]))
        if stamp_us < start_us:
            continue
        if end_us is not None and stamp_us > end_us:
            continue
        records.append(
            ImuRecord(
                stamp_us=stamp_us,
                wx=row[3],
                wy=row[2],
                wz=row[1],
                ax=row[6],
                ay=row[5],
                az=row[4],
            )
        )
    if not records:
        raise RuntimeError(f"No IMU rows selected from {path}")
    return records


def list_lidar_files(lidar_dir: Path, start_us: int, end_us: Optional[int], stride: int) -> List[Path]:
    files = sorted(lidar_dir.glob("*.bin"), key=lambda path: int(path.stem))
    selected: List[Path] = []
    for index, path in enumerate(files):
        if index % max(1, stride) != 0:
            continue
        stamp_us = int(path.stem)
        if stamp_us < start_us:
            continue
        if end_us is not None and stamp_us > end_us:
            continue
        selected.append(path)
    if not selected:
        raise RuntimeError(f"No lidar scans selected from {lidar_dir}")
    return selected


def stamp_us_to_ns(stamp_us: int) -> int:
    return stamp_us * 1000


def time_msg(stamp_ns: int):
    return Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def load_lidar_scan(path: Path, point_stride: int) -> np.ndarray:
    points = np.fromfile(path, dtype=np.float32).reshape((-1, 6))
    if point_stride > 1:
        points = points[::point_stride]
    return points


def pointcloud2_message(points: np.ndarray, stamp_us: int, frame_id: str):
    point_fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    structured = np.zeros(points.shape[0], dtype=POINT_DTYPE)
    structured["x"] = points[:, 0].astype(np.float32)
    structured["y"] = points[:, 1].astype(np.float32)
    structured["z"] = points[:, 2].astype(np.float32)
    structured["intensity"] = points[:, 3].astype(np.float32)
    structured["ring"] = np.clip(points[:, 4], 0.0, 65535.0).astype(np.uint16)
    structured["time"] = (points[:, 5] * 1e-6).astype(np.float32)
    stamp_ns = stamp_us_to_ns(stamp_us)
    header = Header(stamp=time_msg(stamp_ns), frame_id=frame_id)
    payload = np.frombuffer(np.ascontiguousarray(structured).tobytes(), dtype=np.uint8)
    return PointCloud2(
        header=header,
        height=1,
        width=int(points.shape[0]),
        fields=point_fields,
        is_bigendian=False,
        point_step=POINT_STEP,
        row_step=int(points.shape[0]) * POINT_STEP,
        data=payload,
        is_dense=True,
    )


def pose_message(record: PoseRecord, frame_id: str):
    qx, qy, qz, qw = quaternion_from_rpy(record.roll, record.pitch, record.yaw)
    return PoseWithCovarianceStamped(
        header=Header(stamp=time_msg(stamp_us_to_ns(record.stamp_us)), frame_id=frame_id),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=record.x, y=record.y, z=record.z),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )


def twist_message(record: PoseRecord, frame_id: str):
    rotation = rotation_matrix(record.roll, record.pitch, record.yaw)
    velocity_sensor = rotation.T @ np.array([record.vx_enu, record.vy_enu, record.vz_enu], dtype=np.float64)
    return TwistWithCovarianceStamped(
        header=Header(stamp=time_msg(stamp_us_to_ns(record.stamp_us)), frame_id=frame_id),
        twist=TwistWithCovariance(
            twist=Twist(
                linear=Vector3(
                    x=float(velocity_sensor[0]),
                    y=float(velocity_sensor[1]),
                    z=float(velocity_sensor[2]),
                ),
                angular=Vector3(x=record.wx, y=record.wy, z=record.wz),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )


def imu_message(record: ImuRecord, frame_id: str):
    orientation_covariance = np.zeros(9, dtype=np.float64)
    orientation_covariance[0] = -1.0
    return Imu(
        header=Header(stamp=time_msg(stamp_us_to_ns(record.stamp_us)), frame_id=frame_id),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=orientation_covariance,
        angular_velocity=Vector3(x=record.wx, y=record.wy, z=record.wz),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration=Vector3(x=record.ax, y=record.ay, z=record.az),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )


def write_reference_csv(path: Path, poses: Sequence[PoseRecord], frame_id: str) -> None:
    ensure_parent(path)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "message_index",
                "stamp_sec",
                "frame_id",
                "position_x",
                "position_y",
                "position_z",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
                "covariance",
            ]
        )
        for index, record in enumerate(poses):
            qx, qy, qz, qw = quaternion_from_rpy(record.roll, record.pitch, record.yaw)
            writer.writerow(
                [
                    index,
                    f"{record.stamp_us * 1e-6:.9f}",
                    frame_id,
                    f"{record.x:.10f}",
                    f"{record.y:.10f}",
                    f"{record.z:.10f}",
                    f"{qx:.10f}",
                    f"{qy:.10f}",
                    f"{qz:.10f}",
                    f"{qw:.10f}",
                    " ".join("0" for _ in range(36)),
                ]
            )


def write_initial_pose_yaml(path: Path, record: PoseRecord) -> None:
    qx, qy, qz, qw = quaternion_from_rpy(record.roll, record.pitch, record.yaw)
    data = {
        "/**": {
            "ros__parameters": {
                "set_initial_pose": True,
                "initial_pose_x": record.x,
                "initial_pose_y": record.y,
                "initial_pose_z": record.z,
                "initial_pose_qx": qx,
                "initial_pose_qy": qy,
                "initial_pose_qz": qz,
                "initial_pose_qw": qw,
            }
        }
    }
    ensure_parent(path)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def select_initial_pose_record(poses: Sequence[PoseRecord], skip_sec: float) -> PoseRecord:
    if not poses:
        raise RuntimeError("No poses available for initial pose selection")
    if skip_sec <= 0.0:
        return poses[0]
    threshold_us = poses[0].stamp_us + int(round(skip_sec * 1e6))
    for record in poses:
        if record.stamp_us >= threshold_us:
            return record
    raise RuntimeError(f"Could not find a pose after initial_pose_skip_sec={skip_sec:.3f}")


def main() -> int:
    args = parse_args()

    sequence_dir = Path(args.sequence_dir).expanduser().resolve()
    bag_dir = Path(args.bag_dir).expanduser().resolve()
    pose_csv_path = (sequence_dir / args.pose_csv).resolve()
    imu_csv_path = (sequence_dir / args.imu_csv).resolve()
    lidar_dir = (sequence_dir / "lidar").resolve()

    if not sequence_dir.is_dir():
        raise FileNotFoundError(f"Sequence dir not found: {sequence_dir}")
    if not lidar_dir.is_dir():
        raise FileNotFoundError(f"Lidar dir not found: {lidar_dir}")
    if not pose_csv_path.is_file():
        raise FileNotFoundError(f"Pose CSV not found: {pose_csv_path}")
    if not imu_csv_path.is_file():
        raise FileNotFoundError(f"IMU CSV not found: {imu_csv_path}")

    reference_csv_path = (
        Path(args.output_reference_csv).expanduser().resolve() if args.output_reference_csv else None
    )
    initial_pose_yaml_path = (
        Path(args.output_initial_pose_yaml).expanduser().resolve()
        if args.output_initial_pose_yaml
        else None
    )

    prepare_output_path(bag_dir, args.force)
    if reference_csv_path is not None:
        prepare_output_path(reference_csv_path, args.force)
    if initial_pose_yaml_path is not None:
        prepare_output_path(initial_pose_yaml_path, args.force)

    lidar_files_all = sorted(lidar_dir.glob("*.bin"), key=lambda path: int(path.stem))
    if not lidar_files_all:
        raise RuntimeError(f"No lidar scans found under {lidar_dir}")
    sequence_start_us = int(lidar_files_all[0].stem)
    window_start_us = sequence_start_us + int(round(args.start_offset_sec * 1e6))
    window_end_us = None
    if args.duration_sec > 0.0:
        window_end_us = window_start_us + int(round(args.duration_sec * 1e6))

    lidar_files = list_lidar_files(lidar_dir, window_start_us, window_end_us, args.lidar_stride)
    poses = load_pose_records(pose_csv_path, window_start_us, window_end_us)
    imus = load_imu_records(imu_csv_path, window_start_us, window_end_us)

    with Writer(bag_dir, version=8) as writer:
        cloud_connection = writer.add_connection(
            args.cloud_topic,
            PointCloud2.__msgtype__,
            typestore=TYPESTORE,
        )
        pose_connection = writer.add_connection(
            args.pose_topic,
            PoseWithCovarianceStamped.__msgtype__,
            typestore=TYPESTORE,
        )
        twist_connection = writer.add_connection(
            args.twist_topic,
            TwistWithCovarianceStamped.__msgtype__,
            typestore=TYPESTORE,
        )
        imu_connection = writer.add_connection(
            args.imu_topic,
            Imu.__msgtype__,
            typestore=TYPESTORE,
        )

        for record in poses:
            msg = pose_message(record, args.map_frame)
            writer.write(
                pose_connection,
                stamp_us_to_ns(record.stamp_us),
                TYPESTORE.serialize_cdr(msg, PoseWithCovarianceStamped.__msgtype__),
            )
            twist_msg = twist_message(record, args.cloud_frame)
            writer.write(
                twist_connection,
                stamp_us_to_ns(record.stamp_us),
                TYPESTORE.serialize_cdr(twist_msg, TwistWithCovarianceStamped.__msgtype__),
            )

        for record in imus:
            msg = imu_message(record, args.imu_frame)
            writer.write(
                imu_connection,
                stamp_us_to_ns(record.stamp_us),
                TYPESTORE.serialize_cdr(msg, Imu.__msgtype__),
            )

        for lidar_path in lidar_files:
            stamp_us = int(lidar_path.stem)
            points = load_lidar_scan(lidar_path, args.point_stride)
            cloud_msg = pointcloud2_message(points, stamp_us, args.cloud_frame)
            writer.write(
                cloud_connection,
                stamp_us_to_ns(stamp_us),
                TYPESTORE.serialize_cdr(cloud_msg, PointCloud2.__msgtype__),
            )

    if reference_csv_path is not None:
        write_reference_csv(reference_csv_path, poses, args.map_frame)
    if initial_pose_yaml_path is not None:
        write_initial_pose_yaml(
            initial_pose_yaml_path,
            select_initial_pose_record(poses, args.initial_pose_skip_sec),
        )

    print(f"sequence_dir: {sequence_dir}")
    print(f"bag_dir: {bag_dir}")
    print(f"lidar_scans: {len(lidar_files)}")
    print(f"pose_rows: {len(poses)}")
    print(f"imu_rows: {len(imus)}")
    print(f"window_start_us: {window_start_us}")
    print(f"window_end_us: {window_end_us if window_end_us is not None else 'end'}")
    print(f"cloud_topic: {args.cloud_topic}")
    print(f"imu_topic: {args.imu_topic}")
    print(f"pose_topic: {args.pose_topic}")
    print(f"twist_topic: {args.twist_topic}")
    print(f"output_reference_csv: {reference_csv_path or '-'}")
    print(f"output_initial_pose_yaml: {initial_pose_yaml_path or '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
