#!/usr/bin/env python3
"""Convert a JEPLO raw bag (Livox CustomMsg) into the PointCloud2 bag the Go2 suite replays.

The public JEPLO dataset (https://huggingface.co/datasets/ASIG-Lab/JEPLO-Dataset)
ships ``bags/<sequence>`` with ``/livox/lidar`` as ``livox_ros_driver2/msg/CustomMsg``
at 20 Hz. This writes ``/livox/lidar`` as ``sensor_msgs/PointCloud2`` at 10 Hz,
two consecutive sweeps per cloud, with an absolute per-point ``timestamp`` (float64,
seconds). Each cloud is stamped with its second sweep, i.e. mid-sweep, which is why
``rko_lio_jeplo.yaml`` forces absolute point times. ``/livox/imu`` is copied as is.

The CustomMsg CDR is decoded here, so livox_ros_driver2 does not need to be built.

usage: jeplo_to_pc2_bag.py RAW_BAG_DIR OUT_BAG_DIR [--sweeps-per-cloud 2]
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"

# livox_ros_driver2/msg/CustomPoint in CDR: uint32 offset_time, float32 x, y, z,
# uint8 reflectivity, tag, line, padded to the 4-byte alignment of the next point.
CUSTOM_POINT = np.dtype(
    {
        "names": ["offset_time", "x", "y", "z", "reflectivity", "tag", "line"],
        "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
        "offsets": [0, 4, 8, 12, 16, 17, 18],
        "itemsize": 20,
    }
)

# Output layout: x, y, z, intensity (float32) and timestamp (float64, seconds).
OUT_POINT = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<f8"],
        "offsets": [0, 4, 8, 12, 16],
        "itemsize": 24,
    }
)


def _align(offset: int, size: int) -> int:
    return (offset + size - 1) // size * size


def decode_custom_msg(data: bytes):
    """(stamp_sec, stamp_nanosec, frame_id, timebase_ns, points) of a CDR CustomMsg."""
    if data[1] not in (0x01, 0x03):  # CDR_LE / PL_CDR_LE
        raise ValueError("only little-endian CDR is supported")
    body = memoryview(data)[4:]
    sec, nanosec = struct.unpack_from("<iI", body, 0)
    (frame_len,) = struct.unpack_from("<I", body, 8)
    frame_id = bytes(body[12 : 12 + frame_len]).rstrip(b"\0").decode()
    offset = _align(12 + frame_len, 8)
    (timebase,) = struct.unpack_from("<Q", body, offset)
    offset += 8
    (point_num,) = struct.unpack_from("<I", body, offset)
    offset += 4 + 1 + 3  # point_num, lidar_id, rsvd[3]
    offset = _align(offset, 4)
    (count,) = struct.unpack_from("<I", body, offset)
    offset += 4
    if count != point_num:
        raise ValueError(f"point_num {point_num} != sequence length {count}")
    if count == 0:
        return sec, nanosec, frame_id, timebase, np.empty(0, dtype=CUSTOM_POINT)
    # The last point has no trailing padding.
    needed = (count - 1) * CUSTOM_POINT.itemsize + 19
    raw = bytes(body[offset : offset + needed]) + b"\0"
    points = np.frombuffer(raw, dtype=CUSTOM_POINT, count=count)
    return sec, nanosec, frame_id, timebase, points


def merge_sweeps(sweeps) -> tuple[np.ndarray, int, int, str]:
    """One output cloud from decoded sweeps, stamped with the last sweep's header."""
    clouds = []
    for _sec, _nanosec, _frame, timebase, points in sweeps:
        valid = (points["x"] != 0.0) | (points["y"] != 0.0) | (points["z"] != 0.0)
        valid &= (
            np.isfinite(points["x"])
            & np.isfinite(points["y"])
            & np.isfinite(points["z"])
        )
        points = points[valid]
        out = np.empty(points.shape[0], dtype=OUT_POINT)
        out["x"], out["y"], out["z"] = points["x"], points["y"], points["z"]
        out["intensity"] = points["reflectivity"].astype(np.float32)
        out["timestamp"] = (timebase + points["offset_time"].astype(np.uint64)) * 1.0e-9
        clouds.append(out)
    sec, nanosec, frame_id, _timebase, _points = sweeps[-1]
    return np.concatenate(clouds), sec, nanosec, frame_id


def make_pointcloud2(points: np.ndarray, sec: int, nanosec: int, frame_id: str):
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = OUT_POINT.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.data = points.tobytes()
    msg.is_dense = True
    return msg


def convert(raw_bag: Path, out_bag: Path, sweeps_per_cloud: int) -> tuple[int, int]:
    import rosbag2_py
    from rclpy.serialization import serialize_message

    info = rosbag2_py.Info().read_metadata(str(raw_bag), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(raw_bag), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[LIDAR_TOPIC, IMU_TOPIC]))

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(out_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for topic_id, (name, type_name) in enumerate(
        (
            (LIDAR_TOPIC, "sensor_msgs/msg/PointCloud2"),
            (IMU_TOPIC, "sensor_msgs/msg/Imu"),
        )
    ):
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=topic_id, name=name, type=type_name, serialization_format="cdr"
            )
        )

    pending = []
    clouds = imus = 0
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        if topic == IMU_TOPIC:
            writer.write(IMU_TOPIC, data, recv_ns)
            imus += 1
            continue
        pending.append(decode_custom_msg(data))
        if len(pending) < sweeps_per_cloud:
            continue
        points, sec, nanosec, frame_id = merge_sweeps(pending)
        pending = []
        writer.write(
            LIDAR_TOPIC,
            serialize_message(make_pointcloud2(points, sec, nanosec, frame_id)),
            recv_ns,
        )
        clouds += 1
    del writer
    return clouds, imus


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("out_bag", type=Path)
    parser.add_argument("--sweeps-per-cloud", type=int, default=2)
    args = parser.parse_args(argv)
    if args.out_bag.exists():
        print(f"{args.out_bag} exists", file=sys.stderr)
        return 1
    clouds, imus = convert(args.raw_bag, args.out_bag, max(1, args.sweeps_per_cloud))
    print(f"{args.out_bag}: {clouds} clouds, {imus} IMU messages")
    return 0


if __name__ == "__main__":
    sys.exit(main())
