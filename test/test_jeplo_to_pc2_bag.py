"""The JEPLO CustomMsg decoder and sweep merge (tools/benchmark/jeplo_to_pc2_bag.py)."""

import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "benchmark"))

import jeplo_to_pc2_bag as jeplo


def custom_msg_cdr(sec, nanosec, frame_id, timebase, points):
    """Little-endian CDR of livox_ros_driver2/msg/CustomMsg."""
    body = bytearray(struct.pack("<iI", sec, nanosec))
    name = frame_id.encode() + b"\0"
    body += struct.pack("<I", len(name)) + name
    body += b"\0" * (-len(body) % 8)
    body += struct.pack("<QIB3x", timebase, len(points), 1)
    body += b"\0" * (-len(body) % 4)
    body += struct.pack("<I", len(points))
    for index, (offset_time, x, y, z, reflectivity) in enumerate(points):
        if index:
            body += b"\0" * (-len(body) % 4)
        body += struct.pack("<I3f3B", offset_time, x, y, z, reflectivity, 0, 0)
    return b"\x00\x01\x00\x00" + bytes(body)


def test_custom_msg_points_are_decoded():
    data = custom_msg_cdr(
        12,
        500,
        "livox_frame",
        12_000_000_500,
        [(0, 1.0, 2.0, 3.0, 7), (1000, -1.5, 0.5, 0.25, 9)],
    )
    sec, nanosec, frame_id, timebase, points = jeplo.decode_custom_msg(data)
    assert (sec, nanosec, frame_id, timebase) == (
        12,
        500,
        "livox_frame",
        12_000_000_500,
    )
    assert points["offset_time"].tolist() == [0, 1000]
    assert np.allclose(points["x"], [1.0, -1.5])
    assert np.allclose(points["z"], [3.0, 0.25])
    assert points["reflectivity"].tolist() == [7, 9]


def test_merged_cloud_has_absolute_times_and_the_last_stamp():
    first = jeplo.decode_custom_msg(
        custom_msg_cdr(
            1, 0, "f", 1_000_000_000, [(0, 1.0, 0.0, 0.0, 1), (0, 0.0, 0.0, 0.0, 1)]
        )
    )
    second = jeplo.decode_custom_msg(
        custom_msg_cdr(
            1, 50_000_000, "f", 1_050_000_000, [(25_000_000, 2.0, 0.0, 0.0, 3)]
        )
    )
    points, sec, nanosec, frame_id = jeplo.merge_sweeps([first, second])
    # The zero point (an invalid return) is dropped.
    assert np.allclose(points["x"], [1.0, 2.0])
    assert np.allclose(points["timestamp"], [1.0, 1.075])
    assert np.allclose(points["intensity"], [1.0, 3.0])
    assert (sec, nanosec, frame_id) == (1, 50_000_000, "f")


if __name__ == "__main__":
    test_custom_msg_points_are_decoded()
    test_merged_cloud_has_absolute_times_and_the_last_stamp()
    print("test_jeplo_to_pc2_bag: all tests passed")
