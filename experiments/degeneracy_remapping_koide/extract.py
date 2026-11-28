"""Writes replay.c's inputs from a Koide indoor sequence.

    python extract.py BAG_DIR MAP.ply GT.txt OUT_DIR [--topic /points2/decompressed]

Needs `rosbags` and `numpy` (pip). Writes OUT_DIR/map.bin, scans.bin and gt.bin:
each scan's points (finite, voxel-downsampled at 5 cm) and the ground-truth pose
nearest its stamp (TUM: stamp x y z qx qy qz qw), skipping scans with no ground
truth within 20 ms.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def read_ply(path: Path) -> np.ndarray:
    data = path.read_bytes()
    header_end = data.index(b"end_header\n") + len(b"end_header\n")
    header = data[:header_end].decode()
    assert "binary_little_endian" in header, "expected a binary little-endian PLY"
    count = int(next(line.split()[2] for line in header.splitlines() if line.startswith("element vertex")))
    properties = [line.split()[2] for line in header.splitlines() if line.startswith("property")]
    assert properties[:3] == ["x", "y", "z"] and len(properties) == 3, properties
    return np.frombuffer(data, dtype="<f4", count=count * 3, offset=header_end).reshape(-1, 3)


def pose_matrix(row: np.ndarray) -> np.ndarray:
    x, y, z, qx, qy, qz, qw = row
    n = qx * qx + qy * qy + qz * qz + qw * qw
    s = 2.0 / n
    m = np.eye(4)
    m[:3, :3] = [
        [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
    ]
    m[:3, 3] = [x, y, z]
    return m


def cloud_xyz(msg) -> np.ndarray:
    fields = {f.name: f.offset for f in msg.fields}
    step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, step)
    xyz = np.stack(
        [raw[:, fields[a] : fields[a] + 4].copy().view("<f4")[:, 0] for a in "xyz"], axis=1
    )
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    keys = np.floor(xyz / 0.05).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return xyz[np.sort(keep)].astype("<f4")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("map", type=Path)
    parser.add_argument("gt", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--topic", default="/points2/decompressed")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    points = read_ply(args.map)
    with (args.out / "map.bin").open("wb") as f:
        f.write(struct.pack("<Q", len(points)))
        f.write(points.astype("<f4").tobytes())

    gt = np.loadtxt(args.gt)
    written = 0
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([args.bag], default_typestore=typestore) as reader, (
        args.out / "scans.bin"
    ).open("wb") as scans, (args.out / "gt.bin").open("wb") as poses:
        connections = [c for c in reader.connections if c.topic == args.topic]
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            nearest = int(np.argmin(np.abs(gt[:, 0] - stamp)))
            if abs(gt[nearest, 0] - stamp) > 0.02:
                continue
            xyz = cloud_xyz(msg)
            scans.write(struct.pack("<dI", stamp, len(xyz)))
            scans.write(xyz.tobytes())
            poses.write(struct.pack("<d", stamp))
            poses.write(pose_matrix(gt[nearest, 1:]).astype("<f8").tobytes())
            written += 1
    print(f"map {len(points)} points, {written} scans")


if __name__ == "__main__":
    main()
