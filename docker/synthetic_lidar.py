#!/usr/bin/env python3
"""Synthetic room map and LiDAR stream for the Docker smoke test.

Not a benchmark: a hand-made L-shaped hall with pillars, scanned from a sensor
that circles near the origin. The scan condition is read from a mode file each
cycle, so the smoke test can switch it while quickstart runs:

  ok      scans from the true pose
  sparse  only 40 points per scan
  kidnap  scans from 7 m and 70 deg away from the tracked pose
  stop    no scans

Usage:
  synthetic_lidar.py --write-map map.pcd
  synthetic_lidar.py --mode-file mode.txt [--topic /cloud] [--frame velodyne]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

SENSOR_HEIGHT_M = 1.0


def make_map(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    parts = []

    def wall(x0, y0, x1, y1, count):
        t = rng.random(count)
        parts.append(
            np.c_[x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, rng.random(count) * 3.0]
        )

    wall(-10, -6, 20, -6, 3000)
    wall(20, -6, 20, 6, 1200)
    wall(20, 6, 4, 6, 1600)
    wall(4, 6, 4, 20, 1400)
    wall(4, 20, -10, 20, 1400)
    wall(-10, 20, -10, -6, 2600)
    for cx, cy in [(0, 0), (8, 2), (14, -3), (-5, 10), (0, 15), (-6, -2)]:
        for angle in np.linspace(0.0, 2.0 * math.pi, 60, endpoint=False):
            parts.append(
                np.c_[
                    np.full(20, cx + 0.4 * math.cos(angle)),
                    np.full(20, cy + 0.4 * math.sin(angle)),
                    np.linspace(0.0, 3.0, 20),
                ]
            )
    floor = np.c_[rng.uniform(-10, 20, 6000), rng.uniform(-6, 20, 6000), np.zeros(6000)]
    parts.append(floor[~((floor[:, 0] > 4) & (floor[:, 1] > 6))])
    return np.vstack(parts).astype(np.float32)


def write_pcd(path: Path, points: np.ndarray) -> None:
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    path.write_bytes(header.encode("ascii") + points.astype(np.float32).tobytes())


def sensor_pose(t: float, mode: str) -> tuple[float, float, float]:
    x = 1.0 * math.sin(0.1 * t)
    y = 0.5 * (1.0 - math.cos(0.1 * t))
    yaw = 0.05 * math.sin(0.1 * t)
    if mode == "kidnap":
        return x + 6.0, y + 4.0, yaw + 1.2
    return x, y, yaw


def scan(cloud: np.ndarray, pose, mode: str, rng) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = cloud - np.array([x, y, SENSOR_HEIGHT_M], dtype=np.float32)
    local = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]]
    r = np.linalg.norm(local[:, :2], axis=1)
    local = local[(r > 0.5) & (r < 25.0)]
    if mode == "sparse":
        local = local[:40]
    return (local + rng.normal(0.0, 0.01, local.shape)).astype(np.float32)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--write-map", type=Path)
    parser.add_argument("--mode-file", type=Path)
    parser.add_argument("--topic", default="/cloud")
    parser.add_argument("--frame", default="velodyne")
    args = parser.parse_args()
    cloud = make_map()
    if args.write_map:
        write_pcd(args.write_map, cloud)
        print(f"wrote {len(cloud)} points to {args.write_map}")
        return 0

    import rclpy
    from sensor_msgs.msg import PointCloud2, PointField

    rclpy.init()
    node = rclpy.create_node("synthetic_lidar")
    publisher = node.create_publisher(PointCloud2, args.topic, 10)
    rng = np.random.default_rng(1)
    state = {"t": 0.0}

    def tick():
        mode = "ok"
        if args.mode_file and args.mode_file.exists():
            mode = args.mode_file.read_text().strip() or "ok"
        if mode == "stop":
            return
        state["t"] += 0.1
        points = scan(cloud, sensor_pose(state["t"], mode), mode, rng)
        msg = PointCloud2()
        msg.header.frame_id = args.frame
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(points)
        msg.fields = [
            PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate("xyz")
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = points.tobytes()
        publisher.publish(msg)

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
