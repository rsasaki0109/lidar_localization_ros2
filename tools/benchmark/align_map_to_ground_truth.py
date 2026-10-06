#!/usr/bin/env python3
"""Move a SLAM map into the ground-truth frame of its own mapping run.

The SE(3) transform is the least-squares fit (Umeyama, no scale) of the
mapping trajectory to the ground truth at matching stamps, so the benchmark
can score localization on that map without any alignment.

usage: align_map_to_ground_truth.py --map map.pcd --trajectory traj.tum
           --ground-truth gt.txt --out map_gt.pcd
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import benchmark_eval


def fit_se3(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotation and translation that map source points onto target points."""
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (target - target_mean).T @ (source - source_mean)
    u, _, vt = np.linalg.svd(covariance)
    sign = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
    rotation = u @ sign @ vt
    return rotation, target_mean - rotation @ source_mean


def lzf_decompress(data: bytes, size: int) -> bytes:
    """Decompress LZF, the codec of PCD binary_compressed (lidar_slam_ros2's maps)."""
    out = bytearray(size)
    ip = op = 0
    while ip < len(data):
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:
            count = ctrl + 1
            out[op : op + count] = data[ip : ip + count]
            ip += count
            op += count
            continue
        length = ctrl >> 5
        if length == 7:
            length += data[ip]
            ip += 1
        ref = op - ((ctrl & 0x1F) << 8) - data[ip] - 1
        ip += 1
        for _ in range(length + 2):  # back references may overlap the output
            out[op] = out[ref]
            op += 1
            ref += 1
    if op != size:
        raise ValueError(f"LZF data decompressed to {op} bytes, expected {size}")
    return bytes(out)


def read_pcd(path: Path) -> tuple[list[str], np.ndarray]:
    """Header lines and the points of a PCD with float x y z fields."""
    raw = path.read_bytes()
    data_start = raw.index(b"\n", raw.index(b"\nDATA") + 1) + 1
    header = raw[:data_start].decode("ascii").splitlines()
    fields = {line.split()[0]: line.split()[1:] for line in header if line.strip()}
    names, sizes, types = fields["FIELDS"], fields["SIZE"], fields["TYPE"]
    counts = fields.get("COUNT", ["1"] * len(names))
    dtype = np.dtype(
        [
            (
                name,
                {"F": "f", "U": "u", "I": "i"}[kind] + size,
                (int(count),) if int(count) > 1 else (),
            )
            for name, size, kind, count in zip(names, sizes, types, counts, strict=True)
        ]
    )
    if any(name not in dtype.names for name in ("x", "y", "z")):
        raise ValueError(f"{path}: a map needs x, y and z fields")
    points = int(fields["POINTS"][0])
    kind = fields["DATA"][0]
    if kind == "ascii":
        rows = np.loadtxt(raw[data_start:].decode("ascii").splitlines(), ndmin=2)
        cloud = np.zeros(points, dtype=dtype)
        column = 0
        for name in dtype.names:
            width = int(np.prod(dtype[name].shape)) if dtype[name].shape else 1
            values = rows[:, column : column + width]
            cloud[name] = values.reshape(cloud[name].shape)
            column += width
        return header, cloud
    if kind == "binary":
        return header, np.frombuffer(
            raw, dtype=dtype, count=points, offset=data_start
        ).copy()
    if kind == "binary_compressed":
        compressed, size = np.frombuffer(raw, dtype="<u4", count=2, offset=data_start)
        body = lzf_decompress(
            raw[data_start + 8 : data_start + 8 + int(compressed)], int(size)
        )
        cloud = np.zeros(points, dtype=dtype)
        offset = 0
        for name in dtype.names:  # stored field by field
            field = dtype[name]
            nbytes = field.itemsize * points
            cloud[name] = np.frombuffer(
                body,
                dtype=field.base,
                count=points * max(1, int(np.prod(field.shape))),
                offset=offset,
            ).reshape(cloud[name].shape)
            offset += nbytes
        return header, cloud
    raise ValueError(f"{path}: unsupported PCD data type {kind}")


def write_transformed_pcd(path: Path, out: Path, rotation, translation) -> int:
    """Write the map moved by (rotation, translation) as a binary PCD."""
    header, cloud = read_pcd(path)
    xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(np.float64)
    moved = xyz @ rotation.T + translation
    for index, name in enumerate(("x", "y", "z")):
        cloud[name] = moved[:, index]
    lines = [line for line in header if line.split() and line.split()[0] != "DATA"]
    lines = [
        "VIEWPOINT 0 0 0 1 0 0 0" if line.startswith("VIEWPOINT") else line
        for line in lines
    ]
    out.write_bytes(
        ("\n".join([*lines, "DATA binary"]) + "\n").encode("ascii") + cloud.tobytes()
    )
    return len(cloud)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)

    trajectory, truth = benchmark_eval.match_to_ground_truth(
        benchmark_eval.load_tum(args.trajectory),
        benchmark_eval.load_tum(args.ground_truth),
    )
    if len(trajectory) < 10:
        print(
            "too few trajectory poses match the ground truth in time", file=sys.stderr
        )
        return 1
    rotation, translation = fit_se3(trajectory[:, 1:4], truth[:, 1:4])
    residual = np.linalg.norm(
        trajectory[:, 1:4] @ rotation.T + translation - truth[:, 1:4], axis=1
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    points = write_transformed_pcd(args.map, args.out, rotation, translation)
    report = {
        "alignment": "se3_umeyama_trajectory_to_ground_truth",
        "map": str(args.map),
        "trajectory": str(args.trajectory),
        "ground_truth": str(args.ground_truth),
        "pairs": len(trajectory),
        "residual_rmse_m": float(np.sqrt(np.mean(residual**2))),
        "rotation": rotation.tolist(),
        "translation": translation.tolist(),
        "points": points,
    }
    args.out.with_suffix(".align.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("pairs", "residual_rmse_m", "points")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
