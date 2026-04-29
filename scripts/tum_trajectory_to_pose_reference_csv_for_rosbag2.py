#!/usr/bin/env python3
"""Crop a TUM trajectory to a rosbag2 time window and export benchmark reference CSV / initial pose YAML."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterator
from typing import Optional
from typing import Tuple

import yaml


def iter_tum_poses(path: Path) -> Iterator[Tuple[float, float, float, float, float, float, float, float]]:
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            yield tuple(float(part) for part in parts[:8])  # type: ignore[return-value]


def resolve_bag_metadata_path(bag_path: Path) -> Path:
    if bag_path.is_file():
        if bag_path.name != "metadata.yaml":
            raise FileNotFoundError(f"Expected a rosbag2 metadata.yaml or bag directory, got file: {bag_path}")
        return bag_path

    metadata_path = bag_path / "metadata.yaml"
    if metadata_path.exists():
        return metadata_path

    candidates = sorted(bag_path.rglob("metadata.yaml"))
    if len(candidates) == 1:
        return candidates[0]
    raise FileNotFoundError(f"Could not resolve rosbag2 metadata.yaml under {bag_path}")


def load_bag_time_window(
    metadata_path: Path,
    bag_start_offset: float,
    bag_duration: float,
) -> Tuple[float, float]:
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = metadata.get("rosbag2_bagfile_information", {})
    starting_time = info.get("starting_time", {})
    duration = info.get("duration", {})

    start_ns = starting_time.get("nanoseconds_since_epoch")
    duration_ns = duration.get("nanoseconds")
    if not isinstance(start_ns, int) or not isinstance(duration_ns, int):
        raise RuntimeError(f"Failed to read starting_time/duration from {metadata_path}")

    bag_full_start = float(start_ns) * 1e-9
    bag_full_end = bag_full_start + float(duration_ns) * 1e-9
    start_sec = bag_full_start + bag_start_offset
    end_sec = bag_full_end if bag_duration <= 0.0 else min(start_sec + bag_duration, bag_full_end)
    return start_sec, end_sec


def export_initial_pose_yaml(
    path: Path,
    position: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> None:
    data = {
        "/**": {
            "ros__parameters": {
                "set_initial_pose": True,
                "initial_pose_x": position[0],
                "initial_pose_y": position[1],
                "initial_pose_z": position[2],
                "initial_pose_qx": quaternion[0],
                "initial_pose_qy": quaternion[1],
                "initial_pose_qz": quaternion[2],
                "initial_pose_qw": quaternion[3],
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def write_reference_csv(
    rows: list[Tuple[float, float, float, float, float, float, float, float]],
    output_csv: Path,
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
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
        for index, (t, x, y, z, qx, qy, qz, qw) in enumerate(rows):
            writer.writerow(
                [
                    index,
                    f"{t:.9f}",
                    "map",
                    f"{x:.10f}",
                    f"{y:.10f}",
                    f"{z:.10f}",
                    f"{qx:.10f}",
                    f"{qy:.10f}",
                    f"{qz:.10f}",
                    f"{qw:.10f}",
                    "",
                ]
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Input TUM trajectory text file")
    parser.add_argument("--bag-path", required=True, type=Path, help="rosbag2 directory or metadata.yaml")
    parser.add_argument("--output-csv", required=True, type=Path, help="Destination reference CSV")
    parser.add_argument(
        "--output-initial-pose-yaml",
        default="",
        type=Path,
        help="Optional parameter YAML from the chosen initial row",
    )
    parser.add_argument(
        "--initial-pose-skip-sec",
        type=float,
        default=0.05,
        help="Choose the first row at least this many seconds after the cropped window start",
    )
    parser.add_argument(
        "--bag-start-offset",
        type=float,
        default=0.0,
        help="Seconds to skip from the bag start time when cropping the TUM trajectory",
    )
    parser.add_argument(
        "--bag-duration",
        type=float,
        default=0.0,
        help="Seconds of bag data to export. 0 means until bag end",
    )
    parser.add_argument(
        "--time-padding",
        type=float,
        default=0.0,
        help="Optional symmetric padding in seconds around the bag time window",
    )
    args = parser.parse_args()

    metadata_path = resolve_bag_metadata_path(args.bag_path.expanduser().resolve())
    start_sec, end_sec = load_bag_time_window(metadata_path, args.bag_start_offset, args.bag_duration)
    start_sec -= args.time_padding
    end_sec += args.time_padding

    rows = [row for row in iter_tum_poses(args.input.expanduser().resolve()) if start_sec <= row[0] <= end_sec]
    if not rows:
        raise SystemExit(
            f"No poses left after cropping to [{start_sec:.9f}, {end_sec:.9f}] from {args.input}"
        )

    write_reference_csv(rows, args.output_csv.expanduser().resolve())

    chosen: Optional[Tuple[float, float, float, float, float, float, float, float]]
    if args.output_initial_pose_yaml:
        threshold = rows[0][0] + max(args.initial_pose_skip_sec, 0.0)
        chosen = next((row for row in rows if row[0] >= threshold), None)
        if chosen is None:
            raise SystemExit(
                f"No cropped row with t >= {threshold:.9f} for initial pose (window starts at {rows[0][0]:.9f})"
            )
        _, x, y, z, qx, qy, qz, qw = chosen
        export_initial_pose_yaml(
            args.output_initial_pose_yaml.expanduser().resolve(),
            (x, y, z),
            (qx, qy, qz, qw),
        )
        print(f"initial_pose_stamp_sec: {chosen[0]:.9f}")
        print(f"output_initial_pose_yaml: {args.output_initial_pose_yaml.expanduser().resolve()}")

    print(f"metadata_path: {metadata_path}")
    print(f"crop_start_sec: {start_sec:.9f}")
    print(f"crop_end_sec: {end_sec:.9f}")
    print(f"rows_written: {len(rows)}")
    print(f"output_csv: {args.output_csv.expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
