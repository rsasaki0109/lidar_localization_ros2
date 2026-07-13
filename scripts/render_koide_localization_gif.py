#!/usr/bin/env python3
"""Render a compact Koide localization replay GIF from benchmark CSV files."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
import numpy as np
import yaml
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_occupancy(yaml_path: Path):
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float32)
    probability = pixels / 255.0 if metadata.get("negate", 0) else (255.0 - pixels) / 255.0
    occupied = np.flipud(probability >= float(metadata.get("occupied_thresh", 0.65)))
    render = np.where(occupied, 35, 245).astype(np.uint8)
    resolution = float(metadata["resolution"])
    origin = metadata.get("origin", [0.0, 0.0, 0.0])
    height, width = render.shape
    extent = (
        float(origin[0]),
        float(origin[0]) + width * resolution,
        float(origin[1]),
        float(origin[1]) + height * resolution,
    )
    return render, extent


def load_poses(csv_path: Path):
    poses = []
    with csv_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamp = float(row["stamp_sec"])
            if stamp <= 0.0:
                continue
            qx = float(row["orientation_x"])
            qy = float(row["orientation_y"])
            qz = float(row["orientation_z"])
            qw = float(row["orientation_w"])
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
            poses.append((stamp, float(row["position_x"]), float(row["position_y"]), yaw))
    if len(poses) < 2:
        raise ValueError(f"need at least two timestamped poses: {csv_path}")
    return np.asarray(poses, dtype=np.float64)


def interpolate_poses(poses, frame_count):
    source_time = poses[:, 0]
    frame_time = np.linspace(source_time[0], source_time[-1], frame_count)
    xs = np.interp(frame_time, source_time, poses[:, 1])
    ys = np.interp(frame_time, source_time, poses[:, 2])
    yaws = np.interp(frame_time, source_time, np.unwrap(poses[:, 3]))
    return np.column_stack((frame_time, xs, ys, yaws))


def reference_window(reference, start, end):
    mask = (reference[:, 0] >= start - 0.2) & (reference[:, 0] <= end + 0.2)
    window = reference[mask]
    if len(window) < 2:
        raise ValueError("reference CSV does not overlap the estimated trajectory")
    return window


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--occupancy-yaml", type=Path, required=True)
    parser.add_argument("--estimated-csv", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output-gif", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args()

    pixels, extent = load_occupancy(args.occupancy_yaml)
    estimated_raw = load_poses(args.estimated_csv)
    reference = reference_window(
        load_poses(args.reference_csv), estimated_raw[0, 0], estimated_raw[-1, 0]
    )
    estimated = interpolate_poses(estimated_raw, max(2, args.frames))

    all_x = np.concatenate((estimated[:, 1], reference[:, 1]))
    all_y = np.concatenate((estimated[:, 2], reference[:, 2]))
    margin = 7.0
    view = (all_x.min() - margin, all_x.max() + margin, all_y.min() - margin, all_y.max() + margin)

    fig, ax = plt.subplots(figsize=(7.2, 5.0), dpi=90)
    fig.patch.set_facecolor("#0b1220")
    frames = []
    frame_duration = int(round(1000 / max(1, args.fps)))

    for index, (stamp, x, y, yaw) in enumerate(estimated):
        ax.clear()
        ax.set_facecolor("#e5e7eb")
        ax.imshow(
            pixels,
            cmap="gray",
            extent=extent,
            origin="lower",
            vmin=0,
            vmax=255,
            interpolation="nearest",
        )
        ax.plot(reference[:, 1], reference[:, 2], "--", color="#2563eb", linewidth=1.5,
                alpha=0.8, label="Ground truth")
        ax.plot(estimated[: index + 1, 1], estimated[: index + 1, 2], color="#16a34a",
                linewidth=2.8, label="NDT_OMP estimate")
        ax.arrow(
            x,
            y,
            2.8 * math.cos(yaw),
            2.8 * math.sin(yaw),
            width=0.35,
            head_width=1.5,
            head_length=1.5,
            color="#dc2626",
            length_includes_head=True,
            zorder=5,
        )

        nearest = reference[np.argmin(np.abs(reference[:, 0] - stamp))]
        error = math.hypot(x - nearest[1], y - nearest[2])
        elapsed = stamp - estimated[0, 0]
        ax.text(
            0.02,
            0.97,
            f"Koide outdoor_hard_01a  |  t = {elapsed:4.1f} s  |  error = {error:.2f} m",
            transform=ax.transAxes,
            va="top",
            color="white",
            fontsize=10,
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "#0f172a", "alpha": 0.9,
                  "edgecolor": "none"},
        )
        ax.legend(loc="lower right", framealpha=0.92, fontsize=9)
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title("ROS 2 LiDAR localization replay (LiDAR-only)", color="white", pad=8)
        fig.tight_layout(pad=0.8)
        fig.canvas.draw()
        rgb = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        frames.append(Image.fromarray(rgb.copy()))

    plt.close(fig)
    durations = [frame_duration] * len(frames)
    durations[-1] = 1600
    args.output_gif.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.output_gif,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
        optimize=True,
    )
    print(f"wrote {args.output_gif} ({len(frames)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
