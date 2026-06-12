#!/usr/bin/env python3
"""Render the global-localization demo GIF from recorded demo artifacts.

Storyboard: kidnapped start (no pose) -> BBS_2D candidates appear on the map
-> top candidate becomes /initialpose -> NDT localization trajectory resumes.

Inputs are the artifacts produced by the manual demo flow:
  - occupancy yaml/pgm (map background)
  - candidates.yaml (PoseArray echo of ~/candidates)
  - pose_trace.csv (recorded /pcl_pose after reinitialization)
  - event_initialpose.json (top candidate summary)
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_map(yaml_path: Path):
    # Same occupancy semantics as the BBS engine: threshold the probability
    # and flip pgm rows (top-down) into world rows (bottom-up).
    metadata = yaml.safe_load(yaml_path.read_text())
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float32)
    if int(metadata.get("negate", 0)):
        probability = pixels / 255.0
    else:
        probability = (255.0 - pixels) / 255.0
    occupied = np.flipud(probability >= float(metadata.get("occupied_thresh", 0.65)))
    render = np.where(occupied, 40, 255).astype(np.uint8)
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


def load_candidates(candidates_yaml: Path):
    # ros2 topic echo terminates messages with a --- document separator.
    data = next(iter(yaml.safe_load_all(candidates_yaml.read_text())))
    poses = []
    for pose in data["poses"]:
        qz = float(pose["orientation"]["z"])
        qw = float(pose["orientation"]["w"])
        poses.append(
            (
                float(pose["position"]["x"]),
                float(pose["position"]["y"]),
                math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz),
            ))
    return poses


def load_trajectory(pose_trace_csv: Path):
    xs, ys = [], []
    with pose_trace_csv.open() as stream:
        for row in csv.DictReader(stream):
            xs.append(float(row["position_x"]))
            ys.append(float(row["position_y"]))
    return np.asarray(xs), np.asarray(ys)


def draw_frame(ax, pixels, extent, view, title):
    ax.clear()
    ax.imshow(pixels, cmap="gray", extent=extent, origin="lower",
              vmin=0, vmax=255, interpolation="nearest")
    ax.set_xlim(view[0], view[1])
    ax.set_ylim(view[2], view[3])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=11)


def arrow(ax, x, y, yaw, length, color, width, alpha=1.0, zorder=3):
    ax.arrow(
        x, y, length * math.cos(yaw), length * math.sin(yaw),
        head_width=length * 0.45, head_length=length * 0.45,
        fc=color, ec=color, width=width, alpha=alpha,
        length_includes_head=True, zorder=zorder)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--occupancy-yaml", type=Path, required=True)
    parser.add_argument("--candidates-yaml", type=Path, required=True)
    parser.add_argument("--pose-trace-csv", type=Path, required=True)
    parser.add_argument("--event-json", type=Path, required=True)
    parser.add_argument("--output-gif", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=6)
    parser.add_argument("--trajectory-frames", type=int, default=42)
    args = parser.parse_args()

    pixels, extent = load_map(args.occupancy_yaml)
    candidates = load_candidates(args.candidates_yaml)
    xs, ys = load_trajectory(args.pose_trace_csv)
    event = json.loads(args.event_json.read_text())
    top = event["top"]

    margin = 8.0
    view = (
        min(xs.min(), min(c[0] for c in candidates)) - margin,
        max(xs.max(), max(c[0] for c in candidates)) + margin,
        min(ys.min(), min(c[1] for c in candidates)) - margin,
        max(ys.max(), max(c[1] for c in candidates)) + margin,
    )

    fig, ax = plt.subplots(figsize=(6.0, 7.0), dpi=80)
    fig.tight_layout(pad=1.5)
    frames = []
    durations = []

    def snapshot(duration_ms):
        fig.canvas.draw()
        buffer = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        frames.append(Image.fromarray(buffer.copy()))
        durations.append(int(duration_ms))

    # Phase 1: kidnapped start, no pose on the map.
    draw_frame(ax, pixels, extent, view, "Kidnapped start: no initial pose")
    snapshot(1800)

    # Phase 2: BBS_2D candidates appear, top candidate highlighted.
    top_yaw = math.radians(top["yaw_deg"])
    for _ in range(1):
        draw_frame(
            ax, pixels, extent, view,
            f"BBS_2D global localization: {len(candidates)} candidates "
            f"(top score {top['score']:.3f})")
        for cx, cy, cyaw in candidates[1:]:
            arrow(ax, cx, cy, cyaw, 3.0, "tab:blue", 0.35, alpha=0.55)
        arrow(ax, top["x"], top["y"], top_yaw, 4.0, "tab:red", 0.6, zorder=4)
        snapshot(2600)

    # Phase 3: top candidate becomes /initialpose.
    for _ in range(1):
        draw_frame(ax, pixels, extent, view,
                   "Top candidate published as /initialpose")
        arrow(ax, top["x"], top["y"], top_yaw, 4.0, "tab:red", 0.6, zorder=4)
        ax.plot(top["x"], top["y"], "o", color="tab:red", markersize=10,
                fillstyle="none", markeredgewidth=2, zorder=4)
        snapshot(2000)

    # Phase 4: NDT localization resumes; the trajectory grows.
    steps = max(1, args.trajectory_frames)
    for frame_index in range(1, steps + 1):
        upto = max(2, int(len(xs) * frame_index / steps))
        draw_frame(ax, pixels, extent, view,
                   "NDT localization resumed from the candidate")
        arrow(ax, top["x"], top["y"], top_yaw, 4.0, "tab:red", 0.6,
              alpha=0.7, zorder=3)
        ax.plot(xs[:upto], ys[:upto], "-", color="tab:green", linewidth=2.2,
                zorder=5)
        ax.plot(xs[upto - 1], ys[upto - 1], "o", color="tab:green",
                markersize=7, zorder=6)
        snapshot(1000 / max(1, args.fps))

    # Hold the final frame.
    durations[-1] = 2500

    args.output_gif.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.output_gif,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
    )
    print(f"wrote {args.output_gif} ({len(frames)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
