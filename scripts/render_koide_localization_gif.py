#!/usr/bin/env python3
"""Render a compact Koide localization replay GIF from benchmark CSV files."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
import numpy as np
import yaml
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402


ALIGNMENT_COLORS = {
    "matched": "#16a34a",
    "bridge": "#f59e0b",
    "request": "#dc2626",
}

ALIGNMENT_LABELS = {
    "matched": "NDT accepted",
    "bridge": "Odom-bridge output",
    "request": "Bridge + reinit request",
}


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


def load_alignment(csv_path: Path):
    samples = []
    with csv_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamp = float(row["stamp_sec"])
            values = json.loads(row.get("values_json") or "{}")
            action = values.get("recovery_action", "")
            if action == "request_reinitialization":
                state = "request"
            elif action in ("accept_measurement", "accept_measurement_with_warning"):
                state = "matched"
            else:
                state = "bridge"
            samples.append((stamp, state))
    if not samples:
        raise ValueError(f"no alignment samples: {csv_path}")
    return samples


def crop_poses(poses, start_offset_sec, duration_sec):
    start = float(poses[0, 0]) + max(0.0, start_offset_sec)
    end = float(poses[-1, 0]) if duration_sec is None else start + max(0.0, duration_sec)
    mask = (poses[:, 0] >= start) & (poses[:, 0] <= end)
    cropped = poses[mask]
    if len(cropped) < 2:
        raise ValueError(
            f"requested replay window has fewer than two poses: start={start:.3f}, end={end:.3f}"
        )
    return cropped


def load_metrics_label(trajectory_eval_path: Path, bridge_coverage_path: Path, prefix: str):
    trajectory = json.loads(trajectory_eval_path.read_text(encoding="utf-8"))
    coverage = json.loads(bridge_coverage_path.read_text(encoding="utf-8"))
    return (
        f"{prefix}RMSE {float(trajectory['translation_rmse_m']):.3f} m | "
        f"output {float(coverage['output_coverage_percent']):.1f}% | "
        f"NDT matched {float(coverage['matched_coverage_percent']):.1f}% | "
        f"max gap {float(coverage['max_output_gap_sec']):.3f} s"
    )


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


def motion_headings(poses):
    """Estimate direction of travel from the trajectory, not sensor orientation."""
    headings = np.empty(len(poses), dtype=np.float64)
    last_heading = float(poses[0, 3])
    minimum_baseline = 0.3
    for index in range(len(poses)):
        before = index
        while before > 0:
            before -= 1
            if math.hypot(
                float(poses[index, 1] - poses[before, 1]),
                float(poses[index, 2] - poses[before, 2]),
            ) >= minimum_baseline:
                break

        after = index
        while after < len(poses) - 1:
            after += 1
            if math.hypot(
                float(poses[after, 1] - poses[index, 1]),
                float(poses[after, 2] - poses[index, 2]),
            ) >= minimum_baseline:
                break

        dx = float(poses[after, 1] - poses[before, 1])
        dy = float(poses[after, 2] - poses[before, 2])
        if before != after and math.hypot(dx, dy) >= minimum_baseline:
            last_heading = math.atan2(dy, dx)
        headings[index] = last_heading
    return headings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--occupancy-yaml", type=Path, required=True)
    parser.add_argument("--estimated-csv", type=Path)
    parser.add_argument(
        "--alignment-csv",
        type=Path,
        help="Optional alignment diagnostics used to distinguish NDT matches, bridge output, and requests",
    )
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument(
        "--reference-only",
        action="store_true",
        help="Animate the reference route when no localization estimate is available",
    )
    parser.add_argument("--output-gif", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--start-offset-sec", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float)
    parser.add_argument("--sequence-label", default="Koide outdoor_hard_01a")
    parser.add_argument("--estimate-label", default="NDT_OMP estimate")
    parser.add_argument("--title", default="ROS 2 LiDAR localization replay (LiDAR-only)")
    parser.add_argument(
        "--metrics-label",
        default="",
        help="Verified run-level metrics displayed below the replay status",
    )
    parser.add_argument("--trajectory-eval-json", type=Path)
    parser.add_argument("--bridge-coverage-json", type=Path)
    parser.add_argument("--metrics-prefix", default="")
    args = parser.parse_args()

    if args.reference_only and args.estimated_csv is not None:
        parser.error("--reference-only cannot be combined with --estimated-csv")
    if args.reference_only and args.alignment_csv is not None:
        parser.error("--reference-only cannot be combined with --alignment-csv")
    if not args.reference_only and args.estimated_csv is None:
        parser.error("--estimated-csv is required unless --reference-only is used")
    if bool(args.trajectory_eval_json) != bool(args.bridge_coverage_json):
        parser.error("--trajectory-eval-json and --bridge-coverage-json must be provided together")
    if args.metrics_label and args.trajectory_eval_json:
        parser.error("--metrics-label cannot be combined with metric JSON inputs")

    metrics_label = args.metrics_label
    if args.trajectory_eval_json:
        metrics_label = load_metrics_label(
            args.trajectory_eval_json, args.bridge_coverage_json, args.metrics_prefix
        )

    pixels, extent = load_occupancy(args.occupancy_yaml)
    reference_raw = load_poses(args.reference_csv)
    if args.reference_only:
        estimated_raw = reference_raw
        pose_time_origin = float(estimated_raw[0, 0])
        reference = reference_raw
        alignment = []
    else:
        complete_estimated = load_poses(args.estimated_csv)
        pose_time_origin = float(complete_estimated[0, 0])
        estimated_raw = crop_poses(
            complete_estimated, args.start_offset_sec, args.duration_sec
        )
        reference = reference_window(
            reference_raw, estimated_raw[0, 0], estimated_raw[-1, 0]
        )
        alignment = load_alignment(args.alignment_csv) if args.alignment_csv else []
        alignment = [
            sample for sample in alignment
            if estimated_raw[0, 0] <= sample[0] <= estimated_raw[-1, 0]
        ]
        if args.alignment_csv and not alignment:
            raise ValueError("alignment CSV does not overlap the estimated trajectory")
    estimated = interpolate_poses(estimated_raw, max(2, args.frames))
    travel_yaws = motion_headings(estimated)

    alignment_stamp = np.asarray([sample[0] for sample in alignment], dtype=np.float64)
    alignment_state = [sample[1] for sample in alignment]
    if alignment:
        alignment_x = np.interp(alignment_stamp, estimated_raw[:, 0], estimated_raw[:, 1])
        alignment_y = np.interp(alignment_stamp, estimated_raw[:, 0], estimated_raw[:, 2])

    all_x = np.concatenate((estimated[:, 1], reference[:, 1]))
    all_y = np.concatenate((estimated[:, 2], reference[:, 2]))
    margin = 7.0
    view = (all_x.min() - margin, all_x.max() + margin, all_y.min() - margin, all_y.max() + margin)

    fig, ax = plt.subplots(figsize=(7.2, 5.0), dpi=90)
    fig.patch.set_facecolor("#0b1220")
    fig.subplots_adjust(left=0.015, right=0.985, bottom=0.025, top=0.91)
    frames = []
    frame_duration = int(round(1000 / max(1, args.fps)))

    for index, (stamp, x, y, _) in enumerate(estimated):
        yaw = travel_yaws[index]
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
        ax.plot(estimated[: index + 1, 1], estimated[: index + 1, 2], color="#0f766e",
                linewidth=2.8, label=args.estimate_label)
        current_state = None
        if alignment:
            visible_count = int(np.searchsorted(alignment_stamp, stamp, side="right"))
            for state in ("matched", "bridge", "request"):
                indices = [
                    sample_index for sample_index in range(visible_count)
                    if alignment_state[sample_index] == state
                ]
                if indices:
                    ax.scatter(
                        alignment_x[indices], alignment_y[indices],
                        s=10 if state != "request" else 18,
                        color=ALIGNMENT_COLORS[state], alpha=0.75,
                        edgecolors="none", zorder=4,
                    )
            nearest_index = int(np.argmin(np.abs(alignment_stamp - stamp)))
            current_state = alignment_state[nearest_index]
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

        elapsed = stamp - pose_time_origin
        status = f"{args.sequence_label}  |  t = {elapsed:5.1f} s"
        if not args.reference_only:
            nearest = reference[np.argmin(np.abs(reference[:, 0] - stamp))]
            error = math.hypot(x - nearest[1], y - nearest[2])
            status += f"  |  error = {error:.2f} m"
        if current_state is not None:
            status += f"\nState: {ALIGNMENT_LABELS[current_state]}"
        ax.text(
            0.02,
            0.97,
            status,
            transform=ax.transAxes,
            va="top",
            color="white",
            fontsize=10,
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "#0f172a", "alpha": 0.9,
                  "edgecolor": "none"},
        )
        if metrics_label:
            ax.text(
                0.02,
                0.835,
                metrics_label,
                transform=ax.transAxes,
                va="top",
                color="#e2e8f0",
                fontsize=8.5,
                bbox={"boxstyle": "round,pad=0.3", "facecolor": "#0f172a", "alpha": 0.86,
                      "edgecolor": "none"},
            )
        handles, labels = ax.get_legend_handles_labels()
        if alignment:
            handles.extend(
                Line2D(
                    [0], [0], marker="o", linestyle="none",
                    markerfacecolor=ALIGNMENT_COLORS[state], markeredgecolor="none",
                    markersize=6, label=ALIGNMENT_LABELS[state],
                )
                for state in ("matched", "bridge", "request")
            )
            labels.extend(ALIGNMENT_LABELS[state] for state in ("matched", "bridge", "request"))
        ax.legend(handles, labels, loc="lower right", framealpha=0.92, fontsize=8)
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(args.title, color="white", pad=8)
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
