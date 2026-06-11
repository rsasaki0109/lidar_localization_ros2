#!/usr/bin/env python3
"""Calibrate /pcl_pose covariance against ground truth from benchmark runs.

Joins pose_trace.csv (estimated poses + published covariance) with the run's
reference trajectory and per-scan fitness_score from alignment_status.csv, then
reports, per fitness bin:

- actual per-axis translation error and yaw error quantiles
- coverage of the currently published covariance (fraction of errors within
  1/2/3 published sigma)
- coverage of a candidate linear-std model
  ``std = clamp(base + gain * fitness, min, max)``

Usage:
  python3 scripts/analyze_pose_covariance_calibration.py \
    LABEL=/path/to/run_dir [LABEL=/path/to/run_dir ...] \
    [--output report.json]

Each run dir must contain pose_trace.csv, alignment_status.csv, and
trajectory_eval.json (whose reference_csv points at the ground truth trace).
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from pathlib import Path


def quaternion_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_pose_csv(path: Path) -> list[dict]:
    rows = []
    with path.open() as handle:
        for row in csv.DictReader(handle):
            stamp = float(row["stamp_sec"])
            if stamp <= 0.0:
                continue
            covariance = [float(v) for v in row["covariance"].split()] if row.get(
                "covariance", "").strip() else []
            rows.append(
                {
                    "stamp": stamp,
                    "x": float(row["position_x"]),
                    "y": float(row["position_y"]),
                    "z": float(row["position_z"]),
                    "yaw": quaternion_yaw(
                        float(row["orientation_x"]),
                        float(row["orientation_y"]),
                        float(row["orientation_z"]),
                        float(row["orientation_w"]),
                    ),
                    "covariance": covariance,
                }
            )
    rows.sort(key=lambda item: item["stamp"])
    return rows


def load_fitness(path: Path) -> list[tuple[float, float]]:
    samples = []
    with path.open() as handle:
        for row in csv.DictReader(handle):
            try:
                values = json.loads(row["values_json"])
                fitness = float(values["fitness_score"])
            except (KeyError, ValueError, json.JSONDecodeError):
                continue
            if math.isfinite(fitness):
                samples.append((float(row["stamp_sec"]), fitness))
    samples.sort()
    return samples


def nearest(sorted_pairs: list, stamps: list[float], stamp: float, tolerance: float):
    index = bisect.bisect_left(stamps, stamp)
    best = None
    best_diff = tolerance
    for candidate in (index - 1, index):
        if 0 <= candidate < len(stamps):
            diff = abs(stamps[candidate] - stamp)
            if diff <= best_diff:
                best_diff = diff
                best = sorted_pairs[candidate]
    return best


def collect_samples(run_dir: Path, max_time_diff: float) -> list[dict]:
    eval_data = json.loads((run_dir / "trajectory_eval.json").read_text())
    reference = load_pose_csv(Path(eval_data["reference_csv"]))
    estimated = load_pose_csv(run_dir / "pose_trace.csv")
    fitness = load_fitness(run_dir / "alignment_status.csv")

    reference_stamps = [row["stamp"] for row in reference]
    fitness_stamps = [pair[0] for pair in fitness]

    samples = []
    for pose in estimated:
        truth = nearest(reference, reference_stamps, pose["stamp"], max_time_diff)
        matched_fitness = nearest(fitness, fitness_stamps, pose["stamp"], max_time_diff)
        if truth is None or matched_fitness is None:
            continue
        covariance = pose["covariance"]
        samples.append(
            {
                "stamp": pose["stamp"],
                "fitness": matched_fitness[1],
                "error_x": pose["x"] - truth["x"],
                "error_y": pose["y"] - truth["y"],
                "error_z": pose["z"] - truth["z"],
                "error_yaw": wrap_angle(pose["yaw"] - truth["yaw"]),
                "published_var_x": covariance[0] if covariance else float("nan"),
                "published_var_y": covariance[7] if covariance else float("nan"),
                "published_var_yaw": covariance[35] if covariance else float("nan"),
            }
        )
    return samples


def quantile(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def coverage(samples: list[dict], std_fn, sigma: float) -> dict:
    if not samples:
        return {"xy": float("nan"), "yaw": float("nan")}
    xy_hits = 0
    yaw_hits = 0
    for sample in samples:
        std_xy, std_yaw = std_fn(sample)
        # Pool x and y: each axis is one trial against the shared xy sigma.
        xy_hits += int(abs(sample["error_x"]) <= sigma * std_xy)
        xy_hits += int(abs(sample["error_y"]) <= sigma * std_xy)
        yaw_hits += int(abs(sample["error_yaw"]) <= sigma * std_yaw)
    return {
        "xy": xy_hits / (2 * len(samples)),
        "yaw": yaw_hits / len(samples),
    }


def published_std(sample: dict) -> tuple[float, float]:
    return (
        math.sqrt(max(sample["published_var_x"], sample["published_var_y"])),
        math.sqrt(sample["published_var_yaw"]),
    )


def model_std(params: dict):
    def std_fn(sample: dict) -> tuple[float, float]:
        fitness = max(0.0, sample["fitness"])
        std_xy = min(
            params["xy_max_std_m"],
            max(params["xy_min_std_m"], params["xy_base_std_m"] +
                params["xy_std_per_fitness_m"] * fitness))
        std_yaw = min(
            params["yaw_max_std_rad"],
            max(params["yaw_min_std_rad"], params["yaw_base_std_rad"] +
                params["yaw_std_per_fitness_rad"] * fitness))
        return std_xy, std_yaw
    return std_fn


def bin_report(samples: list[dict], edges: list[float]) -> list[dict]:
    report = []
    for low, high in zip(edges[:-1], edges[1:]):
        in_bin = [s for s in samples if low <= s["fitness"] < high]
        abs_xy = [max(abs(s["error_x"]), abs(s["error_y"])) for s in in_bin]
        abs_yaw = [abs(s["error_yaw"]) for s in in_bin]
        report.append(
            {
                "fitness_range": [low, high],
                "count": len(in_bin),
                "abs_xy_error_m": {
                    "p50": quantile(abs_xy, 0.5),
                    "p68": quantile(abs_xy, 0.68),
                    "p95": quantile(abs_xy, 0.95),
                    "max": max(abs_xy) if abs_xy else float("nan"),
                },
                "abs_yaw_error_deg": {
                    "p50": math.degrees(quantile(abs_yaw, 0.5)),
                    "p68": math.degrees(quantile(abs_yaw, 0.68)),
                    "p95": math.degrees(quantile(abs_yaw, 0.95)),
                },
            }
        )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", help="LABEL=/path/to/run_dir")
    parser.add_argument("--max-time-diff", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--model", type=json.loads,
        default=None,
        help="candidate model params as JSON; defaults to the shipped defaults")
    args = parser.parse_args()

    candidate = args.model or {
        "xy_base_std_m": 0.1,
        "xy_std_per_fitness_m": 0.1,
        "xy_min_std_m": 0.1,
        "xy_max_std_m": 5.0,
        "yaw_base_std_rad": math.radians(0.5),
        "yaw_std_per_fitness_rad": math.radians(0.5),
        "yaw_min_std_rad": math.radians(0.5),
        "yaw_max_std_rad": math.radians(30.0),
    }

    edges = [0.0, 0.5, 1.0, 2.0, 4.0, 6.0, 10.0, float("inf")]
    report = {"candidate_model": candidate, "runs": {}}
    for spec in args.runs:
        label, _, path = spec.partition("=")
        run_dir = Path(path or label)
        samples = collect_samples(run_dir, args.max_time_diff)
        published = [s for s in samples if math.isfinite(s["published_var_x"])]
        run_report = {
            "run_dir": str(run_dir),
            "matched_samples": len(samples),
            "bins": bin_report(samples, edges),
            "published_coverage": {
                "1_sigma": coverage(published, published_std, 1.0),
                "2_sigma": coverage(published, published_std, 2.0),
                "3_sigma": coverage(published, published_std, 3.0),
            },
            "candidate_coverage": {
                "1_sigma": coverage(samples, model_std(candidate), 1.0),
                "2_sigma": coverage(samples, model_std(candidate), 2.0),
                "3_sigma": coverage(samples, model_std(candidate), 3.0),
            },
        }
        report["runs"][label] = run_report

        print(f"== {label} ({len(samples)} matched samples)")
        for bin_row in run_report["bins"]:
            if bin_row["count"] == 0:
                continue
            low, high = bin_row["fitness_range"]
            xy = bin_row["abs_xy_error_m"]
            yaw = bin_row["abs_yaw_error_deg"]
            print(
                f"  fitness [{low:5.1f},{high:5.1f}) n={bin_row['count']:4d}  "
                f"|xy|err p50={xy['p50']:.3f} p68={xy['p68']:.3f} "
                f"p95={xy['p95']:.3f} max={xy['max']:.3f} m  "
                f"|yaw|err p68={yaw['p68']:.2f} deg")
        for name in ("published_coverage", "candidate_coverage"):
            row = run_report[name]
            print(
                f"  {name:19s} xy 1/2/3 sigma: "
                f"{row['1_sigma']['xy']:.2f} / {row['2_sigma']['xy']:.2f} / "
                f"{row['3_sigma']['xy']:.2f}   yaw: "
                f"{row['1_sigma']['yaw']:.2f} / {row['2_sigma']['yaw']:.2f} / "
                f"{row['3_sigma']['yaw']:.2f}")

    if args.output:
        args.output.write_text(json.dumps(report, indent=2))
        print(f"report written to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
