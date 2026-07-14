#!/usr/bin/env python3
"""Evaluate LiDAR-IMU temporal-offset identifiability on every Koide sequence."""

import argparse
import json
from pathlib import Path
import statistics
import subprocess
import sys


SEQUENCES = {
    "indoor_easy_01": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_easy_02": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_hard_01": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_kidnap_01": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_kidnap_02": ("/imu", "/points2/decompressed", "indoor"),
    "outdoor_hard_01a": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_hard_01b": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_hard_02a": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_hard_02b": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_kidnap_a": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_kidnap_b": ("/livox/imu", "/livox/points", "outdoor"),
}


def classify_offset_candidate(metrics):
    reasons = []
    if metrics["window_count"] < 2:
        reasons.append("insufficient_windows")
    if metrics["search_boundary_window_count"] > 0:
        reasons.append("window_hit_search_boundary")
    if metrics["relative_rmse_improvement_over_zero"] < 0.02:
        reasons.append("less_than_2pct_better_than_zero")
    width = metrics["one_percent_best_width_sec"]
    if width is None or width > 0.03:
        reasons.append("broad_minimum")
    mad = metrics["window_offset_mad_sec"]
    if mad is None or mad > 0.01:
        reasons.append("window_offset_unstable")
    median = metrics["window_offset_median_sec"]
    if median is None or abs(metrics["refined_offset_sec"] - median) > 0.015:
        reasons.append("full_and_window_estimates_disagree")
    return {
        "decision": "runtime_ab_candidate" if not reasons else "reject",
        "reasons": reasons,
    }


def compact_result(name, family, report):
    alignment = report["between_cloud_alignment"]
    identifiable = alignment["offset_identifiability"]
    windows = report["time_offset_windows"]
    metrics = {
        "sequence": name,
        "family": family,
        "point_time_available": bool(report["point_time_hypotheses"]),
        "best_grid_offset_sec": alignment["imu_minus_reference_time_offset_sec"],
        "refined_offset_sec": identifiable["quadratic_refined_offset_sec"],
        "zero_offset_rmse_rad_s": identifiable["zero_offset_rmse_rad_s"],
        "best_offset_rmse_rad_s": identifiable["best_offset_rmse_rad_s"],
        "relative_rmse_improvement_over_zero": identifiable[
            "relative_rmse_improvement_over_zero"],
        "one_percent_best_width_sec": identifiable["one_percent_best_width_sec"],
        "best_at_search_boundary": identifiable["best_at_search_boundary"],
        "window_count": windows["window_count"],
        "window_offset_median_sec": windows.get("offset_median_sec"),
        "window_offset_mad_sec": windows.get("offset_mad_sec"),
        "window_offset_min_sec": windows.get("offset_min_sec"),
        "window_offset_max_sec": windows.get("offset_max_sec"),
        "search_boundary_window_count": windows.get(
            "search_boundary_window_count", 0),
    }
    metrics.update(classify_offset_candidate(metrics))
    return metrics


def aggregate(results):
    families = {}
    for family in sorted({item["family"] for item in results}):
        selected = [item for item in results if item["family"] == family]
        families[family] = {
            "sequence_count": len(selected),
            "runtime_ab_candidate_count": sum(
                item["decision"] == "runtime_ab_candidate" for item in selected),
            "median_refined_offset_sec": statistics.median(
                item["refined_offset_sec"] for item in selected),
            "max_abs_refined_offset_sec": max(
                abs(item["refined_offset_sec"]) for item in selected),
        }
    return families


def markdown_report(summary):
    lines = [
        "# Koide LiDAR-IMU time-offset analysis",
        "",
        "Positive offset means the matching IMU measurement has a later timestamp than LiDAR.",
        "The runtime gate requires a narrow optimum, at least 2% improvement over zero, and",
        "agreement across 30-second windows. Indoor clouds have no per-point time field, so",
        "their estimates can validate IMU prediction timing but cannot enable point deskew.",
        "",
        "| sequence | offset ms | zero improvement | width ms | window median/MAD ms | decision |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for item in summary["sequences"]:
        median = item["window_offset_median_sec"]
        mad = item["window_offset_mad_sec"]
        window = "n/a" if median is None or mad is None else f"{median*1000:.1f}/{mad*1000:.1f}"
        width = item["one_percent_best_width_sec"]
        lines.append(
            f"| {item['sequence']} | {item['refined_offset_sec']*1000:.1f} | "
            f"{item['relative_rmse_improvement_over_zero']*100:.2f}% | "
            f"{'n/a' if width is None else f'{width*1000:.1f}'} | {window} | "
            f"{item['decision']} |")
    lines.extend(["", f"Overall decision: **{summary['promotion_decision']}**", ""])
    return "\n".join(lines)


def main():
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sequence", action="append", choices=SEQUENCES)
    parser.add_argument("--max-scans", type=int, default=0)
    parser.add_argument("--offset-step-sec", type=float, default=0.001)
    parser.add_argument("--window-sec", type=float, default=30.0)
    parser.add_argument("--window-step-sec", type=float, default=0.005)
    parser.add_argument(
        "--analyzer", type=Path,
        default=repo / "scripts" / "analyze_koide_imu_consistency.py")
    args = parser.parse_args()

    names = args.sequence or list(SEQUENCES)
    assets = args.data_root / "generated" / "localization_gif_benchmarks" / "assets"
    details = args.output_dir / "sequences"
    details.mkdir(parents=True, exist_ok=True)
    results = []
    for name in names:
        imu_topic, cloud_topic, family = SEQUENCES[name]
        output = details / f"{name}.json"
        command = [
            sys.executable, str(args.analyzer),
            "--bag", str(args.data_root / "sequences" / name),
            "--reference", str(assets / f"{name}_reference.csv"),
            "--output", str(output),
            "--imu-topic", imu_topic,
            "--cloud-topic", cloud_topic,
            "--max-scans", str(args.max_scans),
            "--time-offset-step-sec", str(args.offset_step_sec),
            "--time-offset-window-sec", str(args.window_sec),
            "--time-offset-window-step-sec", str(args.window_step_sec),
        ]
        print(f"Analyzing {name}...", flush=True)
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        results.append(compact_result(name, family, json.loads(output.read_text())))

    candidates = [item for item in results if item["decision"] == "runtime_ab_candidate"]
    deskew_candidates = [item for item in candidates if item["point_time_available"]]
    summary = {
        "schema_version": 1,
        "purpose": "LiDAR-IMU temporal-offset validation; no global localization",
        "sequence_count": len(results),
        "all_expected_sequences_analyzed": set(names) == set(SEQUENCES),
        "sequences": results,
        "families": aggregate(results),
        "runtime_ab_candidates": [item["sequence"] for item in candidates],
        "deskew_runtime_ab_candidates": [item["sequence"] for item in deskew_candidates],
        "promotion_decision": (
            "run_bounded_runtime_ab" if deskew_candidates else
            "reject_runtime_offset_no_stable_deskew_candidate"),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (args.output_dir / "summary.md").write_text(markdown_report(summary))
    print(args.output_dir / "summary.json")


if __name__ == "__main__":
    main()
