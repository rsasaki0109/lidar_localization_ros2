#!/usr/bin/env python3

import argparse
import json
import shlex
import subprocess
from pathlib import Path
from typing import List


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run bounded NDT scorer comparisons for candidate_index, route_proximity, "
            "and oracle_rank candidate ordering. This is diagnostic only and never "
            "accepts or resets pose."
        )
    )
    parser.add_argument("--attempts-csv", required=True)
    parser.add_argument("--candidates-csv", required=True)
    parser.add_argument("--bag-path", required=True)
    parser.add_argument("--map-path", required=True)
    parser.add_argument("--cloud-topic", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--registration-method", default="NDT_OMP")
    parser.add_argument("--local-map-radius", type=float, default=150.0)
    parser.add_argument("--voxel-leaf-size", type=float, default=1.0)
    parser.add_argument("--scan-min-range", type=float, default=1.0)
    parser.add_argument("--scan-max-range", type=float, default=120.0)
    parser.add_argument("--max-scan-time-diff", type=float, default=0.25)
    parser.add_argument("--ndt-resolution", type=float, default=1.0)
    parser.add_argument("--ndt-step-size", type=float, default=0.1)
    parser.add_argument("--transform-epsilon", type=float, default=0.01)
    parser.add_argument("--max-iterations", type=int, default=30)
    parser.add_argument("--num-threads", type=int, default=1)
    parser.add_argument("--score-gate-threshold", type=float, default=6.0)
    parser.add_argument("--refinement-delta-gate-m", type=float, default=2.0)
    parser.add_argument("--refinement-yaw-gate-rad", type=float, default=0.7853981633974483)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--print-only", action="store_true")
    return parser.parse_args()


def quote_command(command: List[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def run(command: List[str], print_only: bool) -> None:
    print(quote_command(command), flush=True)
    if print_only:
        return
    subprocess.run(command, check=True)


def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    scan_dir = output_dir / "registration_scans"
    labels = ["candidate_index", "route_proximity", "oracle_rank"]
    summary_args: List[str] = []

    script_dir = Path(__file__).resolve().parent
    for label in labels:
        jobs_csv = output_dir / f"relocalization_registration_jobs_{label}_top{args.top_k}.csv"
        scan_scores_csv = (
            output_dir / f"relocalization_registration_scores_{label}_top{args.top_k}.csv"
        )
        ndt_scores_csv = (
            output_dir / f"relocalization_registration_scores_ndt_{label}_top{args.top_k}.csv"
        )
        summary_json = (
            output_dir / f"relocalization_registration_score_summary_{label}_top{args.top_k}.json"
        )
        summary_md = (
            output_dir / f"relocalization_registration_score_summary_{label}_top{args.top_k}.md"
        )

        job_cmd = [
            str(script_dir / "make_registration_relocalization_jobs.py"),
            "--attempts-csv",
            str(Path(args.attempts_csv).expanduser().resolve()),
            "--candidates-csv",
            str(Path(args.candidates_csv).expanduser().resolve()),
            "--bag-path",
            str(Path(args.bag_path).expanduser().resolve()),
            "--map-path",
            str(Path(args.map_path).expanduser().resolve()),
            "--cloud-topic",
            args.cloud_topic,
            "--output-csv",
            str(jobs_csv),
            "--registration-method",
            args.registration_method,
            "--max-candidates-per-attempt",
            str(args.top_k),
            "--selection-source",
            label,
            "--voxel-leaf-size",
            str(args.voxel_leaf_size),
            "--local-map-radius",
            str(args.local_map_radius),
            "--timeout-sec",
            "1.0",
        ]
        if args.overwrite:
            job_cmd.append("--overwrite")
        run(job_cmd, args.print_only)

        resolve_cmd = [
            str(script_dir / "resolve_registration_relocalization_scans.py"),
            "--jobs-csv",
            str(jobs_csv),
            "--output-csv",
            str(scan_scores_csv),
            "--max-scan-time-diff",
            str(args.max_scan_time_diff),
            "--min-range",
            str(args.scan_min_range),
            "--max-range",
            str(args.scan_max_range),
            "--export-scan-pcd-dir",
            str(scan_dir),
        ]
        if args.overwrite:
            resolve_cmd.append("--overwrite")
        run(resolve_cmd, args.print_only)

        ndt_cmd = [
            "ros2",
            "run",
            "lidar_localization_ros2",
            "relocalization_ndt_score_jobs",
            "--input-csv",
            str(scan_scores_csv),
            "--output-csv",
            str(ndt_scores_csv),
            "--max-jobs",
            str(args.top_k),
            "--ndt-resolution",
            str(args.ndt_resolution),
            "--ndt-step-size",
            str(args.ndt_step_size),
            "--transform-epsilon",
            str(args.transform_epsilon),
            "--max-iterations",
            str(args.max_iterations),
            "--num-threads",
            str(args.num_threads),
            "--scan-voxel-leaf-size",
            str(args.voxel_leaf_size),
            "--target-voxel-leaf-size",
            str(args.voxel_leaf_size),
            "--local-map-radius",
            str(args.local_map_radius),
            "--score-gate-threshold",
            str(args.score_gate_threshold),
            "--refinement-delta-gate-m",
            str(args.refinement_delta_gate_m),
            "--refinement-yaw-gate-rad",
            str(args.refinement_yaw_gate_rad),
        ]
        run(ndt_cmd, args.print_only)

        summary_cmd = [
            str(script_dir / "summarize_registration_relocalization_scores.py"),
            "--scores-csv",
            str(ndt_scores_csv),
            "--output-json",
            str(summary_json),
            "--output-md",
            str(summary_md),
        ]
        run(summary_cmd, args.print_only)
        summary_args.extend(["--summary", f"{label}_top{args.top_k}:{summary_json}"])

    comparison_json = (
        output_dir / f"relocalization_registration_score_comparison_top{args.top_k}.json"
    )
    comparison_md = (
        output_dir / f"relocalization_registration_score_comparison_top{args.top_k}.md"
    )
    comparison_cmd = [
        str(script_dir / "compare_registration_relocalization_score_summaries.py"),
        *summary_args,
        "--output-json",
        str(comparison_json),
        "--output-md",
        str(comparison_md),
    ]
    run(comparison_cmd, args.print_only)
    if not args.print_only:
        comparison = json.loads(comparison_json.read_text(encoding="utf-8"))
        print(
            json.dumps(
                {
                    "output_json": str(comparison_json),
                    "output_md": str(comparison_md),
                    "best_by_gate_passed": comparison["best_by_gate_passed"],
                }
            )
        )


if __name__ == "__main__":
    main()
