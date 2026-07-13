#!/usr/bin/env python3
"""Aggregate repeated Koide corner no-deskew vs deskew comparisons."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


MODES = (
    "lidar_only",
    "deskew",
    "imu_pose_history",
    "lidar_constant_velocity",
    "localizability_guard",
)
IMU_REQUIRED_MODES = {"deskew", "imu_pose_history"}


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _number(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _median(values: Sequence[Optional[float]]) -> Optional[float]:
    present = [value for value in values if value is not None]
    return statistics.median(present) if present else None


def _mode_runs(comparison: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    result: Dict[str, Dict[str, Any]] = {}
    for run in comparison.get("runs", []):
        if not isinstance(run, dict):
            continue
        name = str(run.get("backend_hint") or Path(str(run.get("run_dir", ""))).name)
        if name:
            result[name] = run
    return result


def _imu_level(document: Dict[str, Any]) -> str:
    checks = document.get("checks", [])
    if any(isinstance(item, dict) and item.get("level") == "FAIL" for item in checks):
        return "FAIL"
    if any(isinstance(item, dict) and item.get("level") == "WARN" for item in checks):
        return "WARN"
    return "OK" if checks else "n/a"


def load_rows(output_dir: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for repeat_dir in sorted(output_dir.glob("repeat_*")):
        comparison = _load_json(repeat_dir / "comparison.json")
        by_mode = _mode_runs(comparison)
        for mode in MODES:
            run = by_mode.get(mode)
            if run is None:
                continue
            trajectory = run.get("trajectory_eval", {})
            alignment = run.get("alignment", {})
            pose_trace = run.get("pose_trace", {})
            imu_document = _load_json(repeat_dir / mode / "imu_validation.json")
            imu_summary = imu_document.get("summary", {})
            rows.append({
                "repeat": repeat_dir.name,
                "mode": mode,
                "translation_rmse_m": _number(trajectory.get("translation_rmse_m")),
                "translation_error_last_m": _number(
                    trajectory.get("translation_error_last_m")),
                "translation_error_max_m": _number(
                    trajectory.get("translation_error_max_m")),
                "rotation_rmse_deg": _number(trajectory.get("rotation_rmse_deg")),
                "rotation_error_last_deg": _number(
                    trajectory.get("rotation_error_last_deg")),
                "matched_sample_count": _number(trajectory.get("matched_sample_count")),
                "pose_requested_duration_ratio": _number(
                    pose_trace.get("requested_duration_ratio")),
                "rejected_streak_max": _number(alignment.get("rejected_streak_max")),
                "reinitialization_requested_rows": _number(
                    alignment.get("reinitialization_requested_rows")),
                "alignment_time_p95_sec": _number(alignment.get("alignment_time_p95_sec")),
                "imu_check": _imu_level(imu_document),
                "deskew_applied_ratio": _number(
                    imu_summary.get("continuous_time_deskew_applied_ratio")),
                "scan_time_duration_median_sec": _number(
                    imu_summary.get("scan_time_duration_median_sec")),
                "scan_time_duration_p95_sec": _number(
                    imu_summary.get("scan_time_duration_p95_sec")),
                "scan_time_duration_max_sec": _number(
                    imu_summary.get("scan_time_duration_max_sec")),
                "deskew_invalid_time_count_max": _number(
                    imu_summary.get(
                        "continuous_time_deskew_skipped_invalid_time_count_max")),
                "deskew_clamped_time_count_max": _number(
                    imu_summary.get("continuous_time_deskew_clamped_time_count_max")),
                "pose_history_coverage_median": _number(
                    imu_summary.get(
                        "continuous_time_deskew_pose_history_coverage_median")),
            })
    return rows


def summarize(
    rows: Sequence[Dict[str, Any]],
    expected_repeats: int,
    max_end_error_m: float,
    max_translation_error_m: float,
    max_rotation_rmse_deg: float,
    max_rejected_streak: int,
    min_coverage_ratio: float,
    max_latency_ratio: float,
) -> Dict[str, Any]:
    summaries: Dict[str, Dict[str, Any]] = {}
    for mode in MODES:
        selected = [row for row in rows if row["mode"] == mode]
        success_count = sum(
            1 for row in selected
            if row["translation_error_last_m"] is not None
            and row["translation_error_last_m"] <= max_end_error_m
            and row["translation_error_max_m"] is not None
            and row["translation_error_max_m"] <= max_translation_error_m
            and row["rotation_rmse_deg"] is not None
            and row["rotation_rmse_deg"] <= max_rotation_rmse_deg
            and (row["pose_requested_duration_ratio"] or 0.0) >= min_coverage_ratio
            and (row["rejected_streak_max"] or 0.0) <= max_rejected_streak
            and (row["reinitialization_requested_rows"] or 0.0) == 0.0
            and (mode not in IMU_REQUIRED_MODES or row["imu_check"] == "OK")
        )
        summaries[mode] = {
            "repeat_count": len(selected),
            "success_count": success_count,
            "translation_rmse_median": _median(
                [row["translation_rmse_m"] for row in selected]),
            "translation_error_last_median": _median(
                [row["translation_error_last_m"] for row in selected]),
            "translation_error_last_worst": max(
                (row["translation_error_last_m"] for row in selected
                 if row["translation_error_last_m"] is not None),
                default=None,
            ),
            "translation_error_max_worst": max(
                (row["translation_error_max_m"] for row in selected
                 if row["translation_error_max_m"] is not None),
                default=None,
            ),
            "rotation_rmse_median_deg": _median(
                [row["rotation_rmse_deg"] for row in selected]),
            "alignment_time_p95_median_sec": _median(
                [row["alignment_time_p95_sec"] for row in selected]),
            "rejected_streak_worst": max(
                (row["rejected_streak_max"] for row in selected
                 if row["rejected_streak_max"] is not None),
                default=None,
            ),
            "deskew_applied_ratio_median": _median(
                [row["deskew_applied_ratio"] for row in selected]),
            "scan_time_duration_p95_median_sec": _median(
                [row["scan_time_duration_p95_sec"] for row in selected]),
            "pose_history_coverage_median": _median(
                [row["pose_history_coverage_median"] for row in selected]),
        }

    lidar = summaries["lidar_only"]
    lidar_latency = lidar["alignment_time_p95_median_sec"]
    latency_ratios: Dict[str, Optional[float]] = {}
    candidate_pass: Dict[str, bool] = {}
    gates = {"repeat_matrix_complete": all(
        summaries[mode]["repeat_count"] >= expected_repeats for mode in MODES)}
    for mode in MODES[1:]:
        candidate = summaries[mode]
        latency = candidate["alignment_time_p95_median_sec"]
        latency_ratio = (
            latency / lidar_latency
            if lidar_latency is not None and lidar_latency > 0.0 and latency is not None
            else None
        )
        latency_ratios[mode] = latency_ratio
        improves_accuracy = (
            candidate["success_count"] > lidar["success_count"]
            or (
                candidate["translation_rmse_median"] is not None
                and lidar["translation_rmse_median"] is not None
                and candidate["translation_rmse_median"] < lidar["translation_rmse_median"]
            )
        )
        stable = candidate["success_count"] >= expected_repeats
        timely = latency_ratio is not None and latency_ratio <= max_latency_ratio
        coverage_ok = (
            mode != "imu_pose_history"
            or (
                candidate["pose_history_coverage_median"] is not None
                and candidate["pose_history_coverage_median"] >= min_coverage_ratio
            )
        )
        gates[f"{mode}_success_all_repeats"] = stable
        gates[f"{mode}_improves_accuracy_or_success_rate"] = improves_accuracy
        gates[f"{mode}_latency_ratio_at_most_limit"] = timely
        gates[f"{mode}_coverage_at_least_minimum"] = coverage_ok
        candidate_pass[mode] = stable and improves_accuracy and timely and coverage_ok
    overall_pass = gates["repeat_matrix_complete"] and any(candidate_pass.values())
    return {
        "expected_repeats": expected_repeats,
        "max_end_error_m": max_end_error_m,
        "max_translation_error_m": max_translation_error_m,
        "max_rotation_rmse_deg": max_rotation_rmse_deg,
        "max_rejected_streak": max_rejected_streak,
        "min_coverage_ratio": min_coverage_ratio,
        "max_latency_ratio": max_latency_ratio,
        "latency_ratios": latency_ratios,
        "candidate_pass": candidate_pass,
        "modes": summaries,
        "rows": list(rows),
        "gates": gates,
        "overall_pass": overall_pass,
    }


def _fmt(value: Any, digits: int = 3) -> str:
    return "n/a" if value is None else f"{float(value):.{digits}f}"


def markdown(summary: Dict[str, Any]) -> str:
    lines = [
        f"# Koide Corner Deskew A/B: {'PASS' if summary['overall_pass'] else 'FAIL'}",
        "",
        "| Repeat | Mode | RMSE m | Max error m | End error m | Rot RMSE deg | Coverage | Reject max | Reinit rows | Align p95 s | Deskew applied | Scan span p95 s | IMU check |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in summary["rows"]:
        lines.append(
            "| " + " | ".join([
                row["repeat"], row["mode"], _fmt(row["translation_rmse_m"]),
                _fmt(row["translation_error_max_m"]),
                _fmt(row["translation_error_last_m"]),
                _fmt(row["rotation_rmse_deg"]),
                _fmt(row["pose_requested_duration_ratio"]),
                _fmt(row["rejected_streak_max"], 0),
                _fmt(row["reinitialization_requested_rows"], 0),
                _fmt(row["alignment_time_p95_sec"]),
                _fmt(row["deskew_applied_ratio"]),
                _fmt(row["scan_time_duration_p95_sec"]), row["imu_check"],
            ]) + " |"
        )
    lines.extend(["", "## Gates", ""])
    for name, passed in summary["gates"].items():
        lines.append(f"- [{'x' if passed else ' '}] `{name}`")
    lines.extend([
        "",
        f"- candidate/lidar alignment p95 ratios: `{summary['latency_ratios']}`",
        f"- allowed ratio: `{_fmt(summary['max_latency_ratio'])}`",
        f"- required end error: `<= {_fmt(summary['max_end_error_m'])} m`",
        f"- required max error: `<= {_fmt(summary['max_translation_error_m'])} m`",
        f"- required rotation RMSE: `<= {_fmt(summary['max_rotation_rmse_deg'])} deg`",
        f"- allowed reject streak: `<= {summary['max_rejected_streak']}`",
        "",
    ])
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--expected-repeats", type=int, default=3)
    parser.add_argument("--max-end-error-m", type=float, default=1.0)
    parser.add_argument("--max-translation-error-m", type=float, default=2.0)
    parser.add_argument("--max-rotation-rmse-deg", type=float, default=5.0)
    parser.add_argument("--max-rejected-streak", type=int, default=3)
    parser.add_argument("--min-coverage-ratio", type=float, default=0.8)
    parser.add_argument("--max-latency-ratio", type=float, default=1.2)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = Path(args.output_dir).expanduser().resolve()
    try:
        result = summarize(
            load_rows(output_dir),
            max(1, args.expected_repeats),
            max(0.0, args.max_end_error_m),
            max(0.0, args.max_translation_error_m),
            max(0.0, args.max_rotation_rmse_deg),
            max(0, args.max_rejected_streak),
            max(0.0, min(1.0, args.min_coverage_ratio)),
            max(0.0, args.max_latency_ratio),
        )
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"summary error: {error}", file=sys.stderr)
        return 2
    json_path = Path(args.output_json).expanduser() if args.output_json else output_dir / "summary.json"
    md_path = Path(args.output_md).expanduser() if args.output_md else output_dir / "summary.md"
    json_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md_path.write_text(markdown(result), encoding="utf-8")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    return 0 if result["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
