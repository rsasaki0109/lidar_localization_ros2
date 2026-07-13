#!/usr/bin/env python3
"""Summarize whether NDT Hessian degeneracy precedes Koide failures."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def load_pose_rows(path):
    rows = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append({
                "stamp": float(row["stamp_sec"]),
                "position": tuple(float(row[f"position_{axis}"]) for axis in "xyz"),
                "quaternion": tuple(float(row[f"orientation_{axis}"]) for axis in "xyzw"),
            })
    return rows


def nearest(rows, stamp, max_difference=0.1):
    candidate = min(rows, key=lambda row: abs(row["stamp"] - stamp))
    return candidate if abs(candidate["stamp"] - stamp) <= max_difference else None


def pose_error(estimated, reference):
    translation = math.sqrt(sum(
        (estimated["position"][i] - reference["position"][i]) ** 2 for i in range(3)))
    dot = abs(sum(estimated["quaternion"][i] * reference["quaternion"][i] for i in range(4)))
    en = math.sqrt(sum(value * value for value in estimated["quaternion"]))
    rn = math.sqrt(sum(value * value for value in reference["quaternion"]))
    rotation = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot / (en * rn)))))
    return translation, rotation


def load_diagnostics(path):
    rows = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            values = json.loads(row["values_json"])
            if values.get("registration_localizability_enabled") != "true":
                continue
            rows.append({
                "stamp": float(row["stamp_sec"]),
                "message": row["message"],
                "valid": values.get("registration_localizability_valid") == "true",
                "weak_ratio": float(values["registration_localizability_weak_ratio"]),
                "condition": float(values["registration_localizability_absolute_condition_number"]),
                "nonpositive": int(values["registration_localizability_nonpositive_eigenvalue_count"]),
                "rejected_streak": int(values["consecutive_rejected_updates"]),
                "alignment_time": float(values["alignment_time_sec"]),
            })
    return rows


def summarize_run(run_dir):
    diagnostics = load_diagnostics(run_dir / "alignment_status.csv")
    estimates = load_pose_rows(run_dir / "pose_trace.csv")
    reference = load_pose_rows(run_dir.parent / "reference.csv")
    first_reject_index = next(
        (index for index, row in enumerate(diagnostics) if row["message"] != "ok"), None)
    first_reject_stamp = diagnostics[first_reject_index]["stamp"] if first_reject_index is not None else None

    # A causal alarm uses only earlier rows: weak ratio <= half the expanding
    # median and condition >= twice the expanding median, after four warm-up rows.
    alarm_indices = []
    for index in range(4, len(diagnostics)):
        history = diagnostics[:index]
        weak_median = statistics.median(row["weak_ratio"] for row in history)
        condition_median = statistics.median(row["condition"] for row in history)
        row = diagnostics[index]
        if row["weak_ratio"] <= 0.5 * weak_median and row["condition"] >= 2.0 * condition_median:
            alarm_indices.append(index)

    pose_errors = []
    for estimate in estimates:
        match = nearest(reference, estimate["stamp"])
        if match is not None:
            translation, rotation = pose_error(estimate, match)
            pose_errors.append({"stamp": estimate["stamp"], "translation_m": translation,
                                "rotation_deg": rotation})
    first_gt_error = next(
        (row for row in pose_errors if row["translation_m"] > 2.0 or row["rotation_deg"] > 5.0), None)
    event_stamps = [stamp for stamp in (
        first_reject_stamp, first_gt_error["stamp"] if first_gt_error else None) if stamp is not None]
    first_failure_stamp = min(event_stamps) if event_stamps else None
    causal_alarms = [diagnostics[index]["stamp"] for index in alarm_indices
                     if first_failure_stamp is not None and diagnostics[index]["stamp"] < first_failure_stamp]
    pre_reject = diagnostics[:first_reject_index] if first_reject_index is not None else diagnostics
    return {
        "run_dir": str(run_dir),
        "diagnostic_rows": len(diagnostics),
        "valid_rows": sum(row["valid"] for row in diagnostics),
        "nonpositive_rows": sum(row["nonpositive"] > 0 for row in diagnostics),
        "first_reject_stamp": first_reject_stamp,
        "max_rejected_streak": max((row["rejected_streak"] for row in diagnostics), default=0),
        "first_gt_error_stamp": first_gt_error["stamp"] if first_gt_error else None,
        "translation_error_max_m": max((row["translation_m"] for row in pose_errors), default=None),
        "rotation_error_max_deg": max((row["rotation_deg"] for row in pose_errors), default=None),
        "pre_reject_weak_ratio_min": min((row["weak_ratio"] for row in pre_reject), default=None),
        "pre_reject_weak_ratio_last": pre_reject[-1]["weak_ratio"] if pre_reject else None,
        "pre_reject_condition_max": max((row["condition"] for row in pre_reject), default=None),
        "causal_alarm_stamps": causal_alarms,
        "indicator_preceded_failure": bool(causal_alarms),
        "alignment_time_median_sec": statistics.median(
            row["alignment_time"] for row in diagnostics) if diagnostics else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-markdown", type=Path)
    args = parser.parse_args()
    run_dirs = sorted(args.output_dir.glob("repeat_*/registration_localizability"))
    runs = []
    for run_dir in run_dirs:
        if (run_dir / "alignment_status.csv").exists() and (run_dir / "pose_trace.csv").exists():
            try:
                run = summarize_run(run_dir)
                if run["diagnostic_rows"]:
                    runs.append(run)
            except (ValueError, FileNotFoundError):
                continue
    report = {
        "schema_version": 1,
        "repeat_count": len(runs),
        "all_repeats_valid": len(runs) >= 3 and all(
            run["valid_rows"] == run["diagnostic_rows"] for run in runs),
        "indicator_preceded_failure_repeat_count": sum(
            run["indicator_preceded_failure"] for run in runs),
        "eligible_for_mitigation_variant": len(runs) >= 3 and all(
            run["indicator_preceded_failure"] for run in runs),
        "decision": "evaluate_mitigation" if len(runs) >= 3 and all(
            run["indicator_preceded_failure"] for run in runs) else "negative_result_no_mitigation",
        "runs": runs,
    }
    output_json = args.output_json or args.output_dir / "localizability_summary.json"
    output_markdown = args.output_markdown or args.output_dir / "localizability_summary.md"
    output_json.write_text(json.dumps(report, indent=2) + "\n")
    lines = [
        "# Registration localizability result", "",
        f"- Valid repeats: {len(runs)}",
        f"- Indicator preceded failure: {report['indicator_preceded_failure_repeat_count']}/{len(runs)}",
        f"- Decision: `{report['decision']}`", "",
        "| repeat | rows | first reject | max GT translation | max GT rotation | precursor |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for index, run in enumerate(runs, 1):
        lines.append(
            f"| {index} | {run['diagnostic_rows']} | {run['first_reject_stamp']} | "
            f"{run['translation_error_max_m']:.3f} | {run['rotation_error_max_deg']:.3f} | "
            f"{run['indicator_preceded_failure']} |")
    output_markdown.write_text("\n".join(lines) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
