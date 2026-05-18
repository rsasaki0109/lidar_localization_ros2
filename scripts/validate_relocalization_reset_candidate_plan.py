#!/usr/bin/env python3

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple


REQUIRED_PLAN_COLUMNS = [
    "attempt_id",
    "selected",
    "accepted",
    "rejection_reason",
    "selected_job_id",
    "selected_candidate_index",
    "selected_score",
    "selected_converged",
    "selected_registration_gate_passed",
    "selected_final_pose_x",
    "selected_final_pose_y",
    "selected_final_pose_z",
    "selected_final_yaw_rad",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a relocalization reset candidate plan artifact. This is a contract "
            "checker only; it does not execute or accept resets."
        )
    )
    parser.add_argument("--plan-csv", required=True)
    parser.add_argument(
        "--scores-csv",
        default="",
        help="Optional registration score CSV to cross-check selected job/candidate rows.",
    )
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    parser.add_argument("--max-score", type=float, default=6.0)
    parser.add_argument("--max-refinement-delta-m", type=float, default=2.0)
    parser.add_argument("--max-refinement-yaw-rad", type=float, default=0.7853981633974483)
    parser.add_argument("--min-selected-count", type=int, default=0)
    parser.add_argument("--max-selected-count", type=int, default=-1)
    parser.add_argument("--allow-accepted", action="store_true")
    parser.add_argument("--allow-oracle-selection", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def _read_csv(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def _as_bool(value: Any) -> Optional[bool]:
    if value is None or str(value).strip() == "":
        return None
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "y"}:
        return True
    if normalized in {"0", "false", "no", "n"}:
        return False
    return None


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _selected_key(row: Dict[str, str]) -> Tuple[str, str, str]:
    return (
        str(row.get("attempt_id", "")),
        str(row.get("selected_job_id", "")),
        str(row.get("selected_candidate_index", "")),
    )


def _score_key(row: Dict[str, str]) -> Tuple[str, str, str]:
    return (
        str(row.get("attempt_id", "")),
        str(row.get("job_id", "")),
        str(row.get("candidate_index", "")),
    )


def _counts(values: Iterable[str]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def validate(
    plan_header: List[str],
    plan_rows: List[Dict[str, str]],
    scores_rows: List[Dict[str, str]],
    args: argparse.Namespace,
) -> Dict[str, Any]:
    failures: List[str] = []
    warnings: List[str] = []

    missing = [column for column in REQUIRED_PLAN_COLUMNS if column not in plan_header]
    for column in missing:
        failures.append(f"missing required plan column: {column}")

    if not plan_rows:
        failures.append("plan CSV has no rows")

    attempts = [str(row.get("attempt_id", "")) for row in plan_rows]
    for attempt_id, count in _counts(attempts).items():
        if not attempt_id:
            failures.append("plan row has empty attempt_id")
        if count > 1:
            failures.append(f"attempt_id appears more than once: {attempt_id}")

    selected_rows = [row for row in plan_rows if _as_bool(row.get("selected")) is True]
    accepted_rows = [row for row in plan_rows if _as_bool(row.get("accepted")) is True]
    if accepted_rows and not args.allow_accepted:
        failures.append(
            f"accepted=true rows are not allowed in artifact-only mode: {len(accepted_rows)}"
        )

    if len(selected_rows) < args.min_selected_count:
        failures.append(
            f"selected row count {len(selected_rows)} is below minimum {args.min_selected_count}"
        )
    if args.max_selected_count >= 0 and len(selected_rows) > args.max_selected_count:
        failures.append(
            f"selected row count {len(selected_rows)} is above maximum {args.max_selected_count}"
        )

    scores_by_key = {_score_key(row): row for row in scores_rows}
    for row in selected_rows:
        attempt_id = str(row.get("attempt_id", ""))
        prefix = f"attempt {attempt_id}: "
        if not str(row.get("selected_job_id", "")):
            failures.append(prefix + "selected_job_id is empty")
        if not str(row.get("selected_candidate_index", "")):
            failures.append(prefix + "selected_candidate_index is empty")
        if (
            str(row.get("selected_selection_source", "")) == "oracle_rank"
            and not args.allow_oracle_selection
        ):
            failures.append(prefix + "oracle_rank selection is not allowed by default")
        if _as_bool(row.get("selected_converged")) is not True:
            failures.append(prefix + "selected_converged is not true")
        if _as_bool(row.get("selected_registration_gate_passed")) is not True:
            failures.append(prefix + "selected_registration_gate_passed is not true")

        selected_score = _as_float(row.get("selected_score"))
        if selected_score is None:
            failures.append(prefix + "selected_score is missing or non-finite")
        elif selected_score > args.max_score:
            failures.append(
                prefix + f"selected_score {selected_score:.6f} exceeds {args.max_score:.6f}"
            )

        refinement_delta = _as_float(row.get("selected_refinement_delta_m"))
        if refinement_delta is None:
            failures.append(prefix + "selected_refinement_delta_m is missing or non-finite")
        elif refinement_delta > args.max_refinement_delta_m:
            failures.append(
                prefix
                + f"selected_refinement_delta_m {refinement_delta:.6f} exceeds "
                + f"{args.max_refinement_delta_m:.6f}"
            )

        refinement_yaw = _as_float(row.get("selected_refinement_delta_yaw_rad"))
        if refinement_yaw is None:
            failures.append(
                prefix + "selected_refinement_delta_yaw_rad is missing or non-finite"
            )
        elif refinement_yaw > args.max_refinement_yaw_rad:
            failures.append(
                prefix
                + f"selected_refinement_delta_yaw_rad {refinement_yaw:.6f} exceeds "
                + f"{args.max_refinement_yaw_rad:.6f}"
            )

        for field in [
            "selected_final_pose_x",
            "selected_final_pose_y",
            "selected_final_pose_z",
            "selected_final_yaw_rad",
        ]:
            if _as_float(row.get(field)) is None:
                failures.append(prefix + f"{field} is missing or non-finite")

        if scores_rows:
            key = _selected_key(row)
            score_row = scores_by_key.get(key)
            if score_row is None:
                failures.append(prefix + "selected job/candidate does not exist in scores CSV")
            else:
                if _as_bool(score_row.get("registration_gate_passed")) is not True:
                    failures.append(prefix + "scores CSV row did not pass registration gate")
                score_value = _as_float(score_row.get("score"))
                if selected_score is not None and score_value is not None:
                    if abs(score_value - selected_score) > 1e-5:
                        failures.append(prefix + "selected_score differs from scores CSV")
                elif selected_score is not None or score_value is not None:
                    failures.append(prefix + "score presence differs from scores CSV")

    for row in plan_rows:
        if _as_bool(row.get("selected")) is not True and str(row.get("rejection_reason", "")) == "":
            warnings.append(
                f"attempt {row.get('attempt_id', '')}: unselected row has empty rejection_reason"
            )

    return {
        "validation_passed": not failures,
        "failure_count": len(failures),
        "warning_count": len(warnings),
        "failures": failures,
        "warnings": warnings,
        "attempt_count": len(plan_rows),
        "selected_count": len(selected_rows),
        "accepted_count": len(accepted_rows),
        "rejection_reason_counts": _counts(
            str(row.get("rejection_reason", "")) for row in plan_rows
        ),
        "selected_selection_source_counts": _counts(
            str(row.get("selected_selection_source", "")) for row in selected_rows
        ),
    }


def write_json(path: Path, data: Dict[str, Any], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output JSON already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_md(path: Path, data: Dict[str, Any], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output Markdown already exists: {path}")
    lines = [
        "# Relocalization Reset Candidate Plan Validation",
        "",
        f"- validation passed: `{str(data['validation_passed']).lower()}`",
        f"- failures: `{data['failure_count']}`",
        f"- warnings: `{data['warning_count']}`",
        f"- attempts: `{data['attempt_count']}`",
        f"- selected: `{data['selected_count']}`",
        f"- accepted: `{data['accepted_count']}`",
        f"- rejection reasons: `{json.dumps(data['rejection_reason_counts'], sort_keys=True)}`",
        f"- selected sources: `{json.dumps(data['selected_selection_source_counts'], sort_keys=True)}`",
        "",
        "## Failures",
        "",
    ]
    if data["failures"]:
        lines.extend(f"- {failure}" for failure in data["failures"])
    else:
        lines.append("- none")
    lines.extend(["", "## Warnings", ""])
    if data["warnings"]:
        lines.extend(f"- {warning}" for warning in data["warnings"])
    else:
        lines.append("- none")
    lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    plan_csv = Path(args.plan_csv).expanduser().resolve()
    scores_csv = Path(args.scores_csv).expanduser().resolve() if args.scores_csv else None
    if not plan_csv.exists():
        raise FileNotFoundError(f"plan CSV does not exist: {plan_csv}")
    if scores_csv is not None and not scores_csv.exists():
        raise FileNotFoundError(f"scores CSV does not exist: {scores_csv}")

    plan_header, plan_rows = _read_csv(plan_csv)
    _, scores_rows = _read_csv(scores_csv) if scores_csv is not None else ([], [])
    result = validate(plan_header, plan_rows, scores_rows, args)
    result.update(
        {
            "plan_csv": str(plan_csv),
            "scores_csv": "" if scores_csv is None else str(scores_csv),
            "notes": [
                "this validator does not execute resets",
                "accepted=true is rejected by default for artifact-only plans",
                "oracle_rank selection is rejected by default",
            ],
        }
    )
    if args.output_json:
        write_json(Path(args.output_json).expanduser().resolve(), result, args.overwrite)
    if args.output_md:
        write_md(Path(args.output_md).expanduser().resolve(), result, args.overwrite)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["validation_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
