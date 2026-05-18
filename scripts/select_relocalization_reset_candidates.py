#!/usr/bin/env python3

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple


PLAN_FIELDNAMES = [
    "attempt_id",
    "trigger_stamp_sec",
    "selected",
    "accepted",
    "rejection_reason",
    "policy_name",
    "selected_job_id",
    "selected_candidate_index",
    "selected_selection_source",
    "selected_selection_rank",
    "selected_score",
    "second_score",
    "registration_score_margin",
    "selected_converged",
    "selected_registration_gate_passed",
    "selected_registration_gate_reason",
    "selected_refinement_delta_m",
    "selected_refinement_delta_yaw_rad",
    "selected_initial_pose_x",
    "selected_initial_pose_y",
    "selected_initial_pose_z",
    "selected_initial_yaw_rad",
    "selected_final_pose_x",
    "selected_final_pose_y",
    "selected_final_pose_z",
    "selected_final_yaw_rad",
    "eligible_candidate_count",
    "scored_candidate_count",
    "generated_at",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select one registration-scored relocalization candidate per attempt for the future "
            "guarded reset path. This writes an artifact only; it never accepts or resets pose."
        )
    )
    parser.add_argument("--scores-csv", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    parser.add_argument("--policy-name", default="lowest_registration_score_with_gate")
    parser.add_argument("--max-score", type=float, default=6.0)
    parser.add_argument("--max-refinement-delta-m", type=float, default=2.0)
    parser.add_argument("--max-refinement-yaw-rad", type=float, default=0.7853981633974483)
    parser.add_argument(
        "--min-score-margin",
        type=float,
        default=0.0,
        help="Require second_score - selected_score to be at least this value. 0 disables it.",
    )
    parser.add_argument(
        "--require-registration-gate",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Require registration_gate_passed=true when the column is present.",
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def _read_csv(path: Path) -> Tuple[List[str], List[Dict[str, Any]]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _as_int(value: Any) -> Optional[int]:
    if value is None or str(value).strip() == "":
        return None
    try:
        return int(float(str(value)))
    except (TypeError, ValueError):
        return None


def _as_bool(value: Any) -> Optional[bool]:
    if value is None or str(value).strip() == "":
        return None
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "y"}:
        return True
    if normalized in {"0", "false", "no", "n"}:
        return False
    return None


def _fmt(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.9f}"
    return str(value)


def _row_sort_key(row: Dict[str, Any]) -> Tuple[float, int, int]:
    score = _as_float(row.get("score"))
    if score is None:
        score = float("inf")
    selection_rank = _as_int(row.get("selection_rank"))
    if selection_rank is None:
        selection_rank = 10**9
    candidate_index = _as_int(row.get("candidate_index"))
    if candidate_index is None:
        candidate_index = 10**9
    return (score, selection_rank, candidate_index)


def _is_scored(row: Dict[str, Any]) -> bool:
    return _as_float(row.get("score")) is not None


def _is_eligible(
    row: Dict[str, Any],
    max_score: float,
    max_refinement_delta_m: float,
    max_refinement_yaw_rad: float,
    require_registration_gate: bool,
) -> bool:
    if _as_bool(row.get("converged")) is not True:
        return False
    if require_registration_gate and _as_bool(row.get("registration_gate_passed")) is not True:
        return False
    score = _as_float(row.get("score"))
    if score is None or score > max_score:
        return False
    refinement_delta = _as_float(row.get("refinement_delta_m"))
    if refinement_delta is None or refinement_delta > max_refinement_delta_m:
        return False
    refinement_yaw = _as_float(row.get("refinement_delta_yaw_rad"))
    if refinement_yaw is None or refinement_yaw > max_refinement_yaw_rad:
        return False
    return True


def _blank_plan(
    attempt_id: str,
    trigger_stamp_sec: str,
    rejection_reason: str,
    policy_name: str,
    scored_count: int,
    eligible_count: int,
    generated_at: str,
) -> Dict[str, str]:
    row = {field: "" for field in PLAN_FIELDNAMES}
    row.update(
        {
            "attempt_id": attempt_id,
            "trigger_stamp_sec": trigger_stamp_sec,
            "selected": "false",
            "accepted": "false",
            "rejection_reason": rejection_reason,
            "policy_name": policy_name,
            "eligible_candidate_count": str(eligible_count),
            "scored_candidate_count": str(scored_count),
            "generated_at": generated_at,
        }
    )
    return row


def build_plan(
    rows: List[Dict[str, Any]],
    policy_name: str,
    max_score: float,
    max_refinement_delta_m: float,
    max_refinement_yaw_rad: float,
    min_score_margin: float,
    require_registration_gate: bool,
) -> List[Dict[str, str]]:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    by_attempt: Dict[str, List[Dict[str, Any]]] = {}
    for row in rows:
        by_attempt.setdefault(str(row.get("attempt_id", "")), []).append(row)

    plan_rows: List[Dict[str, str]] = []
    for attempt_id, attempt_rows in sorted(by_attempt.items()):
        trigger_stamp_sec = str(attempt_rows[0].get("trigger_stamp_sec", ""))
        scored_rows = [row for row in attempt_rows if _is_scored(row)]
        eligible_rows = [
            row
            for row in scored_rows
            if _is_eligible(
                row,
                max_score=max_score,
                max_refinement_delta_m=max_refinement_delta_m,
                max_refinement_yaw_rad=max_refinement_yaw_rad,
                require_registration_gate=require_registration_gate,
            )
        ]
        eligible_rows.sort(key=_row_sort_key)
        scored_rows.sort(key=_row_sort_key)

        if not scored_rows:
            plan_rows.append(
                _blank_plan(
                    attempt_id,
                    trigger_stamp_sec,
                    "no_scored_candidates",
                    policy_name,
                    scored_count=0,
                    eligible_count=0,
                    generated_at=generated_at,
                )
            )
            continue
        if not eligible_rows:
            plan_rows.append(
                _blank_plan(
                    attempt_id,
                    trigger_stamp_sec,
                    "no_candidate_passed_registration_policy",
                    policy_name,
                    scored_count=len(scored_rows),
                    eligible_count=0,
                    generated_at=generated_at,
                )
            )
            continue

        selected = eligible_rows[0]
        selected_score = _as_float(selected.get("score"))
        second_score = None
        for row in scored_rows:
            if row is selected:
                continue
            candidate_score = _as_float(row.get("score"))
            if candidate_score is not None:
                second_score = candidate_score
                break
        margin = None
        if selected_score is not None and second_score is not None:
            margin = second_score - selected_score
        if min_score_margin > 0.0 and (margin is None or margin < min_score_margin):
            plan_rows.append(
                _blank_plan(
                    attempt_id,
                    trigger_stamp_sec,
                    "registration_score_margin_too_small",
                    policy_name,
                    scored_count=len(scored_rows),
                    eligible_count=len(eligible_rows),
                    generated_at=generated_at,
                )
            )
            continue

        plan_rows.append(
            {
                "attempt_id": attempt_id,
                "trigger_stamp_sec": trigger_stamp_sec,
                "selected": "true",
                "accepted": "false",
                "rejection_reason": "reset_execution_not_implemented",
                "policy_name": policy_name,
                "selected_job_id": str(selected.get("job_id", "")),
                "selected_candidate_index": str(selected.get("candidate_index", "")),
                "selected_selection_source": str(selected.get("selection_source", "")),
                "selected_selection_rank": str(selected.get("selection_rank", "")),
                "selected_score": _fmt(selected_score),
                "second_score": _fmt(second_score),
                "registration_score_margin": _fmt(margin),
                "selected_converged": str(selected.get("converged", "")),
                "selected_registration_gate_passed": str(
                    selected.get("registration_gate_passed", "")
                ),
                "selected_registration_gate_reason": str(
                    selected.get("registration_gate_reason", "")
                ),
                "selected_refinement_delta_m": str(selected.get("refinement_delta_m", "")),
                "selected_refinement_delta_yaw_rad": str(
                    selected.get("refinement_delta_yaw_rad", "")
                ),
                "selected_initial_pose_x": str(selected.get("initial_pose_x", "")),
                "selected_initial_pose_y": str(selected.get("initial_pose_y", "")),
                "selected_initial_pose_z": str(selected.get("initial_pose_z", "")),
                "selected_initial_yaw_rad": str(selected.get("initial_yaw_rad", "")),
                "selected_final_pose_x": str(selected.get("final_pose_x", "")),
                "selected_final_pose_y": str(selected.get("final_pose_y", "")),
                "selected_final_pose_z": str(selected.get("final_pose_z", "")),
                "selected_final_yaw_rad": str(selected.get("final_yaw_rad", "")),
                "eligible_candidate_count": str(len(eligible_rows)),
                "scored_candidate_count": str(len(scored_rows)),
                "generated_at": generated_at,
            }
        )
    return plan_rows


def write_csv(path: Path, rows: List[Dict[str, str]], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=PLAN_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def build_summary(plan_rows: List[Dict[str, str]], scores_csv: Path, output_csv: Path) -> Dict[str, Any]:
    selected_rows = [row for row in plan_rows if _as_bool(row.get("selected")) is True]
    selected_scores = [
        score
        for score in (_as_float(row.get("selected_score")) for row in selected_rows)
        if score is not None
    ]
    return {
        "scores_csv": str(scores_csv),
        "output_csv": str(output_csv),
        "attempt_count": len(plan_rows),
        "selected_count": len(selected_rows),
        "accepted_count": 0,
        "selected_rate_percent": (100.0 * len(selected_rows) / len(plan_rows))
        if plan_rows
        else 0.0,
        "rejection_reason_counts": _counts(row.get("rejection_reason", "") for row in plan_rows),
        "selected_score_min": min(selected_scores) if selected_scores else None,
        "selected_score_median": _median(selected_scores),
        "selected_score_max": max(selected_scores) if selected_scores else None,
        "notes": [
            "selected=true means a candidate passed the artifact policy",
            "accepted is always false; this helper never resets pose",
            "oracle_rank is not required or used by the default policy",
        ],
    }


def _counts(values: Any) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _median(values: List[float]) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def write_summary_json(path: Path, summary: Dict[str, Any], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output JSON already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_summary_md(path: Path, summary: Dict[str, Any], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output Markdown already exists: {path}")
    lines = [
        "# Relocalization Reset Candidate Plan",
        "",
        f"- scores CSV: `{summary['scores_csv']}`",
        f"- output CSV: `{summary['output_csv']}`",
        f"- attempts: `{summary['attempt_count']}`",
        f"- selected: `{summary['selected_count']}`",
        f"- accepted: `{summary['accepted_count']}`",
        f"- selected rate: `{summary['selected_rate_percent']:.3f}%`",
        f"- selected score median: `{summary['selected_score_median']}`",
        "",
        "## Rejection Reasons",
        "",
    ]
    for reason, count in sorted(summary["rejection_reason_counts"].items()):
        lines.append(f"- `{reason}`: `{count}`")
    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- `selected=true` means a candidate passed the artifact policy.",
            "- `accepted=false` is intentional; this helper never resets pose.",
            "- The default policy does not use `oracle_rank`.",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    scores_csv = Path(args.scores_csv).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    if not scores_csv.exists():
        raise FileNotFoundError(f"scores CSV does not exist: {scores_csv}")

    _, rows = _read_csv(scores_csv)
    plan_rows = build_plan(
        rows=rows,
        policy_name=args.policy_name,
        max_score=args.max_score,
        max_refinement_delta_m=args.max_refinement_delta_m,
        max_refinement_yaw_rad=args.max_refinement_yaw_rad,
        min_score_margin=args.min_score_margin,
        require_registration_gate=args.require_registration_gate,
    )
    write_csv(output_csv, plan_rows, overwrite=args.overwrite)

    summary = build_summary(plan_rows, scores_csv=scores_csv, output_csv=output_csv)
    if args.output_json:
        write_summary_json(
            Path(args.output_json).expanduser().resolve(), summary, overwrite=args.overwrite
        )
    if args.output_md:
        write_summary_md(
            Path(args.output_md).expanduser().resolve(), summary, overwrite=args.overwrite
        )
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
