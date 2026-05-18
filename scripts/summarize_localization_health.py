#!/usr/bin/env python3

import argparse
import csv
import json
import math
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional


FAILURE_MESSAGES = {
    "local_map_crop_too_small",
    "registration_not_converged",
    "filtered_scan_empty",
    "scan_missing_xyz_field",
}

FAILURE_STATES = {
    "degraded",
    "recovering",
    "reinitialization_requested",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize localization health and reinitialization windows from "
            "benchmark_diagnostic_recorder alignment_status.csv artifacts."
        )
    )
    parser.add_argument(
        "--alignment-csv",
        required=True,
        help="Path to alignment_status.csv produced by benchmark_diagnostic_recorder.",
    )
    parser.add_argument(
        "--trajectory-eval-json",
        default="",
        help="Optional trajectory_eval.json to include overall RMSE context.",
    )
    parser.add_argument("--output-json", default="", help="Optional output JSON path.")
    parser.add_argument("--output-md", default="", help="Optional output Markdown path.")
    parser.add_argument(
        "--stable-ok-rows",
        type=int,
        default=5,
        help="Minimum consecutive ok rows after a request window to count as recovered.",
    )
    parser.add_argument(
        "--recovery-window-sec",
        type=float,
        default=10.0,
        help="Maximum time after a request window start to search for stable recovery.",
    )
    parser.add_argument(
        "--false-recovery-rmse-threshold-m",
        type=float,
        default=5.0,
        help=(
            "Overall translation RMSE threshold used only to flag recovered runs as "
            "reference-risky when trajectory_eval_json is available."
        ),
    )
    return parser.parse_args()


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() == "true"


def _as_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(float(str(value)))
    except (TypeError, ValueError):
        return None


def _percent(numerator: int, denominator: int) -> Optional[float]:
    if denominator <= 0:
        return None
    return 100.0 * float(numerator) / float(denominator)


def load_alignment_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            values = json.loads(record["values_json"])
            message = str(record.get("message", ""))
            requested = _as_bool(values.get("reinitialization_requested"))
            recovery_state = str(values.get("recovery_state", ""))
            failure_like = (
                message in FAILURE_MESSAGES
                or message.startswith("fitness_score_over_")
                or requested
                or recovery_state in FAILURE_STATES
            )
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "level": _as_int(record.get("level")),
                    "message": message,
                    "registration_method": values.get("registration_method"),
                    "has_converged": _as_bool(values.get("has_converged")),
                    "fitness_score": _as_float(values.get("fitness_score")),
                    "alignment_time_sec": _as_float(values.get("alignment_time_sec")),
                    "accepted_gap_sec": _as_float(values.get("accepted_gap_sec")),
                    "consecutive_rejected_updates": _as_int(
                        values.get("consecutive_rejected_updates")
                    ),
                    "recovery_state": recovery_state,
                    "recovery_action": values.get("recovery_action"),
                    "reinitialization_requested": requested,
                    "reinitialization_request_reason": values.get(
                        "reinitialization_request_reason"
                    ),
                    "reinitialization_request_score": _as_float(
                        values.get("reinitialization_request_score")
                    ),
                    "failure_like": failure_like,
                }
            )
    return rows


def _segment_rows(rows: List[Dict[str, Any]], key: str) -> List[Dict[str, Any]]:
    segments: List[Dict[str, Any]] = []
    index = 0
    while index < len(rows):
        if not rows[index][key]:
            index += 1
            continue
        start = index
        while index < len(rows) and rows[index][key]:
            index += 1
        end = index - 1
        segments.append(
            {
                "start_index": start,
                "end_index": end,
                "start_stamp_sec": rows[start]["stamp_sec"],
                "end_stamp_sec": rows[end]["stamp_sec"],
                "duration_sec": max(0.0, rows[end]["stamp_sec"] - rows[start]["stamp_sec"]),
                "row_count": end - start + 1,
                "first_message": rows[start]["message"],
                "last_message": rows[end]["message"],
            }
        )
    return segments


def _max_optional(values: Iterable[Optional[float]]) -> Optional[float]:
    finite_values = [value for value in values if value is not None]
    return max(finite_values) if finite_values else None


def _stable_ok_rows_after(
    rows: List[Dict[str, Any]],
    start_index: int,
    deadline_sec: float,
) -> Dict[str, Any]:
    first_ok_index: Optional[int] = None
    for index in range(start_index, len(rows)):
        if rows[index]["stamp_sec"] > deadline_sec:
            break
        if rows[index]["message"] == "ok":
            first_ok_index = index
            break

    if first_ok_index is None:
        return {
            "first_ok_index": None,
            "first_ok_delay_sec": None,
            "stable_ok_rows": 0,
        }

    stable_ok_rows = 0
    for index in range(first_ok_index, len(rows)):
        if rows[index]["message"] != "ok":
            break
        stable_ok_rows += 1

    return {
        "first_ok_index": first_ok_index,
        "first_ok_delay_sec": rows[first_ok_index]["stamp_sec"] - rows[start_index]["stamp_sec"],
        "stable_ok_rows": stable_ok_rows,
    }


def summarize_request_windows(
    rows: List[Dict[str, Any]],
    stable_ok_rows_required: int,
    recovery_window_sec: float,
) -> List[Dict[str, Any]]:
    request_segments = _segment_rows(rows, "reinitialization_requested")
    windows: List[Dict[str, Any]] = []
    for segment in request_segments:
        start_index = int(segment["start_index"])
        end_index = int(segment["end_index"])
        deadline_sec = rows[start_index]["stamp_sec"] + recovery_window_sec
        stable = _stable_ok_rows_after(rows, end_index + 1, deadline_sec)
        reason_counts = Counter(
            str(rows[index]["reinitialization_request_reason"])
            for index in range(start_index, end_index + 1)
            if rows[index]["reinitialization_request_reason"] is not None
        )
        recovered = stable["stable_ok_rows"] >= stable_ok_rows_required
        windows.append(
            {
                "start_delta_sec": rows[start_index]["stamp_sec"] - rows[0]["stamp_sec"],
                "duration_sec": segment["duration_sec"],
                "row_count": segment["row_count"],
                "reason_counts": dict(reason_counts),
                "first_ok_after_request_sec": stable["first_ok_delay_sec"],
                "stable_ok_rows_after_request": stable["stable_ok_rows"],
                "recovered": recovered,
            }
        )
    return windows


def summarize_alignment(
    rows: List[Dict[str, Any]],
    stable_ok_rows_required: int,
    recovery_window_sec: float,
) -> Dict[str, Any]:
    if not rows:
        return {
            "row_count": 0,
            "ok_rows": 0,
            "failure_like_rows": 0,
            "reinitialization_requested_rows": 0,
            "reinitialization_request_transition_count": 0,
            "reinitialization_windows": [],
            "lost_windows": [],
            "message_counts": {},
            "recovery_state_counts": {},
            "recovery_action_counts": {},
            "max_accepted_gap_sec": None,
            "max_consecutive_rejected_updates": None,
            "max_fitness_score": None,
            "max_alignment_time_sec": None,
        }

    request_transitions = 0
    previous_requested = False
    for row in rows:
        requested = bool(row["reinitialization_requested"])
        if requested and not previous_requested:
            request_transitions += 1
        previous_requested = requested

    request_windows = summarize_request_windows(
        rows,
        stable_ok_rows_required=stable_ok_rows_required,
        recovery_window_sec=recovery_window_sec,
    )
    recovered_count = sum(1 for window in request_windows if window["recovered"])

    first_stamp_sec = rows[0]["stamp_sec"]
    plausible_accepted_gaps = [
        row["accepted_gap_sec"]
        for row in rows
        if row["accepted_gap_sec"] is not None
        and row["accepted_gap_sec"] <= (row["stamp_sec"] - first_stamp_sec + 1.0)
    ]

    return {
        "row_count": len(rows),
        "ok_rows": sum(1 for row in rows if row["message"] == "ok"),
        "ok_rate_percent": _percent(sum(1 for row in rows if row["message"] == "ok"), len(rows)),
        "failure_like_rows": sum(1 for row in rows if row["failure_like"]),
        "failure_like_rate_percent": _percent(
            sum(1 for row in rows if row["failure_like"]),
            len(rows),
        ),
        "reinitialization_requested_rows": sum(
            1 for row in rows if row["reinitialization_requested"]
        ),
        "reinitialization_request_transition_count": request_transitions,
        "reinitialization_recovered_count": recovered_count,
        "reinitialization_unrecovered_count": len(request_windows) - recovered_count,
        "reinitialization_windows": request_windows,
        "lost_windows": _segment_rows(rows, "failure_like"),
        "message_counts": dict(Counter(str(row["message"]) for row in rows)),
        "recovery_state_counts": dict(Counter(str(row["recovery_state"]) for row in rows)),
        "recovery_action_counts": dict(Counter(str(row["recovery_action"]) for row in rows)),
        "request_reason_counts": dict(
            Counter(
                str(row["reinitialization_request_reason"])
                for row in rows
                if row["reinitialization_requested"]
            )
        ),
        "max_accepted_gap_sec": _max_optional(plausible_accepted_gaps),
        "max_consecutive_rejected_updates": max(
            (
                row["consecutive_rejected_updates"]
                for row in rows
                if row["consecutive_rejected_updates"] is not None
            ),
            default=None,
        ),
        "max_fitness_score": _max_optional(row["fitness_score"] for row in rows),
        "max_alignment_time_sec": _max_optional(row["alignment_time_sec"] for row in rows),
    }


def load_trajectory_eval(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None:
        return None
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def build_summary(
    alignment_csv: Path,
    trajectory_eval_json: Optional[Path],
    stable_ok_rows_required: int,
    recovery_window_sec: float,
    false_recovery_rmse_threshold_m: float,
) -> Dict[str, Any]:
    rows = load_alignment_rows(alignment_csv)
    alignment = summarize_alignment(
        rows,
        stable_ok_rows_required=stable_ok_rows_required,
        recovery_window_sec=recovery_window_sec,
    )
    trajectory_eval = load_trajectory_eval(trajectory_eval_json)

    reference_risk = "not_evaluated"
    if trajectory_eval is not None and alignment["reinitialization_recovered_count"] > 0:
        translation_rmse_m = trajectory_eval.get("translation_rmse_m")
        if isinstance(translation_rmse_m, (int, float)):
            reference_risk = (
                "high_overall_rmse_after_recovery"
                if translation_rmse_m > false_recovery_rmse_threshold_m
                else "overall_rmse_within_threshold"
            )

    return {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "alignment_csv": str(alignment_csv),
        "trajectory_eval_json": None if trajectory_eval_json is None else str(trajectory_eval_json),
        "stable_ok_rows_required": stable_ok_rows_required,
        "recovery_window_sec": recovery_window_sec,
        "false_recovery_rmse_threshold_m": false_recovery_rmse_threshold_m,
        "alignment": alignment,
        "trajectory_eval": trajectory_eval,
        "reference_risk": reference_risk,
        "notes": [
            "false recovery cannot be proven from diagnostics alone",
            "event-local false recovery needs per-event reference evaluation in a future relocalization benchmark",
        ],
    }


def _fmt(value: Any) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def build_markdown(summary: Dict[str, Any]) -> str:
    alignment = summary["alignment"]
    trajectory_eval = summary.get("trajectory_eval")
    lines = [
        "# Localization Health Summary",
        "",
        f"Generated at: {summary['generated_at']}",
        "",
        f"- alignment_csv: `{summary['alignment_csv']}`",
        f"- trajectory_eval_json: `{summary['trajectory_eval_json']}`",
        f"- reference_risk: `{summary['reference_risk']}`",
        "",
        "## Alignment",
        "",
        f"- rows: `{alignment['row_count']}`",
        f"- ok rows: `{alignment['ok_rows']}` (`{_fmt(alignment.get('ok_rate_percent'))}%`)",
        (
            f"- failure-like rows: `{alignment['failure_like_rows']}` "
            f"(`{_fmt(alignment.get('failure_like_rate_percent'))}%`)"
        ),
        f"- reinitialization requested rows: `{alignment['reinitialization_requested_rows']}`",
        (
            "- reinitialization transitions: "
            f"`{alignment['reinitialization_request_transition_count']}`"
        ),
        f"- recovered request windows: `{alignment['reinitialization_recovered_count']}`",
        f"- unrecovered request windows: `{alignment['reinitialization_unrecovered_count']}`",
        f"- max accepted gap sec: `{_fmt(alignment['max_accepted_gap_sec'])}`",
        (
            "- max consecutive rejected updates: "
            f"`{_fmt(alignment['max_consecutive_rejected_updates'])}`"
        ),
        f"- max fitness score: `{_fmt(alignment['max_fitness_score'])}`",
        f"- max alignment time sec: `{_fmt(alignment['max_alignment_time_sec'])}`",
        "",
        "## Request Windows",
        "",
    ]

    if alignment["reinitialization_windows"]:
        lines.extend(
            [
                "| Start [s] | Rows | Duration [s] | Recovered | First OK [s] | Stable OK rows | Reasons |",
                "| ---: | ---: | ---: | --- | ---: | ---: | --- |",
            ]
        )
        for window in alignment["reinitialization_windows"]:
            lines.append(
                "| "
                f"{_fmt(window['start_delta_sec'])} | "
                f"{window['row_count']} | "
                f"{_fmt(window['duration_sec'])} | "
                f"{window['recovered']} | "
                f"{_fmt(window['first_ok_after_request_sec'])} | "
                f"{window['stable_ok_rows_after_request']} | "
                f"`{json.dumps(window['reason_counts'], sort_keys=True)}` |"
            )
    else:
        lines.append("- none")

    lines.extend(
        [
            "",
            "## Counts",
            "",
            f"- message_counts: `{json.dumps(alignment['message_counts'], sort_keys=True)}`",
            (
                "- recovery_state_counts: "
                f"`{json.dumps(alignment['recovery_state_counts'], sort_keys=True)}`"
            ),
            (
                "- recovery_action_counts: "
                f"`{json.dumps(alignment['recovery_action_counts'], sort_keys=True)}`"
            ),
            (
                "- request_reason_counts: "
                f"`{json.dumps(alignment['request_reason_counts'], sort_keys=True)}`"
            ),
        ]
    )

    if trajectory_eval is not None:
        lines.extend(
            [
                "",
                "## Trajectory Eval",
                "",
                f"- matched_sample_count: `{trajectory_eval.get('matched_sample_count')}`",
                f"- translation_rmse_m: `{_fmt(trajectory_eval.get('translation_rmse_m'))}`",
                f"- rotation_rmse_deg: `{_fmt(trajectory_eval.get('rotation_rmse_deg'))}`",
            ]
        )

    lines.extend(
        [
            "",
            "## Caveat",
            "",
            "- Diagnostics can show request/recovery timing, but event-local false recovery needs reference evaluation around each reset.",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    alignment_csv = Path(args.alignment_csv).expanduser().resolve()
    if not alignment_csv.exists():
        raise FileNotFoundError(f"alignment CSV does not exist: {alignment_csv}")

    trajectory_eval_json: Optional[Path] = None
    if args.trajectory_eval_json:
        trajectory_eval_json = Path(args.trajectory_eval_json).expanduser().resolve()
        if not trajectory_eval_json.exists():
            raise FileNotFoundError(
                f"trajectory eval JSON does not exist: {trajectory_eval_json}"
            )

    summary = build_summary(
        alignment_csv=alignment_csv,
        trajectory_eval_json=trajectory_eval_json,
        stable_ok_rows_required=args.stable_ok_rows,
        recovery_window_sec=args.recovery_window_sec,
        false_recovery_rmse_threshold_m=args.false_recovery_rmse_threshold_m,
    )

    if args.output_json:
        output_json = Path(args.output_json).expanduser().resolve()
        output_json.parent.mkdir(parents=True, exist_ok=True)
        output_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.output_md:
        output_md = Path(args.output_md).expanduser().resolve()
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_md.write_text(build_markdown(summary) + "\n", encoding="utf-8")

    if not args.output_json and not args.output_md:
        print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
