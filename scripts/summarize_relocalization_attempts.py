#!/usr/bin/env python3

import argparse
import csv
import json
import math
from collections import Counter
from datetime import datetime
from pathlib import Path
from statistics import median
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize experimental relocalization attempt artifacts and match them "
            "against reinitialization request windows from alignment_status.csv."
        )
    )
    parser.add_argument(
        "--alignment-csv",
        required=True,
        help="Path to alignment_status.csv produced by benchmark_diagnostic_recorder.",
    )
    parser.add_argument(
        "--attempts-csv",
        default="",
        help=(
            "Optional relocalization_attempts.csv. When omitted or missing with "
            "--allow-missing-attempts, the summary records zero attempts."
        ),
    )
    parser.add_argument(
        "--trajectory-eval-json",
        default="",
        help="Optional trajectory_eval.json for run-level reference-risk context.",
    )
    parser.add_argument("--output-json", default="", help="Optional output JSON path.")
    parser.add_argument("--output-md", default="", help="Optional output Markdown path.")
    parser.add_argument(
        "--allow-missing-attempts",
        action="store_true",
        help="Treat a missing attempts CSV as an empty attempt set.",
    )
    parser.add_argument(
        "--request-match-window-sec",
        type=float,
        default=10.0,
        help="Seconds after a request window ends in which an attempt still matches it.",
    )
    parser.add_argument(
        "--post-reset-stable-ok-rows",
        type=int,
        default=5,
        help="Stable accepted rows required for an accepted attempt to count as recovered.",
    )
    parser.add_argument(
        "--false-recovery-rmse-threshold-m",
        type=float,
        default=5.0,
        help=(
            "Overall translation RMSE threshold used only to mark accepted attempts "
            "as reference-risky when event-local false recovery is unknown."
        ),
    )
    return parser.parse_args()


def _as_bool(value: Any) -> Optional[bool]:
    if value is None:
        return None
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "y"}:
        return True
    if normalized in {"false", "0", "no", "n"}:
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


def _as_int(value: Any) -> Optional[int]:
    if value is None or str(value).strip() == "":
        return None
    try:
        return int(float(str(value)))
    except (TypeError, ValueError):
        return None


def _percent(numerator: int, denominator: int) -> Optional[float]:
    if denominator <= 0:
        return None
    return 100.0 * float(numerator) / float(denominator)


def _first_present(row: Dict[str, Any], keys: Iterable[str]) -> Any:
    for key in keys:
        if key in row and str(row[key]).strip() != "":
            return row[key]
    return None


def _fmt(value: Any) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def _stats(values: Iterable[Optional[float]]) -> Dict[str, Optional[float]]:
    finite_values = [value for value in values if value is not None]
    if not finite_values:
        return {"min": None, "median": None, "max": None}
    return {
        "min": min(finite_values),
        "median": float(median(finite_values)),
        "max": max(finite_values),
    }


def load_alignment_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            values = json.loads(record["values_json"])
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "message": str(record.get("message", "")),
                    "reinitialization_requested": _as_bool(
                        values.get("reinitialization_requested")
                    )
                    is True,
                    "reinitialization_request_reason": values.get(
                        "reinitialization_request_reason"
                    ),
                }
            )
    return rows


def request_windows(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    windows: List[Dict[str, Any]] = []
    index = 0
    while index < len(rows):
        if not rows[index]["reinitialization_requested"]:
            index += 1
            continue
        start = index
        while index < len(rows) and rows[index]["reinitialization_requested"]:
            index += 1
        end = index - 1
        reasons = Counter(
            str(rows[row_index]["reinitialization_request_reason"])
            for row_index in range(start, end + 1)
            if rows[row_index]["reinitialization_request_reason"] is not None
        )
        windows.append(
            {
                "start_index": start,
                "end_index": end,
                "start_stamp_sec": rows[start]["stamp_sec"],
                "end_stamp_sec": rows[end]["stamp_sec"],
                "start_delta_sec": rows[start]["stamp_sec"] - rows[0]["stamp_sec"],
                "duration_sec": max(0.0, rows[end]["stamp_sec"] - rows[start]["stamp_sec"]),
                "row_count": end - start + 1,
                "reason_counts": dict(reasons),
            }
        )
    return windows


def normalize_attempt(row: Dict[str, Any], row_number: int) -> Dict[str, Any]:
    trigger_stamp = _as_float(
        _first_present(row, ["trigger_stamp_sec", "trigger_time_sec", "stamp_sec", "start_stamp_sec"])
    )
    start_stamp = _as_float(_first_present(row, ["start_stamp_sec", "start_time_sec"]))
    end_stamp = _as_float(_first_present(row, ["end_stamp_sec", "end_time_sec"]))
    runtime = _as_float(_first_present(row, ["runtime_sec", "duration_sec", "elapsed_sec"]))
    if runtime is None and start_stamp is not None and end_stamp is not None:
        runtime = max(0.0, end_stamp - start_stamp)

    accepted = _as_bool(_first_present(row, ["accepted", "reset_accepted"]))
    false_recovery = _as_bool(_first_present(row, ["false_recovery", "event_false_recovery"]))
    post_reset_ok_rows = _as_int(
        _first_present(row, ["post_reset_ok_rows", "stable_ok_rows_after_reset"])
    )
    recovered = accepted is True and (
        post_reset_ok_rows is not None and post_reset_ok_rows > 0
    )

    return {
        "attempt_id": str(_first_present(row, ["attempt_id", "id"]) or row_number),
        "trigger_stamp_sec": trigger_stamp,
        "start_stamp_sec": start_stamp,
        "end_stamp_sec": end_stamp,
        "source": str(_first_present(row, ["source", "candidate_source"]) or "unknown"),
        "mode": str(_first_present(row, ["mode", "roi_mode"]) or "unknown"),
        "roi_type": str(_first_present(row, ["roi_type", "roi"]) or "unknown"),
        "candidate_count": _as_int(_first_present(row, ["candidate_count", "candidates"])),
        "accepted": accepted,
        "accepted_candidate_rank": _as_int(
            _first_present(row, ["accepted_candidate_rank", "accepted_rank"])
        ),
        "rejection_reason": str(
            _first_present(row, ["rejection_reason", "reason", "accepted_reason"]) or ""
        ),
        "best_score": _as_float(_first_present(row, ["best_score", "score"])),
        "second_score": _as_float(_first_present(row, ["second_score", "runner_up_score"])),
        "candidate_margin": _as_float(_first_present(row, ["candidate_margin", "margin"])),
        "overlap": _as_float(_first_present(row, ["overlap", "effective_overlap"])),
        "converged": _as_bool(_first_present(row, ["converged", "has_converged"])),
        "refinement_delta_m": _as_float(
            _first_present(row, ["refinement_delta_m", "delta_m"])
        ),
        "refinement_delta_yaw_rad": _as_float(
            _first_present(row, ["refinement_delta_yaw_rad", "delta_yaw_rad"])
        ),
        "runtime_sec": runtime,
        "post_reset_ok_rows": post_reset_ok_rows,
        "post_reset_window_sec": _as_float(
            _first_present(row, ["post_reset_window_sec", "stable_window_sec"])
        ),
        "recovered": recovered,
        "false_recovery": false_recovery,
    }


def load_attempts(path: Optional[Path], allow_missing: bool) -> List[Dict[str, Any]]:
    if path is None:
        return []
    if not path.exists():
        if allow_missing:
            return []
        raise FileNotFoundError(f"attempts CSV does not exist: {path}")
    with path.open("r", encoding="utf-8", newline="") as stream:
        return [
            normalize_attempt(row, row_number)
            for row_number, row in enumerate(csv.DictReader(stream), start=1)
        ]


def match_attempts_to_windows(
    windows: List[Dict[str, Any]],
    attempts: List[Dict[str, Any]],
    match_window_sec: float,
) -> List[Dict[str, Any]]:
    matched_windows: List[Dict[str, Any]] = []
    for window_index, window in enumerate(windows):
        start = float(window["start_stamp_sec"])
        end = float(window["end_stamp_sec"]) + match_window_sec
        matching_attempts = []
        for attempt in attempts:
            stamp = attempt["trigger_stamp_sec"]
            if stamp is None:
                stamp = attempt["start_stamp_sec"]
            if stamp is not None and start <= stamp <= end:
                matching_attempts.append(attempt)
        accepted_attempts = [attempt for attempt in matching_attempts if attempt["accepted"] is True]
        recovered_attempts = [attempt for attempt in matching_attempts if attempt["recovered"]]
        matched_windows.append(
            {
                "window_index": window_index,
                "start_delta_sec": window["start_delta_sec"],
                "duration_sec": window["duration_sec"],
                "row_count": window["row_count"],
                "reason_counts": window["reason_counts"],
                "attempt_count": len(matching_attempts),
                "accepted_attempt_count": len(accepted_attempts),
                "recovered_attempt_count": len(recovered_attempts),
                "attempt_ids": [attempt["attempt_id"] for attempt in matching_attempts],
            }
        )
    return matched_windows


def load_trajectory_eval(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None:
        return None
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def build_summary(
    alignment_csv: Path,
    attempts_csv: Optional[Path],
    trajectory_eval_json: Optional[Path],
    allow_missing_attempts: bool,
    request_match_window_sec: float,
    post_reset_stable_ok_rows: int,
    false_recovery_rmse_threshold_m: float,
) -> Dict[str, Any]:
    rows = load_alignment_rows(alignment_csv)
    windows = request_windows(rows)
    attempts = load_attempts(attempts_csv, allow_missing=allow_missing_attempts)
    matched_windows = match_attempts_to_windows(windows, attempts, request_match_window_sec)
    trajectory_eval = load_trajectory_eval(trajectory_eval_json)

    accepted_attempts = [attempt for attempt in attempts if attempt["accepted"] is True]
    rejected_attempts = [attempt for attempt in attempts if attempt["accepted"] is False]
    unknown_decision_attempts = [attempt for attempt in attempts if attempt["accepted"] is None]
    recovered_attempts = [
        attempt
        for attempt in accepted_attempts
        if (attempt["post_reset_ok_rows"] or 0) >= post_reset_stable_ok_rows
    ]
    false_recovery_known = [
        attempt for attempt in attempts if attempt["false_recovery"] is not None
    ]
    false_recovery_count = sum(
        1 for attempt in false_recovery_known if attempt["false_recovery"] is True
    )

    reference_risk = "not_evaluated"
    if accepted_attempts and not false_recovery_known and trajectory_eval is not None:
        translation_rmse_m = trajectory_eval.get("translation_rmse_m")
        if isinstance(translation_rmse_m, (int, float)):
            reference_risk = (
                "accepted_attempts_high_overall_rmse_event_local_unknown"
                if translation_rmse_m > false_recovery_rmse_threshold_m
                else "accepted_attempts_overall_rmse_within_threshold_event_local_unknown"
            )

    request_window_count = len(windows)
    attempted_window_count = sum(1 for window in matched_windows if window["attempt_count"] > 0)
    accepted_window_count = sum(
        1 for window in matched_windows if window["accepted_attempt_count"] > 0
    )

    return {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "alignment_csv": str(alignment_csv),
        "attempts_csv": None if attempts_csv is None else str(attempts_csv),
        "attempts_csv_exists": attempts_csv.exists() if attempts_csv is not None else False,
        "trajectory_eval_json": None if trajectory_eval_json is None else str(trajectory_eval_json),
        "request_match_window_sec": request_match_window_sec,
        "post_reset_stable_ok_rows": post_reset_stable_ok_rows,
        "false_recovery_rmse_threshold_m": false_recovery_rmse_threshold_m,
        "request_windows": {
            "count": request_window_count,
            "attempted_count": attempted_window_count,
            "unattempted_count": request_window_count - attempted_window_count,
            "accepted_count": accepted_window_count,
            "attempted_rate_percent": _percent(attempted_window_count, request_window_count),
            "matched": matched_windows,
        },
        "attempts": {
            "count": len(attempts),
            "accepted_count": len(accepted_attempts),
            "rejected_count": len(rejected_attempts),
            "unknown_decision_count": len(unknown_decision_attempts),
            "recovered_count": len(recovered_attempts),
            "false_recovery_count": false_recovery_count,
            "false_recovery_unknown_count": len(attempts) - len(false_recovery_known),
            "source_counts": dict(Counter(attempt["source"] for attempt in attempts)),
            "mode_counts": dict(Counter(attempt["mode"] for attempt in attempts)),
            "roi_type_counts": dict(Counter(attempt["roi_type"] for attempt in attempts)),
            "rejection_reason_counts": dict(
                Counter(
                    attempt["rejection_reason"] or "unknown"
                    for attempt in attempts
                    if attempt["accepted"] is not True
                )
            ),
            "candidate_count_stats": _stats(
                float(attempt["candidate_count"])
                if attempt["candidate_count"] is not None
                else None
                for attempt in attempts
            ),
            "runtime_sec_stats": _stats(attempt["runtime_sec"] for attempt in attempts),
            "best_score_stats": _stats(attempt["best_score"] for attempt in attempts),
            "candidate_margin_stats": _stats(
                attempt["candidate_margin"] for attempt in attempts
            ),
            "refinement_delta_m_stats": _stats(
                attempt["refinement_delta_m"] for attempt in attempts
            ),
        },
        "trajectory_eval": trajectory_eval,
        "reference_risk": reference_risk,
        "schema": {
            "required_csv_columns": [
                "attempt_id",
                "trigger_stamp_sec",
                "source",
                "candidate_count",
                "accepted",
                "rejection_reason",
                "runtime_sec",
            ],
            "recommended_csv_columns": [
                "start_stamp_sec",
                "end_stamp_sec",
                "mode",
                "roi_type",
                "accepted_candidate_rank",
                "best_score",
                "second_score",
                "candidate_margin",
                "overlap",
                "converged",
                "refinement_delta_m",
                "refinement_delta_yaw_rad",
                "post_reset_ok_rows",
                "post_reset_window_sec",
                "false_recovery",
            ],
        },
        "notes": [
            "an absent attempts CSV means no relocalization candidate generator ran",
            "false recovery is authoritative only when event-local false_recovery is recorded",
            "overall trajectory RMSE is only a coarse reference-risk signal",
        ],
    }


def build_markdown(summary: Dict[str, Any]) -> str:
    requests = summary["request_windows"]
    attempts = summary["attempts"]
    lines = [
        "# Relocalization Attempt Summary",
        "",
        f"Generated at: {summary['generated_at']}",
        "",
        f"- alignment_csv: `{summary['alignment_csv']}`",
        f"- attempts_csv: `{summary['attempts_csv']}`",
        f"- attempts_csv_exists: `{summary['attempts_csv_exists']}`",
        f"- trajectory_eval_json: `{summary['trajectory_eval_json']}`",
        f"- reference_risk: `{summary['reference_risk']}`",
        "",
        "## Attempts",
        "",
        f"- attempts: `{attempts['count']}`",
        f"- accepted: `{attempts['accepted_count']}`",
        f"- rejected: `{attempts['rejected_count']}`",
        f"- unknown decision: `{attempts['unknown_decision_count']}`",
        f"- recovered: `{attempts['recovered_count']}`",
        f"- false recovery: `{attempts['false_recovery_count']}`",
        f"- false recovery unknown: `{attempts['false_recovery_unknown_count']}`",
        f"- source_counts: `{json.dumps(attempts['source_counts'], sort_keys=True)}`",
        f"- rejection_reason_counts: `{json.dumps(attempts['rejection_reason_counts'], sort_keys=True)}`",
        f"- runtime_sec_stats: `{json.dumps(attempts['runtime_sec_stats'], sort_keys=True)}`",
        "",
        "## Request Window Coverage",
        "",
        f"- request windows: `{requests['count']}`",
        f"- attempted windows: `{requests['attempted_count']}` (`{_fmt(requests['attempted_rate_percent'])}%`)",
        f"- unattempted windows: `{requests['unattempted_count']}`",
        f"- windows with accepted attempt: `{requests['accepted_count']}`",
        "",
    ]

    if requests["matched"]:
        lines.extend(
            [
                "| Window | Start [s] | Rows | Attempts | Accepted | Recovered | Attempt IDs | Reasons |",
                "| ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
            ]
        )
        for window in requests["matched"]:
            lines.append(
                "| "
                f"{window['window_index']} | "
                f"{_fmt(window['start_delta_sec'])} | "
                f"{window['row_count']} | "
                f"{window['attempt_count']} | "
                f"{window['accepted_attempt_count']} | "
                f"{window['recovered_attempt_count']} | "
                f"`{json.dumps(window['attempt_ids'])}` | "
                f"`{json.dumps(window['reason_counts'], sort_keys=True)}` |"
            )
    else:
        lines.append("- no reinitialization request windows")

    lines.extend(
        [
            "",
            "## Attempt CSV Schema",
            "",
            f"- required: `{', '.join(summary['schema']['required_csv_columns'])}`",
            f"- recommended: `{', '.join(summary['schema']['recommended_csv_columns'])}`",
            "",
            "## Caveat",
            "",
            "- Use this artifact to measure candidate generation and guarded reset behavior; it does not implement global relocalization by itself.",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    alignment_csv = Path(args.alignment_csv).expanduser().resolve()
    if not alignment_csv.exists():
        raise FileNotFoundError(f"alignment CSV does not exist: {alignment_csv}")

    attempts_csv: Optional[Path] = None
    if args.attempts_csv:
        attempts_csv = Path(args.attempts_csv).expanduser().resolve()

    trajectory_eval_json: Optional[Path] = None
    if args.trajectory_eval_json:
        trajectory_eval_json = Path(args.trajectory_eval_json).expanduser().resolve()
        if not trajectory_eval_json.exists():
            raise FileNotFoundError(
                f"trajectory eval JSON does not exist: {trajectory_eval_json}"
            )

    summary = build_summary(
        alignment_csv=alignment_csv,
        attempts_csv=attempts_csv,
        trajectory_eval_json=trajectory_eval_json,
        allow_missing_attempts=args.allow_missing_attempts,
        request_match_window_sec=args.request_match_window_sec,
        post_reset_stable_ok_rows=args.post_reset_stable_ok_rows,
        false_recovery_rmse_threshold_m=args.false_recovery_rmse_threshold_m,
    )

    if args.output_json:
        output_json = Path(args.output_json).expanduser().resolve()
        output_json.parent.mkdir(parents=True, exist_ok=True)
        output_json.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.output_md:
        output_md = Path(args.output_md).expanduser().resolve()
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_md.write_text(build_markdown(summary) + "\n", encoding="utf-8")

    if not args.output_json and not args.output_md:
        print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
