#!/usr/bin/env python3

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional


FIELDNAMES = [
    "attempt_id",
    "trigger_stamp_sec",
    "start_stamp_sec",
    "end_stamp_sec",
    "source",
    "mode",
    "roi_type",
    "candidate_count",
    "accepted",
    "accepted_candidate_rank",
    "rejection_reason",
    "best_score",
    "second_score",
    "candidate_margin",
    "overlap",
    "converged",
    "refinement_delta_m",
    "refinement_delta_yaw_rad",
    "runtime_sec",
    "post_reset_ok_rows",
    "post_reset_window_sec",
    "false_recovery",
    "request_reason",
    "request_score",
    "request_window_rows",
    "request_window_duration_sec",
    "generated_at",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a disabled relocalization-attempt baseline CSV from alignment_status.csv. "
            "Each reinitialization request window becomes one rejected attempt with "
            "candidate_count=0."
        )
    )
    parser.add_argument("--alignment-csv", required=True, help="Input alignment_status.csv")
    parser.add_argument("--output-csv", required=True, help="Output relocalization_attempts.csv")
    parser.add_argument(
        "--source",
        default="disabled",
        help="Attempt source label to write into the CSV.",
    )
    parser.add_argument(
        "--mode",
        default="diagnostic_request",
        help="Attempt mode label to write into the CSV.",
    )
    parser.add_argument(
        "--roi-type",
        default="none",
        help="ROI type label to write into the CSV.",
    )
    parser.add_argument(
        "--rejection-reason",
        default="candidate_generator_disabled",
        help="Rejection reason written for every generated attempt.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite output CSV when it already exists.",
    )
    return parser.parse_args()


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        return float(str(value))
    except (TypeError, ValueError):
        return None


def load_alignment_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            values = json.loads(record["values_json"])
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "message": str(record.get("message", "")),
                    "requested": _as_bool(values.get("reinitialization_requested")),
                    "reason": values.get("reinitialization_request_reason"),
                    "score": _as_float(values.get("reinitialization_request_score")),
                }
            )
    return rows


def build_attempt_rows(
    rows: List[Dict[str, Any]],
    source: str,
    mode: str,
    roi_type: str,
    rejection_reason: str,
) -> List[Dict[str, Any]]:
    attempts: List[Dict[str, Any]] = []
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    index = 0
    while index < len(rows):
        if not rows[index]["requested"]:
            index += 1
            continue

        start = index
        while index < len(rows) and rows[index]["requested"]:
            index += 1
        end = index - 1

        attempts.append(
            {
                "attempt_id": f"disabled_{len(attempts) + 1:04d}",
                "trigger_stamp_sec": f"{rows[start]['stamp_sec']:.9f}",
                "start_stamp_sec": f"{rows[start]['stamp_sec']:.9f}",
                "end_stamp_sec": f"{rows[start]['stamp_sec']:.9f}",
                "source": source,
                "mode": mode,
                "roi_type": roi_type,
                "candidate_count": "0",
                "accepted": "false",
                "accepted_candidate_rank": "",
                "rejection_reason": rejection_reason,
                "best_score": "",
                "second_score": "",
                "candidate_margin": "",
                "overlap": "",
                "converged": "false",
                "refinement_delta_m": "",
                "refinement_delta_yaw_rad": "",
                "runtime_sec": "0.0",
                "post_reset_ok_rows": "0",
                "post_reset_window_sec": "",
                "false_recovery": "",
                "request_reason": rows[start]["reason"] or "",
                "request_score": (
                    "" if rows[start]["score"] is None else f"{rows[start]['score']:.10g}"
                ),
                "request_window_rows": str(end - start + 1),
                "request_window_duration_sec": f"{max(0.0, rows[end]['stamp_sec'] - rows[start]['stamp_sec']):.9f}",
                "generated_at": generated_at,
            }
        )
    return attempts


def write_attempts(path: Path, attempts: List[Dict[str, Any]], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(attempts)


def main() -> None:
    args = parse_args()
    alignment_csv = Path(args.alignment_csv).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    if not alignment_csv.exists():
        raise FileNotFoundError(f"alignment CSV does not exist: {alignment_csv}")

    attempts = build_attempt_rows(
        load_alignment_rows(alignment_csv),
        source=args.source,
        mode=args.mode,
        roi_type=args.roi_type,
        rejection_reason=args.rejection_reason,
    )
    write_attempts(output_csv, attempts, overwrite=args.overwrite)
    print(json.dumps({"output_csv": str(output_csv), "attempt_count": len(attempts)}))


if __name__ == "__main__":
    main()
