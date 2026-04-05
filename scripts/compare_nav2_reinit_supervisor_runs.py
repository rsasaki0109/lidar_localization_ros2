#!/usr/bin/env python3

import argparse
import csv
import json
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize one or more Nav2 replay runs for reinitialization-supervisor comparison."
    )
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        help="Run label and log dir in the form label=/absolute/path/to/log_dir",
    )
    parser.add_argument("--output-json", default="", help="Optional output JSON path")
    parser.add_argument("--output-md", default="", help="Optional output Markdown path")
    return parser.parse_args()


def parse_run_arg(value: str) -> Tuple[str, Path]:
    if "=" not in value:
        raise ValueError(f"Invalid --run value: {value}")
    label, path_str = value.split("=", 1)
    label = label.strip()
    path = Path(path_str).expanduser().resolve()
    if not label:
        raise ValueError(f"Empty run label: {value}")
    if not path.exists():
        raise FileNotFoundError(f"Run log dir does not exist: {path}")
    return label, path


def load_alignment_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            values = json.loads(record["values_json"])
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "message": record["message"],
                    "requested": str(values.get("reinitialization_requested", "")).lower() == "true",
                    "reason": values.get("reinitialization_request_reason"),
                    "score": float(values.get("reinitialization_request_score", "nan")),
                    "accepted_gap_sec": values.get("accepted_gap_sec"),
                    "consecutive_rejected_updates": values.get("consecutive_rejected_updates"),
                }
            )
    return rows


def parse_supervisor_log(path: Path) -> Dict[str, int]:
    counts = {
        "request_received_count": 0,
        "burst_started_count": 0,
        "request_cleared_count": 0,
    }
    if not path.exists():
        return counts

    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if "Received reinitialization request" in line:
                counts["request_received_count"] += 1
            if "Starting reinitialization publish burst" in line:
                counts["burst_started_count"] += 1
            if "Reinitialization request cleared" in line:
                counts["request_cleared_count"] += 1
    return counts


def parse_goal_status(path: Path) -> str:
    if not path.exists():
        return "MISSING"

    status = "UNKNOWN"
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if "Goal finished with status:" in stripped:
                status = stripped.rsplit(":", 1)[-1].strip().upper()
            elif "goal finished with status=" in stripped.lower():
                status = stripped.rsplit("=", 1)[-1].strip().upper()
    return status


def summarize_alignment(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not rows:
        return {
            "row_count": 0,
            "reinitialization_requested_rows": 0,
            "request_transition_count": 0,
            "first_trigger_delta_sec": None,
            "first_trigger_reason": None,
            "ok_rows_after_first_trigger": 0,
            "tail_ok_rows": 0,
            "reason_counts": {},
            "recovery_windows": [],
        }

    first_stamp = rows[0]["stamp_sec"]
    requested_rows = [row for row in rows if row["requested"]]
    first_trigger = requested_rows[0] if requested_rows else None
    reason_counts = Counter(
        str(row["reason"]) for row in requested_rows if row["reason"] is not None
    )

    transition_count = 0
    previous_requested = False
    for row in rows:
        if row["requested"] and not previous_requested:
            transition_count += 1
        previous_requested = row["requested"]

    ok_rows_after_first_trigger = 0
    if first_trigger is not None:
        trigger_stamp = float(first_trigger["stamp_sec"])
        ok_rows_after_first_trigger = sum(
            1 for row in rows if row["stamp_sec"] > trigger_stamp and row["message"] == "ok"
        )

    recovery_windows: List[Dict[str, Any]] = []
    index = 0
    while index < len(rows):
        if not rows[index]["requested"]:
            index += 1
            continue

        start_index = index
        while index < len(rows) and rows[index]["requested"]:
            index += 1
        end_index = index - 1

        first_ok_after_sec: Optional[float] = None
        probe = index
        while probe < len(rows):
            if rows[probe]["message"] == "ok":
                first_ok_after_sec = rows[probe]["stamp_sec"] - rows[start_index]["stamp_sec"]
                break
            if rows[probe]["requested"]:
                break
            probe += 1

        recovery_windows.append(
            {
                "start_delta_sec": rows[start_index]["stamp_sec"] - first_stamp,
                "duration_rows": end_index - start_index + 1,
                "first_ok_after_sec": first_ok_after_sec,
                "reason": rows[start_index]["reason"],
            }
        )

    return {
        "row_count": len(rows),
        "reinitialization_requested_rows": len(requested_rows),
        "request_transition_count": transition_count,
        "first_trigger_delta_sec": (
            None if first_trigger is None else first_trigger["stamp_sec"] - first_stamp
        ),
        "first_trigger_reason": None if first_trigger is None else first_trigger["reason"],
        "ok_rows_after_first_trigger": ok_rows_after_first_trigger,
        "tail_ok_rows": sum(1 for row in rows[-100:] if row["message"] == "ok"),
        "reason_counts": dict(reason_counts),
        "recovery_windows": recovery_windows,
    }


def score_run(run: Dict[str, Any]) -> Tuple[Any, ...]:
    alignment = run["alignment"]
    goal_status = run["goal_status"]
    return (
        1 if goal_status == "SUCCEEDED" else 0,
        alignment["ok_rows_after_first_trigger"],
        alignment["tail_ok_rows"],
        -alignment["reinitialization_requested_rows"],
        -alignment["request_transition_count"],
    )


def build_markdown(comparison: Dict[str, Any]) -> str:
    lines = [
        "# Nav2 Reinitialization Supervisor Compare",
        "",
        f"Generated at: {comparison['generated_at']}",
        "",
        f"Recommended run: `{comparison['recommended_run']}`",
        "",
        "| Run | Goal status | Requested rows | Request transitions | First trigger [s] | OK rows after first trigger | Tail OK rows | Burst starts |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for run in comparison["runs"]:
        alignment = run["alignment"]
        supervisor = run["supervisor_log"]
        lines.append(
            "| "
            f"{run['label']} | "
            f"{run['goal_status']} | "
            f"{alignment['reinitialization_requested_rows']} | "
            f"{alignment['request_transition_count']} | "
            f"{alignment['first_trigger_delta_sec']} | "
            f"{alignment['ok_rows_after_first_trigger']} | "
            f"{alignment['tail_ok_rows']} | "
            f"{supervisor['burst_started_count']} |"
        )
    lines.append("")
    for run in comparison["runs"]:
        lines.append(f"## {run['label']}")
        lines.append("")
        lines.append(f"- log_dir: `{run['log_dir']}`")
        lines.append(f"- goal status: `{run['goal_status']}`")
        lines.append(
            f"- first trigger reason: `{run['alignment']['first_trigger_reason']}`"
        )
        lines.append(
            f"- request reason counts: `{json.dumps(run['alignment']['reason_counts'], sort_keys=True)}`"
        )
        lines.append(
            f"- recovery windows: `{json.dumps(run['alignment']['recovery_windows'])}`"
        )
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    runs: List[Dict[str, Any]] = []
    for raw_run in args.run:
        label, log_dir = parse_run_arg(raw_run)
        alignment_path = log_dir / "alignment_status.csv"
        if not alignment_path.exists():
            raise FileNotFoundError(f"alignment_status.csv not found under {log_dir}")
        rows = load_alignment_rows(alignment_path)
        runs.append(
            {
                "label": label,
                "log_dir": str(log_dir),
                "alignment": summarize_alignment(rows),
                "supervisor_log": parse_supervisor_log(log_dir / "nav2_launch.log"),
                "goal_status": parse_goal_status(log_dir / "navigate_to_pose.log"),
            }
        )

    recommended_run = max(runs, key=score_run)["label"]
    comparison = {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "run_count": len(runs),
        "recommended_run": recommended_run,
        "runs": runs,
    }

    if args.output_json:
        output_json = Path(args.output_json).expanduser().resolve()
        output_json.parent.mkdir(parents=True, exist_ok=True)
        output_json.write_text(json.dumps(comparison, indent=2, sort_keys=True), encoding="utf-8")

    markdown = build_markdown(comparison)
    if args.output_md:
        output_md = Path(args.output_md).expanduser().resolve()
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_md.write_text(markdown, encoding="utf-8")

    print(json.dumps(comparison, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
