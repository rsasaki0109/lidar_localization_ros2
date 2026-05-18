#!/usr/bin/env python3

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional


COMMAND_FIELDNAMES = [
    "command_id",
    "attempt_id",
    "trigger_stamp_sec",
    "command_type",
    "dry_run",
    "publish_topic",
    "frame_id",
    "stamp_sec",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "yaw_rad",
    "source_plan_row_selected",
    "source_plan_row_accepted",
    "source_job_id",
    "source_candidate_index",
    "source_selection_source",
    "source_selection_rank",
    "source_score",
    "source_registration_gate_reason",
    "status",
    "rejection_reason",
    "generated_at",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a relocalization reset candidate plan into dry-run reset command artifacts. "
            "This does not publish /initialpose and does not execute a reset."
        )
    )
    parser.add_argument("--plan-csv", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    parser.add_argument("--publish-topic", default="/initialpose")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument(
        "--stamp-source",
        choices=["trigger", "zero"],
        default="trigger",
        help="Stamp to write into the dry-run command artifact.",
    )
    parser.add_argument("--allow-accepted-plan", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def _read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


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


def _fmt(value: Optional[float]) -> str:
    return "" if value is None else f"{value:.9f}"


def _yaw_to_quaternion(yaw: float) -> Dict[str, float]:
    half = 0.5 * yaw
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def _counts(values: Iterable[str]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _rejected_command(
    command_id: str,
    row: Dict[str, str],
    publish_topic: str,
    frame_id: str,
    reason: str,
    generated_at: str,
) -> Dict[str, str]:
    command = {field: "" for field in COMMAND_FIELDNAMES}
    command.update(
        {
            "command_id": command_id,
            "attempt_id": str(row.get("attempt_id", "")),
            "trigger_stamp_sec": str(row.get("trigger_stamp_sec", "")),
            "command_type": "initialpose",
            "dry_run": "true",
            "publish_topic": publish_topic,
            "frame_id": frame_id,
            "source_plan_row_selected": str(row.get("selected", "")),
            "source_plan_row_accepted": str(row.get("accepted", "")),
            "source_job_id": str(row.get("selected_job_id", "")),
            "source_candidate_index": str(row.get("selected_candidate_index", "")),
            "source_selection_source": str(row.get("selected_selection_source", "")),
            "source_selection_rank": str(row.get("selected_selection_rank", "")),
            "source_score": str(row.get("selected_score", "")),
            "source_registration_gate_reason": str(
                row.get("selected_registration_gate_reason", "")
            ),
            "status": "rejected",
            "rejection_reason": reason,
            "generated_at": generated_at,
        }
    )
    return command


def build_commands(
    plan_rows: List[Dict[str, str]],
    publish_topic: str,
    frame_id: str,
    stamp_source: str,
    allow_accepted_plan: bool,
) -> List[Dict[str, str]]:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    commands: List[Dict[str, str]] = []
    for index, row in enumerate(plan_rows, start=1):
        command_id = f"reset_command_{index:06d}"
        selected = _as_bool(row.get("selected"))
        accepted = _as_bool(row.get("accepted"))
        if selected is not True:
            commands.append(
                _rejected_command(
                    command_id,
                    row,
                    publish_topic,
                    frame_id,
                    "plan_row_not_selected",
                    generated_at,
                )
            )
            continue
        if accepted is True and not allow_accepted_plan:
            commands.append(
                _rejected_command(
                    command_id,
                    row,
                    publish_topic,
                    frame_id,
                    "accepted_plan_rows_not_allowed",
                    generated_at,
                )
            )
            continue

        x = _as_float(row.get("selected_final_pose_x"))
        y = _as_float(row.get("selected_final_pose_y"))
        z = _as_float(row.get("selected_final_pose_z"))
        yaw = _as_float(row.get("selected_final_yaw_rad"))
        if x is None or y is None or z is None or yaw is None:
            commands.append(
                _rejected_command(
                    command_id,
                    row,
                    publish_topic,
                    frame_id,
                    "selected_final_pose_missing",
                    generated_at,
                )
            )
            continue

        quat = _yaw_to_quaternion(yaw)
        stamp = _as_float(row.get("trigger_stamp_sec")) if stamp_source == "trigger" else 0.0
        commands.append(
            {
                "command_id": command_id,
                "attempt_id": str(row.get("attempt_id", "")),
                "trigger_stamp_sec": str(row.get("trigger_stamp_sec", "")),
                "command_type": "initialpose",
                "dry_run": "true",
                "publish_topic": publish_topic,
                "frame_id": frame_id,
                "stamp_sec": _fmt(stamp),
                "position_x": _fmt(x),
                "position_y": _fmt(y),
                "position_z": _fmt(z),
                "orientation_x": _fmt(quat["x"]),
                "orientation_y": _fmt(quat["y"]),
                "orientation_z": _fmt(quat["z"]),
                "orientation_w": _fmt(quat["w"]),
                "yaw_rad": _fmt(yaw),
                "source_plan_row_selected": str(row.get("selected", "")),
                "source_plan_row_accepted": str(row.get("accepted", "")),
                "source_job_id": str(row.get("selected_job_id", "")),
                "source_candidate_index": str(row.get("selected_candidate_index", "")),
                "source_selection_source": str(row.get("selected_selection_source", "")),
                "source_selection_rank": str(row.get("selected_selection_rank", "")),
                "source_score": str(row.get("selected_score", "")),
                "source_registration_gate_reason": str(
                    row.get("selected_registration_gate_reason", "")
                ),
                "status": "dry_run_command_generated",
                "rejection_reason": "publish_disabled",
                "generated_at": generated_at,
            }
        )
    return commands


def write_csv(path: Path, rows: List[Dict[str, str]], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=COMMAND_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def build_summary(plan_csv: Path, output_csv: Path, commands: List[Dict[str, str]]) -> Dict[str, Any]:
    generated = [row for row in commands if row.get("status") == "dry_run_command_generated"]
    return {
        "plan_csv": str(plan_csv),
        "output_csv": str(output_csv),
        "command_count": len(commands),
        "dry_run_generated_count": len(generated),
        "published_count": 0,
        "status_counts": _counts(row.get("status", "") for row in commands),
        "rejection_reason_counts": _counts(row.get("rejection_reason", "") for row in commands),
        "notes": [
            "this helper never publishes /initialpose",
            "commands use selected_final_pose_* from the reset candidate plan",
            "selected_initial_pose_* remains candidate seed provenance, not the reset command pose",
        ],
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
        "# Relocalization Reset Commands",
        "",
        f"- plan CSV: `{data['plan_csv']}`",
        f"- output CSV: `{data['output_csv']}`",
        f"- commands: `{data['command_count']}`",
        f"- dry-run generated: `{data['dry_run_generated_count']}`",
        f"- published: `{data['published_count']}`",
        f"- status counts: `{json.dumps(data['status_counts'], sort_keys=True)}`",
        f"- rejection reasons: `{json.dumps(data['rejection_reason_counts'], sort_keys=True)}`",
        "",
        "## Notes",
        "",
    ]
    lines.extend(f"- {note}" for note in data["notes"])
    lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    plan_csv = Path(args.plan_csv).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    if not plan_csv.exists():
        raise FileNotFoundError(f"plan CSV does not exist: {plan_csv}")

    plan_rows = _read_csv(plan_csv)
    commands = build_commands(
        plan_rows=plan_rows,
        publish_topic=args.publish_topic,
        frame_id=args.frame_id,
        stamp_source=args.stamp_source,
        allow_accepted_plan=args.allow_accepted_plan,
    )
    write_csv(output_csv, commands, args.overwrite)
    summary = build_summary(plan_csv, output_csv, commands)
    if args.output_json:
        write_json(Path(args.output_json).expanduser().resolve(), summary, args.overwrite)
    if args.output_md:
        write_md(Path(args.output_md).expanduser().resolve(), summary, args.overwrite)
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
