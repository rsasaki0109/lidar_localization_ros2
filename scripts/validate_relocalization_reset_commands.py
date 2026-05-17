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


REQUIRED_COLUMNS = [
    "command_id",
    "attempt_id",
    "command_type",
    "dry_run",
    "publish_topic",
    "frame_id",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "yaw_rad",
    "source_plan_row_accepted",
    "source_selection_source",
    "status",
    "rejection_reason",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate dry-run relocalization reset command artifacts. This checker never "
            "publishes /initialpose; it only enforces the dry-run command contract."
        )
    )
    parser.add_argument("--commands-csv", required=True)
    parser.add_argument(
        "--summary-json",
        default="",
        help="Optional relocalization_reset_commands.json summary to cross-check published_count.",
    )
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    parser.add_argument("--expected-topic", default="/initialpose")
    parser.add_argument("--expected-frame-id", default="map")
    parser.add_argument("--quaternion-norm-tolerance", type=float, default=1e-3)
    parser.add_argument("--min-generated-count", type=int, default=0)
    parser.add_argument("--max-generated-count", type=int, default=-1)
    parser.add_argument("--allow-published", action="store_true")
    parser.add_argument("--allow-accepted-source-plan", action="store_true")
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


def _counts(values: Iterable[str]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _load_summary(path: Optional[Path]) -> Dict[str, Any]:
    if path is None:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def validate(
    header: List[str],
    rows: List[Dict[str, str]],
    summary: Dict[str, Any],
    args: argparse.Namespace,
) -> Dict[str, Any]:
    failures: List[str] = []
    warnings: List[str] = []

    missing = [column for column in REQUIRED_COLUMNS if column not in header]
    for column in missing:
        failures.append(f"missing required command column: {column}")
    if not rows:
        failures.append("commands CSV has no rows")

    command_ids = [str(row.get("command_id", "")) for row in rows]
    for command_id, count in _counts(command_ids).items():
        if not command_id:
            failures.append("command row has empty command_id")
        if count > 1:
            failures.append(f"command_id appears more than once: {command_id}")

    generated_rows = [row for row in rows if row.get("status") == "dry_run_command_generated"]
    if len(generated_rows) < args.min_generated_count:
        failures.append(
            f"generated command count {len(generated_rows)} is below minimum {args.min_generated_count}"
        )
    if args.max_generated_count >= 0 and len(generated_rows) > args.max_generated_count:
        failures.append(
            f"generated command count {len(generated_rows)} is above maximum {args.max_generated_count}"
        )

    for row in rows:
        prefix = f"command {row.get('command_id', '')}: "
        dry_run = _as_bool(row.get("dry_run"))
        if dry_run is not True:
            failures.append(prefix + "dry_run is not true")
        if row.get("command_type") != "initialpose":
            failures.append(prefix + "command_type is not initialpose")
        if row.get("publish_topic") != args.expected_topic:
            failures.append(prefix + f"publish_topic is not {args.expected_topic}")
        if row.get("frame_id") != args.expected_frame_id:
            failures.append(prefix + f"frame_id is not {args.expected_frame_id}")
        if (
            _as_bool(row.get("source_plan_row_accepted")) is True
            and not args.allow_accepted_source_plan
        ):
            failures.append(prefix + "source_plan_row_accepted=true is not allowed")
        if row.get("source_selection_source") == "oracle_rank" and not args.allow_oracle_selection:
            failures.append(prefix + "oracle_rank selection is not allowed")

        if row.get("status") == "dry_run_command_generated":
            if row.get("rejection_reason") != "publish_disabled":
                failures.append(prefix + "generated dry-run command must use publish_disabled")
            for field in [
                "stamp_sec",
                "position_x",
                "position_y",
                "position_z",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
                "yaw_rad",
            ]:
                if _as_float(row.get(field)) is None:
                    failures.append(prefix + f"{field} is missing or non-finite")
            qx = _as_float(row.get("orientation_x"))
            qy = _as_float(row.get("orientation_y"))
            qz = _as_float(row.get("orientation_z"))
            qw = _as_float(row.get("orientation_w"))
            if None not in (qx, qy, qz, qw):
                assert qx is not None and qy is not None and qz is not None and qw is not None
                norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
                if abs(norm - 1.0) > args.quaternion_norm_tolerance:
                    failures.append(prefix + f"quaternion norm {norm:.9f} is not near 1.0")
        elif row.get("status") == "rejected":
            if not row.get("rejection_reason"):
                warnings.append(prefix + "rejected command has empty rejection_reason")
        else:
            failures.append(prefix + f"unknown status: {row.get('status', '')}")

    published_count = summary.get("published_count")
    if published_count is not None:
        try:
            published_count_int = int(published_count)
        except (TypeError, ValueError):
            failures.append("summary published_count is not an integer")
        else:
            if published_count_int != 0 and not args.allow_published:
                failures.append(f"summary published_count is not zero: {published_count_int}")
    elif summary:
        warnings.append("summary JSON has no published_count field")

    return {
        "validation_passed": not failures,
        "failure_count": len(failures),
        "warning_count": len(warnings),
        "failures": failures,
        "warnings": warnings,
        "command_count": len(rows),
        "dry_run_generated_count": len(generated_rows),
        "status_counts": _counts(row.get("status", "") for row in rows),
        "rejection_reason_counts": _counts(row.get("rejection_reason", "") for row in rows),
        "source_selection_source_counts": _counts(
            row.get("source_selection_source", "") for row in generated_rows
        ),
        "summary_published_count": published_count,
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
        "# Relocalization Reset Command Validation",
        "",
        f"- validation passed: `{str(data['validation_passed']).lower()}`",
        f"- failures: `{data['failure_count']}`",
        f"- warnings: `{data['warning_count']}`",
        f"- commands: `{data['command_count']}`",
        f"- dry-run generated: `{data['dry_run_generated_count']}`",
        f"- summary published count: `{data['summary_published_count']}`",
        f"- status counts: `{json.dumps(data['status_counts'], sort_keys=True)}`",
        f"- rejection reasons: `{json.dumps(data['rejection_reason_counts'], sort_keys=True)}`",
        f"- selected sources: `{json.dumps(data['source_selection_source_counts'], sort_keys=True)}`",
        "",
        "## Failures",
        "",
    ]
    lines.extend(f"- {failure}" for failure in data["failures"]) if data["failures"] else lines.append("- none")
    lines.extend(["", "## Warnings", ""])
    lines.extend(f"- {warning}" for warning in data["warnings"]) if data["warnings"] else lines.append("- none")
    lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    commands_csv = Path(args.commands_csv).expanduser().resolve()
    summary_json = Path(args.summary_json).expanduser().resolve() if args.summary_json else None
    if not commands_csv.exists():
        raise FileNotFoundError(f"commands CSV does not exist: {commands_csv}")
    if summary_json is not None and not summary_json.exists():
        raise FileNotFoundError(f"summary JSON does not exist: {summary_json}")

    header, rows = _read_csv(commands_csv)
    summary = _load_summary(summary_json)
    result = validate(header, rows, summary, args)
    result.update(
        {
            "commands_csv": str(commands_csv),
            "summary_json": "" if summary_json is None else str(summary_json),
            "notes": [
                "this validator never publishes /initialpose",
                "dry_run=true is required by default",
                "published_count must be zero by default when summary JSON is provided",
                "oracle_rank command provenance is rejected by default",
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
