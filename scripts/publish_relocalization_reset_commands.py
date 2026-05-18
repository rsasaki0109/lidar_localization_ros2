#!/usr/bin/env python3

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional


EXECUTION_FIELDNAMES = [
    "command_id",
    "attempt_id",
    "execute",
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
    "publish_repeat",
    "published_count",
    "status",
    "rejection_reason",
    "validation_json",
    "validation_passed",
    "generated_at",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish validated relocalization reset commands to /initialpose. By default this "
            "only writes an execution report; actual publishing requires --execute."
        )
    )
    parser.add_argument("--commands-csv", required=True)
    parser.add_argument("--validation-json", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--allow-unvalidated", action="store_true")
    parser.add_argument("--allow-failed-validation", action="store_true")
    parser.add_argument("--allow-accepted-source-plan", action="store_true")
    parser.add_argument("--allow-oracle-selection", action="store_true")
    parser.add_argument("--publish-repeat", type=int, default=1)
    parser.add_argument("--publish-period-sec", type=float, default=0.1)
    parser.add_argument("--spin-before-sec", type=float, default=0.2)
    parser.add_argument("--node-name", default="relocalization_reset_command_publisher")
    parser.add_argument("--xy-covariance", type=float, default=0.25)
    parser.add_argument("--z-covariance", type=float, default=0.25)
    parser.add_argument("--roll-pitch-covariance", type=float, default=0.06853892326654787)
    parser.add_argument("--yaw-covariance", type=float, default=0.06853892326654787)
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


def _counts(values: Iterable[str]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _load_validation(path: Path, allow_unvalidated: bool) -> Dict[str, Any]:
    if not path.exists():
        if allow_unvalidated:
            return {"validation_passed": False, "missing": True}
        raise FileNotFoundError(f"validation JSON does not exist: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _validation_ok(validation: Dict[str, Any], args: argparse.Namespace) -> bool:
    if args.allow_unvalidated:
        return True
    if bool(validation.get("validation_passed")):
        return True
    return bool(args.allow_failed_validation)


def _safe_float(row: Dict[str, str], key: str) -> Optional[float]:
    return _as_float(row.get(key))


def _command_rejection(row: Dict[str, str], args: argparse.Namespace) -> str:
    if row.get("status") != "dry_run_command_generated":
        return "command_status_not_generated"
    if _as_bool(row.get("dry_run")) is not True:
        return "command_not_marked_dry_run"
    if _as_bool(row.get("source_plan_row_accepted")) is True and not args.allow_accepted_source_plan:
        return "accepted_source_plan_not_allowed"
    if row.get("source_selection_source") == "oracle_rank" and not args.allow_oracle_selection:
        return "oracle_selection_not_allowed"
    for field in [
        "position_x",
        "position_y",
        "position_z",
        "orientation_x",
        "orientation_y",
        "orientation_z",
        "orientation_w",
    ]:
        if _safe_float(row, field) is None:
            return f"{field}_missing"
    return ""


def _empty_execution(
    row: Dict[str, str],
    args: argparse.Namespace,
    validation_json: Path,
    validation_passed: bool,
    status: str,
    rejection_reason: str,
    generated_at: str,
) -> Dict[str, str]:
    return {
        "command_id": str(row.get("command_id", "")),
        "attempt_id": str(row.get("attempt_id", "")),
        "execute": "true" if args.execute else "false",
        "publish_topic": str(row.get("publish_topic", "")),
        "frame_id": str(row.get("frame_id", "")),
        "position_x": str(row.get("position_x", "")),
        "position_y": str(row.get("position_y", "")),
        "position_z": str(row.get("position_z", "")),
        "orientation_x": str(row.get("orientation_x", "")),
        "orientation_y": str(row.get("orientation_y", "")),
        "orientation_z": str(row.get("orientation_z", "")),
        "orientation_w": str(row.get("orientation_w", "")),
        "yaw_rad": str(row.get("yaw_rad", "")),
        "publish_repeat": str(args.publish_repeat),
        "published_count": "0",
        "status": status,
        "rejection_reason": rejection_reason,
        "validation_json": str(validation_json),
        "validation_passed": "true" if validation_passed else "false",
        "generated_at": generated_at,
    }


def _covariance(args: argparse.Namespace) -> List[float]:
    covariance = [0.0] * 36
    covariance[0] = args.xy_covariance
    covariance[7] = args.xy_covariance
    covariance[14] = args.z_covariance
    covariance[21] = args.roll_pitch_covariance
    covariance[28] = args.roll_pitch_covariance
    covariance[35] = args.yaw_covariance
    return covariance


def publish_rows(rows: List[Dict[str, str]], args: argparse.Namespace) -> Dict[str, int]:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rclpy.init()
    node = rclpy.create_node(args.node_name)
    publishers: Dict[str, Any] = {}
    counts: Dict[str, int] = {}
    try:
        deadline = time.monotonic() + max(0.0, args.spin_before_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        covariance = _covariance(args)
        for row in rows:
            topic = row["publish_topic"]
            if topic not in publishers:
                publishers[topic] = node.create_publisher(PoseWithCovarianceStamped, topic, 10)
            publisher = publishers[topic]
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = row["frame_id"]
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(row["position_x"])
            msg.pose.pose.position.y = float(row["position_y"])
            msg.pose.pose.position.z = float(row["position_z"])
            msg.pose.pose.orientation.x = float(row["orientation_x"])
            msg.pose.pose.orientation.y = float(row["orientation_y"])
            msg.pose.pose.orientation.z = float(row["orientation_z"])
            msg.pose.pose.orientation.w = float(row["orientation_w"])
            msg.pose.covariance = covariance
            count = 0
            for _ in range(max(1, args.publish_repeat)):
                msg.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.0)
                count += 1
                if args.publish_period_sec > 0:
                    time.sleep(args.publish_period_sec)
            counts[row["command_id"]] = count
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return counts


def build_execution_rows(
    command_rows: List[Dict[str, str]],
    validation_json: Path,
    validation_passed: bool,
    args: argparse.Namespace,
) -> List[Dict[str, str]]:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    execution_rows: List[Dict[str, str]] = []
    for row in command_rows:
        reason = _command_rejection(row, args)
        if reason:
            execution_rows.append(
                _empty_execution(
                    row,
                    args,
                    validation_json,
                    validation_passed,
                    status="not_published",
                    rejection_reason=reason,
                    generated_at=generated_at,
                )
            )
            continue
        if not validation_passed:
            execution_rows.append(
                _empty_execution(
                    row,
                    args,
                    validation_json,
                    validation_passed,
                    status="not_published",
                    rejection_reason="validation_not_passed",
                    generated_at=generated_at,
                )
            )
            continue
        if not args.execute:
            execution_rows.append(
                _empty_execution(
                    row,
                    args,
                    validation_json,
                    validation_passed,
                    status="not_published",
                    rejection_reason="execute_flag_required",
                    generated_at=generated_at,
                )
            )
            continue
        execution_rows.append(
            _empty_execution(
                row,
                args,
                validation_json,
                validation_passed,
                status="pending_publish",
                rejection_reason="",
                generated_at=generated_at,
            )
        )
    return execution_rows


def write_csv(path: Path, rows: List[Dict[str, str]], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=EXECUTION_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def build_summary(commands_csv: Path, output_csv: Path, rows: List[Dict[str, str]]) -> Dict[str, Any]:
    published_count = sum(int(row.get("published_count", "0") or "0") for row in rows)
    return {
        "commands_csv": str(commands_csv),
        "output_csv": str(output_csv),
        "execution_row_count": len(rows),
        "published_count": published_count,
        "status_counts": _counts(row.get("status", "") for row in rows),
        "rejection_reason_counts": _counts(row.get("rejection_reason", "") for row in rows),
        "notes": [
            "actual publishing requires --execute",
            "validation_passed=true is required unless explicitly overridden",
            "publishing uses geometry_msgs/PoseWithCovarianceStamped",
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
        "# Relocalization Reset Command Execution",
        "",
        f"- commands CSV: `{data['commands_csv']}`",
        f"- output CSV: `{data['output_csv']}`",
        f"- execution rows: `{data['execution_row_count']}`",
        f"- published count: `{data['published_count']}`",
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
    commands_csv = Path(args.commands_csv).expanduser().resolve()
    validation_json = Path(args.validation_json).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    if not commands_csv.exists():
        raise FileNotFoundError(f"commands CSV does not exist: {commands_csv}")
    if args.publish_repeat < 1:
        raise ValueError("--publish-repeat must be >= 1")

    command_rows = _read_csv(commands_csv)
    validation = _load_validation(validation_json, args.allow_unvalidated)
    validation_passed = _validation_ok(validation, args)
    execution_rows = build_execution_rows(command_rows, validation_json, validation_passed, args)
    publishable = [row for row in execution_rows if row["status"] == "pending_publish"]
    if publishable:
        published = publish_rows(publishable, args)
        for row in execution_rows:
            if row["status"] == "pending_publish":
                count = published.get(row["command_id"], 0)
                row["published_count"] = str(count)
                row["status"] = "published_initialpose" if count > 0 else "publish_failed"
                row["rejection_reason"] = "" if count > 0 else "publisher_returned_zero"
    write_csv(output_csv, execution_rows, args.overwrite)
    summary = build_summary(commands_csv, output_csv, execution_rows)
    if args.output_json:
        write_json(Path(args.output_json).expanduser().resolve(), summary, args.overwrite)
    if args.output_md:
        write_md(Path(args.output_md).expanduser().resolve(), summary, args.overwrite)
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
