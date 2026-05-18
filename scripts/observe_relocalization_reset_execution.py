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


OBSERVATION_FIELDNAMES = [
    "command_id",
    "attempt_id",
    "published_count",
    "execution_status",
    "command_stamp_sec",
    "observation_window_sec",
    "alignment_rows_in_window",
    "ok_rows_in_window",
    "failure_like_rows_in_window",
    "first_ok_latency_sec",
    "stable_ok_rows_after_reset",
    "max_reject_streak_after_reset",
    "observed_recovered",
    "accepted",
    "false_recovery_observable",
    "reference_risky",
    "rejection_reason",
    "generated_at",
]


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
            "Observe post-reset localization health from reset execution and alignment_status "
            "CSV artifacts. This is offline and does not subscribe to ROS topics."
        )
    )
    parser.add_argument("--execution-csv", required=True)
    parser.add_argument("--alignment-csv", required=True)
    parser.add_argument(
        "--commands-csv",
        default="",
        help="Optional reset commands CSV. Used to recover command stamp/trigger fields.",
    )
    parser.add_argument("--trajectory-eval-json", default="")
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    parser.add_argument("--post-reset-window-sec", type=float, default=10.0)
    parser.add_argument("--stable-ok-rows", type=int, default=5)
    parser.add_argument("--false-recovery-rmse-threshold-m", type=float, default=5.0)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def _read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


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


def _fmt(value: Optional[float]) -> str:
    return "" if value is None else f"{value:.9f}"


def _counts(values: Iterable[str]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def load_alignment_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for record in _read_csv(path):
        values = json.loads(record["values_json"])
        message = str(record.get("message", ""))
        requested = _as_bool(values.get("reinitialization_requested")) is True
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
                "message": message,
                "failure_like": failure_like,
                "consecutive_rejected_updates": _as_int(
                    values.get("consecutive_rejected_updates")
                )
                or 0,
            }
        )
    rows.sort(key=lambda row: row["stamp_sec"])
    return rows


def load_trajectory_risk(path: Optional[Path], threshold_m: float) -> Dict[str, Any]:
    if path is None or not path.exists():
        return {"available": False, "translation_rmse_m": None, "reference_risky": None}
    data = json.loads(path.read_text(encoding="utf-8"))
    rmse = _as_float(
        data.get("translation_rmse_m")
        or data.get("rmse_m")
        or data.get("translation", {}).get("rmse_m", "")
    )
    return {
        "available": rmse is not None,
        "translation_rmse_m": rmse,
        "reference_risky": None if rmse is None else rmse > threshold_m,
    }


def _commands_by_id(commands_csv: Optional[Path]) -> Dict[str, Dict[str, str]]:
    if commands_csv is None or not commands_csv.exists():
        return {}
    return {str(row.get("command_id", "")): row for row in _read_csv(commands_csv)}


def _command_stamp(execution: Dict[str, str], command: Optional[Dict[str, str]]) -> Optional[float]:
    for row in [execution, command or {}]:
        for key in ["stamp_sec", "trigger_stamp_sec", "command_stamp_sec"]:
            value = _as_float(row.get(key))
            if value is not None:
                return value
    return None


def _stable_ok_streak(rows: List[Dict[str, Any]]) -> int:
    max_streak = 0
    current = 0
    for row in rows:
        if row["message"] == "ok":
            current += 1
            max_streak = max(max_streak, current)
        else:
            current = 0
    return max_streak


def _first_ok_latency(rows: List[Dict[str, Any]], start_stamp: float) -> Optional[float]:
    for row in rows:
        if row["message"] == "ok":
            return row["stamp_sec"] - start_stamp
    return None


def observe_execution(
    execution_rows: List[Dict[str, str]],
    alignment_rows: List[Dict[str, Any]],
    commands_by_id: Dict[str, Dict[str, str]],
    trajectory_risk: Dict[str, Any],
    window_sec: float,
    stable_ok_rows_required: int,
) -> List[Dict[str, str]]:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    observations: List[Dict[str, str]] = []
    for execution in execution_rows:
        command_id = str(execution.get("command_id", ""))
        command = commands_by_id.get(command_id)
        published_count = _as_int(execution.get("published_count")) or 0
        execution_status = str(execution.get("status", ""))
        command_stamp = _command_stamp(execution, command)
        base = {
            "command_id": command_id,
            "attempt_id": str(execution.get("attempt_id", "")),
            "published_count": str(published_count),
            "execution_status": execution_status,
            "command_stamp_sec": _fmt(command_stamp),
            "observation_window_sec": f"{window_sec:.9f}",
            "generated_at": generated_at,
        }
        if published_count <= 0:
            observations.append(
                {
                    **base,
                    "alignment_rows_in_window": "0",
                    "ok_rows_in_window": "0",
                    "failure_like_rows_in_window": "0",
                    "first_ok_latency_sec": "",
                    "stable_ok_rows_after_reset": "0",
                    "max_reject_streak_after_reset": "0",
                    "observed_recovered": "false",
                    "accepted": "false",
                    "false_recovery_observable": "false",
                    "reference_risky": "",
                    "rejection_reason": "reset_command_not_published",
                }
            )
            continue
        if command_stamp is None:
            observations.append(
                {
                    **base,
                    "alignment_rows_in_window": "0",
                    "ok_rows_in_window": "0",
                    "failure_like_rows_in_window": "0",
                    "first_ok_latency_sec": "",
                    "stable_ok_rows_after_reset": "0",
                    "max_reject_streak_after_reset": "0",
                    "observed_recovered": "false",
                    "accepted": "false",
                    "false_recovery_observable": "false",
                    "reference_risky": "",
                    "rejection_reason": "command_stamp_missing",
                }
            )
            continue

        deadline = command_stamp + window_sec
        window_rows = [
            row for row in alignment_rows if command_stamp <= row["stamp_sec"] <= deadline
        ]
        ok_rows = [row for row in window_rows if row["message"] == "ok"]
        failure_like_rows = [row for row in window_rows if row["failure_like"]]
        stable_ok = _stable_ok_streak(window_rows)
        recovered = stable_ok >= stable_ok_rows_required
        reference_risky = trajectory_risk.get("reference_risky")
        observations.append(
            {
                **base,
                "alignment_rows_in_window": str(len(window_rows)),
                "ok_rows_in_window": str(len(ok_rows)),
                "failure_like_rows_in_window": str(len(failure_like_rows)),
                "first_ok_latency_sec": _fmt(_first_ok_latency(window_rows, command_stamp)),
                "stable_ok_rows_after_reset": str(stable_ok),
                "max_reject_streak_after_reset": str(
                    max(
                        [int(row["consecutive_rejected_updates"]) for row in window_rows],
                        default=0,
                    )
                ),
                "observed_recovered": "true" if recovered else "false",
                "accepted": "true" if recovered else "false",
                "false_recovery_observable": "true" if trajectory_risk["available"] else "false",
                "reference_risky": ""
                if reference_risky is None
                else ("true" if reference_risky else "false"),
                "rejection_reason": "" if recovered else "stable_ok_rows_below_threshold",
            }
        )
    return observations


def write_csv(path: Path, rows: List[Dict[str, str]], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OBSERVATION_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def build_summary(
    execution_csv: Path,
    alignment_csv: Path,
    output_csv: Path,
    observations: List[Dict[str, str]],
    trajectory_risk: Dict[str, Any],
) -> Dict[str, Any]:
    recovered_rows = [row for row in observations if _as_bool(row.get("observed_recovered"))]
    accepted_rows = [row for row in observations if _as_bool(row.get("accepted"))]
    return {
        "execution_csv": str(execution_csv),
        "alignment_csv": str(alignment_csv),
        "output_csv": str(output_csv),
        "command_count": len(observations),
        "observed_recovered_count": len(recovered_rows),
        "accepted_count": len(accepted_rows),
        "published_command_count": len(
            [row for row in observations if (_as_int(row.get("published_count")) or 0) > 0]
        ),
        "rejection_reason_counts": _counts(row.get("rejection_reason", "") for row in observations),
        "execution_status_counts": _counts(row.get("execution_status", "") for row in observations),
        "trajectory_risk": trajectory_risk,
        "notes": [
            "accepted=true here means offline post-reset observation met stable-ok criteria",
            "unpublished commands are not evaluated for recovery",
            "false recovery is only observable when trajectory_eval_json is provided",
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
        "# Relocalization Reset Execution Observation",
        "",
        f"- execution CSV: `{data['execution_csv']}`",
        f"- alignment CSV: `{data['alignment_csv']}`",
        f"- output CSV: `{data['output_csv']}`",
        f"- commands: `{data['command_count']}`",
        f"- published commands: `{data['published_command_count']}`",
        f"- observed recovered: `{data['observed_recovered_count']}`",
        f"- accepted: `{data['accepted_count']}`",
        f"- rejection reasons: `{json.dumps(data['rejection_reason_counts'], sort_keys=True)}`",
        f"- execution statuses: `{json.dumps(data['execution_status_counts'], sort_keys=True)}`",
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
    execution_csv = Path(args.execution_csv).expanduser().resolve()
    alignment_csv = Path(args.alignment_csv).expanduser().resolve()
    commands_csv = Path(args.commands_csv).expanduser().resolve() if args.commands_csv else None
    trajectory_eval = (
        Path(args.trajectory_eval_json).expanduser().resolve()
        if args.trajectory_eval_json
        else None
    )
    output_csv = Path(args.output_csv).expanduser().resolve()
    if not execution_csv.exists():
        raise FileNotFoundError(f"execution CSV does not exist: {execution_csv}")
    if not alignment_csv.exists():
        raise FileNotFoundError(f"alignment CSV does not exist: {alignment_csv}")
    if commands_csv is not None and not commands_csv.exists():
        raise FileNotFoundError(f"commands CSV does not exist: {commands_csv}")
    if trajectory_eval is not None and not trajectory_eval.exists():
        raise FileNotFoundError(f"trajectory eval JSON does not exist: {trajectory_eval}")

    execution_rows = _read_csv(execution_csv)
    alignment_rows = load_alignment_rows(alignment_csv)
    command_rows = _commands_by_id(commands_csv)
    trajectory_risk = load_trajectory_risk(trajectory_eval, args.false_recovery_rmse_threshold_m)
    observations = observe_execution(
        execution_rows=execution_rows,
        alignment_rows=alignment_rows,
        commands_by_id=command_rows,
        trajectory_risk=trajectory_risk,
        window_sec=args.post_reset_window_sec,
        stable_ok_rows_required=args.stable_ok_rows,
    )
    write_csv(output_csv, observations, args.overwrite)
    summary = build_summary(
        execution_csv=execution_csv,
        alignment_csv=alignment_csv,
        output_csv=output_csv,
        observations=observations,
        trajectory_risk=trajectory_risk,
    )
    if args.output_json:
        write_json(Path(args.output_json).expanduser().resolve(), summary, args.overwrite)
    if args.output_md:
        write_md(Path(args.output_md).expanduser().resolve(), summary, args.overwrite)
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
