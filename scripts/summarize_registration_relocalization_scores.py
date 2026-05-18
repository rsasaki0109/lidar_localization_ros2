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
            "Summarize relocalization registration score artifacts. This helper "
            "does not accept or reset poses; it reports scorer coverage and gate "
            "diagnostics from relocalization_registration_scores*.csv."
        )
    )
    parser.add_argument("--scores-csv", required=True, help="Input registration scores CSV")
    parser.add_argument("--output-json", default="", help="Optional output JSON path")
    parser.add_argument("--output-md", default="", help="Optional output Markdown path")
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


def _percent(numerator: int, denominator: int) -> Optional[float]:
    if denominator <= 0:
        return None
    return 100.0 * float(numerator) / float(denominator)


def _stats(values: Iterable[Optional[float]]) -> Dict[str, Optional[float]]:
    finite_values = [value for value in values if value is not None]
    if not finite_values:
        return {"min": None, "median": None, "max": None}
    return {
        "min": min(finite_values),
        "median": float(median(finite_values)),
        "max": max(finite_values),
    }


def _fmt(value: Any) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def load_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append(
                {
                    "job_id": row.get("job_id", ""),
                    "attempt_id": row.get("attempt_id", ""),
                    "candidate_index": row.get("candidate_index", ""),
                    "status": row.get("status", ""),
                    "rejection_reason": row.get("rejection_reason", ""),
                    "registration_method": row.get("registration_method", ""),
                    "converged": _as_bool(row.get("converged")),
                    "registration_gate_passed": _as_bool(
                        row.get("registration_gate_passed")
                    ),
                    "registration_gate_reason": row.get("registration_gate_reason", ""),
                    "score": _as_float(row.get("score")),
                    "overlap": _as_float(row.get("overlap")),
                    "refinement_delta_m": _as_float(row.get("refinement_delta_m")),
                    "refinement_delta_yaw_rad": _as_float(
                        row.get("refinement_delta_yaw_rad")
                    ),
                    "runtime_sec": _as_float(row.get("runtime_sec")),
                    "scan_time_delta_sec": _as_float(row.get("scan_time_delta_sec")),
                    "scan_point_count_valid": _as_float(row.get("scan_point_count_valid")),
                    "source_point_count": _as_float(row.get("source_point_count")),
                    "target_point_count": _as_float(row.get("target_point_count")),
                }
            )
    return rows


def build_summary(scores_csv: Path) -> Dict[str, Any]:
    rows = load_rows(scores_csv)
    scored_rows = [row for row in rows if row["status"] == "registration_scored_no_reset"]
    scan_resolved_rows = [
        row for row in rows if row["status"] == "scan_resolved_no_registration"
    ]
    converged_rows = [row for row in rows if row["converged"] is True]
    gate_passed_rows = [row for row in rows if row["registration_gate_passed"] is True]
    gate_failed_rows = [row for row in rows if row["registration_gate_passed"] is False]

    attempt_ids = {row["attempt_id"] for row in rows if row["attempt_id"]}
    scored_attempt_ids = {row["attempt_id"] for row in scored_rows if row["attempt_id"]}
    gate_passed_attempt_ids = {
        row["attempt_id"] for row in gate_passed_rows if row["attempt_id"]
    }

    return {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "scores_csv": str(scores_csv),
        "rows": {
            "count": len(rows),
            "scored_count": len(scored_rows),
            "scan_resolved_unscored_count": len(scan_resolved_rows),
            "converged_count": len(converged_rows),
            "gate_passed_count": len(gate_passed_rows),
            "gate_failed_count": len(gate_failed_rows),
            "scored_rate_percent": _percent(len(scored_rows), len(rows)),
            "gate_passed_rate_percent": _percent(len(gate_passed_rows), len(rows)),
        },
        "attempts": {
            "count": len(attempt_ids),
            "scored_count": len(scored_attempt_ids),
            "gate_passed_count": len(gate_passed_attempt_ids),
        },
        "counts": {
            "status": dict(Counter(row["status"] or "unknown" for row in rows)),
            "rejection_reason": dict(
                Counter(row["rejection_reason"] or "unknown" for row in rows)
            ),
            "registration_method": dict(
                Counter(row["registration_method"] or "unknown" for row in rows)
            ),
            "registration_gate_reason": dict(
                Counter(row["registration_gate_reason"] or "unknown" for row in rows)
            ),
        },
        "stats": {
            "score": _stats(row["score"] for row in rows),
            "overlap": _stats(row["overlap"] for row in rows),
            "refinement_delta_m": _stats(row["refinement_delta_m"] for row in rows),
            "refinement_delta_yaw_rad": _stats(
                row["refinement_delta_yaw_rad"] for row in rows
            ),
            "runtime_sec": _stats(row["runtime_sec"] for row in rows),
            "scan_time_delta_sec_abs": _stats(
                abs(row["scan_time_delta_sec"])
                if row["scan_time_delta_sec"] is not None
                else None
                for row in rows
            ),
            "scan_point_count_valid": _stats(row["scan_point_count_valid"] for row in rows),
            "source_point_count": _stats(row["source_point_count"] for row in rows),
            "target_point_count": _stats(row["target_point_count"] for row in rows),
        },
        "scored_stats": {
            "score": _stats(row["score"] for row in scored_rows),
            "overlap": _stats(row["overlap"] for row in scored_rows),
            "refinement_delta_m": _stats(
                row["refinement_delta_m"] for row in scored_rows
            ),
            "refinement_delta_yaw_rad": _stats(
                row["refinement_delta_yaw_rad"] for row in scored_rows
            ),
            "runtime_sec": _stats(row["runtime_sec"] for row in scored_rows),
            "source_point_count": _stats(
                row["source_point_count"] for row in scored_rows
            ),
            "target_point_count": _stats(
                row["target_point_count"] for row in scored_rows
            ),
        },
        "notes": [
            "registration gate pass does not mean pose reset was accepted",
            "rows marked registration_scored_no_reset are scorer artifacts only",
            "scan_resolved_no_registration rows have scan inputs but no backend score yet",
        ],
    }


def build_markdown(summary: Dict[str, Any]) -> str:
    rows = summary["rows"]
    attempts = summary["attempts"]
    counts = summary["counts"]
    stats = summary["stats"]
    scored_stats = summary["scored_stats"]
    lines = [
        "# Registration Relocalization Score Summary",
        "",
        f"Generated at: {summary['generated_at']}",
        "",
        f"- scores_csv: `{summary['scores_csv']}`",
        "",
        "## Coverage",
        "",
        f"- rows: `{rows['count']}`",
        f"- scored rows: `{rows['scored_count']}` (`{_fmt(rows['scored_rate_percent'])}%`)",
        f"- scan-resolved unscored rows: `{rows['scan_resolved_unscored_count']}`",
        f"- converged rows: `{rows['converged_count']}`",
        f"- gate-passed rows: `{rows['gate_passed_count']}` (`{_fmt(rows['gate_passed_rate_percent'])}%`)",
        f"- gate-failed rows: `{rows['gate_failed_count']}`",
        f"- attempts: `{attempts['count']}`",
        f"- attempts with scored rows: `{attempts['scored_count']}`",
        f"- attempts with gate-passed rows: `{attempts['gate_passed_count']}`",
        "",
        "## Counts",
        "",
        f"- status: `{json.dumps(counts['status'], sort_keys=True)}`",
        f"- rejection_reason: `{json.dumps(counts['rejection_reason'], sort_keys=True)}`",
        f"- registration_gate_reason: `{json.dumps(counts['registration_gate_reason'], sort_keys=True)}`",
        "",
        "## Stats",
        "",
        f"- score: `{json.dumps(stats['score'], sort_keys=True)}`",
        f"- refinement_delta_m: `{json.dumps(stats['refinement_delta_m'], sort_keys=True)}`",
        f"- refinement_delta_yaw_rad: `{json.dumps(stats['refinement_delta_yaw_rad'], sort_keys=True)}`",
        f"- runtime_sec: `{json.dumps(stats['runtime_sec'], sort_keys=True)}`",
        f"- source_point_count: `{json.dumps(stats['source_point_count'], sort_keys=True)}`",
        f"- target_point_count: `{json.dumps(stats['target_point_count'], sort_keys=True)}`",
        "",
        "## Scored-Only Stats",
        "",
        f"- score: `{json.dumps(scored_stats['score'], sort_keys=True)}`",
        f"- refinement_delta_m: `{json.dumps(scored_stats['refinement_delta_m'], sort_keys=True)}`",
        f"- refinement_delta_yaw_rad: `{json.dumps(scored_stats['refinement_delta_yaw_rad'], sort_keys=True)}`",
        f"- runtime_sec: `{json.dumps(scored_stats['runtime_sec'], sort_keys=True)}`",
        "",
        "## Caveat",
        "",
        "- A passed registration gate is not an accepted reset. Reset acceptance remains disabled.",
    ]
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    scores_csv = Path(args.scores_csv).expanduser().resolve()
    if not scores_csv.exists():
        raise FileNotFoundError(f"scores CSV does not exist: {scores_csv}")

    summary = build_summary(scores_csv)
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
