#!/usr/bin/env python3

import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare registration relocalization score summaries, for example "
            "candidate_index, route_proximity, and oracle_rank diagnostic order."
        )
    )
    parser.add_argument(
        "--summary",
        action="append",
        required=True,
        metavar="LABEL:PATH",
        help="Summary JSON with a label, e.g. candidate_index:path/to/summary.json",
    )
    parser.add_argument("--output-json", default="", help="Optional comparison JSON")
    parser.add_argument("--output-md", default="", help="Optional comparison Markdown")
    return parser.parse_args()


def _fmt(value: Any) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def parse_summary_arg(raw: str) -> Tuple[str, Path]:
    if ":" not in raw:
        raise ValueError(f"expected LABEL:PATH, got {raw!r}")
    label, path = raw.split(":", 1)
    if not label:
        raise ValueError(f"summary label is empty in {raw!r}")
    return label, Path(path).expanduser().resolve()


def load_summary(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"summary JSON does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def stat_value(summary: Dict[str, Any], group: str, metric: str, key: str) -> Optional[float]:
    value = summary.get(group, {}).get(metric, {}).get(key)
    return value if isinstance(value, (int, float)) else None


def scored_stat_value(summary: Dict[str, Any], metric: str, key: str) -> Optional[float]:
    value = stat_value(summary, "scored_stats", metric, key)
    if value is not None:
        return value
    return stat_value(summary, "stats", metric, key)


def compact_row(label: str, path: Path, summary: Dict[str, Any]) -> Dict[str, Any]:
    rows = summary.get("rows", {})
    attempts = summary.get("attempts", {})
    return {
        "label": label,
        "summary_json": str(path),
        "scores_csv": summary.get("scores_csv"),
        "row_count": rows.get("count"),
        "scored_count": rows.get("scored_count"),
        "scored_rate_percent": rows.get("scored_rate_percent"),
        "converged_count": rows.get("converged_count"),
        "gate_passed_count": rows.get("gate_passed_count"),
        "gate_passed_rate_percent": rows.get("gate_passed_rate_percent"),
        "attempt_count": attempts.get("count"),
        "attempt_gate_passed_count": attempts.get("gate_passed_count"),
        "score_median": scored_stat_value(summary, "score", "median"),
        "score_min": scored_stat_value(summary, "score", "min"),
        "score_max": scored_stat_value(summary, "score", "max"),
        "refinement_delta_m_median": scored_stat_value(
            summary, "refinement_delta_m", "median"
        ),
        "runtime_sec_median": scored_stat_value(summary, "runtime_sec", "median"),
        "runtime_sec_max": scored_stat_value(summary, "runtime_sec", "max"),
        "status_counts": summary.get("counts", {}).get("status", {}),
        "gate_reason_counts": summary.get("counts", {}).get("registration_gate_reason", {}),
    }


def build_comparison(summary_args: List[str]) -> Dict[str, Any]:
    rows = []
    for raw in summary_args:
        label, path = parse_summary_arg(raw)
        rows.append(compact_row(label, path, load_summary(path)))
    best_by_gate = sorted(
        rows,
        key=lambda row: (
            row["gate_passed_count"] or 0,
            row["gate_passed_rate_percent"] or 0.0,
            -(row["score_median"] or float("inf")),
        ),
        reverse=True,
    )
    return {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "rows": rows,
        "best_by_gate_passed": best_by_gate[0]["label"] if best_by_gate else None,
        "notes": [
            "oracle_rank comparisons are diagnostic upper bounds, not runtime candidate selection claims",
            "registration gate pass does not mean pose reset was accepted",
        ],
    }


def build_markdown(comparison: Dict[str, Any]) -> str:
    lines = [
        "# Registration Relocalization Score Comparison",
        "",
        f"Generated at: {comparison['generated_at']}",
        "",
        f"- best_by_gate_passed: `{comparison['best_by_gate_passed']}`",
        "",
        "| Label | Rows | Scored | Gate Passed | Score Median | Runtime Median [s] | Status Counts |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in comparison["rows"]:
        lines.append(
            "| "
            f"{row['label']} | "
            f"{_fmt(row['row_count'])} | "
            f"{_fmt(row['scored_count'])} ({_fmt(row['scored_rate_percent'])}%) | "
            f"{_fmt(row['gate_passed_count'])} ({_fmt(row['gate_passed_rate_percent'])}%) | "
            f"{_fmt(row['score_median'])} | "
            f"{_fmt(row['runtime_sec_median'])} | "
            f"`{json.dumps(row['status_counts'], sort_keys=True)}` |"
        )
    lines.extend(
        [
            "",
            "## Caveat",
            "",
            "- `oracle_rank` ordering is an offline upper-bound diagnostic. It must not be described as runtime relocalization behavior.",
            "- A passed registration gate is not an accepted reset.",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    comparison = build_comparison(args.summary)
    if args.output_json:
        output_json = Path(args.output_json).expanduser().resolve()
        output_json.parent.mkdir(parents=True, exist_ok=True)
        output_json.write_text(
            json.dumps(comparison, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.output_md:
        output_md = Path(args.output_md).expanduser().resolve()
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_md.write_text(build_markdown(comparison) + "\n", encoding="utf-8")
    if not args.output_json and not args.output_md:
        print(json.dumps(comparison, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
