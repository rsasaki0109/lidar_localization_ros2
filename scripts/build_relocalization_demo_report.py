#!/usr/bin/env python3

import argparse
import csv
import html
import json
import math
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build a self-contained HTML relocalization demo report from offline artifacts. "
            "The report never publishes /initialpose; it visualizes candidate, score, dry-run, "
            "execution, and post-reset observation artifacts when they exist."
        )
    )
    parser.add_argument("--artifact-dir", required=True)
    parser.add_argument("--output-html", required=True)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--title", default="Relocalization Demo Report")
    parser.add_argument("--max-score-bars", type=int, default=16)
    parser.add_argument("--max-timeline-rows", type=int, default=600)
    return parser.parse_args()


def _read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _read_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _as_bool(value: Any) -> Optional[bool]:
    if value is None or str(value).strip() == "":
        return None
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "y"}:
        return True
    if normalized in {"0", "false", "no", "n"}:
        return False
    return None


def _counts(values: Iterable[str]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value or "unknown")
        counts[key] = counts.get(key, 0) + 1
    return counts


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def _h(value: Any) -> str:
    return html.escape(str(value), quote=True)


def _artifact_path(artifact_dir: Path, filename: str) -> Path:
    return artifact_dir / filename


def _first_existing(artifact_dir: Path, filenames: List[str]) -> Path:
    for filename in filenames:
        path = artifact_dir / filename
        if path.exists():
            return path
    return artifact_dir / filenames[0]


def _xy(row: Dict[str, str], x_key: str, y_key: str) -> Optional[Tuple[float, float]]:
    x = _as_float(row.get(x_key))
    y = _as_float(row.get(y_key))
    if x is None or y is None:
        return None
    return (x, y)


def load_artifacts(artifact_dir: Path) -> Dict[str, Any]:
    score_path = _first_existing(
        artifact_dir,
        [
            "relocalization_registration_scores_ndt_candidate_index_top5.csv",
            "relocalization_registration_scores_ndt.csv",
            "relocalization_registration_scores.csv",
        ],
    )
    return {
        "artifact_dir": artifact_dir,
        "paths": {
            "candidates": _artifact_path(artifact_dir, "relocalization_candidates.csv"),
            "scores": score_path,
            "plan": _artifact_path(artifact_dir, "relocalization_reset_candidate_plan.csv"),
            "commands": _artifact_path(artifact_dir, "relocalization_reset_commands.csv"),
            "execution": _artifact_path(artifact_dir, "relocalization_reset_command_execution.csv"),
            "observation": _artifact_path(
                artifact_dir, "relocalization_reset_execution_observation.csv"
            ),
            "health": _artifact_path(artifact_dir, "health_summary.json"),
            "attempt_summary": _artifact_path(artifact_dir, "relocalization_attempt_summary.json"),
            "alignment": _artifact_path(artifact_dir, "alignment_status.csv"),
        },
        "candidates": _read_csv(_artifact_path(artifact_dir, "relocalization_candidates.csv")),
        "scores": _read_csv(score_path),
        "plan": _read_csv(_artifact_path(artifact_dir, "relocalization_reset_candidate_plan.csv")),
        "commands": _read_csv(_artifact_path(artifact_dir, "relocalization_reset_commands.csv")),
        "execution": _read_csv(
            _artifact_path(artifact_dir, "relocalization_reset_command_execution.csv")
        ),
        "observation": _read_csv(
            _artifact_path(artifact_dir, "relocalization_reset_execution_observation.csv")
        ),
        "health": _read_json(_artifact_path(artifact_dir, "health_summary.json")),
        "attempt_summary": _read_json(
            _artifact_path(artifact_dir, "relocalization_attempt_summary.json")
        ),
        "alignment": _read_csv(_artifact_path(artifact_dir, "alignment_status.csv")),
    }


def best_score_row(scores: List[Dict[str, str]]) -> Optional[Dict[str, str]]:
    eligible = [
        row
        for row in scores
        if _as_float(row.get("score")) is not None
        and (_as_bool(row.get("registration_gate_passed")) is not False)
    ]
    if not eligible:
        eligible = [row for row in scores if _as_float(row.get("score")) is not None]
    if not eligible:
        return None
    return min(eligible, key=lambda row: _as_float(row.get("score")) or float("inf"))


def selected_candidate_indexes(plan: List[Dict[str, str]], scores: List[Dict[str, str]]) -> set:
    selected = {
        str(row.get("selected_candidate_index", ""))
        for row in plan
        if _as_bool(row.get("selected")) is True
    }
    selected.discard("")
    if selected:
        return selected
    best = best_score_row(scores)
    if best is None:
        return set()
    return {str(best.get("candidate_index", ""))}


def score_by_candidate(scores: List[Dict[str, str]]) -> Dict[str, Dict[str, str]]:
    out: Dict[str, Dict[str, str]] = {}
    for row in scores:
        key = str(row.get("candidate_index", ""))
        if not key:
            continue
        score = _as_float(row.get("score"))
        current = out.get(key)
        if current is None or (score or float("inf")) < (
            _as_float(current.get("score")) or float("inf")
        ):
            out[key] = row
    return out


def summarize(data: Dict[str, Any]) -> Dict[str, Any]:
    candidates = data["candidates"]
    scores = data["scores"]
    plan = data["plan"]
    commands = data["commands"]
    execution = data["execution"]
    observation = data["observation"]
    best = best_score_row(scores)
    published_count = sum(int(float(row.get("published_count", "0") or 0)) for row in execution)
    return {
        "artifact_dir": str(data["artifact_dir"]),
        "candidate_count": len(candidates),
        "attempt_count": len({row.get("attempt_id", "") for row in candidates if row.get("attempt_id")}),
        "scored_count": len([row for row in scores if _as_float(row.get("score")) is not None]),
        "gate_passed_count": len(
            [row for row in scores if _as_bool(row.get("registration_gate_passed")) is True]
        ),
        "selected_count": len([row for row in plan if _as_bool(row.get("selected")) is True]),
        "dry_run_command_count": len(
            [row for row in commands if row.get("status") == "dry_run_command_generated"]
        ),
        "execution_row_count": len(execution),
        "published_count": published_count,
        "observation_count": len(observation),
        "observed_recovered_count": len(
            [row for row in observation if _as_bool(row.get("observed_recovered")) is True]
        ),
        "false_recovery_count": len(
            [row for row in observation if _as_bool(row.get("false_recovery_observable")) is True]
        ),
        "best_candidate_index": None if best is None else best.get("candidate_index"),
        "best_score": None if best is None else _as_float(best.get("score")),
        "best_gate_reason": None if best is None else best.get("registration_gate_reason"),
        "score_status_counts": _counts(row.get("status", "") for row in scores),
        "execution_status_counts": _counts(row.get("status", "") for row in execution),
        "observation_rejection_counts": _counts(row.get("rejection_reason", "") for row in observation),
    }


def card_grid(summary: Dict[str, Any]) -> str:
    cards = [
        ("candidates", summary["candidate_count"]),
        ("scored", summary["scored_count"]),
        ("gate passed", summary["gate_passed_count"]),
        ("selected", summary["selected_count"]),
        ("dry-run commands", summary["dry_run_command_count"]),
        ("published", summary["published_count"]),
        ("observed recovered", summary["observed_recovered_count"]),
        ("false recovery", summary["false_recovery_count"]),
    ]
    return "".join(
        f'<div class="metric"><div class="metric-value">{_h(value)}</div>'
        f'<div class="metric-label">{_h(label)}</div></div>'
        for label, value in cards
    )


def stage_table(data: Dict[str, Any]) -> str:
    rows = [
        ("Health summary", bool(data["health"]), "health_summary.json"),
        ("Candidate generation", bool(data["candidates"]), "relocalization_candidates.csv"),
        ("NDT scoring", bool(data["scores"]), Path(data["paths"]["scores"]).name),
        ("Reset plan", bool(data["plan"]), "relocalization_reset_candidate_plan.csv"),
        ("Dry-run command", bool(data["commands"]), "relocalization_reset_commands.csv"),
        ("Guarded execution", bool(data["execution"]), "relocalization_reset_command_execution.csv"),
        ("Post-reset observation", bool(data["observation"]), "relocalization_reset_execution_observation.csv"),
    ]
    body = []
    for stage, present, artifact in rows:
        badge = "present" if present else "missing"
        body.append(
            "<tr>"
            f"<td>{_h(stage)}</td>"
            f"<td><span class=\"badge {'ok' if present else 'missing'}\">{badge}</span></td>"
            f"<td><code>{_h(artifact)}</code></td>"
            "</tr>"
        )
    return "<table><thead><tr><th>Stage</th><th>Status</th><th>Artifact</th></tr></thead><tbody>" + "".join(body) + "</tbody></table>"


def _bounds(points: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if math.isclose(min_x, max_x):
        min_x -= 1.0
        max_x += 1.0
    if math.isclose(min_y, max_y):
        min_y -= 1.0
        max_y += 1.0
    pad_x = (max_x - min_x) * 0.12
    pad_y = (max_y - min_y) * 0.12
    return min_x - pad_x, max_x + pad_x, min_y - pad_y, max_y + pad_y


def map_svg(data: Dict[str, Any]) -> str:
    candidates = data["candidates"]
    scores = score_by_candidate(data["scores"])
    selected = selected_candidate_indexes(data["plan"], data["scores"])
    commands = data["commands"]
    points = []
    for row in candidates:
        xy = _xy(row, "pose_x", "pose_y")
        if xy:
            points.append(xy)
    for row in commands:
        xy = _xy(row, "position_x", "position_y")
        if xy:
            points.append(xy)
    if not points:
        return '<div class="empty">No candidate or command poses available.</div>'

    width, height = 820, 430
    min_x, max_x, min_y, max_y = _bounds(points)

    def sx(x: float) -> float:
        return 40.0 + (x - min_x) / (max_x - min_x) * (width - 80.0)

    def sy(y: float) -> float:
        return height - 35.0 - (y - min_y) / (max_y - min_y) * (height - 70.0)

    elements = [
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="candidate map">',
        '<rect x="0" y="0" width="820" height="430" rx="6" fill="#f8fafc"/>',
        '<line x1="40" y1="395" x2="780" y2="395" stroke="#cbd5e1"/>',
        '<line x1="40" y1="35" x2="40" y2="395" stroke="#cbd5e1"/>',
    ]
    for row in candidates:
        xy = _xy(row, "pose_x", "pose_y")
        if xy is None:
            continue
        idx = str(row.get("candidate_index", ""))
        score_row = scores.get(idx, {})
        score = _as_float(score_row.get("score"))
        gate = _as_bool(score_row.get("registration_gate_passed"))
        fill = "#64748b"
        if gate is True:
            fill = "#16a34a"
        elif gate is False:
            fill = "#dc2626"
        stroke = "#111827" if idx in selected else "#ffffff"
        radius = 9 if idx in selected else 6
        title = f"candidate {idx}, score {_fmt(score)}, gate {score_row.get('registration_gate_reason', 'n/a')}"
        elements.append(
            f'<circle cx="{sx(xy[0]):.1f}" cy="{sy(xy[1]):.1f}" r="{radius}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="2"><title>{_h(title)}</title></circle>'
        )
        if idx in selected:
            elements.append(
                f'<text x="{sx(xy[0]) + 11:.1f}" y="{sy(xy[1]) - 8:.1f}" '
                'font-size="12" fill="#111827">selected</text>'
            )
    for row in commands:
        xy = _xy(row, "position_x", "position_y")
        if xy is None:
            continue
        x, y = sx(xy[0]), sy(xy[1])
        elements.append(
            f'<path d="M{x:.1f},{y - 11:.1f} L{x + 3:.1f},{y - 3:.1f} '
            f'L{x + 11:.1f},{y:.1f} L{x + 3:.1f},{y + 3:.1f} L{x:.1f},{y + 11:.1f} '
            f'L{x - 3:.1f},{y + 3:.1f} L{x - 11:.1f},{y:.1f} L{x - 3:.1f},{y - 3:.1f} Z" '
            'fill="#2563eb" stroke="#1e3a8a"><title>dry-run /initialpose command</title></path>'
        )
    elements.extend(
        [
            f'<text x="40" y="420" font-size="11" fill="#475569">x {_fmt(min_x)} to {_fmt(max_x)} m</text>',
            f'<text x="670" y="420" font-size="11" fill="#475569">y {_fmt(min_y)} to {_fmt(max_y)} m</text>',
            "</svg>",
        ]
    )
    return "".join(elements)


def score_bars(data: Dict[str, Any], max_rows: int) -> str:
    rows = [row for row in data["scores"] if _as_float(row.get("score")) is not None]
    rows.sort(key=lambda row: _as_float(row.get("score")) or float("inf"))
    rows = rows[: max(1, max_rows)]
    if not rows:
        return '<div class="empty">No scored candidates available.</div>'
    max_score = max(_as_float(row.get("score")) or 0.0 for row in rows) or 1.0
    selected = selected_candidate_indexes(data["plan"], data["scores"])
    bars = []
    for row in rows:
        idx = str(row.get("candidate_index", ""))
        score = _as_float(row.get("score")) or 0.0
        width = max(2.0, score / max_score * 100.0)
        gate = _as_bool(row.get("registration_gate_passed"))
        cls = "gate-pass" if gate is True else "gate-fail" if gate is False else "gate-unknown"
        marker = " selected" if idx in selected else ""
        bars.append(
            f'<div class="bar-row{marker}"><div class="bar-label">#{_h(idx)}</div>'
            f'<div class="bar-track"><div class="bar {cls}" style="width:{width:.1f}%"></div></div>'
            f'<div class="bar-value">{_fmt(score)}</div></div>'
        )
    return "".join(bars)


def alignment_events(alignment_rows: List[Dict[str, str]], max_rows: int) -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    for row in alignment_rows:
        stamp = _as_float(row.get("stamp_sec"))
        if stamp is None:
            continue
        values: Dict[str, Any] = {}
        try:
            values = json.loads(row.get("values_json", "{}"))
        except json.JSONDecodeError:
            pass
        requested = _as_bool(values.get("reinitialization_requested")) is True
        message = str(row.get("message", ""))
        state = str(values.get("recovery_state", ""))
        score = _as_float(values.get("fitness_score"))
        kind = "ok"
        if requested:
            kind = "request"
        elif message != "ok" or state in {"degraded", "recovering", "reinitialization_requested"}:
            kind = "failure"
        events.append({"stamp": stamp, "kind": kind, "message": message, "score": score})
    if len(events) <= max_rows:
        return events
    step = max(1, math.ceil(len(events) / max_rows))
    return events[::step]


def timeline_svg(data: Dict[str, Any], max_rows: int) -> str:
    events = alignment_events(data["alignment"], max_rows)
    stamps = [event["stamp"] for event in events]
    for row in data["scores"]:
        stamp = _as_float(row.get("trigger_stamp_sec"))
        if stamp is not None:
            stamps.append(stamp)
    for row in data["commands"]:
        stamp = _as_float(row.get("stamp_sec")) or _as_float(row.get("trigger_stamp_sec"))
        if stamp is not None:
            stamps.append(stamp)
    if not stamps:
        return '<div class="empty">No timeline stamps available.</div>'
    min_t, max_t = min(stamps), max(stamps)
    if math.isclose(min_t, max_t):
        max_t += 1.0
    width, height = 820, 170

    def sx(t: float) -> float:
        return 40.0 + (t - min_t) / (max_t - min_t) * (width - 80.0)

    lanes = {"ok": 105.0, "request": 75.0, "failure": 45.0}
    colors = {"ok": "#16a34a", "request": "#f59e0b", "failure": "#dc2626"}
    elements = [
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="health timeline">',
        '<rect x="0" y="0" width="820" height="170" rx="6" fill="#f8fafc"/>',
    ]
    for label, y in [("failure", 45), ("request", 75), ("ok", 105)]:
        elements.append(f'<line x1="40" y1="{y}" x2="780" y2="{y}" stroke="#e2e8f0"/>')
        elements.append(f'<text x="8" y="{y + 4}" font-size="11" fill="#475569">{label}</text>')
    for event in events:
        x = sx(event["stamp"])
        y = lanes[event["kind"]]
        title = f'{event["kind"]}: {event["message"]}, score {_fmt(event["score"])}'
        elements.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.4" fill="{colors[event["kind"]]}">'
            f"<title>{_h(title)}</title></circle>"
        )
    for row in data["scores"]:
        stamp = _as_float(row.get("trigger_stamp_sec"))
        if stamp is None:
            continue
        x = sx(stamp)
        elements.append(
            f'<line x1="{x:.1f}" y1="22" x2="{x:.1f}" y2="130" stroke="#7c3aed" '
            'stroke-width="1.5" stroke-dasharray="4 4"><title>relocalization attempt trigger</title></line>'
        )
    for row in data["commands"]:
        stamp = _as_float(row.get("stamp_sec")) or _as_float(row.get("trigger_stamp_sec"))
        if stamp is None:
            continue
        x = sx(stamp)
        elements.append(
            f'<line x1="{x:.1f}" y1="18" x2="{x:.1f}" y2="138" stroke="#2563eb" '
            'stroke-width="2"><title>dry-run reset command</title></line>'
        )
    elements.extend(
        [
            f'<text x="40" y="154" font-size="11" fill="#475569">t0 {_fmt(min_t)} sec</text>',
            f'<text x="640" y="154" font-size="11" fill="#475569">duration {_fmt(max_t - min_t)} sec</text>',
            "</svg>",
        ]
    )
    return "".join(elements)


def details_table(title: str, rows: List[Dict[str, str]], columns: List[str], limit: int = 8) -> str:
    if not rows:
        return f"<h3>{_h(title)}</h3><div class=\"empty\">No rows available.</div>"
    header = "".join(f"<th>{_h(col)}</th>" for col in columns)
    body = []
    for row in rows[:limit]:
        body.append("<tr>" + "".join(f"<td>{_h(row.get(col, ''))}</td>" for col in columns) + "</tr>")
    return (
        f"<h3>{_h(title)}</h3><table><thead><tr>{header}</tr></thead>"
        f"<tbody>{''.join(body)}</tbody></table>"
    )


def build_html(data: Dict[str, Any], summary: Dict[str, Any], title: str, max_score_bars: int, max_timeline_rows: int) -> str:
    best_line = (
        f"best candidate #{_h(summary['best_candidate_index'])}, "
        f"score {_h(_fmt(summary['best_score']))}, gate {_h(summary['best_gate_reason'])}"
        if summary["best_candidate_index"] is not None
        else "no scored candidate"
    )
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{_h(title)}</title>
<style>
body {{ margin: 0; font: 14px/1.45 system-ui, -apple-system, Segoe UI, sans-serif; color: #111827; background: #f1f5f9; }}
main {{ max-width: 1180px; margin: 0 auto; padding: 24px; }}
h1 {{ font-size: 28px; margin: 0 0 4px; }}
h2 {{ font-size: 18px; margin: 24px 0 10px; }}
h3 {{ font-size: 15px; margin: 18px 0 8px; }}
.subtle {{ color: #64748b; }}
.panel {{ background: white; border: 1px solid #e2e8f0; border-radius: 8px; padding: 16px; margin-top: 14px; }}
.metrics {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; margin-top: 16px; }}
.metric {{ background: #0f172a; color: white; border-radius: 6px; padding: 12px; }}
.metric-value {{ font-size: 24px; font-weight: 700; }}
.metric-label {{ color: #cbd5e1; font-size: 12px; text-transform: uppercase; }}
table {{ border-collapse: collapse; width: 100%; font-size: 13px; }}
th, td {{ border-bottom: 1px solid #e2e8f0; padding: 7px 8px; text-align: left; vertical-align: top; }}
th {{ color: #475569; background: #f8fafc; }}
code {{ font-size: 12px; }}
.badge {{ display: inline-block; border-radius: 999px; padding: 2px 8px; font-size: 12px; font-weight: 700; }}
.badge.ok {{ background: #dcfce7; color: #166534; }}
.badge.missing {{ background: #fee2e2; color: #991b1b; }}
.grid2 {{ display: grid; grid-template-columns: minmax(0, 1.35fr) minmax(320px, .65fr); gap: 14px; }}
.empty {{ padding: 18px; border: 1px dashed #cbd5e1; border-radius: 6px; color: #64748b; background: #f8fafc; }}
.bar-row {{ display: grid; grid-template-columns: 52px 1fr 72px; gap: 8px; align-items: center; margin: 8px 0; }}
.bar-row.selected .bar-label {{ font-weight: 800; color: #7c2d12; }}
.bar-track {{ height: 14px; background: #e2e8f0; border-radius: 999px; overflow: hidden; }}
.bar {{ height: 14px; border-radius: 999px; }}
.gate-pass {{ background: #16a34a; }}
.gate-fail {{ background: #dc2626; }}
.gate-unknown {{ background: #64748b; }}
.bar-label, .bar-value {{ font-variant-numeric: tabular-nums; }}
.legend {{ display: flex; flex-wrap: wrap; gap: 12px; color: #475569; font-size: 12px; }}
.dot {{ display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 4px; vertical-align: -1px; }}
@media (max-width: 860px) {{ .grid2 {{ grid-template-columns: 1fr; }} main {{ padding: 14px; }} }}
</style>
</head>
<body>
<main>
<h1>{_h(title)}</h1>
<div class="subtle">artifact dir: <code>{_h(summary['artifact_dir'])}</code></div>
<div class="subtle">{best_line}</div>
<section class="metrics">{card_grid(summary)}</section>
<section class="panel">
<h2>Pipeline Coverage</h2>
{stage_table(data)}
</section>
<section class="grid2">
<div class="panel">
<h2>Candidate Map</h2>
<div class="legend">
<span><span class="dot" style="background:#16a34a"></span>gate passed</span>
<span><span class="dot" style="background:#dc2626"></span>gate failed</span>
<span><span class="dot" style="background:#64748b"></span>unscored</span>
<span><span class="dot" style="background:#2563eb"></span>dry-run command</span>
</div>
{map_svg(data)}
</div>
<div class="panel">
<h2>NDT Score Ranking</h2>
{score_bars(data, max_score_bars)}
</div>
</section>
<section class="panel">
<h2>Health Timeline</h2>
{timeline_svg(data, max_timeline_rows)}
</section>
<section class="panel">
{details_table('Reset Plan', data['plan'], ['attempt_id', 'selected', 'accepted', 'selected_candidate_index', 'selected_score', 'rejection_reason'])}
{details_table('Dry-Run Commands', data['commands'], ['command_id', 'attempt_id', 'dry_run', 'publish_topic', 'frame_id', 'status', 'rejection_reason'])}
{details_table('Execution / Observation', data['observation'], ['command_id', 'published_count', 'observed_recovered', 'accepted', 'false_recovery_observable', 'rejection_reason'])}
</section>
</main>
</body>
</html>
"""


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    if not artifact_dir.exists():
        raise FileNotFoundError(f"artifact dir does not exist: {artifact_dir}")
    data = load_artifacts(artifact_dir)
    summary = summarize(data)
    output_html = Path(args.output_html).expanduser().resolve()
    output_html.parent.mkdir(parents=True, exist_ok=True)
    output_html.write_text(
        build_html(data, summary, args.title, args.max_score_bars, args.max_timeline_rows),
        encoding="utf-8",
    )
    if args.output_json:
        write_json(Path(args.output_json).expanduser().resolve(), summary)
    print(json.dumps({"output_html": str(output_html), **summary}, sort_keys=True))


if __name__ == "__main__":
    main()
