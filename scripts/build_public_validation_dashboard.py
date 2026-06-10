#!/usr/bin/env python3

import argparse
import json
import html
import time
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a public validation dashboard from demo and regression summaries."
    )
    parser.add_argument(
        "--workspace-root",
        default="",
        help="lidarloc_ws root. Default: parent of repo root.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Dashboard output directory. Default: <workspace>/artifacts/public/dashboard",
    )
    parser.add_argument(
        "--demo-report-json",
        default="",
        help="Optional demo_report.json override.",
    )
    parser.add_argument(
        "--release-summary-json",
        default="",
        help="Optional release regression summary.json override.",
    )
    parser.add_argument(
        "--public-summary-json",
        default="",
        help="Optional public regression summary.json override.",
    )
    return parser.parse_args()


def resolve_repo_root() -> Path:
    script_dir = Path(__file__).resolve().parent
    if (script_dir.parent / "CMakeLists.txt").exists():
        return script_dir.parent
    return script_dir.parent.parent.parent / "repo"


def load_json(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None or not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def fmt_bool(value: Any) -> str:
    if value is None:
        return "n/a"
    return "pass" if bool(value) else "fail"


def fmt_float(value: Any, digits: int = 3) -> str:
    if value in (None, "", "None"):
        return "n/a"
    return f"{float(value):.{digits}f}"


def fmt_int(value: Any) -> str:
    if value in (None, "", "None"):
        return "n/a"
    return str(int(value))


def discover_paths(args: argparse.Namespace, repo_root: Path) -> Dict[str, Optional[Path]]:
    ws_root = (
        Path(args.workspace_root).expanduser().resolve()
        if args.workspace_root
        else repo_root.parent
    )
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else ws_root / "artifacts/public/dashboard"
    )

    demo_path = (
        Path(args.demo_report_json).expanduser().resolve()
        if args.demo_report_json
        else ws_root / "artifacts/public/demo/latest/demo_report.json"
    )
    release_path = (
        Path(args.release_summary_json).expanduser().resolve()
        if args.release_summary_json
        else ws_root / "artifacts/public/release_regression_suite/summary.json"
    )
    public_path = (
        Path(args.public_summary_json).expanduser().resolve()
        if args.public_summary_json
        else None
    )

    release_data = load_json(release_path)
    if public_path is None:
        if release_data and release_data.get("public_regression_suite", {}).get("summary_json"):
            public_path = Path(release_data["public_regression_suite"]["summary_json"])
        else:
            public_path = ws_root / "artifacts/public/public_regression_suite/summary.json"

    return {
        "ws_root": ws_root,
        "output_dir": output_dir,
        "demo_report_json": demo_path if demo_path.exists() else None,
        "release_summary_json": release_path if release_path.exists() else None,
        "public_summary_json": public_path if public_path.exists() else None,
        "demo_png": (
            demo_path.parent / "trajectory_xy.png"
            if demo_path.exists() and (demo_path.parent / "trajectory_xy.png").exists()
            else None
        ),
    }


def build_dashboard_data(paths: Dict[str, Optional[Path]]) -> Dict[str, Any]:
    demo = load_json(paths["demo_report_json"])
    release = load_json(paths["release_summary_json"])
    public = load_json(paths["public_summary_json"])

    istanbul = (public or {}).get("istanbul", {})
    hdl = (public or {}).get("hdl", {})
    nav2 = (release or {}).get("nav2_reinitialization_supervisor_regression", {})

    rows: List[Dict[str, str]] = []

    if demo:
        rows.append(
            {
                "suite": "Public demo",
                "dataset": demo.get("dataset", "Autoware Istanbul 60s"),
                "status": "ok" if not demo.get("target_process_died_during_run") else "fail",
                "translation_rmse_m": fmt_float(demo.get("translation_rmse_m")),
                "rotation_rmse_deg": fmt_float(demo.get("rotation_rmse_deg")),
                "matched": fmt_int(demo.get("matched_sample_count")),
                "notes": "Star-friendly entry path; same public Istanbul dataset",
            }
        )

    if istanbul:
        rows.append(
            {
                "suite": "Public regression",
                "dataset": "Autoware Istanbul 60s (no-IMU safety)",
                "status": fmt_bool(istanbul.get("pass")),
                "translation_rmse_m": fmt_float(istanbul.get("translation_rmse_m")),
                "rotation_rmse_deg": fmt_float(istanbul.get("rotation_rmse_deg")),
                "matched": fmt_int(istanbul.get("matched_sample_count")),
                "notes": "Release gate; not an IMU benefit benchmark",
            }
        )

    if hdl:
        rows.append(
            {
                "suite": "Public regression",
                "dataset": "HDL hdl_400 60s (IMU safety)",
                "status": fmt_bool(hdl.get("pass")),
                "translation_rmse_m": "n/a",
                "rotation_rmse_deg": "n/a",
                "matched": "n/a",
                "notes": (
                    f"pose rows median {fmt_float(hdl.get('false_pose_rows'))} -> "
                    f"{fmt_float(hdl.get('true_pose_rows'))}; "
                    f"align median {fmt_float(hdl.get('false_alignment_time_median_sec'), 4)}s -> "
                    f"{fmt_float(hdl.get('true_alignment_time_median_sec'), 4)}s"
                ),
            }
        )

    if nav2:
        rows.append(
            {
                "suite": "Release regression",
                "dataset": "Nav2 reinit supervisor 150s",
                "status": fmt_bool(nav2.get("overall_pass")),
                "translation_rmse_m": "n/a",
                "rotation_rmse_deg": "n/a",
                "matched": "n/a",
                "notes": (
                    f"requested rows {fmt_int(nav2.get('baseline_requested_rows'))} -> "
                    f"{fmt_int(nav2.get('candidate_requested_rows'))}; "
                    f"goal {nav2.get('candidate_goal_status', 'n/a')}"
                ),
            }
        )

    return {
        "generated_at_unix": time.time(),
        "generated_at_iso": datetime.now().astimezone().isoformat(timespec="seconds"),
        "sources": {
            "demo_report_json": None if paths["demo_report_json"] is None else str(paths["demo_report_json"]),
            "release_summary_json": None
            if paths["release_summary_json"] is None
            else str(paths["release_summary_json"]),
            "public_summary_json": None
            if paths["public_summary_json"] is None
            else str(paths["public_summary_json"]),
            "demo_trajectory_png": None if paths["demo_png"] is None else str(paths["demo_png"]),
        },
        "overall": {
            "demo_available": demo is not None,
            "release_regression_available": release is not None,
            "public_regression_available": public is not None,
            "release_overall_pass": None if release is None else bool(release.get("overall_pass")),
            "public_overall_pass": None if public is None else bool(public.get("overall_pass")),
        },
        "rows": rows,
        "claim_boundary": [
            "Only public / reproducible datasets belong in outward-facing claims.",
            "Istanbul 60s numbers are no-IMU safety checks, not IMU benefit benchmarks.",
            "Koide / Boreas / GLIL numbers are engineering evidence unless the claim gate is explicitly met.",
            "Public demo and regression rows must use the same official dataset sources and commands.",
        ],
        "commands": {
            "public_demo": "scripts/run_public_demo.sh",
            "public_regression": "scripts/run_public_regression_suite.sh",
            "release_regression": "scripts/run_release_regression_suite.sh",
            "dashboard": "scripts/run_public_validation_dashboard.sh",
        },
    }


def render_markdown(data: Dict[str, Any], output_dir: Path) -> str:
    lines = [
        "# Public Validation Dashboard",
        "",
        f"- generated_at: `{data['generated_at_iso']}`",
        f"- release_overall_pass: `{data['overall']['release_overall_pass']}`",
        f"- public_overall_pass: `{data['overall']['public_overall_pass']}`",
        "",
        "## Latest Public Validation",
        "",
        "| Suite | Dataset | Status | Trans RMSE [m] | Rot RMSE [deg] | Matched | Notes |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]

    for row in data["rows"]:
        lines.append(
            "| {suite} | {dataset} | {status} | {translation_rmse_m} | "
            "{rotation_rmse_deg} | {matched} | {notes} |".format(**row)
        )

    if data["sources"]["demo_trajectory_png"]:
        lines.extend(
            [
                "",
                "## Public Demo Trajectory",
                "",
                "![demo trajectory](../demo/latest/trajectory_xy.png)",
                "",
            ]
        )

    lines.extend(
        [
            "## Reproduce",
            "",
            "```bash",
            "cd /path/to/lidarloc_ws/repo",
            "source scripts/setup_local_env.sh",
            data["commands"]["public_demo"],
            data["commands"]["release_regression"],
            data["commands"]["dashboard"],
            "```",
            "",
            "## Claim Boundary",
            "",
        ]
    )
    for note in data["claim_boundary"]:
        lines.append(f"- {note}")

    lines.extend(
        [
            "",
            "## Source Artifacts",
            "",
            f"- demo_report.json: `{data['sources']['demo_report_json']}`",
            f"- release summary.json: `{data['sources']['release_summary_json']}`",
            f"- public summary.json: `{data['sources']['public_summary_json']}`",
            "",
        ]
    )
    return "\n".join(lines)


def render_html(data: Dict[str, Any]) -> str:
    table_rows = []
    for row in data["rows"]:
        table_rows.append(
            "<tr>"
            f"<td>{html.escape(row['suite'])}</td>"
            f"<td>{html.escape(row['dataset'])}</td>"
            f"<td>{html.escape(row['status'])}</td>"
            f"<td>{html.escape(row['translation_rmse_m'])}</td>"
            f"<td>{html.escape(row['rotation_rmse_deg'])}</td>"
            f"<td>{html.escape(row['matched'])}</td>"
            f"<td>{html.escape(row['notes'])}</td>"
            "</tr>"
        )

    image_block = ""
    if data["sources"]["demo_trajectory_png"]:
        image_block = (
            '<section><h2>Public Demo Trajectory</h2>'
            '<img alt="demo trajectory" src="../demo/latest/trajectory_xy.png" '
            'style="max-width: 720px; width: 100%;"></section>'
        )

    claim_items = "".join(f"<li>{html.escape(note)}</li>" for note in data["claim_boundary"])

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>lidar_localization_ros2 public validation</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 2rem auto; max-width: 1100px; line-height: 1.5; }}
    table {{ border-collapse: collapse; width: 100%; margin: 1rem 0; }}
    th, td {{ border: 1px solid #cbd5e1; padding: 0.5rem 0.75rem; text-align: left; vertical-align: top; }}
    th {{ background: #f8fafc; }}
    code {{ background: #f1f5f9; padding: 0.1rem 0.3rem; border-radius: 0.25rem; }}
    .meta {{ color: #475569; }}
  </style>
</head>
<body>
  <h1>Public Validation Dashboard</h1>
  <p class="meta">generated_at: <code>{html.escape(data['generated_at_iso'])}</code></p>
  <p class="meta">
    release_overall_pass: <code>{html.escape(str(data['overall']['release_overall_pass']))}</code>;
    public_overall_pass: <code>{html.escape(str(data['overall']['public_overall_pass']))}</code>
  </p>
  <section>
    <h2>Latest Public Validation</h2>
    <table>
      <thead>
        <tr>
          <th>Suite</th><th>Dataset</th><th>Status</th><th>Trans RMSE [m]</th>
          <th>Rot RMSE [deg]</th><th>Matched</th><th>Notes</th>
        </tr>
      </thead>
      <tbody>
        {''.join(table_rows)}
      </tbody>
    </table>
  </section>
  {image_block}
  <section>
    <h2>Reproduce</h2>
    <pre><code>cd /path/to/lidarloc_ws/repo
source scripts/setup_local_env.sh
{html.escape(data['commands']['public_demo'])}
{html.escape(data['commands']['release_regression'])}
{html.escape(data['commands']['dashboard'])}</code></pre>
  </section>
  <section>
    <h2>Claim Boundary</h2>
    <ul>{claim_items}</ul>
  </section>
</body>
</html>
"""


def main() -> int:
    args = parse_args()
    repo_root = resolve_repo_root()
    paths = discover_paths(args, repo_root)
    data = build_dashboard_data(paths)

    if not data["rows"]:
        raise SystemExit(
            "No validation summaries found. Run scripts/run_public_demo.sh or "
            "scripts/run_release_regression_suite.sh first."
        )

    output_dir = paths["output_dir"]
    output_dir.mkdir(parents=True, exist_ok=True)

    dashboard_json = output_dir / "dashboard.json"
    dashboard_md = output_dir / "index.md"
    dashboard_html = output_dir / "index.html"

    dashboard_json.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    dashboard_md.write_text(render_markdown(data, output_dir), encoding="utf-8")
    dashboard_html.write_text(render_html(data), encoding="utf-8")

    print(f"dashboard_json: {dashboard_json}")
    print(f"dashboard_md: {dashboard_md}")
    print(f"dashboard_html: {dashboard_html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
