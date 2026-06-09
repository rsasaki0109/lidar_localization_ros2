#!/usr/bin/env python3

import argparse
import csv
import json
import time
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a lightweight public demo report (PNG + Markdown + JSON)."
    )
    parser.add_argument("--estimated-csv", required=True, help="pose_trace.csv from benchmark_runner")
    parser.add_argument("--reference-csv", required=True, help="Reference trajectory CSV")
    parser.add_argument("--trajectory-eval-json", required=True, help="trajectory_eval.json")
    parser.add_argument("--summary-json", default="", help="Optional benchmark summary.json")
    parser.add_argument("--output-dir", required=True, help="Directory for report artifacts")
    parser.add_argument("--dataset-name", default="Autoware Istanbul 60s")
    parser.add_argument("--elapsed-sec", type=float, default=None, help="Wall-clock demo runtime")
    return parser.parse_args()


def load_xy(path: Path) -> Dict[str, List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    with path.open("r", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            xs.append(float(row["position_x"]))
            ys.append(float(row["position_y"]))
    return {"x": xs, "y": ys}


def load_json(path: Optional[Path]) -> Dict[str, Any]:
    if path is None or not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def write_png(
    estimated: Dict[str, List[float]],
    reference: Dict[str, List[float]],
    output_png: Path,
    title: str,
) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    fig, ax = plt.subplots(figsize=(8, 6), dpi=120)
    if reference["x"]:
        ax.plot(reference["x"], reference["y"], color="#94a3b8", linewidth=1.5, label="reference")
    if estimated["x"]:
        ax.plot(estimated["x"], estimated["y"], color="#16a34a", linewidth=2.0, label="localized")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png)
    plt.close(fig)
    return True


def fmt_float(value: Any, digits: int = 3) -> str:
    if value in (None, "", "None"):
        return "n/a"
    return f"{float(value):.{digits}f}"


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    estimated_csv = Path(args.estimated_csv).expanduser().resolve()
    reference_csv = Path(args.reference_csv).expanduser().resolve()
    eval_json_path = Path(args.trajectory_eval_json).expanduser().resolve()
    summary_json_path = (
        Path(args.summary_json).expanduser().resolve() if args.summary_json else None
    )

    estimated = load_xy(estimated_csv)
    reference = load_xy(reference_csv)
    eval_data = load_json(eval_json_path)
    summary_data = load_json(summary_json_path)

    png_path = output_dir / "trajectory_xy.png"
    png_written = write_png(
        estimated,
        reference,
        png_path,
        title=f"{args.dataset_name} trajectory (XY)",
    )

    report_json = {
        "dataset": args.dataset_name,
        "generated_at_unix": time.time(),
        "elapsed_sec": args.elapsed_sec,
        "translation_rmse_m": eval_data.get("translation_rmse_m"),
        "rotation_rmse_deg": eval_data.get("rotation_rmse_deg"),
        "matched_sample_count": eval_data.get("matched_sample_count"),
        "pose_rows": len(estimated["x"]),
        "target_process_died_during_run": summary_data.get("target_process_died_during_run"),
        "artifacts": {
            "estimated_csv": str(estimated_csv),
            "reference_csv": str(reference_csv),
            "trajectory_eval_json": str(eval_json_path),
            "summary_json": None if summary_json_path is None else str(summary_json_path),
            "trajectory_xy_png": str(png_path) if png_written else None,
        },
    }
    report_json_path = output_dir / "demo_report.json"
    report_json_path.write_text(json.dumps(report_json, indent=2) + "\n", encoding="utf-8")

    elapsed_line = (
        f"- elapsed_sec: `{args.elapsed_sec:.1f}`"
        if args.elapsed_sec is not None
        else "- elapsed_sec: `n/a`"
    )
    png_line = (
        f"![trajectory XY](trajectory_xy.png)"
        if png_written
        else "_trajectory_xy.png was not generated (matplotlib unavailable)_"
    )

    report_md = "\n".join(
        [
            f"# Public Demo Report — {args.dataset_name}",
            "",
            png_line,
            "",
            "## Metrics",
            "",
            f"- translation_rmse_m: `{fmt_float(report_json['translation_rmse_m'])}`",
            f"- rotation_rmse_deg: `{fmt_float(report_json['rotation_rmse_deg'])}`",
            f"- matched_sample_count: `{report_json['matched_sample_count']}`",
            f"- pose_rows: `{report_json['pose_rows']}`",
            f"- target_process_died_during_run: `{report_json['target_process_died_during_run']}`",
            elapsed_line,
            "",
            "## Artifacts",
            "",
            f"- demo_report.json: `demo_report.json`",
            f"- trajectory_eval.json: `{eval_json_path}`",
            f"- pose_trace.csv: `{estimated_csv}`",
            "",
        ]
    )
    report_md_path = output_dir / "demo_report.md"
    report_md_path.write_text(report_md, encoding="utf-8")

    print(f"demo_report_md: {report_md_path}")
    print(f"demo_report_json: {report_json_path}")
    if png_written:
        print(f"trajectory_xy_png: {png_path}")
    else:
        print("warning: trajectory_xy.png was not generated", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
