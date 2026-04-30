#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict
from typing import Optional

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.spatial.transform import Rotation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate an HTML comparison report for two localization runs.")
    parser.add_argument("--lidarloc-csv", required=True, help="pose_trace.csv from lidar_localization_ros2")
    parser.add_argument(
        "--hdl-csv",
        "--reference-csv",
        dest="hdl_csv",
        required=True,
        help="Reference pose CSV used for comparison",
    )
    parser.add_argument("--output-html", required=True, help="Destination HTML report path")
    parser.add_argument("--lidarloc-summary", default="", help="Optional summary.json from lidar_localization_ros2")
    parser.add_argument("--lidarloc-diagnostics", default="", help="Optional alignment_status.csv from lidar_localization_ros2")
    parser.add_argument(
        "--hdl-status",
        "--reference-status",
        dest="hdl_status",
        default="",
        help="Optional reference runtime/status CSV",
    )
    parser.add_argument("--reference-label", default="reference", help="Legend and metric label for the reference trajectory")
    parser.add_argument("--title", default="Localization Comparison Report", help="HTML report title")
    return parser.parse_args()


def load_csv(path: Path) -> pd.DataFrame:
    return pd.read_csv(path)


def keep_main_time_cluster(df: pd.DataFrame, stamp_col: str) -> pd.DataFrame:
    if df.empty:
      return df.copy()
    stamps = df[stamp_col].astype(float)
    center = stamps.median()
    window = 1.0e6
    filtered = df.loc[(stamps >= center - window) & (stamps <= center + window)].copy()
    if filtered.empty:
        return df.copy()
    return filtered


def quat_to_rpy_deg(df: pd.DataFrame, cols: Dict[str, str]) -> pd.DataFrame:
    quat = df[[cols["x"], cols["y"], cols["z"], cols["w"]]].to_numpy()
    euler = Rotation.from_quat(quat).as_euler("xyz", degrees=True)
    out = df.copy()
    out["roll_deg"] = euler[:, 0]
    out["pitch_deg"] = euler[:, 1]
    out["yaw_deg"] = np.unwrap(np.deg2rad(euler[:, 2]))
    out["yaw_deg"] = np.rad2deg(out["yaw_deg"])
    return out


def prepare_lidarloc(path: Path) -> pd.DataFrame:
    df = load_csv(path)
    df = keep_main_time_cluster(df, "stamp_sec")
    df["stamp_sec"] = df["stamp_sec"].astype(float)
    for col in ["position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"]:
        df[col] = df[col].astype(float)
    df = df.sort_values("stamp_sec").drop_duplicates("stamp_sec")
    df = quat_to_rpy_deg(
        df,
        {"x": "orientation_x", "y": "orientation_y", "z": "orientation_z", "w": "orientation_w"},
    )
    return df


def prepare_hdl(path: Path) -> pd.DataFrame:
    df = load_csv(path)
    df["stamp_sec"] = df["stamp_sec"].astype(float)
    for col in ["position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"]:
        df[col] = df[col].astype(float)
    df = keep_main_time_cluster(df, "stamp_sec")
    df = df.sort_values("stamp_sec").drop_duplicates("stamp_sec")
    df = quat_to_rpy_deg(
        df,
        {"x": "orientation_x", "y": "orientation_y", "z": "orientation_z", "w": "orientation_w"},
    )
    return df


def path_length(df: pd.DataFrame) -> float:
    xyz = df[["position_x", "position_y", "position_z"]].to_numpy()
    if len(xyz) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def interpolate_columns(source: pd.DataFrame, target_times: np.ndarray, columns: list[str]) -> Dict[str, np.ndarray]:
    src_t = source["stamp_sec"].to_numpy()
    result: Dict[str, np.ndarray] = {}
    for col in columns:
        result[col] = np.interp(target_times, src_t, source[col].to_numpy())
    return result


def overlapping_frames(lhs: pd.DataFrame, rhs: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    start = max(lhs["stamp_sec"].min(), rhs["stamp_sec"].min())
    end = min(lhs["stamp_sec"].max(), rhs["stamp_sec"].max())
    lhs_o = lhs[(lhs["stamp_sec"] >= start) & (lhs["stamp_sec"] <= end)].copy()
    rhs_o = rhs[(rhs["stamp_sec"] >= start) & (rhs["stamp_sec"] <= end)].copy()
    return lhs_o, rhs_o


def summarize_pair(lidarloc: pd.DataFrame, hdl: pd.DataFrame) -> Dict[str, Optional[float]]:
    overlap = build_overlap_series(lidarloc, hdl)
    if overlap.empty:
        return {
            "overlap_duration_sec": None,
            "translation_delta_rmse_m": None,
            "rotation_delta_rmse_deg": None,
            "endpoint_delta_m": None,
            "path_length_ratio": None,
            "first_translation_delta_gt_1m_sec": None,
            "first_translation_delta_gt_2m_sec": None,
            "first_translation_delta_gt_3m_sec": None,
            "first_translation_delta_gt_5m_sec": None,
            "first_translation_delta_gt_10m_sec": None,
        }

    translation_rmse = float(np.sqrt(np.mean(overlap["translation_delta_m"].to_numpy() ** 2)))
    rotation_rmse = float(np.sqrt(np.mean(overlap["rotation_delta_deg"].to_numpy() ** 2)))
    endpoint_delta = float(overlap["translation_delta_m"].iloc[-1])

    lidarloc_o, hdl_o = overlapping_frames(lidarloc, hdl)
    hdl_path = path_length(hdl_o)
    lidarloc_path = path_length(lidarloc_o)
    ratio = lidarloc_path / hdl_path if hdl_path > 0.0 else None

    def first_crossing(threshold: float) -> Optional[float]:
        crossed = overlap[overlap["translation_delta_m"] > threshold]
        if crossed.empty:
            return None
        return float(crossed["time_rel"].iloc[0])

    return {
        "overlap_duration_sec": float(overlap["time_rel"].iloc[-1]) if len(overlap) >= 2 else 0.0,
        "translation_delta_rmse_m": translation_rmse,
        "rotation_delta_rmse_deg": rotation_rmse,
        "endpoint_delta_m": endpoint_delta,
        "path_length_ratio": ratio,
        "first_translation_delta_gt_1m_sec": first_crossing(1.0),
        "first_translation_delta_gt_2m_sec": first_crossing(2.0),
        "first_translation_delta_gt_3m_sec": first_crossing(3.0),
        "first_translation_delta_gt_5m_sec": first_crossing(5.0),
        "first_translation_delta_gt_10m_sec": first_crossing(10.0),
    }


def build_overlap_series(lidarloc: pd.DataFrame, hdl: pd.DataFrame) -> pd.DataFrame:
    lidarloc_o, hdl_o = overlapping_frames(lidarloc, hdl)
    if lidarloc_o.empty or hdl_o.empty:
        return pd.DataFrame()

    target_t = lidarloc_o["stamp_sec"].to_numpy()
    hdl_interp = interpolate_columns(
        hdl_o,
        target_t,
        ["position_x", "position_y", "position_z", "roll_deg", "pitch_deg", "yaw_deg"],
    )

    pos_l = lidarloc_o[["position_x", "position_y", "position_z"]].to_numpy()
    pos_h = np.column_stack([hdl_interp["position_x"], hdl_interp["position_y"], hdl_interp["position_z"]])
    rpy_l = lidarloc_o[["roll_deg", "pitch_deg", "yaw_deg"]].to_numpy()
    rpy_h = np.column_stack([hdl_interp["roll_deg"], hdl_interp["pitch_deg"], hdl_interp["yaw_deg"]])

    overlap = pd.DataFrame(
        {
            "stamp_sec": target_t,
            "time_rel": target_t - target_t[0],
            "translation_delta_m": np.linalg.norm(pos_l - pos_h, axis=1),
            "rotation_delta_deg": np.sqrt(np.sum((rpy_l - rpy_h) ** 2, axis=1) / 3.0),
        }
    )
    return overlap


def summarize_lidarloc_diagnostics(path: Optional[Path]) -> Dict[str, Optional[float]]:
    if path is None or not path.exists():
        return {}
    rows = list(csv.DictReader(path.open("r", encoding="utf-8", newline="")))
    align_times = []
    fitness = []
    for row in rows:
        try:
            values = json.loads(row["values_json"])
        except Exception:
            continue
        if "alignment_time_sec" in values:
            align_times.append(float(values["alignment_time_sec"]))
        if "fitness_score" in values:
            fitness.append(float(values["fitness_score"]))
    return {
        "diagnostic_rows": len(rows),
        "alignment_time_median_sec": float(np.median(align_times)) if align_times else None,
        "alignment_time_max_sec": float(np.max(align_times)) if align_times else None,
        "fitness_median": float(np.median(fitness)) if fitness else None,
    }


def summarize_hdl_status(path: Optional[Path]) -> Dict[str, Optional[float]]:
    if path is None or not path.exists():
        return {}
    df = pd.read_csv(path)
    return {
        "status_rows": int(len(df)),
        "matching_error_median": float(df["matching_error"].median()) if len(df) else None,
        "matching_error_max": float(df["matching_error"].max()) if len(df) else None,
        "inlier_fraction_median": float(df["inlier_fraction"].median()) if len(df) else None,
        "non_converged_count": int((df["has_converged"] == 0).sum()) if len(df) else None,
    }


def build_trajectory_figure(lidarloc: pd.DataFrame, hdl: pd.DataFrame, reference_label: str) -> str:
    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(
            x=hdl["position_x"],
            y=hdl["position_y"],
            z=hdl["position_z"],
            mode="lines",
            name=reference_label,
            line={"width": 4},
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=lidarloc["position_x"],
            y=lidarloc["position_y"],
            z=lidarloc["position_z"],
            mode="lines",
            name="lidar_localization_ros2",
            line={"width": 4},
        )
    )
    fig.update_layout(
        title="3D Trajectory Overlay",
        scene={
            "xaxis_title": "X [m]",
            "yaxis_title": "Y [m]",
            "zaxis_title": "Z [m]",
            "aspectmode": "data",
        },
        margin={"l": 0, "r": 0, "t": 40, "b": 0},
        height=700,
    )
    return fig.to_html(full_html=False, include_plotlyjs="cdn")


def build_timeseries_figure(lidarloc: pd.DataFrame, hdl: pd.DataFrame, reference_label: str) -> str:
    start = min(lidarloc["stamp_sec"].min(), hdl["stamp_sec"].min())
    ll = lidarloc.copy()
    hh = hdl.copy()
    ll["time_rel"] = ll["stamp_sec"] - start
    hh["time_rel"] = hh["stamp_sec"] - start

    fig = make_subplots(rows=6, cols=1, shared_xaxes=True, vertical_spacing=0.02, subplot_titles=[
        "X [m]", "Y [m]", "Z [m]", "Roll [deg]", "Pitch [deg]", "Yaw [deg]"
    ])
    series = [
        ("position_x", "X [m]"),
        ("position_y", "Y [m]"),
        ("position_z", "Z [m]"),
        ("roll_deg", "Roll [deg]"),
        ("pitch_deg", "Pitch [deg]"),
        ("yaw_deg", "Yaw [deg]"),
    ]
    for idx, (col, _) in enumerate(series, start=1):
        fig.add_trace(go.Scatter(x=hh["time_rel"], y=hh[col], mode="lines", name=f"{reference_label}:{col}", legendgroup="reference", showlegend=idx == 1), row=idx, col=1)
        fig.add_trace(go.Scatter(x=ll["time_rel"], y=ll[col], mode="lines", name=f"lidarloc:{col}", legendgroup="lidarloc", showlegend=idx == 1), row=idx, col=1)
    fig.update_layout(height=1400, title="XYZ / RPY Time Series", margin={"l": 50, "r": 20, "t": 50, "b": 40})
    fig.update_xaxes(title_text="Time from first sample [s]", row=6, col=1)
    return fig.to_html(full_html=False, include_plotlyjs=False)


def build_delta_figure(overlap: pd.DataFrame, reference_label: str) -> str:
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.08, subplot_titles=[
        f"Translation Delta vs {reference_label} [m]",
        f"Rotation Delta vs {reference_label} [deg]",
    ])
    fig.add_trace(
        go.Scatter(x=overlap["time_rel"], y=overlap["translation_delta_m"], mode="lines", name="translation delta"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=overlap["time_rel"], y=overlap["rotation_delta_deg"], mode="lines", name="rotation delta"),
        row=2,
        col=1,
    )
    fig.update_layout(height=750, title="Delta Time Series", margin={"l": 50, "r": 20, "t": 50, "b": 40})
    fig.update_xaxes(title_text="Time from overlap start [s]", row=2, col=1)
    return fig.to_html(full_html=False, include_plotlyjs=False)


def fmt(value: Optional[float], digits: int = 3) -> str:
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def render_table(rows: list[tuple[str, str]]) -> str:
    body = "\n".join(f"<tr><th>{label}</th><td>{value}</td></tr>" for label, value in rows)
    return f"<table>{body}</table>"


def main() -> int:
    args = parse_args()
    lidarloc = prepare_lidarloc(Path(args.lidarloc_csv))
    hdl = prepare_hdl(Path(args.hdl_csv))
    reference_label = args.reference_label

    pair = summarize_pair(lidarloc, hdl)
    overlap = build_overlap_series(lidarloc, hdl)
    lidarloc_diag = summarize_lidarloc_diagnostics(Path(args.lidarloc_diagnostics)) if args.lidarloc_diagnostics else {}
    hdl_status = summarize_hdl_status(Path(args.hdl_status)) if args.hdl_status else {}

    lidarloc_summary = {}
    if args.lidarloc_summary:
        lidarloc_summary = json.loads(Path(args.lidarloc_summary).read_text(encoding="utf-8"))

    report_dir = Path(args.output_html).resolve().parent
    report_dir.mkdir(parents=True, exist_ok=True)

    metrics_table = render_table(
        [
            ("Overlap duration [s]", fmt(pair.get("overlap_duration_sec"))),
            (f"Translation delta RMSE vs {reference_label} [m]", fmt(pair.get("translation_delta_rmse_m"))),
            (f"Rotation delta RMSE vs {reference_label} [deg]", fmt(pair.get("rotation_delta_rmse_deg"))),
            ("Endpoint delta [m]", fmt(pair.get("endpoint_delta_m"))),
            (f"Path length ratio lidarloc/{reference_label}", fmt(pair.get("path_length_ratio"))),
            ("First time translation delta > 1m [s]", fmt(pair.get("first_translation_delta_gt_1m_sec"))),
            ("First time translation delta > 2m [s]", fmt(pair.get("first_translation_delta_gt_2m_sec"))),
            ("First time translation delta > 3m [s]", fmt(pair.get("first_translation_delta_gt_3m_sec"))),
            ("First time translation delta > 5m [s]", fmt(pair.get("first_translation_delta_gt_5m_sec"))),
            ("First time translation delta > 10m [s]", fmt(pair.get("first_translation_delta_gt_10m_sec"))),
            ("lidarloc samples", str(len(lidarloc))),
            (f"{reference_label} samples", str(len(hdl))),
            ("lidarloc path length [m]", fmt(path_length(lidarloc))),
            (f"{reference_label} path length [m]", fmt(path_length(hdl))),
        ]
    )

    runtime_rows = []
    if lidarloc_summary:
        stats = lidarloc_summary.get("resource_monitor", {}).get("stats", {})
        runtime_rows.extend(
            [
                ("lidarloc cpu max [%]", fmt(stats.get("cpu_percent_max"))),
                ("lidarloc rss max [MB]", fmt(stats.get("rss_mb_max"))),
            ]
        )
    runtime_rows.extend(
        [
            ("lidarloc align median [s]", fmt(lidarloc_diag.get("alignment_time_median_sec"))),
            ("lidarloc align max [s]", fmt(lidarloc_diag.get("alignment_time_max_sec"))),
            ("lidarloc fitness median", fmt(lidarloc_diag.get("fitness_median"))),
            (f"{reference_label} matching error median", fmt(hdl_status.get("matching_error_median"))),
            (f"{reference_label} matching error max", fmt(hdl_status.get("matching_error_max"))),
            (f"{reference_label} inlier fraction median", fmt(hdl_status.get("inlier_fraction_median"))),
            (f"{reference_label} non-converged count", str(hdl_status.get("non_converged_count", "-"))),
        ]
    )
    runtime_table = render_table(runtime_rows)

    trajectory_fig = build_trajectory_figure(lidarloc, hdl, reference_label)
    timeseries_fig = build_timeseries_figure(lidarloc, hdl, reference_label)
    delta_fig = build_delta_figure(overlap, reference_label) if not overlap.empty else "<p>No overlap window.</p>"

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{args.title}</title>
  <style>
    body {{
      margin: 0;
      font-family: 'IBM Plex Sans', 'Segoe UI', sans-serif;
      color: #111;
      background: linear-gradient(180deg, #f4f0e8 0%, #ffffff 35%);
    }}
    main {{
      max-width: 1400px;
      margin: 0 auto;
      padding: 32px 28px 80px;
    }}
    h1, h2 {{
      margin: 0 0 12px;
    }}
    p {{
      line-height: 1.5;
      max-width: 900px;
    }}
    .card {{
      background: rgba(255,255,255,0.9);
      border: 1px solid #ddd3c2;
      border-radius: 18px;
      padding: 20px 22px;
      margin: 18px 0 24px;
      box-shadow: 0 12px 40px rgba(0,0,0,0.06);
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 18px;
    }}
    table {{
      border-collapse: collapse;
      width: 100%;
    }}
    th, td {{
      text-align: left;
      padding: 10px 12px;
      border-bottom: 1px solid #eee4d6;
      vertical-align: top;
    }}
    th {{
      width: 60%;
      color: #5b5247;
      font-weight: 600;
    }}
    .note {{
      color: #5b5247;
      font-size: 0.95rem;
    }}
  </style>
</head>
<body>
<main>
  <h1>{args.title}</h1>
  <p>
    This report compares <code>lidar_localization_ros2</code> against <code>{reference_label}</code>.
    The comparison is relative, not absolute ground truth, unless the supplied reference trajectory itself is ground truth.
  </p>
  <div class="grid">
    <section class="card">
      <h2>Relative Metrics</h2>
      {metrics_table}
    </section>
    <section class="card">
      <h2>Runtime Snapshot</h2>
      {runtime_table}
    </section>
  </div>
  <section class="card">
    <h2>Reading Guide</h2>
    <p class="note">
      <code>translation delta RMSE</code> and <code>rotation delta RMSE</code> are differences versus
      <code>{reference_label}</code>, not necessarily errors against ground truth.
      Small values mean the ROS 2 trajectory stays close to the reference trajectory over the overlapping window.
    </p>
  </section>
  <section class="card">
    <h2>3D Trajectory</h2>
    {trajectory_fig}
  </section>
  <section class="card">
    <h2>XYZ / RPY Time Series</h2>
    {timeseries_fig}
  </section>
  <section class="card">
    <h2>Delta Time Series</h2>
    {delta_fig}
  </section>
</main>
</body>
</html>
"""

    Path(args.output_html).write_text(html, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
