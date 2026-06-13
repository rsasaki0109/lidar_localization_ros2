#!/usr/bin/env python3
"""Offline diagnostic: is `local_map_crop_too_small` a map-coverage cliff?

The Boreas localization run holds ~0.04 m error for ~12 s and then collapses to
~45 m RMSE, with `local_map_crop_too_small` and reject streaks as the reported
failure modes (development_plan.md, Phase 2). With the shipped Boreas params
(`local_map_radius: 300`, `local_map_min_points: 100`) a 300 m disk around any
on-map point holds far more than 100 points, so `local_map_crop_too_small` can
only fire when the crop *center* is hundreds of metres off the mapped area. That
leaves two very different root causes:

  (A) prediction divergence -- the crop is centered on the predicted seed
      (`runAlignmentAttempt(init_guess, init_guess, ...)`), so a runaway
      prediction starves the target cloud, which guarantees the next reject, a
      positive-feedback divergence; or
  (C) map coverage -- the map simply does not cover the whole trajectory (a
      map-split / partial rebuild), so once the vehicle drives past the mapped
      region the crop around the *true* pose legitimately falls below
      `min_points`.

These have opposite fixes (estimator-side vs map-side), so they must be told
apart before touching runtime behaviour -- the Phase 3 lesson about not shipping
an unreplayed change applies here too.

This script settles it without ROS or a running localizer: it counts, for every
ground-truth pose, how many map points fall within `local_map_radius`, and
reports the first time that count drops below `local_map_min_points`. If the true
trajectory stays well-covered for the whole run, a real `local_map_crop_too_small`
must be prediction-driven (A). If coverage itself collapses around the cliff time,
it is a map-coverage cliff (C) and the map must be extended/retiled.

Usage:
    diagnose_local_map_crop_coverage.py \
        --map-pcd boreas_map.pcd --reference-csv boreas_reference.csv \
        --local-map-radius 300 --local-map-min-points 100 \
        --out-csv coverage.csv [--plot coverage.png]
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


def count_points_within_radius(map_xy, query_xy, radius_m, chunk=256):
    """Points of ``map_xy`` within ``radius_m`` of each row of ``query_xy``.

    Pure numpy so it is testable without scipy. Uses a KD-tree fast path when
    scipy is available, otherwise a chunked brute-force scan. Returns an int
    array of length ``len(query_xy)``.
    """
    map_xy = np.asarray(map_xy, dtype=np.float64)
    query_xy = np.asarray(query_xy, dtype=np.float64)
    if query_xy.ndim != 2 or query_xy.shape[1] != 2:
        raise ValueError("query_xy must be (N, 2)")
    if map_xy.size == 0:
        return np.zeros(len(query_xy), dtype=np.int64)

    try:
        from scipy.spatial import cKDTree  # type: ignore

        tree = cKDTree(map_xy)
        counts = tree.query_ball_point(query_xy, radius_m, return_length=True)
        return np.asarray(counts, dtype=np.int64)
    except Exception:  # noqa: BLE001 - scipy absent or too old; brute force.
        pass

    r2 = float(radius_m) ** 2
    counts = np.empty(len(query_xy), dtype=np.int64)
    mx = map_xy[:, 0]
    my = map_xy[:, 1]
    # Bound the (chunk x map) intermediate so a large map cannot OOM the fallback:
    # cap ~20M float64 elements (~160 MB) per temporary.
    eff_chunk = max(1, min(chunk, int(20_000_000 // max(1, len(map_xy)))))
    for start in range(0, len(query_xy), eff_chunk):
        block = query_xy[start:start + eff_chunk]
        # (block, map) squared distances, chunked over queries to bound memory.
        dx = block[:, 0][:, None] - mx[None, :]
        dy = block[:, 1][:, None] - my[None, :]
        counts[start:start + len(block)] = np.count_nonzero(dx * dx + dy * dy <= r2, axis=1)
    return counts


def analyze_coverage(map_xy, traj_xy, traj_t, radius_m, min_points, count_scale=1):
    """Per-pose coverage and the first under-covered (cliff) index.

    Returns a dict with: ``counts`` (per pose), ``too_small`` (bool mask),
    ``out_of_bounds`` (crop center beyond map bbox + radius), ``cliff_index``
    (first too_small index or None), ``cliff_time`` (its timestamp or None),
    ``map_bounds`` and ``min_count``.
    """
    map_xy = np.asarray(map_xy, dtype=np.float64)
    traj_xy = np.asarray(traj_xy, dtype=np.float64)
    # Scale by the subsample stride so we compare a full-map estimate against the
    # absolute min_points (a strided map otherwise reports a false cliff).
    counts = count_points_within_radius(map_xy, traj_xy, radius_m) * int(count_scale)
    too_small = counts < min_points

    min_x, min_y = map_xy.min(axis=0)
    max_x, max_y = map_xy.max(axis=0)
    out_of_bounds = (
        (traj_xy[:, 0] < min_x - radius_m)
        | (traj_xy[:, 0] > max_x + radius_m)
        | (traj_xy[:, 1] < min_y - radius_m)
        | (traj_xy[:, 1] > max_y + radius_m)
    )

    cliff_idx = int(np.argmax(too_small)) if too_small.any() else None
    cliff_time = float(traj_t[cliff_idx]) if cliff_idx is not None else None
    return {
        "counts": counts,
        "too_small": too_small,
        "out_of_bounds": out_of_bounds,
        "cliff_index": cliff_idx,
        "cliff_time": cliff_time,
        "map_bounds": (float(min_x), float(max_x), float(min_y), float(max_y)),
        "min_count": int(counts.min()) if len(counts) else 0,
    }


def verdict(result, min_points):
    """A human-readable (A)/(C) verdict from an analyze_coverage result."""
    if result["cliff_index"] is None:
        return (
            "COVERAGE-OK: the true trajectory stays at or above min_points for the "
            "whole run (min in-radius count %d >= %d). A real local_map_crop_too_small "
            "is therefore NOT a coverage problem -- it must be prediction-driven (the "
            "crop center diverged from truth). Look estimator-side (twist prediction / "
            "reject streak), not at the map."
            % (result["min_count"], min_points)
        )
    t = result["cliff_time"]
    oob = result["out_of_bounds"][result["cliff_index"]]
    return (
        "COVERAGE-CLIFF at t=%.2fs (in-radius count drops below %d along the *true* "
        "trajectory; crop center %s map bbox+radius). This is a map-coverage cliff: the "
        "map does not cover the trajectory past this point. Fix is map-side (extend / "
        "retile / rebuild the map to cover the full route), not estimator-side."
        % (t, min_points, "outside" if oob else "inside")
    )


def load_reference(reference_csv):
    t, xs, ys = [], [], []
    with open(reference_csv, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            t.append(float(row["stamp_sec"]))
            xs.append(float(row["position_x"]))
            ys.append(float(row["position_y"]))
    if not t:
        raise SystemExit("reference CSV had no rows")
    order = np.argsort(t)
    t = np.asarray(t)[order]
    xy = np.column_stack([np.asarray(xs)[order], np.asarray(ys)[order]])
    # Re-zero time to the first pose so the cliff time is "seconds into the run".
    return t - t[0], xy


def load_map_xy(map_pcd, max_points=None):
    import open3d as o3d  # imported lazily so the pure core stays dependency-light

    cloud = o3d.io.read_point_cloud(str(map_pcd))
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.size == 0:
        raise SystemExit(f"map PCD is empty: {map_pcd}")
    xy = points[:, :2]
    stride = 1
    if max_points and len(xy) > max_points:
        # Uniform stride subsample keeps spatial coverage representative; the
        # caller scales counts by `stride` so min_points stays comparable.
        stride = int(np.ceil(len(xy) / max_points))
        xy = xy[::stride]
    return xy, stride


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--map-pcd", required=True, type=Path)
    p.add_argument("--reference-csv", required=True, type=Path)
    p.add_argument("--local-map-radius", type=float, default=300.0)
    p.add_argument("--local-map-min-points", type=int, default=100)
    p.add_argument("--max-map-points", type=int, default=None,
                   help="Optional: subsample the map above this many points for "
                        "speed; counts are scaled back by the stride so min_points "
                        "stays comparable (an estimate). Default: no subsample.")
    p.add_argument("--out-csv", type=Path, default=None)
    p.add_argument("--plot", type=Path, default=None)
    return p.parse_args()


def write_csv(path, t, xy, result):
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_sec", "x", "y", "points_in_radius", "too_small", "out_of_bounds"])
        for i in range(len(t)):
            writer.writerow([
                f"{t[i]:.3f}", f"{xy[i, 0]:.3f}", f"{xy[i, 1]:.3f}",
                int(result["counts"][i]), int(result["too_small"][i]),
                int(result["out_of_bounds"][i]),
            ])


def render_plot(path, t, result, min_points):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"(plot skipped: matplotlib unavailable: {exc})", file=sys.stderr)
        return
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, result["counts"], label="map points within radius")
    ax.axhline(min_points, color="red", linestyle="--", label=f"min_points={min_points}")
    if result["cliff_index"] is not None:
        ax.axvline(result["cliff_time"], color="orange", linestyle=":",
                   label=f"cliff t={result['cliff_time']:.1f}s")
    ax.set_xlabel("time into run (s)")
    ax.set_ylabel("in-radius map points")
    ax.set_yscale("symlog")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"wrote plot {path}")


def main():
    args = parse_args()
    t, xy = load_reference(args.reference_csv)
    map_xy, stride = load_map_xy(args.map_pcd, args.max_map_points)
    result = analyze_coverage(
        map_xy, xy, t, args.local_map_radius, args.local_map_min_points, count_scale=stride)

    if stride > 1:
        print(f"NOTE: map subsampled by stride {stride}; in-radius counts are "
              f"estimated (raw count x {stride}).")
    print(f"poses: {len(t)}  map points (after subsample): {len(map_xy)}")
    print("map bbox x[%.1f, %.1f] y[%.1f, %.1f]" % result["map_bounds"])
    print("in-radius count: min %d, median %d, max %d" % (
        result["min_count"], int(np.median(result["counts"])), int(result["counts"].max())))
    print(verdict(result, args.local_map_min_points))

    if args.out_csv:
        write_csv(args.out_csv, t, xy, result)
        print(f"wrote per-pose coverage {args.out_csv}")
    if args.plot:
        render_plot(args.plot, t, result, args.local_map_min_points)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
