#!/usr/bin/env python3
"""WP2 offline A/B: map-wide BBS_2D vs route-crop candidates on real scans.

Compares the two candidate sources for the same query scans from a Koide bag:
- map-wide BBS_2D (make_bbs_relocalization_attempts) may return along-corridor
  aliases 100+ m from the true pose (the documented north/south corridor alias);
- route-crop (make_route_grid_relocalization_attempts) seeds candidates only on
  the known reference route near the query time, so far aliases cannot appear.

For each query stamp the script extracts the real scan, generates both candidate
sets, and evaluates with GT: recall (min XY distance from the true pose to the
top-K) and alias presence (count of top-K candidates farther than
--alias-distance-m from the true pose). Ground truth is used for evaluation
only, never to order or generate candidates.

Usage:
    run_wp2_route_crop_ab.py \\
        --bag /path/outdoor_hard_01a \\
        --occupancy-yaml /path/outdoor_hard_bbs.yaml \\
        --reference-csv /path/reference.csv \\
        --gt-csv /path/gt/traj_lidar_outdoor_hard_01.txt \\
        --output /tmp/wp2_route_crop_ab --query-count 12
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import make_bbs_relocalization_attempts as bbs  # noqa: E402
import make_route_grid_relocalization_attempts as route_grid  # noqa: E402

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
PC2 = TYPESTORE.types["sensor_msgs/msg/PointCloud2"]


def load_gt(gt_txt: Path):
    """GT as {stamp_sec: (x, y)} from the koide traj txt (space-separated).

    Format per line: timestamp x y z qx qy qz qw (no header).
    """
    gt = {}
    with open(gt_txt, encoding="utf-8") as stream:
        for line in stream:
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                gt[float(parts[0])] = (float(parts[1]), float(parts[2]))
            except ValueError:
                continue
    return gt


def gt_pose_at(gt, stamp_sec):
    """Interpolate GT pose at stamp_sec from nearest GT samples."""
    stamps = np.array(sorted(gt.keys()))
    idx = np.searchsorted(stamps, stamp_sec)
    if idx <= 0:
        return gt[stamps[0]]
    if idx >= len(stamps):
        return gt[stamps[-1]]
    t0, t1 = stamps[idx - 1], stamps[idx]
    w = (stamp_sec - t0) / max(1e-9, t1 - t0)
    x = gt[t0][0] * (1 - w) + gt[t1][0] * w
    y = gt[t0][1] * (1 - w) + gt[t1][1] * w
    return (x, y)


def load_reference(reference_csv: Path):
    return route_grid.load_reference_rows(reference_csv)


def read_scan(bag_path, cloud_topic, stamp_ns):
    with Reader(bag_path) as reader:
        conn = [c for c in reader.connections if c.topic == cloud_topic][0]
        best, best_dt = None, None
        for c, ts, raw in reader.messages(connections=[conn]):
            dt = abs(ts - stamp_ns)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = (ts, raw)
        return _deserialize_points(best[1])


def read_scans_batched(bag_path, cloud_topic, stamps_ns):
    """Read one nearest scan per requested stamp in a single bag pass."""
    best_raw = [None] * len(stamps_ns)
    best_dt = [None] * len(stamps_ns)
    with Reader(bag_path) as reader:
        conn = [c for c in reader.connections if c.topic == cloud_topic][0]
        for c, ts, raw in reader.messages(connections=[conn]):
            for i, target in enumerate(stamps_ns):
                if best_raw[i] is None:
                    best_raw[i] = raw
                    best_dt[i] = abs(ts - target)
                    continue
                dt = abs(ts - target)
                if dt < best_dt[i]:
                    best_dt[i] = dt
                    best_raw[i] = raw
    return [_deserialize_points(best_raw[i]) for i in range(len(stamps_ns))]


def _deserialize_points(raw):
    msg = TYPESTORE.deserialize_cdr(raw, PC2.__msgtype__)
    offs = {f.name: f.offset for f in msg.fields}
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = []
    for name in ("x", "y", "z"):
        o = offs[name]
        xyz.append(np.frombuffer(data[:, o:o + 4].tobytes(), dtype="<f4"))
    return np.stack(xyz, axis=1).astype(np.float64)


def bbs_candidates(occupancy_yaml, scan_xyz, resolution_m, angular_res, max_cand, nms_m):
    occ = bbs.load_occupancy_map(Path(occupancy_yaml))
    matching = occ.occupied
    for _ in range(max(0, 1)):
        matching = bbs._dilate_one_cell(matching)
    scan_xy = bbs.prepare_scan_xy(
        scan_xyz, z_min_m=0.5, z_max_m=5.0, voxel_size_m=resolution_m,
        max_scan_points=512, min_range_m=1.0)
    if scan_xy.shape[0] == 0:
        return []
    search = bbs.branch_and_bound_candidates
    if getattr(bbs, "_CPP_BACKEND", None):
        search = bbs._CPP_BACKEND
    cands = search(
        matching, scan_xy, resolution_m, angular_res, 4, max_cand,
        nms_radius_cells=int(round(nms_m / resolution_m)))
    return [
        (occ.grid_cell_center_to_world(c.tx_cell, c.ty_cell)[0],
         occ.grid_cell_center_to_world(c.tx_cell, c.ty_cell)[1])
        for c in cands
    ]


def route_candidates(reference, trigger_stamp, radius_sec, min_spacing, max_route_poses,
                     max_cand, yaw_offsets, lateral_offsets, longitudinal_offsets):
    rows, _nearest = route_grid.select_route_rows(
        reference, trigger_stamp, radius_sec, min_spacing, max_route_poses)
    cands = route_grid.build_candidates(
        "ab", rows, trigger_stamp, "route", longitudinal_offsets,
        lateral_offsets, yaw_offsets, 0)
    return [(float(c["pose_x"]), float(c["pose_y"])) for c in cands]


def _load_cpp_backend():
    try:
        import bbs_cpp  # noqa: F401
        bbs._CPP_BACKEND = bbs_cpp.branch_and_bound_candidates
        print("using bbs_cpp backend")
    except ImportError as exc:
        bbs._CPP_BACKEND = None
        print("bbs_cpp unavailable (%s); using python backend" % exc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--cloud-topic", default="/livox/points")
    parser.add_argument("--occupancy-yaml", required=True)
    parser.add_argument("--reference-csv", required=True)
    parser.add_argument("--gt-csv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--query-count", type=int, default=12)
    parser.add_argument("--query-stamps", default="", help="comma-separated override")
    parser.add_argument("--topk", type=int, default=8)
    parser.add_argument("--alias-distance-m", type=float, default=50.0)
    parser.add_argument("--route-time-radius-sec", type=float, default=20.0)
    parser.add_argument("--route-min-spacing-m", type=float, default=8.0)
    parser.add_argument("--max-route-poses", type=int, default=60)
    parser.add_argument("--max-candidates", type=int, default=32)
    parser.add_argument("--yaw-offsets-deg", default="-15,0,15")
    parser.add_argument("--lateral-offsets-m", default="-2,0,2")
    parser.add_argument("--longitudinal-offsets-m", default="-1,0,1")
    args = parser.parse_args()

    resolution_m = float(bbs.load_occupancy_map(Path(args.occupancy_yaml)).resolution_m)
    reference = load_reference(Path(args.reference_csv))
    gt = load_gt(Path(args.gt_csv))
    _load_cpp_backend()
    t0 = float(reference[0]["stamp_sec"])
    t1 = float(reference[-1]["stamp_sec"])

    if args.query_stamps:
        stamps = [float(s) for s in args.query_stamps.split(",")]
    else:
        stamps = [t0 + (t1 - t0) * i / max(1, args.query_count) for i in range(args.query_count)]

    yaw_offsets = route_grid._parse_float_list(args.yaw_offsets_deg)
    lateral = route_grid._parse_float_list(args.lateral_offsets_m)
    longitudinal = route_grid._parse_float_list(args.longitudinal_offsets_m)

    rows = []
    scans = read_scans_batched(args.bag, args.cloud_topic, [int(s * 1e9) for s in stamps])
    for i, stamp in enumerate(stamps):
        true_x, true_y = gt_pose_at(gt, stamp)
        scan = scans[i]
        bbs_top = bbs_candidates(
            args.occupancy_yaml, scan, resolution_m, math.radians(5.0),
            args.max_candidates, 2.0)[: args.topk]
        # Route-crop candidates are route-bounded but unscored here (registration
        # scoring is downstream); measure the full candidate set, not a truncated
        # slice, so a later route row's near-true candidate is not lost.
        route_top = route_candidates(
            reference, stamp, args.route_time_radius_sec, args.route_min_spacing_m,
            args.max_route_poses, args.max_candidates, yaw_offsets, lateral, longitudinal)

        def recall(cands):
            if not cands:
                return None, None
            dists = [math.hypot(x - true_x, y - true_y) for x, y in cands]
            return min(dists), dists

        bbs_best, bbs_dists = recall(bbs_top)
        route_best, route_dists = recall(route_top)
        bbs_aliases = sum(1 for d in bbs_dists if d > args.alias_distance_m) if bbs_dists else 0
        route_aliases = sum(1 for d in route_dists if d > args.alias_distance_m) if route_dists else 0
        rows.append({
            "stamp_sec": round(stamp, 3),
            "true_x": round(true_x, 3),
            "true_y": round(true_y, 3),
            "bbs_topk_recall_m": round(bbs_best, 3) if bbs_best is not None else None,
            "route_topk_recall_m": round(route_best, 3) if route_best is not None else None,
            "bbs_topk_alias_count": bbs_aliases,
            "route_topk_alias_count": route_aliases,
        })

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    recall_ok = sum(1 for r in rows if r["route_topk_recall_m"] is not None and r["route_topk_recall_m"] <= 5.0)
    alias_free = sum(1 for r in rows if r["route_topk_alias_count"] == 0)
    bbs_alias_free = sum(1 for r in rows if r["bbs_topk_alias_count"] == 0)
    summary = {
        "query_count": len(rows),
        "route_topk_recall_le_5m": recall_ok,
        "route_topk_alias_free": alias_free,
        "bbs_topk_alias_free": bbs_alias_free,
        "note": "route-crop must keep recall@K (<=5 m) while eliminating far aliases; GT used only for evaluation.",
    }
    summary_path = out.with_name("summary.json")
    summary_path.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    for r in rows:
        print(f"t={r['stamp_sec']:9.1f} BBS recall={r['bbs_topk_recall_m']} "
              f"aliases={r['bbs_topk_alias_count']} | route recall={r['route_topk_recall_m']} "
              f"aliases={r['route_topk_alias_count']}")


if __name__ == "__main__":
    main()
