#!/usr/bin/env python3
"""WP1 G2 NDT search-method A/B benchmark (KDTREE vs DIRECT7 x threads).

Feeds identical seeded scans and identical candidate poses to the G2 NDT scorer
(g2_ndt_score.MapNdtScorer) under every requested (search_method, num_threads)
combination and records per-candidate latency plus per-combination summary.

The wall-clock numbers only matter on an idle machine: every rep records the
1-min load average and any rep taken at load >= 5 is flagged invalid, consistent
with benchmark_bbs_backends.py and docs/g2_bbs_speedup.md.

The map and occupancy grid are required inputs; the scan is a deterministic
seeded sample of occupied cells (same shape as benchmark_bbs_backends.py), so a
stock checkout can exercise the entire path without rosbag playback. Use it
directly:
    g++ -O2 -std=c++17 -shared -fPIC -I include -I /usr/include/pybind11 \\
        $(python3-config --includes) -I "$(python3 -c 'import numpy;print(numpy.get_include())')" \\
        src/g2_ndt_score_pybind.cpp -o /tmp/g2/g2_ndt_score$(python3-config --extension-suffix) \\
        -I src/ndt_omp_ros2/include
or rely on the colcon-built module (installed next to the node) when the package
is built with pybind11/Python3 Development.Module.

The runtime G2 default is unchanged: ndt_search_method=direct7, num_threads=1.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import global_localization_query as glq  # noqa: E402
import make_bbs_relocalization_attempts as bbs  # noqa: E402

LOAD_GATE = 5.0


def load1():
    return os.getloadavg()[0]


def median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def build_matching_grid(occ_map, dilate_cells):
    grid = occ_map.occupied
    for _ in range(max(0, dilate_cells)):
        grid = bbs._dilate_one_cell(grid)
    return grid


def seeded_scan_xy(occ_map, pose_xy, radius_m, max_points, seed):
    occ = occ_map.occupied
    ys, xs = np.nonzero(occ)
    wx = occ_map.origin_x_m + (xs + 0.5) * occ_map.resolution_m
    wy = occ_map.origin_y_m + (ys + 0.5) * occ_map.resolution_m
    d = np.hypot(wx - pose_xy[0], wy - pose_xy[1])
    sel = np.nonzero(d < radius_m)[0]
    rng = np.random.default_rng(seed)
    if len(sel) > max_points:
        sel = rng.choice(sel, max_points, replace=False)
    return np.stack([wx[sel] - pose_xy[0], wy[sel] - pose_xy[1]], axis=1)


def seeded_candidate_poses(occ_map, center_xy, radius_m, count, seed):
    """Determine deterministic candidate poses near center_xy for scoring."""
    rng = np.random.default_rng(seed)
    xs = center_xy[0] + rng.uniform(-radius_m, radius_m, size=count)
    ys = center_xy[1] + rng.uniform(-radius_m, radius_m, size=count)
    yaws = rng.uniform(-np.pi, np.pi, size=count)
    # score_candidates expects (x, y, z, yaw); z is fixed at the map seed plane.
    return [tuple(float(v) for v in pose) for pose in zip(xs, ys, np.zeros_like(xs), yaws)]

def score_all(scorer, scan_xyz, poses, repeats):
    """Time one scoring pass over all poses; return per-pose timings."""
    counts = np.zeros(len(poses), dtype=int)
    total = 0.0
    for _ in range(repeats):
        start = time.perf_counter()
        results = scorer.score_candidates(scan_xyz, poses)
        total += time.perf_counter() - start
        for i, result in enumerate(results):
            if result.converged:
                counts[i] += 1
    per_pose = total / len(poses) / repeats
    return per_pose, total / repeats, counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, help="3D map (PLY/PCD) for NDT scoring")
    parser.add_argument(
        "--occupancy-yaml", required=True,
        help="occupancy map YAML used to synthesize the scan and candidates")
    parser.add_argument(
        "--methods", default="kdtree,direct7",
        help="comma-separated search methods to compare (kdtree/direct7)")
    parser.add_argument("--threads", default="1,4", help="comma-separated thread counts")
    parser.add_argument("--repeats", type=int, default=5, help="scoring passes per combo")
    parser.add_argument("--candidates", type=int, default=32)
    parser.add_argument("--candidate-radius-m", type=float, default=30.0)
    parser.add_argument("--scan-radius-m", type=float, default=60.0)
    parser.add_argument("--max-scan-points", type=int, default=512)
    parser.add_argument("--center-x", type=float, default=0.0)
    parser.add_argument("--center-y", type=float, default=0.0)
    parser.add_argument("--output", required=True, help="output directory")
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    glq._append_module_dirs("g2_ndt_score")
    try:
        import g2_ndt_score  # noqa: E402
    except ImportError as exc:
        parser.error("g2_ndt_score module unavailable: %s" % exc)

    occ_map = bbs.load_occupancy_map(Path(args.occupancy_yaml))
    scan_xy = seeded_scan_xy(
        occ_map,
        (args.center_x, args.center_y),
        args.scan_radius_m,
        args.max_scan_points,
        seed=0,
    )
    if len(scan_xy) == 0:
        parser.error("no occupied cells within scan-radius of the center pose")
    scan_xyz = np.hstack([scan_xy, np.full((len(scan_xy), 1), 1.0)])

    poses = seeded_candidate_poses(
        occ_map, (args.center_x, args.center_y), args.candidate_radius_m,
        args.candidates, seed=1)

    methods = [m.strip().lower() for m in args.methods.split(",")]
    threads = [int(t) for t in args.threads.split(",")]

    summary = {"runs": [], "methods": methods, "threads": threads}
    csv_lines = ["method,threads,rep,load1,passes_sec,candidate_count,converged_total"]
    for method in methods:
        for thread_count in threads:
            config = glq.GlobalLocalizationConfig(
                enable_registration_scoring=True,
                map_path=args.map,
                ndt_num_threads=thread_count,
                ndt_search_method=method,
            )
            scorer = g2_ndt_score.MapNdtScorer(
                args.map,
                config.ndt_resolution,
                config.ndt_step_size,
                config.ndt_transform_epsilon,
                config.ndt_max_iterations,
                config.ndt_num_threads,
                glq.resolve_pclomp_search_method(config.ndt_search_method),
                config.ndt_scan_voxel_leaf_size,
                config.ndt_target_voxel_leaf_size,
                config.ndt_local_map_radius,
                config.ndt_min_target_points,
            )
            start = time.perf_counter()
            passes_sec, once_sec, converged = score_all(scorer, scan_xyz, poses, args.repeats)
            per_candidate_ms = 1000.0 * passes_sec
            load = load1()
            valid = load < LOAD_GATE
            summary["runs"].append({
                "method": method,
                "threads": thread_count,
                "load1": load,
                "valid": valid,
                "passes_sec": passes_sec,
                "once_sec": once_sec,
                "per_candidate_ms_median": per_candidate_ms,
                "converged_total": int(converged.sum()),
                "candidate_count": len(poses),
                "wall_clock_sec": time.perf_counter() - start,
            })
            for rep in range(1, args.repeats + 1):
                csv_lines.append(
                    f"{method},{thread_count},{rep},{load:.2f},"
                    f"{passes_sec:.6f},{len(poses)},{int(converged.sum())}")
            print(f"{method}/t{thread_count}: per-candidate {per_candidate_ms:.3f} ms "
                  f"(load {load:.2f}, {'valid' if valid else 'INVALID'})")

    overpass = None
    for item in summary["runs"]:
        if item["valid"] and overpass is None:
            overpass = item  # first valid run is written for reference only
    summary["note"] = (
        "Read per_candidate_ms_median once; only runs with valid=true are "
        "timing claims. Candidate rank, fitness, and seed error stay identical "
        "across methods inside the scorer; search method is a latency factor.")
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    (output / "candidate_latency.csv").write_text("\n".join(csv_lines) + "\n")
    print("wrote", output / "summary.json")
    print("wrote", output / "candidate_latency.csv")


if __name__ == "__main__":
    main()