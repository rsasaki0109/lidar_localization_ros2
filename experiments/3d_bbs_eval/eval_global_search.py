#!/usr/bin/env python3
"""Ask G2 or 3D-BBS about recorded scans and compare the answers with ground truth.

Both methods see the same scans, levelled with the odometry attitude the way
global_localization_node.py levels them (g2_offline_probe.py does the same):

  g2      GlobalLocalizationEngine (scripts/global_localization_query.py): BBS on
          the 2D occupancy grid, then NDT refinement and ranking of the top
          candidates (the cross-session suite settings by default). With
          --config '{"global_search_engine": "bbs3d"}' the engine's own 3D search
          (bbs3d_cpp) answers instead.
  3dbbs   3D-BBS (CPU) over a hierarchical 3D voxel map of the same map.pcd
          (bbs3d_py, built from bbs3d_pybind.cpp), x/y/z and yaw with roll and
          pitch within +-0.02 rad; optionally its k best non-suppressed leaves,
          then the same NDT refinement and ranking as G2 (--refine).

The output JSON has g2_offline_probe.py's layout (queries with candidates and
their errors, a summary) plus height errors, time and peak memory, so the
startup replay (simulate_startup.py) reads either.

usage: eval_global_search.py --method {g2,3dbbs} --bag BAG --ground-truth gt.tum
           --map map.pcd --occupancy occ.yaml --out result.json [options]
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "tools" / "benchmark"))
sys.path.insert(0, str(REPO / "experiments" / "go2_wide_map_global_init"))

import build_map_from_odometry as map_builder  # noqa: E402
import g2_offline_probe as probe  # noqa: E402
import global_localization_query as glq  # noqa: E402

# The cross-session suite's G2 settings
# (tools/benchmark/suites/go2_jeplo_cross_session.yaml) on top of quickstart's.
CROSS_SESSION_CONFIG = {
    **probe.QUICKSTART_CONFIG,
    "max_scan_points": 1024,
    "max_candidates": 16,
    "registration_refine_candidates": True,
    "z_min_m": 0.0,
    "sparse_band_min_points": 512,
    "nms_radius_m": 3.0,
    "registration_sensor_height_m": 1.0,
}


def peak_rss_mb() -> float:
    """Peak resident set size of this process (VmHWM), MB."""
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmHWM:"):
                return float(line.split()[1]) / 1024.0
    except OSError:
        pass
    return float("nan")


def read_ply_xyz(path: Path) -> np.ndarray:
    """x y z of a binary little-endian PLY whose vertices are float32 properties."""
    with open(path, "rb") as stream:
        count, names = 0, []
        while True:
            line = stream.readline().decode("ascii", "replace").strip()
            parts = line.split()
            if parts[:2] == ["element", "vertex"]:
                count = int(parts[2])
            elif parts[:2] == ["property", "float"]:
                names.append(parts[2])
            elif parts and parts[0] == "format" and parts[1] != "binary_little_endian":
                raise ValueError(f"{path}: only binary_little_endian PLY")
            elif line == "end_header":
                break
        raw = np.frombuffer(stream.read(count * 4 * len(names)), dtype="<f4")
        table = raw.reshape(count, len(names))
        data = table[:, [names.index(axis) for axis in ("x", "y", "z")]]
    data = np.asarray(data, dtype=np.float64)
    return data[np.isfinite(data).all(axis=1)]


def read_pcd_xyz(path: Path) -> np.ndarray:
    """x y z of a binary or ascii PCD whose first three fields are float32 x y z."""
    if path.suffix.lower() == ".ply":
        return read_ply_xyz(path)
    with open(path, "rb") as stream:
        header = {}
        while True:
            line = stream.readline().decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key] = value.split()
            if key == "DATA":
                break
        count = int(header["POINTS"][0])
        fields = header["FIELDS"]
        sizes = [int(s) for s in header["SIZE"]]
        if header["DATA"][0] == "ascii":
            data = np.loadtxt(stream, ndmin=2)[:, :3]
        else:
            if header["DATA"][0] != "binary" or fields[:3] != ["x", "y", "z"]:
                raise ValueError(f"{path}: only binary/ascii PCD with x y z first")
            stride = sum(sizes)
            raw = np.frombuffer(stream.read(count * stride), dtype=np.uint8)
            data = (
                raw.reshape(count, stride)[:, :12]
                .copy()
                .view(np.float32)
                .reshape(count, 3)
            )
    data = np.asarray(data, dtype=np.float64)
    return data[np.isfinite(data).all(axis=1)]


def yaw_matrix(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Bbs3dSearch:
    """3D-BBS over a voxelized map; answers candidates as GlobalLocalizationCandidate."""

    def __init__(self, args, map_xyz: np.ndarray):
        sys.path.insert(0, str(args.bbs3d_module_dir))
        import bbs3d_py

        self.args = args
        points = (
            map_builder.voxel_filter(map_xyz, args.map_leaf)
            if args.map_leaf > 0
            else map_xyz
        )
        self.map_points = len(points)
        self.searcher = bbs3d_py.Searcher(
            points, args.min_level_res, args.max_level, args.threads
        )
        lo, hi = points.min(axis=0), points.max(axis=0)
        if args.z_range is not None:
            lo[2], hi[2] = args.z_range
        self.searcher.set_trans_range(lo, hi)
        tilt = args.tilt_rad
        self.searcher.set_angular_range(
            np.array([-tilt, -tilt, 0.0]), np.array([tilt, tilt, 2.0 * math.pi])
        )

    def prepare_scan(self, levelled: np.ndarray) -> np.ndarray:
        ranges = np.linalg.norm(levelled, axis=1)
        keep = (ranges >= self.args.scan_min_range) & (
            ranges <= self.args.scan_max_range
        )
        scan = levelled[keep]
        if self.args.scan_leaf > 0 and len(scan):
            scan = map_builder.voxel_filter(scan, self.args.scan_leaf)
        return scan

    def query(self, levelled: np.ndarray):
        scan = self.prepare_scan(levelled)
        if len(scan) == 0:
            return [], {"scan_points": 0, "elapsed_ms": 0.0}
        if self.args.k <= 1 and self.args.library_localize:
            result = self.searcher.localize(scan, 0.0, self.args.timeout_ms)
            found = (
                [
                    {
                        "pose": result["pose"],
                        "score_percentage": result["score_percentage"],
                    }
                ]
                if result["localized"]
                else []
            )
            info = {
                "elapsed_ms": result["elapsed_ms"],
                "timed_out": result["timed_out"],
            }
        else:
            result = self.searcher.localize_topk(
                scan,
                self.args.k,
                self.args.nms_radius_m,
                math.radians(self.args.nms_yaw_deg),
                0.0,
                self.args.timeout_ms,
                self.args.relative_floor,
            )
            found = result["candidates"]
            info = {
                key: result[key]
                for key in (
                    "elapsed_ms",
                    "timed_out",
                    "expanded",
                    "max_queue",
                    "initial",
                )
            }
        info["scan_points"] = len(scan)
        candidates = []
        for item in found:
            pose = np.asarray(item["pose"])
            score = float(item["score_percentage"])
            candidates.append(
                glq.GlobalLocalizationCandidate(
                    x_m=float(pose[0, 3]),
                    y_m=float(pose[1, 3]),
                    z_m=float(pose[2, 3]),
                    yaw_rad=math.atan2(pose[1, 0], pose[0, 0]),
                    score=score,
                    hit_count=0,
                    point_count=len(scan),
                    bbs_score=score,
                )
            )
        return candidates, info


def candidate_record(candidate, truth_t, truth_yaw) -> dict:
    error = math.hypot(candidate.x_m - truth_t[0], candidate.y_m - truth_t[1])
    return {
        "x": candidate.x_m,
        "y": candidate.y_m,
        "z": candidate.z_m,
        "yaw_deg": math.degrees(candidate.yaw_rad),
        "score": candidate.score,
        "bbs_score": candidate.bbs_score,
        "fitness": candidate.registration_fitness,
        "error_m": error,
        "z_error_m": candidate.z_m - float(truth_t[2]),
        "yaw_error_deg": math.degrees(abs(probe.wrap(candidate.yaw_rad - truth_yaw))),
    }


def run(args) -> dict:
    odometry = map_builder.load_tum(args.odometry or args.ground_truth)
    truth = map_builder.load_tum(args.ground_truth)
    overrides = json.loads(args.config)
    config = glq.GlobalLocalizationConfig(
        **{**CROSS_SESSION_CONFIG, **overrides, "map_path": str(args.map)}
    )
    rss_start = peak_rss_mb()
    setup_start = time.monotonic()
    # The G2 engine also carries the NDT scorer that refines 3D-BBS candidates.
    engine = glq.GlobalLocalizationEngine(config, occupancy_yaml=args.occupancy)
    if engine.registration_scoring_error:
        raise RuntimeError(engine.registration_scoring_error)
    bbs3d = None
    if args.method == "3dbbs":
        bbs3d = Bbs3dSearch(args, read_pcd_xyz(args.map))
    setup_sec = time.monotonic() - setup_start

    start = max(truth[0, 0], odometry[0, 0]) + args.skip_sec
    end = min(truth[-1, 0], odometry[-1, 0], start + args.window_sec)
    gaps = np.where((np.diff(truth[:, 0]) > 2.0) & (truth[:-1, 0] >= start))[0]
    if gaps.size:
        end = min(end, truth[gaps[0], 0])
    stamps = list(np.arange(start, end, args.interval))
    scans = probe.read_scans(args.bag, stamps, args.topic)

    queries = []
    for scan_stamp, xyz in scans:
        odom_r, _odom_t = probe.pose_at(odometry, scan_stamp)
        odom_t = _odom_t
        levelled = probe.level_scan(xyz, odom_r)
        gt_r, gt_t = probe.pose_at(truth, scan_stamp)
        gt_yaw = probe.yaw_of(gt_r)

        search_info = {}
        started = time.monotonic()
        if bbs3d is None:
            result = engine.query(levelled)
            candidates = result.candidates
            # global_search_engine=bbs3d (--config): the package's own engine.
            search_info = dict(result.search_info or {})
            raw_top = None
            search_sec = time.monotonic() - started
            refine_sec = 0.0
            band_points = result.scan_point_count
        else:
            candidates, search_info = bbs3d.query(levelled)
            search_sec = time.monotonic() - started
            raw_top = (
                candidate_record(candidates[0], gt_t, gt_yaw) if candidates else None
            )
            refine_started = time.monotonic()
            if args.refine and candidates:
                candidates = engine._score_with_registration(
                    levelled, candidates, refine=True
                )
            refine_sec = time.monotonic() - refine_started
            band_points = search_info.get("scan_points", 0)
        runtime = time.monotonic() - started

        records = [candidate_record(c, gt_t, gt_yaw) for c in candidates]
        near_rank = next(
            (
                rank
                for rank, c in enumerate(records)
                if c["error_m"] <= probe.NEAR_TRANSLATION_M
                and c["yaw_error_deg"] <= probe.NEAR_YAW_DEG
            ),
            None,
        )
        truth_fitness = None
        if args.truth_fitness:
            truth_candidate = glq.GlobalLocalizationCandidate(
                x_m=float(gt_t[0]),
                y_m=float(gt_t[1]),
                z_m=float(gt_t[2]),
                yaw_rad=gt_yaw,
                score=0.0,
                hit_count=0,
                point_count=0,
                bbs_score=0.0,
            )
            truth_fitness = engine._score_with_registration(
                levelled, [truth_candidate], refine=False
            )[0].registration_fitness
        queries.append(
            {
                "stamp": scan_stamp,
                "odom": [float(odom_t[0]), float(odom_t[1]), probe.yaw_of(odom_r)],
                "truth": [float(gt_t[0]), float(gt_t[1]), gt_yaw, float(gt_t[2])],
                "band_points": band_points,
                "runtime_sec": runtime,
                "search_sec": search_sec,
                "refine_sec": refine_sec,
                "search_info": search_info,
                "near_rank": near_rank,
                "truth_bbs_score": 0.0,
                "truth_fitness": truth_fitness,
                "raw_top": raw_top,
                "candidates": records,
            }
        )
        if args.verbose and records:
            top = records[0]
            print(
                f"{scan_stamp - stamps[0]:7.1f}s err {top['error_m']:6.2f} m "
                f"dz {top['z_error_m']:+6.2f} yaw {top['yaw_error_deg']:6.1f} "
                f"t {runtime:6.2f}s",
                flush=True,
            )
    info = {
        "setup_sec": setup_sec,
        "rss_before_setup_mb": rss_start,
        "peak_rss_mb": peak_rss_mb(),
    }
    if engine.bbs3d_info is not None:
        info["engine_bbs3d"] = engine.bbs3d_info
    if bbs3d is not None:
        info.update(
            {
                "voxelmap_build_ms": bbs3d.searcher.build_ms(),
                "voxelmap_buckets": bbs3d.searcher.voxel_count(),
                "map_points": bbs3d.map_points,
            }
        )
    return {"config": dataclasses.asdict(config), "info": info, "queries": queries}


def success(record: dict, max_error_m: float, max_yaw_deg: float) -> bool:
    return record["error_m"] <= max_error_m and record["yaw_error_deg"] <= max_yaw_deg


def summarize(queries: list, max_error_m: float, max_yaw_deg: float) -> dict:
    n = len(queries)
    if n == 0:
        return {"queries": 0}
    runtimes = np.array([q["runtime_sec"] for q in queries])
    tops = [q["candidates"][0] if q["candidates"] else None for q in queries]
    ok = [t is not None and success(t, max_error_m, max_yaw_deg) for t in tops]
    wrong_floor = [t is not None and abs(t["z_error_m"]) > 1.5 for t in tops]
    raw = [q["raw_top"] for q in queries]
    summary = {
        "queries": n,
        "success": sum(ok) / n,
        "wrong_floor": sum(wrong_floor) / n,
        "any_near": sum(q["near_rank"] is not None for q in queries) / n,
        "runtime_median_sec": float(np.median(runtimes)),
        "runtime_p95_sec": float(np.percentile(runtimes, 95)),
        "search_median_sec": float(np.median([q["search_sec"] for q in queries])),
        "refine_median_sec": float(np.median([q["refine_sec"] for q in queries])),
    }
    if any(r is not None for r in raw):
        summary["raw_success"] = (
            sum(r is not None and success(r, max_error_m, max_yaw_deg) for r in raw) / n
        )
        summary["raw_wrong_floor"] = (
            sum(r is not None and abs(r["z_error_m"]) > 1.5 for r in raw) / n
        )
    summary["startup"] = probe.simulate_startup(queries)
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--method", choices=["g2", "3dbbs"], required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument(
        "--odometry", type=Path, help="attitude for levelling (default: ground truth)"
    )
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--occupancy", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--config", default="{}", help="JSON overrides of the G2 config"
    )
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--skip-sec", type=float, default=2.0)
    parser.add_argument("--window-sec", type=float, default=400.0)
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--success-m", type=float, default=1.0)
    parser.add_argument("--success-yaw-deg", type=float, default=5.0)
    parser.add_argument("--truth-fitness", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    group = parser.add_argument_group("3D-BBS")
    group.add_argument(
        "--bbs3d-module-dir", type=Path, default=Path("."), help="where bbs3d_py*.so is"
    )
    group.add_argument("--min-level-res", type=float, default=0.5)
    group.add_argument("--max-level", type=int, default=6)
    group.add_argument("--map-leaf", type=float, default=0.1)
    group.add_argument("--scan-leaf", type=float, default=0.5)
    group.add_argument("--scan-min-range", type=float, default=1.0)
    group.add_argument("--scan-max-range", type=float, default=30.0)
    group.add_argument("--tilt-rad", type=float, default=0.02)
    group.add_argument(
        "--z-range", type=float, nargs=2, default=None, help="search z (default: map's)"
    )
    group.add_argument("--k", type=int, default=1)
    group.add_argument("--nms-radius-m", type=float, default=3.0)
    group.add_argument("--nms-yaw-deg", type=float, default=360.0)
    group.add_argument(
        "--relative-floor",
        type=float,
        default=0.9,
        help="top-k: prune below this share of the best leaf's score",
    )
    group.add_argument("--timeout-ms", type=int, default=0)
    group.add_argument("--threads", type=int, default=4)
    group.add_argument(
        "--library-localize",
        action="store_true",
        help="with --k 1, call the library's own BBS3D::localize()",
    )
    group.add_argument("--refine", action="store_true", help="NDT-refine and rank")
    args = parser.parse_args(argv)

    data = run(args)
    data["args"] = {k: str(v) for k, v in vars(args).items()}
    data["summary"] = summarize(data["queries"], args.success_m, args.success_yaw_deg)
    data["summary"].update(data["info"])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, indent=1) + "\n")
    print(json.dumps(data["summary"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
