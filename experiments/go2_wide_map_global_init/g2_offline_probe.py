#!/usr/bin/env python3
"""Ask the G2 global search about recorded scans and compare its answers with ground truth.

Runs the runtime GlobalLocalizationEngine (scripts/global_localization_query.py)
on scans of a PointCloud2 bag, levelled with the odometry attitude the way
global_localization_node.py levels them (odom -> cloud frame roll and pitch at
the scan stamp). For every query it records the candidates' errors against the
ground truth, the rank of the first candidate near the truth, the NDT fitness of
the top candidate and of the truth itself, the BBS score of the truth on the
grid, and the number of scan points left in the height band. A startup check
replays the quickstart gates (score, fitness ratio, two-query odometry
consensus) on consecutive queries.

usage: g2_offline_probe.py --bag BAG --odometry odom.tum --ground-truth gt.txt
           --occupancy occ.yaml --map map.pcd --out result.json [--config JSON]
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

import build_map_from_odometry as map_builder  # noqa: E402
import global_localization_query as glq  # noqa: E402
import make_bbs_relocalization_attempts as bbs_engine  # noqa: E402

# Quickstart's G2 settings (scripts/quickstart.py, launch/quickstart.launch.py)
# on top of the node's own defaults.
QUICKSTART_CONFIG = {
    "use_cpp_backend": True,
    "enable_registration_scoring": True,
    "registration_score_gate": 6.0,
    "registration_refine_candidates": False,
    "registration_seed_z_m": 0.0,
    "seed_z_m": 0.0,
    "registration_sensor_height_m": 1.0,
    "max_scan_points": 256,
    "angular_resolution_rad": math.radians(5.0),
    "max_candidates": 8,
    "nms_radius_m": 3.0,
}

# Startup gates (quickstart defaults; scripts/quickstart_model.py).
MIN_CANDIDATE_SCORE = 0.6
MAX_FITNESS_RATIO = 0.5
ALTERNATIVE_SEPARATION_M = 5.0
CONSENSUS_TRANSLATION_M = 2.0
CONSENSUS_PER_ODOM_M = 0.05
CONSENSUS_YAW_DEG = 20.0

# A candidate counts as the truth within these.
NEAR_TRANSLATION_M = 1.0
NEAR_YAW_DEG = 15.0


def yaw_of(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def pose_at(trajectory: np.ndarray, stamp: float):
    rotation, translation = map_builder.interpolate_poses(trajectory, np.array([stamp]))
    return rotation[0], translation[0]


def read_scans(bag: Path, stamps_wanted: list[float], topic: str = "/livox/lidar"):
    """(header stamp, xyz) of the first cloud at or after each wanted stamp."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = rosbag2_py.SequentialReader()
    info = rosbag2_py.Info().read_metadata(str(bag), "")
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    wanted = sorted(stamps_wanted)
    scans = []
    index = 0
    while reader.has_next() and index < len(wanted):
        _topic, data, _recv = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        if stamp + 1.0e-6 < wanted[index]:
            continue
        cloud = np.frombuffer(bytes(msg.data), dtype=map_builder.POINT)
        xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(np.float64)
        scans.append((stamp, xyz))
        while index < len(wanted) and wanted[index] <= stamp + 1.0e-6:
            index += 1
    return scans


def level_scan(xyz: np.ndarray, odom_rotation: np.ndarray) -> np.ndarray:
    _roll, _pitch, level = glq.level_attitude(odom_rotation)
    return xyz @ level.T


def grid_score(engine, scan_xy: np.ndarray, x: float, y: float, yaw: float) -> float:
    """Share of scan points on occupied (dilated) cells at a world pose."""
    occupancy = engine.occupancy_map
    c, s = math.cos(yaw), math.sin(yaw)
    world = scan_xy @ np.array([[c, s], [-s, c]]) + np.array([x, y])
    c, s = math.cos(occupancy.origin_yaw_rad), math.sin(occupancy.origin_yaw_rad)
    local = (world - np.array([occupancy.origin_x_m, occupancy.origin_y_m])) @ np.array(
        [[c, -s], [s, c]]
    )
    cells = np.floor(local / occupancy.resolution_m).astype(np.int64)
    grid = engine.matching_grid
    inside = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < grid.shape[1])
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < grid.shape[0])
    )
    hits = grid[cells[inside, 1], cells[inside, 0]].sum()
    return float(hits) / max(1, len(world))


def probe(args, config: glq.GlobalLocalizationConfig, accumulate_sec: float):
    odometry = map_builder.load_tum(args.odometry)
    truth = map_builder.load_tum(args.ground_truth)
    engine = glq.GlobalLocalizationEngine(config, occupancy_yaml=args.occupancy)
    if engine.registration_scoring_error:
        raise RuntimeError(engine.registration_scoring_error)

    # Query stamps: every --interval seconds where ground truth exists.
    start = max(truth[0, 0], odometry[0, 0]) + args.skip_sec
    end = min(truth[-1, 0], start + args.window_sec)
    gaps = np.where(np.diff(truth[:, 0]) > 0.5)[0]
    if gaps.size:
        end = min(end, truth[gaps[0], 0])
    stamps = list(np.arange(start, end, args.interval))
    needed = []
    for stamp in stamps:
        needed.append(stamp)
        if accumulate_sec > 0.0:
            needed.extend(np.arange(stamp - accumulate_sec, stamp, 0.1).tolist())
    scans = read_scans(args.bag, sorted(set(needed)))
    scan_by_stamp = {round(s, 3): xyz for s, xyz in scans}
    scan_stamps = np.array(sorted(scan_by_stamp))

    queries = []
    for stamp in stamps:
        nearest = scan_stamps[scan_stamps >= stamp - 1.0e-6]
        if nearest.size == 0:
            continue
        scan_stamp = float(nearest[0])
        odom_r, odom_t = pose_at(odometry, scan_stamp)
        xyz = scan_by_stamp[round(scan_stamp, 3)]
        if accumulate_sec > 0.0:
            # Older clouds moved into this cloud's frame with the odometry.
            parts = [xyz]
            for other in scan_stamps[
                (scan_stamps < scan_stamp - 1.0e-6)
                & (scan_stamps >= scan_stamp - accumulate_sec - 1.0e-6)
            ]:
                other_r, other_t = pose_at(odometry, float(other))
                relative_r = odom_r.T @ other_r
                relative_t = odom_r.T @ (other_t - odom_t)
                parts.append(
                    scan_by_stamp[round(float(other), 3)] @ relative_r.T + relative_t
                )
            xyz = np.concatenate(parts)
        levelled = level_scan(xyz, odom_r)
        gt_r, gt_t = pose_at(truth, scan_stamp)
        gt_yaw = yaw_of(gt_r)

        started = time.monotonic()
        result = engine.query(levelled)
        runtime = time.monotonic() - started
        candidates = []
        near_rank = None
        for rank, candidate in enumerate(result.candidates):
            error = math.hypot(candidate.x_m - gt_t[0], candidate.y_m - gt_t[1])
            yaw_error = math.degrees(abs(wrap(candidate.yaw_rad - gt_yaw)))
            if (
                near_rank is None
                and error <= NEAR_TRANSLATION_M
                and yaw_error <= NEAR_YAW_DEG
            ):
                near_rank = rank
            candidates.append(
                {
                    "x": candidate.x_m,
                    "y": candidate.y_m,
                    "yaw_deg": math.degrees(candidate.yaw_rad),
                    "score": candidate.score,
                    "bbs_score": candidate.bbs_score,
                    "fitness": candidate.registration_fitness,
                    "error_m": error,
                    "yaw_error_deg": yaw_error,
                }
            )
        scan_xy = bbs_engine.prepare_scan_xy(
            levelled,
            config.z_min_m,
            config.z_max_m,
            engine.occupancy_map.resolution_m,
            config.max_scan_points,
            min_range_m=config.min_range_m,
        )
        truth_bbs = (
            grid_score(engine, scan_xy, gt_t[0], gt_t[1], gt_yaw)
            if len(scan_xy)
            else 0.0
        )
        truth_fitness = None
        if engine.registration_scorer is not None:
            truth_candidate = glq.GlobalLocalizationCandidate(
                x_m=float(gt_t[0]),
                y_m=float(gt_t[1]),
                z_m=0.0,
                yaw_rad=gt_yaw,
                score=0.0,
                hit_count=0,
                point_count=0,
                bbs_score=0.0,
            )
            scored = engine._score_with_registration(levelled, [truth_candidate])
            truth_fitness = scored[0].registration_fitness
        queries.append(
            {
                "stamp": scan_stamp,
                "odom": [float(odom_t[0]), float(odom_t[1]), yaw_of(odom_r)],
                "truth": [float(gt_t[0]), float(gt_t[1]), gt_yaw],
                "band_points": result.scan_point_count,
                "runtime_sec": runtime,
                "near_rank": near_rank,
                "truth_bbs_score": truth_bbs,
                "truth_fitness": truth_fitness,
                "candidates": candidates,
            }
        )
    return queries


def startup_gates(query: dict) -> tuple[bool, str]:
    """Whether a query's top candidate passes the single-query startup gates."""
    candidates = query["candidates"]
    if not candidates:
        return False, "no_candidates"
    top = candidates[0]
    if top["score"] < MIN_CANDIDATE_SCORE:
        return False, "weak"
    alternative = None
    for other in candidates[1:]:
        fitness = other["fitness"]
        if fitness is None or not math.isfinite(fitness):
            continue
        if (
            math.hypot(other["x"] - top["x"], other["y"] - top["y"])
            < ALTERNATIVE_SEPARATION_M
        ):
            continue
        alternative = fitness if alternative is None else min(alternative, fitness)
    if alternative is None:
        if len(candidates) > 1 and top["score"] - candidates[1]["score"] < 0.05:
            return False, "ambiguous_margin"
    elif top["fitness"] > MAX_FITNESS_RATIO * alternative:
        return False, "ambiguous_registration"
    return True, "gated"


def simulate_startup(queries: list[dict]) -> dict:
    """First query whose gated top agrees, through odometry, with an earlier gated one.

    Follows decide_startup's odometry consensus (two samples, 2 m + 5 cm per metre
    travelled, 20 deg); verification by the localizer is not modelled.
    """
    anchors = []
    for index, query in enumerate(queries):
        ok, _reason = startup_gates(query)
        if not ok:
            continue
        top = query["candidates"][0]
        pose = (top["x"], top["y"], math.radians(top["yaw_deg"]))
        odom = query["odom"]
        for anchor_pose, anchor_odom in anchors:
            # Move the anchor's fix by the odometry motion since then.
            dx, dy = odom[0] - anchor_odom[0], odom[1] - anchor_odom[1]
            c, s = math.cos(-anchor_odom[2]), math.sin(-anchor_odom[2])
            local = (c * dx - s * dy, s * dx + c * dy)
            c, s = math.cos(anchor_pose[2]), math.sin(anchor_pose[2])
            predicted = (
                anchor_pose[0] + c * local[0] - s * local[1],
                anchor_pose[1] + s * local[0] + c * local[1],
                anchor_pose[2] + wrap(odom[2] - anchor_odom[2]),
            )
            travelled = math.hypot(dx, dy)
            if (
                math.hypot(pose[0] - predicted[0], pose[1] - predicted[1])
                <= CONSENSUS_TRANSLATION_M + CONSENSUS_PER_ODOM_M * travelled
                and math.degrees(abs(wrap(pose[2] - predicted[2]))) <= CONSENSUS_YAW_DEG
            ):
                error = math.hypot(
                    pose[0] - query["truth"][0], pose[1] - query["truth"][1]
                )
                return {
                    "initialized": True,
                    "query_index": index,
                    "elapsed_sec": query["stamp"] - queries[0]["stamp"],
                    "error_m": error,
                    "wrong": error > 1.0,
                }
        anchors.append((pose, odom))
        anchors = anchors[-5:]
    return {"initialized": False}


def summarize(queries: list[dict]) -> dict:
    n = len(queries)
    if not n:
        return {"queries": 0}
    top_errors = [q["candidates"][0]["error_m"] for q in queries if q["candidates"]]
    gated = [startup_gates(q) for q in queries]
    gated_tops = [
        q["candidates"][0] for q, (ok, _) in zip(queries, gated, strict=True) if ok
    ]
    return {
        "queries": n,
        "band_points_median": float(np.median([q["band_points"] for q in queries])),
        "top1_near": sum(q["near_rank"] == 0 for q in queries) / n,
        "any_near": sum(q["near_rank"] is not None for q in queries) / n,
        "top1_error_median_m": float(np.median(top_errors)) if top_errors else None,
        "gated": sum(ok for ok, _ in gated) / n,
        "gated_wrong": sum(c["error_m"] > NEAR_TRANSLATION_M for c in gated_tops),
        "gated_right": sum(c["error_m"] <= NEAR_TRANSLATION_M for c in gated_tops),
        "truth_bbs_median": float(np.median([q["truth_bbs_score"] for q in queries])),
        "top_bbs_median": float(
            np.median(
                [q["candidates"][0]["bbs_score"] for q in queries if q["candidates"]]
            )
        ),
        "truth_fitness_median": float(
            np.median(
                [q["truth_fitness"] for q in queries if q["truth_fitness"] is not None]
            )
        ),
        "top_fitness_median": float(
            np.median(
                [q["candidates"][0]["fitness"] for q in queries if q["candidates"]]
            )
        ),
        "runtime_median_sec": float(np.median([q["runtime_sec"] for q in queries])),
        "startup": simulate_startup(queries),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--occupancy", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--config", default="{}", help="JSON overrides of the G2 config"
    )
    parser.add_argument("--accumulate-sec", type=float, default=0.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--skip-sec", type=float, default=3.0)
    parser.add_argument("--window-sec", type=float, default=40.0)
    args = parser.parse_args(argv)

    overrides = json.loads(args.config)
    if "angular_resolution_deg" in overrides:
        overrides["angular_resolution_rad"] = math.radians(
            overrides.pop("angular_resolution_deg")
        )
    config = glq.GlobalLocalizationConfig(
        **{**QUICKSTART_CONFIG, **overrides, "map_path": str(args.map)}
    )
    queries = probe(args, config, args.accumulate_sec)
    summary = summarize(queries)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(
        json.dumps(
            {
                "config": dataclasses.asdict(config),
                "accumulate_sec": args.accumulate_sec,
                "summary": summary,
                "queries": queries,
            },
            indent=1,
        )
        + "\n"
    )
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    sys.exit(main())
