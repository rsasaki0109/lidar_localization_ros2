#!/usr/bin/env python3
"""Run synthetic and Koide correspondence-localizability comparisons."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree

from experiments.correspondence_localizability.interface import CorrespondenceGeometry
from experiments.correspondence_localizability.variants import HessianSpectrum
from experiments.correspondence_localizability.variants import LpicpPlaneLineContribution
from experiments.correspondence_localizability.variants import XicpDirectionContribution
from experiments.point_cloud_io import read_xyz_pcd
from experiments.point_cloud_io import read_xyz_ply


VARIANTS = [HessianSpectrum(), XicpDirectionContribution(), LpicpPlaneLineContribution()]


def _reference_yaw(row: dict[str, str]) -> float:
    x = float(row["orientation_x"])
    y = float(row["orientation_y"])
    z = float(row["orientation_z"])
    w = float(row["orientation_w"])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _geometry(
    points: np.ndarray,
    normals: np.ndarray | None = None,
    lines: np.ndarray | None = None,
) -> CorrespondenceGeometry:
    count = len(points)
    return CorrespondenceGeometry(
        points=np.asarray(points, dtype=np.float64),
        plane_normals=np.zeros((count, 3)) if normals is None else normals,
        plane_mask=np.zeros(count, dtype=bool) if normals is None else np.ones(count, dtype=bool),
        line_directions=np.zeros((count, 3)) if lines is None else lines,
        line_mask=np.zeros(count, dtype=bool) if lines is None else np.ones(count, dtype=bool),
    )


def _synthetic_scenes() -> dict[str, CorrespondenceGeometry]:
    grid = np.linspace(-10.0, 10.0, 21)
    x, y = np.meshgrid(grid, grid)
    plane_points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    plane_normals = np.tile([0.0, 0.0, 1.0], (len(plane_points), 1))

    wall_a = np.column_stack((np.zeros(x.size), x.ravel(), y.ravel()))
    wall_b = np.column_stack((x.ravel(), np.zeros(x.size), y.ravel()))
    corner_points = np.vstack((wall_a, wall_b))
    corner_normals = np.vstack(
        (
            np.tile([1.0, 0.0, 0.0], (len(wall_a), 1)),
            np.tile([0.0, 1.0, 0.0], (len(wall_b), 1)),
        )
    )

    t = np.linspace(-10.0, 10.0, 101)
    line_x = np.column_stack((t, np.full_like(t, 3.0), np.full_like(t, 2.0)))
    line_y = np.column_stack((np.full_like(t, -4.0), t, np.full_like(t, -2.0)))
    line_z = np.column_stack((np.full_like(t, 5.0), np.full_like(t, -5.0), t))
    line_points = np.vstack((line_x, line_y, line_z))
    line_directions = np.vstack(
        (
            np.tile([1.0, 0.0, 0.0], (len(t), 1)),
            np.tile([0.0, 1.0, 0.0], (len(t), 1)),
            np.tile([0.0, 0.0, 1.0], (len(t), 1)),
        )
    )
    return {
        "single_plane": _geometry(plane_points, normals=plane_normals),
        "orthogonal_planes": _geometry(corner_points, normals=corner_normals),
        "three_axis_lines": _geometry(line_points, lines=line_directions),
        "planes_plus_lines": CorrespondenceGeometry(
            points=np.vstack((corner_points, line_points)),
            plane_normals=np.vstack((corner_normals, np.zeros_like(line_points))),
            plane_mask=np.r_[np.ones(len(corner_points), bool), np.zeros(len(line_points), bool)],
            line_directions=np.vstack((np.zeros_like(corner_points), line_directions)),
            line_mask=np.r_[np.zeros(len(corner_points), bool), np.ones(len(line_points), bool)],
        ),
    }


def _voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    keys = np.floor(points / leaf).astype(np.int64)
    _unique, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def _estimate_geometry(
    transformed_scan: np.ndarray,
    map_points: np.ndarray,
    tree: cKDTree,
    max_distance: float,
    neighbors: int,
    feature_threshold: float,
) -> CorrespondenceGeometry:
    distances, nearest = tree.query(transformed_scan, k=1, workers=-1)
    valid = np.isfinite(distances) & (distances <= max_distance)
    points = transformed_scan[valid]
    nearest = nearest[valid]
    if not len(points):
        return _geometry(points)
    _distances, neighborhoods = tree.query(map_points[nearest], k=neighbors, workers=-1)
    samples = map_points[neighborhoods]
    centered = samples - samples.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / neighbors
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    scale = np.maximum(eigenvalues[:, 2], np.finfo(float).eps)
    linearity = (eigenvalues[:, 2] - eigenvalues[:, 1]) / scale
    planarity = (eigenvalues[:, 1] - eigenvalues[:, 0]) / scale
    prefer_line = linearity > planarity
    line_mask = prefer_line & (linearity >= feature_threshold)
    plane_mask = (~prefer_line) & (planarity >= feature_threshold)
    return CorrespondenceGeometry(
        points=points,
        plane_normals=eigenvectors[:, :, 0],
        plane_mask=plane_mask,
        line_directions=eigenvectors[:, :, 2],
        line_mask=line_mask,
    )


def _serialize(result, runtime_sec: float) -> dict:
    return {
        "correspondence_count": result.correspondence_count,
        "eigenvalues": result.eigenvalues.tolist(),
        "weak_ratio": result.weak_ratio,
        "numerical_nullity": result.numerical_nullity,
        "direction_effective_support": result.direction_effective_support.tolist(),
        "runtime_sec": runtime_sec,
    }


def _analyze(geometry: CorrespondenceGeometry) -> dict[str, dict]:
    output = {}
    for variant in VARIANTS:
        started = time.monotonic()
        result = variant.analyze(geometry)
        output[variant.name] = _serialize(result, time.monotonic() - started)
    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-jobs-csv", required=True)
    parser.add_argument("--reference-csv", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--map-voxel-size", type=float, default=0.75)
    parser.add_argument("--scan-voxel-size", type=float, default=0.75)
    parser.add_argument("--max-correspondence-distance", type=float, default=1.5)
    parser.add_argument("--neighbors", type=int, default=12)
    parser.add_argument("--feature-threshold", type=float, default=0.3)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with Path(args.scan_jobs_csv).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    with Path(args.reference_csv).open(newline="", encoding="utf-8") as stream:
        reference = list(csv.DictReader(stream))
    attempts: dict[str, dict[str, str]] = {}
    for row in rows:
        attempts.setdefault(row["attempt_id"], row)
    map_paths = {row["map_path"] for row in rows}
    if len(map_paths) != 1:
        raise RuntimeError("exactly one map is required")
    map_points = _voxel_downsample(
        read_xyz_ply(Path(next(iter(map_paths)))), args.map_voxel_size
    )
    tree = cKDTree(map_points)

    real_results = {}
    for attempt_id, row in sorted(attempts.items()):
        stamp = float(row["trigger_stamp_sec"])
        target = min(reference, key=lambda item: abs(float(item["stamp_sec"]) - stamp))
        yaw = _reference_yaw(target)
        rotation = np.array(
            [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
        )
        translation = np.array(
            [float(target["position_x"]), float(target["position_y"]), float(target["position_z"])]
        )
        scan = _voxel_downsample(
            read_xyz_pcd(Path(row["scan_pcd_path"])), args.scan_voxel_size
        )
        transformed = (rotation @ scan.T).T + translation
        geometry = _estimate_geometry(
            transformed, map_points, tree, args.max_correspondence_distance,
            args.neighbors, args.feature_threshold,
        )
        real_results[attempt_id] = {
            "matched_point_count": len(geometry.points),
            "plane_count": int(geometry.plane_mask.sum()),
            "line_count": int(geometry.line_mask.sum()),
            "variants": _analyze(geometry),
        }

    synthetic = {name: _analyze(scene) for name, scene in _synthetic_scenes().items()}
    report = {
        "schema_version": 1,
        "config": vars(args),
        "synthetic": synthetic,
        "koide": real_results,
        "existing_ndt_hessian_precursor_repeats": "0/3",
        "decision": "negative_result_no_runtime_promotion",
        "decision_reason": (
            "direction-contribution diagnostics are informative offline, but no causal "
            "cross-repeat failure prediction has been demonstrated"
        ),
    }
    output = Path(args.output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output_json": str(output), "decision": report["decision"]}))


if __name__ == "__main__":
    main()
