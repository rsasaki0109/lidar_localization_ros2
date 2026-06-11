#!/usr/bin/env python3
"""Generate BBS_2D global relocalization candidates from an occupancy map.

For each reinitialization request window in alignment_status.csv, the nearest
scan is read from the rosbag2 directory, projected to 2D inside a height band,
and matched against a multi-resolution occupancy pyramid with an admissible
branch-and-bound search over (x, y, yaw). The top-K poses are written using the
same relocalization_attempts.csv / relocalization_candidates.csv contract as
the route-grid and map-grid generators, ordered best-first so that downstream
candidate_index ordering reflects the BBS ranking. Like the sibling
generators, this helper does not score candidates with registration or reset
pose; registration scoring stays downstream.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image

_SCRIPT_DIR = str(Path(__file__).resolve().parent)
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

# The CSV contract is owned by the map-grid generator; import it so the three
# candidate generators cannot drift apart silently.
from make_map_grid_relocalization_attempts import (  # noqa: E402
    ATTEMPT_FIELDNAMES,
    CANDIDATE_FIELDNAMES,
)


@dataclass(frozen=True)
class RequestWindow:
    attempt_id: str
    trigger_stamp_sec: float
    start_stamp_sec: float
    end_stamp_sec: float
    row_count: int
    reason: str
    score: Optional[float]

    @property
    def trigger_stamp_ns(self) -> int:
        return int(round(self.trigger_stamp_sec * 1_000_000_000))


@dataclass(frozen=True)
class OccupancyMap:
    occupied: np.ndarray
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float

    def grid_cell_center_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        gx = (float(ix) + 0.5) * self.resolution_m
        gy = (float(iy) + 0.5) * self.resolution_m
        c = math.cos(self.origin_yaw_rad)
        s = math.sin(self.origin_yaw_rad)
        return (
            self.origin_x_m + c * gx - s * gy,
            self.origin_y_m + s * gx + c * gy,
        )


@dataclass(frozen=True)
class BbsGridCandidate:
    tx_cell: int
    ty_cell: int
    yaw_index: int
    yaw_rad: float
    score: float
    hit_count: int
    point_count: int


def normalize_angle_rad(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def build_occupancy_pyramid(occupancy: np.ndarray, depth: int) -> List[np.ndarray]:
    if occupancy.ndim != 2:
        raise ValueError("occupancy must be a 2D array")
    if depth < 0:
        raise ValueError("pyramid depth must be non-negative")

    levels = [occupancy.astype(bool, copy=True)]
    for _ in range(depth):
        previous = levels[-1]
        if previous.shape[0] == 1 and previous.shape[1] == 1:
            break
        levels.append(_max_pool_2x2(previous))
    return levels


def build_upper_bound_pyramid(occupancy_pyramid: Sequence[np.ndarray]) -> List[np.ndarray]:
    if not occupancy_pyramid:
        raise ValueError("occupancy pyramid must not be empty")
    levels = [occupancy_pyramid[0].astype(bool, copy=True)]
    for level in occupancy_pyramid[1:]:
        levels.append(_dilate_one_cell(level.astype(bool, copy=False)))
    return levels


def bbs_exact_score(
    occupancy: np.ndarray,
    scan_xy_grid: np.ndarray,
    tx_cell: int,
    ty_cell: int,
    yaw_rad: float,
) -> float:
    score, _hits, _count = _score_grid(
        occupancy.astype(bool, copy=False),
        scan_xy_grid,
        tx_cell,
        ty_cell,
        yaw_rad,
        factor=1,
    )
    return score


def bbs_upper_bound(
    upper_bound_pyramid: Sequence[np.ndarray],
    scan_xy_grid: np.ndarray,
    tx_cell: int,
    ty_cell: int,
    yaw_rad: float,
    level: int,
) -> float:
    if level < 0 or level >= len(upper_bound_pyramid):
        raise ValueError("level is outside the upper-bound pyramid")
    factor = 1 << level
    score, _hits, _count = _score_grid(
        upper_bound_pyramid[level].astype(bool, copy=False),
        scan_xy_grid,
        tx_cell,
        ty_cell,
        yaw_rad,
        factor=factor,
    )
    return score


def branch_and_bound_candidates(
    occupancy: np.ndarray,
    scan_xy_m: np.ndarray,
    resolution_m: float,
    angular_resolution_rad: float,
    pyramid_depth: int,
    max_candidates: int,
    nms_radius_cells: int = 0,
) -> List[BbsGridCandidate]:
    if max_candidates <= 0:
        return []
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    if angular_resolution_rad <= 0.0:
        raise ValueError("angular_resolution_rad must be positive")

    occupancy = occupancy.astype(bool, copy=False)
    if occupancy.ndim != 2:
        raise ValueError("occupancy must be 2D")
    if scan_xy_m.size == 0:
        return []

    scan_xy_grid = np.asarray(scan_xy_m, dtype=np.float64) / float(resolution_m)
    if scan_xy_grid.ndim != 2 or scan_xy_grid.shape[1] != 2:
        raise ValueError("scan_xy_m must have shape (N, 2)")

    pyramid = build_occupancy_pyramid(occupancy, pyramid_depth)
    upper_bound_pyramid = build_upper_bound_pyramid(pyramid)
    start_level = len(upper_bound_pyramid) - 1
    yaws = _yaw_samples(angular_resolution_rad)

    height, width = occupancy.shape
    initial_step = 1 << start_level
    n_points = int(scan_xy_grid.shape[0])

    # Every node's (tx_cell, ty_cell) is a multiple of 2^level, so
    # floor((tx + 0.5 + rx) / factor) decomposes into
    # tx / factor + floor((0.5 + rx) / factor). The second term depends only on
    # (yaw, level): precompute it once per pair and merge duplicate coarse cells
    # with counts, turning each node evaluation into a small integer gather
    # instead of a full scan rotation.
    offset_cache: List[List[Tuple[np.ndarray, np.ndarray, np.ndarray]]] = []
    for yaw_rad in yaws:
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        rx = c * scan_xy_grid[:, 0] - s * scan_xy_grid[:, 1]
        ry = s * scan_xy_grid[:, 0] + c * scan_xy_grid[:, 1]
        per_level = []
        for level in range(len(upper_bound_pyramid)):
            factor = float(1 << level)
            qx = np.floor((0.5 + rx) / factor).astype(np.int64)
            qy = np.floor((0.5 + ry) / factor).astype(np.int64)
            unique_cells, cell_counts = np.unique(
                np.stack([qy, qx], axis=1), axis=0, return_counts=True)
            per_level.append(
                (unique_cells[:, 0], unique_cells[:, 1], cell_counts))
        offset_cache.append(per_level)

    def full_hit_map(
        grid: np.ndarray,
        qy: np.ndarray,
        qx: np.ndarray,
        cell_counts: np.ndarray,
        n_blocks_y: int,
        n_blocks_x: int,
    ) -> np.ndarray:
        # hits[ky, kx] = sum over offsets of count * grid[ky + qy, kx + qx]
        # for every block at once, via FFT cross-correlation. Hit counts are
        # integers <= the scan size, far above the FFT round-trip error, so
        # rounding restores the exact integer the direct gather would produce.
        qy_min = int(qy.min())
        qx_min = int(qx.min())
        kernel_h = int(qy.max()) - qy_min + 1
        kernel_w = int(qx.max()) - qx_min + 1
        kernel = np.zeros((kernel_h, kernel_w), dtype=np.float64)
        np.add.at(kernel, (qy - qy_min, qx - qx_min), cell_counts.astype(np.float64))

        grid_height, grid_width = grid.shape
        padded = np.zeros(
            (n_blocks_y + kernel_h - 1, n_blocks_x + kernel_w - 1), dtype=np.float64)
        src_y0 = max(0, qy_min)
        src_x0 = max(0, qx_min)
        src_y1 = min(grid_height, n_blocks_y + qy_min + kernel_h - 1)
        src_x1 = min(grid_width, n_blocks_x + qx_min + kernel_w - 1)
        if src_y1 > src_y0 and src_x1 > src_x0:
            padded[
                src_y0 - qy_min:src_y1 - qy_min,
                src_x0 - qx_min:src_x1 - qx_min,
            ] = grid[src_y0:src_y1, src_x0:src_x1]

        fft_shape = (
            padded.shape[0] + kernel_h - 1,
            padded.shape[1] + kernel_w - 1,
        )
        try:
            from scipy.fft import next_fast_len
            fft_shape = (
                next_fast_len(fft_shape[0], real=True),
                next_fast_len(fft_shape[1], real=True),
            )
        except ImportError:
            pass
        spectrum = np.fft.rfft2(padded, fft_shape) * np.fft.rfft2(
            kernel[::-1, ::-1], fft_shape)
        correlation = np.fft.irfft2(spectrum, fft_shape)
        block_map = correlation[
            kernel_h - 1:kernel_h - 1 + n_blocks_y,
            kernel_w - 1:kernel_w - 1 + n_blocks_x,
        ]
        return np.rint(block_map).astype(np.int64)

    # Adaptive scoring: gather directly while a (yaw, level) pair is cold, and
    # switch to a one-shot FFT hit map for the whole level once the pair is hot
    # enough that the map pays for itself.
    hit_map_cache: dict = {}
    gather_call_counts: dict = {}

    def gather_hits(
        grid: np.ndarray, level: int, yaw_index: int, tx_cell: int, ty_cell: int
    ) -> int:
        key = (yaw_index, level)
        cached_map = hit_map_cache.get(key)
        if cached_map is not None:
            return int(cached_map[ty_cell >> level, tx_cell >> level])

        qy, qx, cell_counts = offset_cache[yaw_index][level]
        grid_height, grid_width = grid.shape
        calls = gather_call_counts.get(key, 0) + 1
        gather_call_counts[key] = calls
        if calls > max(64, (grid_height * grid_width) // 256):
            n_blocks_y = (height + (1 << level) - 1) >> level
            n_blocks_x = (width + (1 << level) - 1) >> level
            cached_map = full_hit_map(
                grid, qy, qx, cell_counts, n_blocks_y, n_blocks_x)
            hit_map_cache[key] = cached_map
            return int(cached_map[ty_cell >> level, tx_cell >> level])

        iy = qy + (ty_cell >> level)
        ix = qx + (tx_cell >> level)
        inside = (ix >= 0) & (iy >= 0) & (ix < grid_width) & (iy < grid_height)
        if not np.any(inside):
            return 0
        return int(
            cell_counts[inside][grid[iy[inside], ix[inside]]].sum())

    # Seed the queue by scoring every top-level block of a yaw at once: the
    # integer hit map is an accumulation of count-weighted shifted views of the
    # (zero-padded) top pyramid level.
    block_tys = list(range(0, height, initial_step))
    block_txs = list(range(0, width, initial_step))
    top_grid = upper_bound_pyramid[start_level]
    top_height, top_width = top_grid.shape

    heap: List[Tuple[float, int, int, int, int, int, float]] = []
    sequence = 0
    for yaw_index, yaw_rad in enumerate(yaws):
        qy, qx, cell_counts = offset_cache[yaw_index][start_level]
        pad_top = int(max(0, -qy.min()))
        pad_left = int(max(0, -qx.min()))
        pad_bottom = int(max(0, len(block_tys) - 1 + qy.max() - (top_height - 1)))
        pad_right = int(max(0, len(block_txs) - 1 + qx.max() - (top_width - 1)))
        padded = np.zeros(
            (pad_top + top_height + pad_bottom, pad_left + top_width + pad_right),
            dtype=np.int64)
        padded[pad_top:pad_top + top_height, pad_left:pad_left + top_width] = top_grid
        hit_map = np.zeros((len(block_tys), len(block_txs)), dtype=np.int64)
        for offset_y, offset_x, count in zip(qy, qx, cell_counts):
            row = pad_top + int(offset_y)
            col = pad_left + int(offset_x)
            hit_map += count * padded[
                row:row + len(block_tys), col:col + len(block_txs)]
        for block_row, ty_cell in enumerate(block_tys):
            for block_col, tx_cell in enumerate(block_txs):
                bound = float(hit_map[block_row, block_col]) / float(n_points)
                heap.append(
                    (-bound, sequence, start_level, tx_cell, ty_cell, yaw_index,
                     yaw_rad))
                sequence += 1
    # Heap keys are unique (sequence tiebreaker), so pop order is the sorted key
    # order regardless of internal heap layout; heapify keeps behavior identical
    # to sequential pushes.
    heapq.heapify(heap)

    best: List[BbsGridCandidate] = []
    kth_score = -math.inf
    eps = 1.0e-12

    while heap:
        neg_bound, _seq, level, tx_cell, ty_cell, yaw_index, yaw_rad = heapq.heappop(heap)
        bound = -neg_bound
        if len(best) >= max_candidates and bound <= kth_score + eps:
            break

        if level == 0:
            cached_level0 = hit_map_cache.get((yaw_index, 0))
            if cached_level0 is not None:
                hit_count = int(cached_level0[ty_cell, tx_cell])
            else:
                hit_count = gather_hits(occupancy, 0, yaw_index, tx_cell, ty_cell)
            candidate = BbsGridCandidate(
                tx_cell=tx_cell,
                ty_cell=ty_cell,
                yaw_index=yaw_index,
                yaw_rad=yaw_rad,
                score=float(hit_count) / float(n_points),
                hit_count=hit_count,
                point_count=n_points,
            )
            if nms_radius_cells > 0:
                # Keep the candidate list spatially diverse: one winner per
                # neighborhood, so downstream registration sees distinct
                # hypotheses instead of duplicates of the global maximum.
                suppressed = False
                for index, existing in enumerate(best):
                    if (
                        abs(existing.tx_cell - candidate.tx_cell) <= nms_radius_cells
                        and abs(existing.ty_cell - candidate.ty_cell) <= nms_radius_cells
                    ):
                        if candidate.score > existing.score:
                            best[index] = candidate
                        suppressed = True
                        break
                if not suppressed:
                    best.append(candidate)
            else:
                best.append(candidate)
            best.sort(key=_candidate_sort_key)
            if len(best) > max_candidates:
                best = best[:max_candidates]
            if len(best) >= max_candidates:
                kth_score = best[-1].score
            continue

        child_level = level - 1
        child_step = 1 << child_level
        for dy in (0, child_step):
            child_ty = ty_cell + dy
            if child_ty >= height:
                continue
            for dx in (0, child_step):
                child_tx = tx_cell + dx
                if child_tx >= width:
                    continue
                cached_child = hit_map_cache.get((yaw_index, child_level))
                if cached_child is not None:
                    child_bound = float(
                        cached_child[
                            child_ty >> child_level, child_tx >> child_level]
                    ) / float(n_points)
                else:
                    child_bound = float(
                        gather_hits(
                            upper_bound_pyramid[child_level], child_level,
                            yaw_index, child_tx, child_ty)) / float(n_points)
                if len(best) >= max_candidates and child_bound < kth_score - eps:
                    continue
                heapq.heappush(
                    heap,
                    (
                        -child_bound,
                        sequence,
                        child_level,
                        child_tx,
                        child_ty,
                        yaw_index,
                        yaw_rad,
                    ),
                )
                sequence += 1

    best.sort(key=_candidate_sort_key)
    return best[:max_candidates]


def load_occupancy_map(yaml_path: Path) -> OccupancyMap:
    with yaml_path.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle)

    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    image = Image.open(image_path).convert("L")
    pixels = np.asarray(image, dtype=np.float32)

    occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
    negate = int(metadata.get("negate", 0))
    if negate:
        occupied_probability = pixels / 255.0
    else:
        occupied_probability = (255.0 - pixels) / 255.0

    occupied_image = occupied_probability >= occupied_thresh
    occupied_grid = np.flipud(occupied_image)

    origin = metadata.get("origin", [0.0, 0.0, 0.0])
    return OccupancyMap(
        occupied=occupied_grid.astype(bool, copy=False),
        resolution_m=float(metadata["resolution"]),
        origin_x_m=float(origin[0]),
        origin_y_m=float(origin[1]),
        origin_yaw_rad=float(origin[2]) if len(origin) > 2 else 0.0,
    )


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_request_windows(alignment_csv: Path, source: str) -> List[RequestWindow]:
    """Group consecutive reinitialization-requested alignment rows into windows.

    This mirrors request_windows() in the route-grid and map-grid generators:
    alignment_status.csv rows carry the request state inside values_json.
    """
    rows: List[Dict[str, Any]] = []
    with alignment_csv.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            values = json.loads(record["values_json"])
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "requested": _as_bool(values.get("reinitialization_requested")),
                    "reason": values.get("reinitialization_request_reason"),
                    "score": _as_float(values.get("reinitialization_request_score")),
                }
            )

    windows: List[RequestWindow] = []
    index = 0
    while index < len(rows):
        if not rows[index]["requested"]:
            index += 1
            continue
        start = index
        while index < len(rows) and rows[index]["requested"]:
            index += 1
        end = index - 1
        windows.append(
            RequestWindow(
                attempt_id=f"{source}_{len(windows) + 1:04d}",
                trigger_stamp_sec=rows[start]["stamp_sec"],
                start_stamp_sec=rows[start]["stamp_sec"],
                end_stamp_sec=rows[end]["stamp_sec"],
                row_count=end - start + 1,
                reason=rows[start]["reason"] or "",
                score=rows[start]["score"],
            )
        )
    return windows


def prepare_scan_xy(
    points_xyz: np.ndarray,
    z_min_m: float,
    z_max_m: float,
    voxel_size_m: float,
    max_scan_points: int,
    min_range_m: float = 1.0,
) -> np.ndarray:
    if points_xyz.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    points_xyz = np.asarray(points_xyz, dtype=np.float64)
    finite = np.isfinite(points_xyz).all(axis=1)
    height_band = (points_xyz[:, 2] >= z_min_m) & (points_xyz[:, 2] <= z_max_m)
    # Livox-style invalid returns are zero-filled at the origin; a minimum
    # range removes them and the sensor-adjacent clutter.
    in_range = np.hypot(points_xyz[:, 0], points_xyz[:, 1]) >= max(0.0, min_range_m)
    xy = points_xyz[finite & height_band & in_range, :2]
    xy = voxel_downsample_2d(xy, voxel_size_m)
    if max_scan_points > 0 and xy.shape[0] > max_scan_points:
        keep = np.linspace(0, xy.shape[0] - 1, max_scan_points, dtype=np.int64)
        xy = xy[keep]
    return xy.astype(np.float64, copy=False)


def voxel_downsample_2d(points_xy: np.ndarray, voxel_size_m: float) -> np.ndarray:
    if points_xy.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    if voxel_size_m <= 0.0:
        raise ValueError("voxel_size_m must be positive")
    cells = np.floor(points_xy / voxel_size_m).astype(np.int64)
    _unique, first_indices = np.unique(cells, axis=0, return_index=True)
    first_indices.sort()
    return points_xy[first_indices]


def make_attempt_row(
    window: RequestWindow,
    source: str,
    mode: str,
    roi_type: str,
    candidate_count: int,
    runtime_sec: float,
    rejection_reason: str,
    yaw_samples_deg: Sequence[float],
    generated_at: str,
    candidates_csv: Path,
) -> Dict[str, str]:
    return {
        "attempt_id": window.attempt_id,
        "trigger_stamp_sec": f"{window.trigger_stamp_sec:.9f}",
        "start_stamp_sec": f"{window.start_stamp_sec:.9f}",
        "end_stamp_sec": f"{window.end_stamp_sec:.9f}",
        "source": source,
        "mode": mode,
        "roi_type": roi_type,
        "candidate_count": str(candidate_count),
        "accepted": "false",
        "accepted_candidate_rank": "",
        "rejection_reason": rejection_reason,
        "best_score": "",
        "second_score": "",
        "candidate_margin": "",
        "overlap": "",
        "converged": "false",
        "refinement_delta_m": "",
        "refinement_delta_yaw_rad": "",
        "runtime_sec": f"{runtime_sec:.6f}",
        "post_reset_ok_rows": "0",
        "post_reset_window_sec": "",
        "false_recovery": "false",
        "request_reason": window.reason,
        "request_score": "" if window.score is None else f"{window.score:.10g}",
        "request_window_rows": str(window.row_count),
        "request_window_duration_sec": f"{max(0.0, window.end_stamp_sec - window.start_stamp_sec):.9f}",
        "generated_at": generated_at,
        "reference_pose_count": "",
        "route_time_radius_sec": "",
        "route_min_spacing_m": "",
        "route_window_start_stamp_sec": "",
        "route_window_end_stamp_sec": "",
        "nearest_reference_stamp_sec": "",
        "nearest_reference_time_delta_sec": "",
        "yaw_offsets_deg": ",".join(f"{value:.9f}" for value in yaw_samples_deg),
        "lateral_offsets_m": "",
        "longitudinal_offsets_m": "",
        "candidates_csv": str(candidates_csv),
    }


def make_candidate_row(
    window: RequestWindow,
    candidate_index: int,
    source: str,
    pose_x: float,
    pose_y: float,
    pose_z: float,
    yaw_rad: float,
) -> Dict[str, str]:
    return {
        "attempt_id": window.attempt_id,
        "candidate_index": str(candidate_index),
        "source": source,
        "pose_x": f"{pose_x:.9f}",
        "pose_y": f"{pose_y:.9f}",
        "pose_z": f"{pose_z:.9f}",
        "yaw_rad": f"{yaw_rad:.9f}",
        "route_stamp_sec": "",
        "route_time_delta_sec": "",
        "route_position_x": "",
        "route_position_y": "",
        "route_position_z": "",
        "route_yaw_rad": "",
        "longitudinal_offset_m": "",
        "lateral_offset_m": "",
        "yaw_offset_deg": f"{math.degrees(yaw_rad):.9f}",
    }


def resolve_nearest_pointclouds(
    bag_path: Path,
    cloud_topic: str,
    windows: Sequence[RequestWindow],
) -> Dict[int, Tuple[int, np.ndarray]]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError("rosbags is required to read rosbag2 point clouds") from exc

    # Bags recorded before type definitions were embedded need an explicit
    # typestore; ROS2_HUMBLE matches the other readers in this repository.
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    targets = sorted((window.trigger_stamp_ns, index) for index, window in enumerate(windows))
    results: Dict[int, Tuple[int, np.ndarray]] = {}
    target_pos = 0
    previous: Optional[Tuple[int, Any, bytes]] = None

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic == cloud_topic]
        if not connections:
            raise ValueError(f"topic {cloud_topic!r} was not found in {bag_path}")

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            current = (int(timestamp), connection, bytes(rawdata))
            while target_pos < len(targets) and targets[target_pos][0] <= current[0]:
                target_ns, window_index = targets[target_pos]
                chosen = _choose_nearest_message(previous, current, target_ns)
                msg = reader.deserialize(chosen[2], chosen[1].msgtype)
                results[window_index] = (chosen[0], pointcloud2_xyz_array(msg))
                target_pos += 1
            previous = current

        while target_pos < len(targets):
            if previous is None:
                raise ValueError(f"topic {cloud_topic!r} had no messages in {bag_path}")
            _target_ns, window_index = targets[target_pos]
            msg = reader.deserialize(previous[2], previous[1].msgtype)
            results[window_index] = (previous[0], pointcloud2_xyz_array(msg))
            target_pos += 1

    return results


def pointcloud2_xyz_array(msg: Any) -> np.ndarray:
    fields = {field.name: field for field in msg.fields}
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError(f"PointCloud2 message lacks field {name!r}")

    endian = ">" if bool(msg.is_bigendian) else "<"
    height = int(msg.height)
    width = int(msg.width)
    row_step = int(msg.row_step)
    point_step = int(msg.point_step)
    data = bytes(msg.data)

    arrays = []
    for name in ("x", "y", "z"):
        field = fields[name]
        dtype = _pointfield_dtype(int(field.datatype), endian)
        values = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=data,
            offset=int(field.offset),
            strides=(row_step, point_step),
        ).reshape(-1)
        arrays.append(values.astype(np.float64, copy=False))

    xyz = np.column_stack(arrays)
    return xyz[np.isfinite(xyz).all(axis=1)]


def write_csv(
    path: Path,
    rows: List[Dict[str, str]],
    fieldnames: Sequence[str],
    overwrite: bool,
) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run(args: argparse.Namespace) -> None:
    alignment_csv = Path(args.alignment_csv).expanduser().resolve()
    occupancy_yaml = Path(args.occupancy_yaml).expanduser().resolve()
    bag_path = Path(args.bag_path).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    candidates_csv = (
        Path(args.output_candidates_csv).expanduser().resolve()
        if args.output_candidates_csv
        else output_csv.with_name("relocalization_candidates.csv")
    )
    for path, label in (
        (alignment_csv, "alignment CSV"),
        (occupancy_yaml, "occupancy yaml"),
        (bag_path, "bag path"),
    ):
        if not path.exists():
            raise FileNotFoundError(f"{label} does not exist: {path}")

    angular_resolution_rad = args.angular_resolution_rad
    if args.angular_resolution_deg is not None:
        angular_resolution_rad = math.radians(args.angular_resolution_deg)
    yaw_samples_deg = [math.degrees(yaw) for yaw in _yaw_samples(angular_resolution_rad)]

    occupancy_map = load_occupancy_map(occupancy_yaml)
    matching_grid = occupancy_map.occupied
    for _ in range(max(0, args.dilate_cells)):
        matching_grid = _dilate_one_cell(matching_grid)
    windows = load_request_windows(alignment_csv, source=args.source)
    clouds = (
        resolve_nearest_pointclouds(bag_path, args.cloud_topic, windows) if windows else {}
    )

    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    attempt_rows: List[Dict[str, str]] = []
    candidate_rows: List[Dict[str, str]] = []

    for window_index, window in enumerate(windows):
        _scan_stamp_ns, points_xyz = clouds[window_index]
        started = time.monotonic()

        scan_xy = prepare_scan_xy(
            points_xyz,
            args.z_min_m,
            args.z_max_m,
            occupancy_map.resolution_m,
            args.max_scan_points,
            min_range_m=args.min_range_m,
        )
        grid_candidates = branch_and_bound_candidates(
            matching_grid,
            scan_xy,
            occupancy_map.resolution_m,
            angular_resolution_rad,
            args.pyramid_depth,
            args.max_candidates,
            nms_radius_cells=int(round(max(0.0, args.nms_radius_m) / occupancy_map.resolution_m)),
        )
        runtime_sec = time.monotonic() - started

        rejection_reason = args.rejection_reason if scan_xy.shape[0] > 0 else "empty_scan"
        attempt_rows.append(
            make_attempt_row(
                window=window,
                source=args.source,
                mode=args.mode,
                roi_type=args.roi_type,
                candidate_count=len(grid_candidates),
                runtime_sec=runtime_sec,
                rejection_reason=rejection_reason,
                yaw_samples_deg=yaw_samples_deg,
                generated_at=generated_at,
                candidates_csv=candidates_csv,
            )
        )

        for candidate_index, candidate in enumerate(grid_candidates):
            pose_x, pose_y = occupancy_map.grid_cell_center_to_world(
                candidate.tx_cell, candidate.ty_cell
            )
            yaw_rad = normalize_angle_rad(occupancy_map.origin_yaw_rad + candidate.yaw_rad)
            candidate_rows.append(
                make_candidate_row(
                    window=window,
                    candidate_index=candidate_index,
                    source=args.source,
                    pose_x=pose_x,
                    pose_y=pose_y,
                    pose_z=args.seed_z_m,
                    yaw_rad=yaw_rad,
                )
            )

    write_csv(output_csv, attempt_rows, ATTEMPT_FIELDNAMES, overwrite=args.overwrite)
    write_csv(candidates_csv, candidate_rows, CANDIDATE_FIELDNAMES, overwrite=args.overwrite)
    print(
        json.dumps(
            {
                "output_csv": str(output_csv),
                "output_candidates_csv": str(candidates_csv),
                "attempt_count": len(attempt_rows),
                "candidate_count": len(candidate_rows),
                "yaw_sample_count": len(yaw_samples_deg),
            }
        )
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Create BBS_2D relocalization candidate artifacts from alignment_status.csv, "
            "an occupancy map, and a rosbag2 directory. Candidates are ordered best-first "
            "by BBS score; registration scoring stays downstream."
        )
    )
    parser.add_argument("--alignment-csv", required=True, help="Input alignment_status.csv")
    parser.add_argument(
        "--occupancy-yaml",
        required=True,
        help="Occupancy map .yaml from generate_occupancy_map_from_pcd.py",
    )
    parser.add_argument("--bag-path", required=True, help="rosbag2 directory with the scan topic")
    parser.add_argument("--cloud-topic", required=True, help="PointCloud2 topic in the bag")
    parser.add_argument("--output-csv", required=True, help="Output relocalization_attempts.csv")
    parser.add_argument(
        "--output-candidates-csv",
        default="",
        help="Optional per-candidate CSV. Defaults to relocalization_candidates.csv next to output CSV.",
    )
    parser.add_argument("--source", default="bbs_2d", help="Attempt source label.")
    parser.add_argument("--mode", default="offline_map_roi", help="Attempt mode label.")
    parser.add_argument("--roi-type", default="bbs_2d", help="ROI type label.")
    parser.add_argument(
        "--angular-resolution-rad",
        type=float,
        default=math.radians(5.0),
        help="Yaw sampling resolution in radians.",
    )
    parser.add_argument(
        "--nms-radius-m",
        type=float,
        default=2.0,
        help=(
            "Spatial non-maximum-suppression radius for the candidate list; keeps "
            "one hypothesis per neighborhood so top-K covers distinct locations."
        ),
    )
    parser.add_argument(
        "--angular-resolution-deg",
        type=float,
        default=None,
        help="Yaw sampling resolution in degrees; overrides --angular-resolution-rad.",
    )
    parser.add_argument(
        "--pyramid-depth",
        type=int,
        default=4,
        help="Number of max-pooled pyramid levels above the full-resolution grid.",
    )
    parser.add_argument(
        "--max-scan-points",
        type=int,
        default=512,
        help="Maximum 2D scan points after the range/height filters and voxel downsample.",
    )
    parser.add_argument(
        "--z-min-m",
        type=float,
        default=0.5,
        help="Lower bound of the sensor-frame height band kept for matching.",
    )
    parser.add_argument(
        "--z-max-m",
        type=float,
        default=5.0,
        help="Upper bound of the sensor-frame height band kept for matching.",
    )
    parser.add_argument(
        "--min-range-m",
        type=float,
        default=1.0,
        help="Minimum XY range; removes zero-filled invalid returns and sensor clutter.",
    )
    parser.add_argument(
        "--dilate-cells",
        type=int,
        default=1,
        help=(
            "Dilate the occupancy grid by this many cells before matching, as a "
            "tolerance for grid discretization and scan noise."
        ),
    )
    parser.add_argument(
        "--seed-z-m",
        type=float,
        default=0.0,
        help="Z value written for every candidate seed pose.",
    )
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=64,
        help="Top-K candidate poses kept per request window.",
    )
    parser.add_argument(
        "--rejection-reason",
        default="candidate_scoring_not_implemented",
        help="Rejection reason written for every generated attempt.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output CSV files.")
    return parser


def main() -> None:
    run(build_arg_parser().parse_args())


def _max_pool_2x2(grid: np.ndarray) -> np.ndarray:
    height, width = grid.shape
    pooled_height = (height + 1) // 2
    pooled_width = (width + 1) // 2
    padded = np.pad(
        grid,
        ((0, pooled_height * 2 - height), (0, pooled_width * 2 - width)),
        mode="constant",
        constant_values=False,
    )
    return padded.reshape(pooled_height, 2, pooled_width, 2).max(axis=(1, 3))


def _dilate_one_cell(grid: np.ndarray) -> np.ndarray:
    padded = np.pad(grid, ((1, 1), (1, 1)), mode="constant", constant_values=False)
    result = np.zeros_like(grid, dtype=bool)
    height, width = grid.shape
    for dy in range(3):
        for dx in range(3):
            result |= padded[dy : dy + height, dx : dx + width]
    return result


def _yaw_samples(angular_resolution_rad: float) -> List[float]:
    count = max(1, int(math.ceil((2.0 * math.pi) / angular_resolution_rad)))
    step = (2.0 * math.pi) / float(count)
    return [normalize_angle_rad(index * step) for index in range(count)]


def _score_grid(
    grid: np.ndarray,
    scan_xy_grid: np.ndarray,
    tx_cell: int,
    ty_cell: int,
    yaw_rad: float,
    factor: int,
) -> Tuple[float, int, int]:
    if scan_xy_grid.size == 0:
        return 0.0, 0, 0

    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    rx = c * scan_xy_grid[:, 0] - s * scan_xy_grid[:, 1]
    ry = s * scan_xy_grid[:, 0] + c * scan_xy_grid[:, 1]

    ix = np.floor((float(tx_cell) + 0.5 + rx) / float(factor)).astype(np.int64)
    iy = np.floor((float(ty_cell) + 0.5 + ry) / float(factor)).astype(np.int64)

    height, width = grid.shape
    inside = (ix >= 0) & (iy >= 0) & (ix < width) & (iy < height)
    hits = int(grid[iy[inside], ix[inside]].sum()) if np.any(inside) else 0
    count = int(scan_xy_grid.shape[0])
    return float(hits) / float(count), hits, count


def _candidate_sort_key(candidate: BbsGridCandidate) -> Tuple[float, int, int, int]:
    return (-candidate.score, candidate.yaw_index, candidate.ty_cell, candidate.tx_cell)


def _choose_nearest_message(
    previous: Optional[Tuple[int, Any, bytes]],
    current: Tuple[int, Any, bytes],
    target_ns: int,
) -> Tuple[int, Any, bytes]:
    if previous is None:
        return current
    if abs(previous[0] - target_ns) <= abs(current[0] - target_ns):
        return previous
    return current


def _pointfield_dtype(datatype: int, endian: str) -> np.dtype:
    mapping = {
        1: "i1",
        2: "u1",
        3: endian + "i2",
        4: endian + "u2",
        5: endian + "i4",
        6: endian + "u4",
        7: endian + "f4",
        8: endian + "f8",
    }
    if datatype not in mapping:
        raise ValueError(f"unsupported PointField datatype {datatype}")
    return np.dtype(mapping[datatype])


if __name__ == "__main__":
    main()
