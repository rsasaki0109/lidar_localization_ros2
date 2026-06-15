#!/usr/bin/env python3
"""ROS-free global localization query logic for the G2 runtime service.

Wraps the BBS_2D engine from make_bbs_relocalization_attempts with occupancy
map loading, scan preparation, and candidate-to-world-pose conversion, so the
runtime node and offline tooling share one code path and the logic stays unit
testable without rclpy.
"""

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List

sys.path.insert(0, str(Path(__file__).resolve().parent))

import make_bbs_relocalization_attempts as bbs_engine  # noqa: E402


@dataclass(frozen=True)
class GlobalLocalizationConfig:
    z_min_m: float = 0.5
    z_max_m: float = 5.0
    min_range_m: float = 1.0
    max_scan_points: int = 512
    angular_resolution_rad: float = math.radians(5.0)
    pyramid_depth: int = 4
    max_candidates: int = 16
    nms_radius_m: float = 2.0
    dilate_cells: int = 1
    seed_z_m: float = 0.0
    # Opt-in: use the compiled C++ branch-and-bound backend (bbs_cpp) when it is
    # importable. Defaults to False so a stock checkout runs pure Python; if the
    # module is missing the engine logs once and silently falls back to Python.
    use_cpp_backend: bool = False


@dataclass(frozen=True)
class GlobalLocalizationCandidate:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    score: float
    hit_count: int
    point_count: int


@dataclass(frozen=True)
class GlobalLocalizationResult:
    candidates: List[GlobalLocalizationCandidate]
    scan_point_count: int


class GlobalLocalizationEngine:
    """Loads the occupancy map once and answers repeated scan queries."""

    def __init__(self, occupancy_yaml, config):
        self.config = config
        self.occupancy_map = bbs_engine.load_occupancy_map(Path(occupancy_yaml))
        matching_grid = self.occupancy_map.occupied
        for _ in range(max(0, config.dilate_cells)):
            matching_grid = bbs_engine._dilate_one_cell(matching_grid)
        self.matching_grid = matching_grid

        # Resolve the search backend once. The C++ module (bbs_cpp) is bit-exact
        # to bbs_engine.branch_and_bound_candidates and exposes the same call
        # signature and candidate attributes, so query() is backend-agnostic.
        self.backend = "python"
        self.backend_error = None
        self._search = bbs_engine.branch_and_bound_candidates
        if config.use_cpp_backend:
            try:
                import bbs_cpp  # noqa: E402
                self._search = bbs_cpp.branch_and_bound_candidates
                self.backend = "cpp"
            except ImportError as exc:  # pragma: no cover - depends on build
                self.backend_error = str(exc)

    def query(self, points_xyz):
        config = self.config
        resolution_m = self.occupancy_map.resolution_m
        scan_xy = bbs_engine.prepare_scan_xy(
            points_xyz,
            config.z_min_m,
            config.z_max_m,
            resolution_m,
            config.max_scan_points,
            min_range_m=config.min_range_m,
        )
        if scan_xy.shape[0] == 0:
            return GlobalLocalizationResult(candidates=[], scan_point_count=0)

        grid_candidates = self._search(
            self.matching_grid,
            scan_xy,
            resolution_m,
            config.angular_resolution_rad,
            config.pyramid_depth,
            config.max_candidates,
            nms_radius_cells=int(
                round(max(0.0, config.nms_radius_m) / resolution_m)),
        )

        candidates = []
        for candidate in grid_candidates:
            x_m, y_m = self.occupancy_map.grid_cell_center_to_world(
                candidate.tx_cell, candidate.ty_cell)
            yaw_rad = bbs_engine.normalize_angle_rad(
                self.occupancy_map.origin_yaw_rad + candidate.yaw_rad)
            candidates.append(
                GlobalLocalizationCandidate(
                    x_m=x_m,
                    y_m=y_m,
                    z_m=config.seed_z_m,
                    yaw_rad=yaw_rad,
                    score=candidate.score,
                    hit_count=candidate.hit_count,
                    point_count=candidate.point_count,
                ))
        return GlobalLocalizationResult(
            candidates=candidates, scan_point_count=int(scan_xy.shape[0]))
