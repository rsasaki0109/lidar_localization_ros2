#!/usr/bin/env python3
"""Compare fixed and refreshed occupancy windows on the same recorded scans."""

import argparse
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
from global_localization_query import GlobalLocalizationConfig, GlobalLocalizationEngine


def crop_map(grid, matching, xy, radius):
    center = (xy - [grid.origin_x_m, grid.origin_y_m]) / grid.resolution_m
    margin = radius / grid.resolution_m
    low = np.maximum(0, np.floor(center - margin)).astype(int)
    high = np.minimum(grid.occupied.shape[::-1], np.ceil(center + margin)).astype(int)
    if np.any(high <= low):
        raise ValueError("Search window does not intersect the occupancy map")
    region = np.s_[low[1] : high[1], low[0] : high[0]]
    return replace(
        grid,
        occupied=grid.occupied[region],
        origin_x_m=grid.origin_x_m + low[0] * grid.resolution_m,
        origin_y_m=grid.origin_y_m + low[1] * grid.resolution_m,
    ), np.ascontiguousarray(matching[region])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", type=Path)
    parser.add_argument("sites", nargs="+", type=int)
    args = parser.parse_args()
    root = args.fixture
    config = GlobalLocalizationConfig(
        use_cpp_backend=True,
        enable_registration_scoring=True,
        registration_refine_candidates=True,
        map_path=str(root / "map.ply"),
        z_min_m=-0.5,
        z_max_m=4.0,
        max_scan_points=512,
        max_candidates=16,
        ndt_num_threads=4,
        ndt_local_map_radius=130.0,
        ndt_target_voxel_leaf_size=0.35,
        ndt_scan_max_range=60.0,
        registration_sensor_height_m=1.66,
    )
    engine = GlobalLocalizationEngine(config, root / "full.yaml")
    if engine.backend != "cpp" or engine.registration_scorer is None:
        raise RuntimeError("Both the C++ BBS and NDT scoring modules are required")
    grid, matching = engine.occupancy_map, engine.matching_grid
    for site in args.sites:
        samples = json.loads((root / "moving" / f"{site}.json").read_text())
        first_prior = np.array(samples[0]["xyz"][:2]) + np.array([10.0, 0.0])
        # Use measured relative translations as an ideal near-prior input.
        # Scans in the fixture have already been levelled with reference attitude.
        for seconds in (12.0, 24.0):
            s = min(samples, key=lambda f: abs(f["time"] - seconds))
            points = np.load(root / "moving" / f"{s['index']}.npy")[:, :3]
            moved_prior = first_prior + np.subtract(s["xyz"][:2], samples[0]["xyz"][:2])
            for name, prior in (("fixed", first_prior), ("refreshed", moved_prior)):
                engine.occupancy_map, engine.matching_grid = crop_map(
                    grid, matching, prior, 70.0
                )
                started = time.monotonic()
                result = engine.query(points)
                elapsed = time.monotonic() - started
                candidate = next(iter(result.candidates), None)
                error = yaw = None
                if candidate is not None:
                    error = math.dist(
                        (candidate.x_m, candidate.y_m, candidate.z_m), s["xyz"]
                    )
                    yaw = abs(math.remainder(candidate.yaw_rad - s["yaw"], math.tau))
                print(
                    json.dumps(
                        {
                            "site": site,
                            "scan_sec": s["time"],
                            "mode": name,
                            "query_sec": elapsed,
                            "position_error_m": error,
                            "yaw_error_rad": yaw,
                            "within_threshold": error is not None
                            and error < 2.0
                            and yaw < 0.05,
                        }
                    ),
                    flush=True,
                )


if __name__ == "__main__":
    main()
