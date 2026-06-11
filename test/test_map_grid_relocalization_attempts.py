#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path

import numpy as np


def _load_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "make_map_grid_relocalization_attempts.py"
    spec = importlib.util.spec_from_file_location("make_map_grid_relocalization_attempts", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _dense_cell(x0, y0, z=0.0, count=30):
    xs = np.linspace(x0 + 0.5, x0 + 9.5, count)
    return np.column_stack([xs, np.full(count, y0 + 5.0), np.full(count, z)])


def test_grid_cells_skip_sparse_cells():
    module = _load_module()
    points = np.vstack(
        [
            _dense_cell(0.0, 0.0),
            _dense_cell(10.0, 0.0),
            np.array([[55.0, 55.0, 0.0]]),  # lone point: below min_points_per_cell
        ]
    )
    cells = module.build_grid_cells(
        points,
        grid_spacing_m=10.0,
        min_points_per_cell=20,
        z_percentile=5.0,
        z_offset_m=0.0,
    )
    assert len(cells) == 2
    assert all(cell["point_count"] >= 20 for cell in cells)


def test_grid_cells_apply_z_percentile_and_offset():
    module = _load_module()
    base = _dense_cell(0.0, 0.0, z=0.0)
    tall = base.copy()
    tall[:, 2] = 10.0  # roof points should not pull the seed z up
    points = np.vstack([base, tall])
    cells = module.build_grid_cells(
        points,
        grid_spacing_m=10.0,
        min_points_per_cell=20,
        z_percentile=5.0,
        z_offset_m=1.5,
    )
    assert len(cells) == 1
    assert abs(cells[0]["z"] - 1.5) < 0.5


def test_candidates_cover_yaws_and_respect_cap():
    module = _load_module()
    cells = [
        {"x": float(x), "y": 0.0, "z": 0.0, "point_count": 30.0}
        for x in range(0, 100, 10)
    ]
    candidates, yaws = module.build_candidates(
        attempt_id="map_grid_0001",
        cells=cells,
        source="map_grid",
        yaw_count=8,
        max_candidates=0,
    )
    assert len(yaws) == 8
    assert len(candidates) == len(cells) * 8
    assert candidates[0]["route_stamp_sec"] == ""

    capped, _ = module.build_candidates(
        attempt_id="map_grid_0002",
        cells=cells,
        source="map_grid",
        yaw_count=8,
        max_candidates=16,
    )
    assert len(capped) <= 16
    xs = {row["pose_x"] for row in capped}
    assert len(xs) > 1  # cell striding keeps spatial coverage
    yaws_kept = {row["yaw_rad"] for row in capped}
    assert len(yaws_kept) == 8  # capping must not collapse the yaw dimension


def test_attempts_emit_contract_fields_per_request_window():
    module = _load_module()
    alignment_rows = [
        {"stamp_sec": 1.0, "requested": False, "reason": "not_failure", "score": None},
        {"stamp_sec": 2.0, "requested": True, "reason": "fitness", "score": 9.0},
        {"stamp_sec": 3.0, "requested": True, "reason": "fitness", "score": 9.5},
        {"stamp_sec": 4.0, "requested": False, "reason": "not_failure", "score": None},
        {"stamp_sec": 5.0, "requested": True, "reason": "gap", "score": 8.0},
    ]
    cells = [{"x": 0.0, "y": 0.0, "z": 0.0, "point_count": 30.0}]
    attempts, candidates = module.build_attempt_artifacts(
        alignment_rows=alignment_rows,
        cells=cells,
        candidates_csv=Path("/tmp/relocalization_candidates.csv"),
        source="map_grid",
        mode="offline_map_roi",
        roi_type="map_grid",
        yaw_count=4,
        max_candidates=0,
        grid_spacing_m=10.0,
        rejection_reason="candidate_scoring_not_implemented",
    )
    assert len(attempts) == 2
    assert attempts[0]["attempt_id"] == "map_grid_0001"
    assert attempts[0]["request_window_rows"] == "2"
    assert attempts[1]["request_reason"] == "gap"
    assert len(candidates) == 2 * 4
    for attempt in attempts:
        assert set(attempt.keys()) == set(module.ATTEMPT_FIELDNAMES)
    for candidate in candidates:
        assert set(candidate.keys()) == set(module.CANDIDATE_FIELDNAMES)
        yaw = float(candidate["yaw_rad"])
        assert -math.pi - 1e-6 <= yaw <= math.pi + 1e-6


def main():
    test_grid_cells_skip_sparse_cells()
    test_grid_cells_apply_z_percentile_and_offset()
    test_candidates_cover_yaws_and_respect_cap()
    test_attempts_emit_contract_fields_per_request_window()
    print("test_map_grid_relocalization_attempts: all tests passed")


if __name__ == "__main__":
    main()
