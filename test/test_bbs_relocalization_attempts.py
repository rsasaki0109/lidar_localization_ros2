#!/usr/bin/env python3

import csv
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import make_bbs_relocalization_attempts as bbs  # noqa: E402
import make_map_grid_relocalization_attempts as map_grid  # noqa: E402


def test_pyramid_upper_bound_is_admissible():
    occupancy = np.zeros((8, 9), dtype=bool)
    occupancy[1, 2] = True
    occupancy[3, 5] = True
    occupancy[6, 7] = True

    scan_xy_grid = np.array(
        [
            [0.1, 0.2],
            [1.4, -0.2],
            [-0.8, 1.1],
            [2.2, 0.7],
        ],
        dtype=np.float64,
    )

    pyramid = bbs.build_occupancy_pyramid(occupancy, depth=3)
    upper_bound_pyramid = bbs.build_upper_bound_pyramid(pyramid)

    for level in range(1, len(upper_bound_pyramid)):
        factor = 1 << level
        for yaw_rad in (0.0, math.pi / 2.0, -math.pi / 3.0):
            for ty_cell in range(0, occupancy.shape[0], factor):
                for tx_cell in range(0, occupancy.shape[1], factor):
                    bound = bbs.bbs_upper_bound(
                        upper_bound_pyramid,
                        scan_xy_grid,
                        tx_cell,
                        ty_cell,
                        yaw_rad,
                        level,
                    )
                    for dy in range(factor):
                        child_ty = ty_cell + dy
                        if child_ty >= occupancy.shape[0]:
                            continue
                        for dx in range(factor):
                            child_tx = tx_cell + dx
                            if child_tx >= occupancy.shape[1]:
                                continue
                            exact = bbs.bbs_exact_score(
                                occupancy,
                                scan_xy_grid,
                                child_tx,
                                child_ty,
                                yaw_rad,
                            )
                            assert bound + 1.0e-12 >= exact


def test_branch_and_bound_top_k_ordering():
    occupancy = np.zeros((6, 6), dtype=bool)
    occupancy[2, 2] = True
    occupancy[2, 3] = True
    occupancy[4, 4] = True

    scan_xy_m = np.array([[0.0, 0.0], [1.0, 0.0]], dtype=np.float64)
    candidates = bbs.branch_and_bound_candidates(
        occupancy=occupancy,
        scan_xy_m=scan_xy_m,
        resolution_m=1.0,
        angular_resolution_rad=2.0 * math.pi,
        pyramid_depth=2,
        max_candidates=5,
    )

    assert len(candidates) == 5
    scores = [candidate.score for candidate in candidates]
    assert scores == sorted(scores, reverse=True)
    assert candidates[0].score == 1.0
    assert any(candidate.score < candidates[0].score for candidate in candidates[1:])


def _write_alignment_csv(path, requested_flags):
    fieldnames = [
        "message_index",
        "stamp_sec",
        "status_index",
        "level",
        "name",
        "message",
        "hardware_id",
        "values_json",
    ]
    with open(path, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index, requested in enumerate(requested_flags):
            values = {
                "reinitialization_requested": "true" if requested else "false",
                "reinitialization_request_reason": "fitness_streak" if requested else "not_failure",
                "reinitialization_request_score": "9.5" if requested else "0.0",
            }
            writer.writerow(
                {
                    "message_index": str(index),
                    "stamp_sec": f"{100.0 + index:.9f}",
                    "status_index": "0",
                    "level": "0",
                    "name": "lidar_localization_ros2/alignment",
                    "message": "ok",
                    "hardware_id": "NDT_OMP",
                    "values_json": json.dumps(values),
                }
            )


def test_request_windows_group_consecutive_requested_rows():
    with tempfile.TemporaryDirectory() as tmp:
        alignment_csv = Path(tmp) / "alignment_status.csv"
        _write_alignment_csv(alignment_csv, [False, True, True, False, True, False])
        windows = bbs.load_request_windows(alignment_csv, source="bbs_2d")

    assert len(windows) == 2
    first, second = windows
    assert first.attempt_id == "bbs_2d_0001"
    assert first.row_count == 2
    assert abs(first.trigger_stamp_sec - 101.0) < 1e-9
    assert abs(first.end_stamp_sec - 102.0) < 1e-9
    assert first.reason == "fitness_streak"
    assert first.score == 9.5
    assert first.trigger_stamp_ns == 101_000_000_000
    assert second.attempt_id == "bbs_2d_0002"
    assert second.row_count == 1


def test_csv_contract_field_sets_match_map_grid():
    assert bbs.ATTEMPT_FIELDNAMES == map_grid.ATTEMPT_FIELDNAMES
    assert bbs.CANDIDATE_FIELDNAMES == map_grid.CANDIDATE_FIELDNAMES

    window = bbs.RequestWindow(
        attempt_id="bbs_2d_0001",
        trigger_stamp_sec=12.345678901,
        start_stamp_sec=12.0,
        end_stamp_sec=13.0,
        row_count=4,
        reason="fitness_streak",
        score=9.5,
    )

    attempt_row = bbs.make_attempt_row(
        window=window,
        source="bbs_2d",
        mode="offline_map_roi",
        roi_type="bbs_2d",
        candidate_count=1,
        runtime_sec=0.25,
        rejection_reason="candidate_scoring_not_implemented",
        yaw_samples_deg=[0.0, 90.0, 180.0, -90.0],
        generated_at="2026-06-11T00:00:00+09:00",
        candidates_csv=Path("/tmp/relocalization_candidates.csv"),
    )
    candidate_row = bbs.make_candidate_row(
        window=window,
        candidate_index=0,
        source="bbs_2d",
        pose_x=1.5,
        pose_y=2.5,
        pose_z=0.0,
        yaw_rad=0.5,
    )

    assert set(attempt_row.keys()) == set(bbs.ATTEMPT_FIELDNAMES)
    assert set(candidate_row.keys()) == set(bbs.CANDIDATE_FIELDNAMES)
    assert attempt_row["attempt_id"] == "bbs_2d_0001"
    assert attempt_row["request_window_rows"] == "4"
    assert candidate_row["pose_x"] == "1.500000000"
    assert candidate_row["yaw_rad"] == "0.500000000"

    for name, value in candidate_row.items():
        if name.startswith("route_"):
            assert value == ""


def main():
    test_pyramid_upper_bound_is_admissible()
    test_branch_and_bound_top_k_ordering()
    test_request_windows_group_consecutive_requested_rows()
    test_csv_contract_field_sets_match_map_grid()
    print("test_bbs_relocalization_attempts: all tests passed")


if __name__ == "__main__":
    main()
