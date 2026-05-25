#!/usr/bin/env python3

import importlib.util
from pathlib import Path


def _load_jobs_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "make_registration_relocalization_jobs.py"
    spec = importlib.util.spec_from_file_location("make_registration_relocalization_jobs", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _build_jobs(selection_source):
    module = _load_jobs_module()
    attempts = [
        {
            "attempt_id": "route_grid_0001",
            "trigger_stamp_sec": "100.0",
        }
    ]
    candidates = [
        {
            "attempt_id": "route_grid_0001",
            "candidate_index": "0",
            "pose_x": "0.0",
            "pose_y": "0.0",
            "pose_z": "0.0",
            "yaw_rad": "0.0",
            "route_stamp_sec": "90.0",
            "route_time_delta_sec": "-10.0",
            "longitudinal_offset_m": "0.0",
            "lateral_offset_m": "0.0",
            "yaw_offset_deg": "0.0",
            "oracle_rank": "4",
            "oracle_score": "10.0",
        },
        {
            "attempt_id": "route_grid_0001",
            "candidate_index": "49",
            "pose_x": "1.0",
            "pose_y": "0.0",
            "pose_z": "0.0",
            "yaw_rad": "0.0",
            "route_stamp_sec": "100.0",
            "route_time_delta_sec": "0.0",
            "longitudinal_offset_m": "0.0",
            "lateral_offset_m": "0.0",
            "yaw_offset_deg": "0.0",
            "oracle_rank": "1",
            "oracle_score": "0.0",
        },
        {
            "attempt_id": "route_grid_0001",
            "candidate_index": "50",
            "pose_x": "1.0",
            "pose_y": "0.0",
            "pose_z": "0.0",
            "yaw_rad": "0.1",
            "route_stamp_sec": "100.0",
            "route_time_delta_sec": "0.0",
            "longitudinal_offset_m": "0.0",
            "lateral_offset_m": "0.0",
            "yaw_offset_deg": "15.0",
            "oracle_rank": "2",
            "oracle_score": "0.2",
        },
        {
            "attempt_id": "route_grid_0001",
            "candidate_index": "46",
            "pose_x": "1.0",
            "pose_y": "1.0",
            "pose_z": "0.0",
            "yaw_rad": "0.0",
            "route_stamp_sec": "100.0",
            "route_time_delta_sec": "0.0",
            "longitudinal_offset_m": "0.0",
            "lateral_offset_m": "1.0",
            "yaw_offset_deg": "0.0",
            "oracle_rank": "3",
            "oracle_score": "1.0",
        },
    ]
    return module.build_jobs(
        attempts=attempts,
        candidates=candidates,
        bag_path=Path("/tmp/bag"),
        map_path=Path("/tmp/map.ply"),
        cloud_topic="/cloud",
        registration_method="NDT_OMP",
        max_candidates_per_attempt=3,
        selection_source=selection_source,
        voxel_leaf_size=1.0,
        local_map_radius=120.0,
        timeout_sec=1.0,
    )


def test_candidate_index_keeps_original_order():
    jobs = _build_jobs("candidate_index")
    assert [row["candidate_index"] for row in jobs] == ["0", "46", "49"]


def test_route_proximity_prefers_trigger_centerline_without_oracle_rank():
    jobs = _build_jobs("route_proximity")
    assert [row["candidate_index"] for row in jobs] == ["49", "50", "46"]
    assert all(row["selection_source"] == "route_proximity" for row in jobs)


def test_oracle_rank_still_orders_by_diagnostic_rank():
    jobs = _build_jobs("oracle_rank")
    assert [row["candidate_index"] for row in jobs] == ["49", "50", "46"]


if __name__ == "__main__":
    test_candidate_index_keeps_original_order()
    test_route_proximity_prefers_trigger_centerline_without_oracle_rank()
    test_oracle_rank_still_orders_by_diagnostic_rank()
