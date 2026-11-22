#!/usr/bin/env python3
"""The opt-in G2 engine bbs3d (global_search_engine=bbs3d) in the query engine.

Everything but the last test runs without the compiled bbs3d_cpp module (a fake
one stands in); the last one uses the real module and skips when it is not built.
"""

import json
import math
import sys
import tempfile
import threading
import types
from pathlib import Path
from types import SimpleNamespace
from typing import ClassVar

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "test"))

import global_localization_query as glq
from test_global_localization_query import make_scan, write_occupancy_map


def pose_matrix(x, y, z, yaw):
    pose = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    pose[:2, :2] = [[c, -s], [s, c]]
    pose[:3, 3] = [x, y, z]
    return pose


class FakeMapSearcher:
    """Stands in for bbs3d_cpp.MapSearcher; answers two fixed poses."""

    instances: ClassVar[list] = []

    def __init__(self, map_path, map_leaf_m, min_level_res_m, max_level):
        self.args = (map_path, map_leaf_m, min_level_res_m, max_level)
        self.searches = []
        self.block = None
        FakeMapSearcher.instances.append(self)

    def info(self):
        return {
            "loaded_points": 1000,
            "map_points": 800,
            "build_ms": 12.5,
            "buckets": 4096,
            "voxelmap_bytes": 65536,
            "min_xyz": np.array([-1.0, -2.0, -3.0]),
            "max_xyz": np.array([10.0, 20.0, 8.0]),
        }

    def search(self, scan, *args):
        self.searches.append((np.array(scan), args))
        if self.block is not None:
            self.block.wait(5.0)
        return {
            "candidates": [
                {
                    "pose": pose_matrix(1.0, 2.0, 5.5, 0.5),
                    "score": 90,
                    "score_ratio": 0.9,
                },
                {
                    "pose": pose_matrix(8.0, 2.0, 0.5, -2.0),
                    "score": 85,
                    "score_ratio": 0.85,
                },
            ],
            "timed_out": False,
            "queue_capped": False,
            "expanded": 42,
            "max_queue": 1000,
            "initial": 10,
            "scan_points": 100,
            "elapsed_ms": 3.0,
        }


class FakeModule:
    """Installs a fake bbs3d_cpp, or a missing one (None), for one test."""

    def __init__(self, module):
        self.module = module
        self.saved = None

    def __enter__(self):
        self.saved = sys.modules.get("bbs3d_cpp", "absent")
        sys.modules["bbs3d_cpp"] = self.module
        FakeMapSearcher.instances = []
        return self

    def __exit__(self, *exc):
        if self.saved == "absent":
            sys.modules.pop("bbs3d_cpp", None)
        else:
            sys.modules["bbs3d_cpp"] = self.saved
        return False


def fake_module():
    module = types.ModuleType("bbs3d_cpp")
    module.MapSearcher = FakeMapSearcher
    return module


class RefiningScorer:
    """Records registration calls; converges every candidate 0.1 m east."""

    def __init__(self, fitness):
        self.fitness = list(fitness)
        self.poses = []

    def score_candidates(self, points_xyz, poses):
        self.poses.append(list(poses))
        return [
            SimpleNamespace(
                fitness=fitness,
                converged=True,
                refined_x=x + 0.1,
                refined_y=y,
                refined_z=z,
                refined_yaw=yaw,
            )
            for (x, y, z, yaw), fitness in zip(poses, self.fitness)
        ]


def test_default_engine_is_bbs2d_and_names_are_checked():
    config = glq.GlobalLocalizationConfig()
    assert config.global_search_engine == "bbs2d"
    assert config.bbs3d_top_k == 8
    assert config.bbs3d_min_separation_m == 3.0
    assert config.bbs3d_timeout_sec == 10.0
    assert glq.normalize_global_search_engine(" BBS3D ") == "bbs3d"
    try:
        glq.normalize_global_search_engine("bbs4d")
    except ValueError as exc:
        assert "bbs2d" in str(exc)
    else:
        raise AssertionError("an unknown engine must be refused")
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        engine = glq.GlobalLocalizationEngine(config, yaml_path)
        assert engine.global_search_engine == "bbs2d"
        assert engine.global_search_engine_error is None
        assert engine.bbs3d_searcher is None
        result = engine.query(make_scan(yaml_path, -1.0, 7.0, 0.5))
        assert result.global_search_engine == "bbs2d"
        assert result.search_info is None and result.search_error is None


def test_invalid_bbs3d_settings_are_refused():
    for field, value in (
        ("bbs3d_top_k", 0),
        ("bbs3d_timeout_sec", 0.0),
        ("bbs3d_timeout_sec", float("inf")),
        ("bbs3d_max_queue_nodes", 0),
        ("bbs3d_relative_score_floor", 1.5),
        ("bbs3d_num_threads", 0),
        ("bbs3d_scan_max_range_m", 0.5),
    ):
        config = glq.GlobalLocalizationConfig(
            global_search_engine="bbs3d", map_path="map.pcd", **{field: value}
        )
        try:
            glq.validate_bbs3d_config(config)
        except ValueError as exc:
            assert field in str(exc), (field, exc)
        else:
            raise AssertionError(f"{field}={value} must be refused")
    glq.validate_bbs3d_config(glq.GlobalLocalizationConfig())


def test_bbs3d_without_the_module_falls_back_to_bbs2d():
    with tempfile.TemporaryDirectory() as tmp, FakeModule(None):
        yaml_path = write_occupancy_map(Path(tmp))
        config = glq.GlobalLocalizationConfig(
            global_search_engine="bbs3d",
            map_path=str(Path(tmp) / "map.pcd"),
            angular_resolution_rad=math.radians(15.0),
        )
        engine = glq.GlobalLocalizationEngine(config, occupancy_yaml=yaml_path)
        assert engine.global_search_engine == "bbs2d"
        assert "bbs3d_cpp" in engine.global_search_engine_error
        result = engine.query(make_scan(yaml_path, -1.0, 7.0, math.radians(30.0)))
        assert result.global_search_engine == "bbs2d"
        top = result.candidates[0]
        assert math.hypot(top.x_m + 1.0, top.y_m - 7.0) <= 1.0


def test_bbs3d_without_a_map_path_falls_back_to_bbs2d():
    with tempfile.TemporaryDirectory() as tmp, FakeModule(fake_module()):
        config = glq.GlobalLocalizationConfig(global_search_engine="bbs3d")
        engine = glq.GlobalLocalizationEngine(config, write_occupancy_map(Path(tmp)))
        assert engine.global_search_engine == "bbs2d"
        assert "map_path" in engine.global_search_engine_error
        assert FakeMapSearcher.instances == []


def test_bbs3d_candidates_conversion():
    result = FakeMapSearcher("m", 0.1, 0.5, 6).search(np.zeros((1, 3)))
    candidates = glq.bbs3d_candidates(result)
    assert len(candidates) == 2
    first = candidates[0]
    assert (first.x_m, first.y_m, first.z_m) == (1.0, 2.0, 5.5)
    assert math.isclose(first.yaw_rad, 0.5)
    assert math.isclose(candidates[1].yaw_rad, -2.0)
    assert first.score == first.bbs_score == 0.9
    assert first.hit_count == 90 and first.point_count == 100
    assert first.registration_fitness is None
    assert glq.bbs3d_candidates({"candidates": []}) == []


def test_bbs3d_answers_are_searched_once_per_map_and_refined():
    with tempfile.TemporaryDirectory() as tmp, FakeModule(fake_module()):
        config = glq.GlobalLocalizationConfig(
            global_search_engine="bbs3d",
            map_path="/maps/stairs.pcd",
            bbs3d_top_k=5,
            bbs3d_min_separation_m=2.5,
            bbs3d_timeout_sec=1.5,
            bbs3d_max_queue_nodes=1234,
            bbs3d_num_threads=2,
            bbs3d_min_level_res_m=1.0,
        )
        engine = glq.GlobalLocalizationEngine(config, write_occupancy_map(Path(tmp)))
        assert engine.global_search_engine == "bbs3d"
        assert engine.backend == "bbs3d"
        assert engine.global_search_engine_error is None
        assert len(FakeMapSearcher.instances) == 1
        searcher = FakeMapSearcher.instances[0]
        assert searcher.args == ("/maps/stairs.pcd", 0.1, 1.0, 6)
        assert engine.bbs3d_info["build_ms"] == 12.5
        assert "rss_after_mb" in engine.bbs3d_info
        # The info is logged and written to result files as JSON.
        assert json.loads(json.dumps(engine.bbs3d_info))["min_xyz"] == [-1, -2, -3]

        # The second answer registers better; both are refined from their 3D pose.
        engine.registration_scorer = RefiningScorer([2.0, 0.5])
        phases = []
        points = np.array([[2.0, 0.0, 0.5], [0.0, 3.0, 1.0]])
        result = engine.query(
            points, progress_callback=lambda p, d, t: phases.append(p)
        )
        assert len(FakeMapSearcher.instances) == 1, "voxel maps are built once"
        scan, args = searcher.searches[0]
        np.testing.assert_array_equal(scan, points)
        # min/max range, scan leaf, k, separation, floors, ms, queue, threads, tilt
        assert args == (1.0, 30.0, 0.5, 5, 2.5, 0.0, 0.9, 1500, 1234, 2, 0.02)
        assert engine.registration_scorer.poses == [
            [(1.0, 2.0, 5.5, 0.5), (8.0, 2.0, 0.5, -2.0)]
        ]
        assert result.global_search_engine == "bbs3d"
        assert result.search_info["answers"] == 2
        assert result.search_info["timed_out"] is False
        assert result.scan_point_count == 100
        assert [round(c.x_m, 3) for c in result.candidates] == [8.1, 1.1]
        assert all(c.registration_refined for c in result.candidates)
        assert phases == ["search", "scoring", "done"]


def test_bbs3d_drops_a_query_while_a_search_runs():
    with tempfile.TemporaryDirectory() as tmp, FakeModule(fake_module()):
        config = glq.GlobalLocalizationConfig(
            global_search_engine="bbs3d", map_path="/maps/stairs.pcd"
        )
        engine = glq.GlobalLocalizationEngine(config, write_occupancy_map(Path(tmp)))
        searcher = FakeMapSearcher.instances[0]
        searcher.block = threading.Event()
        first = {}
        worker = threading.Thread(
            target=lambda: first.update(result=engine.query(np.ones((4, 3))))
        )
        worker.start()
        for _ in range(500):
            if searcher.searches:
                break
            threading.Event().wait(0.01)
        busy = engine.query(np.ones((4, 3)))
        searcher.block.set()
        worker.join(5.0)
        assert busy.candidates == [] and busy.search_error.startswith("busy")
        assert len(searcher.searches) == 1
        assert len(first["result"].candidates) == 2
        # Free again afterwards.
        assert len(engine.query(np.ones((4, 3))).candidates) == 2


def wall_map(z0):
    """Two rooms with different walls stacked 5 m apart (x, y, z rows)."""
    points = []
    for floor_z, walls in (
        (
            z0,
            [
                ((0, 0), (20, 0)),
                ((20, 0), (20, 6)),
                ((20, 6), (10, 6)),
                ((10, 6), (10, 12)),
                ((10, 12), (0, 12)),
                ((0, 12), (0, 0)),
            ],
        ),
        (
            z0 + 5.0,
            [
                ((0, 0), (20, 0)),
                ((20, 0), (20, 12)),
                ((20, 12), (0, 12)),
                ((0, 12), (0, 0)),
                ((13, 0), (13, 7)),
                ((6, 12), (6, 8)),
            ],
        ),
    ):
        xs, ys = np.meshgrid(np.arange(0.0, 20.0, 0.2), np.arange(0.0, 12.0, 0.2))
        points.append(np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, floor_z)], 1))
        for (x0, y0), (x1, y1) in walls:
            steps = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.1))
            t = np.linspace(0.0, 1.0, steps + 1)
            for dz in np.arange(0.0, 2.55, 0.1):
                points.append(
                    np.stack(
                        [
                            x0 + t * (x1 - x0),
                            y0 + t * (y1 - y0),
                            np.full(t.size, floor_z + dz),
                        ],
                        1,
                    )
                )
    return np.vstack(points)


def test_bbs3d_cpp_finds_the_upper_floor():
    try:
        glq._append_module_dirs("bbs3d_cpp")
        import bbs3d_cpp
    except ImportError:
        print("skipping: bbs3d_cpp is not built")
        return
    world = wall_map(0.0)
    searcher = bbs3d_cpp.MapSearcher(world, 0.1, 0.5, 4)
    info = searcher.info()
    assert info["map_points"] > 0 and info["buckets"] > 0
    x, y, z, yaw = 15.3, 9.1, 6.0, 0.7
    upper = world[(world[:, 2] > 4.9) & (world[:, 2] < 7.6)]
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    scan = (upper - np.array([x, y, z])) @ rotation
    result = searcher.search(scan, k=4, min_separation_m=3.0, timeout_ms=60000)
    assert not result["timed_out"] and not result["queue_capped"]
    candidates = glq.bbs3d_candidates(result)
    top = candidates[0]
    assert math.hypot(top.x_m - x, top.y_m - y) < 0.75, (top.x_m, top.y_m)
    assert abs(top.z_m - z) < 0.75, top.z_m
    assert abs(math.remainder(top.yaw_rad - yaw, 2.0 * math.pi)) < 0.1
    for i, a in enumerate(candidates):
        for b in candidates[:i]:
            assert math.dist((a.x_m, a.y_m, a.z_m), (b.x_m, b.y_m, b.z_m)) >= 3.0
    capped = searcher.search(scan, max_queue_nodes=1)
    assert capped["queue_capped"] and capped["candidates"] == []


if __name__ == "__main__":
    test_default_engine_is_bbs2d_and_names_are_checked()
    test_invalid_bbs3d_settings_are_refused()
    test_bbs3d_without_the_module_falls_back_to_bbs2d()
    test_bbs3d_without_a_map_path_falls_back_to_bbs2d()
    test_bbs3d_candidates_conversion()
    test_bbs3d_answers_are_searched_once_per_map_and_refined()
    test_bbs3d_drops_a_query_while_a_search_runs()
    test_bbs3d_cpp_finds_the_upper_floor()
    print("test_global_localization_bbs3d: all tests passed")
