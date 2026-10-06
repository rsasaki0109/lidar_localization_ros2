#!/usr/bin/env python3

import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import global_localization_query as glq


def write_occupancy_map(directory: Path) -> Path:
    # 60x60 grid at 0.5 m; an L-shaped wall pattern with a distinctive corner.
    grid = np.zeros((60, 60), dtype=bool)
    grid[20, 10:50] = True
    grid[20:55, 10] = True
    grid[40, 25:45] = True
    # PGM rows are top-down; the loader flips them back up.
    pixels = np.where(np.flipud(grid), 0, 254).astype(np.uint8)
    image_path = directory / "map.pgm"
    Image.fromarray(pixels, mode="L").save(image_path)
    yaml_path = directory / "map.yaml"
    yaml_path.write_text(
        'image: "map.pgm"\n'
        "resolution: 0.5\n"
        "origin: [-10.0, -5.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    return yaml_path


def make_scan(grid_yaml: Path, true_x: float, true_y: float, true_yaw: float):
    # Sample world points on the walls near the true pose and express them in
    # the scan frame, inside the z band and outside the min-range filter.
    engine_map = glq.bbs_engine.load_occupancy_map(grid_yaml)
    occupied = np.argwhere(engine_map.occupied)
    world_x = engine_map.origin_x_m + (occupied[:, 1] + 0.5) * engine_map.resolution_m
    world_y = engine_map.origin_y_m + (occupied[:, 0] + 0.5) * engine_map.resolution_m
    near = (np.abs(world_x - true_x) < 15.0) & (np.abs(world_y - true_y) < 15.0)
    world = np.stack([world_x[near], world_y[near]], axis=1)

    c = math.cos(-true_yaw)
    s = math.sin(-true_yaw)
    dx = world[:, 0] - true_x
    dy = world[:, 1] - true_y
    scan_x = c * dx - s * dy
    scan_y = s * dx + c * dy
    keep = np.hypot(scan_x, scan_y) >= 1.5
    points = np.stack(
        [scan_x[keep], scan_y[keep], np.full(int(keep.sum()), 1.0)], axis=1
    )
    clutter = np.array([[0.1, 0.0, 1.0], [3.0, 1.0, 9.0], [2.0, -1.0, -3.0]])
    return np.vstack([points, clutter])


def test_query_recovers_known_pose():
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        true_x, true_y, true_yaw = -1.0, 7.0, math.radians(30.0)
        config = glq.GlobalLocalizationConfig(
            angular_resolution_rad=math.radians(15.0),
            max_candidates=8,
            min_range_m=1.0,
        )
        engine = glq.GlobalLocalizationEngine(config, occupancy_yaml=yaml_path)
        result = engine.query(make_scan(yaml_path, true_x, true_y, true_yaw))

        assert result.candidates, "expected at least one candidate"
        assert len(result.candidates) <= 8
        top = result.candidates[0]
        assert math.hypot(top.x_m - true_x, top.y_m - true_y) <= 1.0, (top.x_m, top.y_m)
        yaw_error = abs(glq.bbs_engine.normalize_angle_rad(top.yaw_rad - true_yaw))
        assert yaw_error <= math.radians(15.0) + 1.0e-9, yaw_error
        scores = [candidate.score for candidate in result.candidates]
        assert scores == sorted(scores, reverse=True)


def test_query_handles_empty_scan():
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        engine = glq.GlobalLocalizationEngine(
            glq.GlobalLocalizationConfig(), occupancy_yaml=yaml_path
        )
        result = engine.query(np.empty((0, 3), dtype=np.float64))
        assert result.candidates == []
        assert result.scan_point_count == 0


def test_bbs_cpp_search_path_includes_installed_lib_dir():
    with tempfile.TemporaryDirectory() as tmp:
        prefix = Path(tmp) / "install"
        module_dir = prefix / "lib" / "lidar_localization_ros2"
        module_dir.mkdir(parents=True)
        old_ament = os.environ.get("AMENT_PREFIX_PATH")
        old_path = list(sys.path)
        try:
            os.environ["AMENT_PREFIX_PATH"] = str(prefix)
            glq._append_module_dirs("bbs_cpp")
            assert str(module_dir) in sys.path
        finally:
            if old_ament is None:
                os.environ.pop("AMENT_PREFIX_PATH", None)
            else:
                os.environ["AMENT_PREFIX_PATH"] = old_ament
            sys.path[:] = old_path


class _FakeScoreResult:
    def __init__(
        self, fitness, converged, refined_x, refined_y, refined_z, refined_yaw
    ):
        self.fitness = fitness
        self.converged = converged
        self.refined_x = refined_x
        self.refined_y = refined_y
        self.refined_z = refined_z
        self.refined_yaw = refined_yaw
        self.target_point_count = 1000
        self.source_point_count = 512


class _FakeRegistrationScorer:
    def __init__(self, results):
        self._results = results

    def score_candidates(self, scan_xyz, poses):
        return [self._results[tuple(pose)] for pose in poses]


def test_score_with_registration_rewrites_converged_candidate_pose():
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        engine = glq.GlobalLocalizationEngine(
            glq.GlobalLocalizationConfig(registration_refine_candidates=True),
            occupancy_yaml=yaml_path,
        )
        engine.registration_scorer = _FakeRegistrationScorer(
            {
                (-13.5, 29.7, 0.0, math.radians(10.0)): _FakeScoreResult(
                    fitness=0.042,
                    converged=True,
                    refined_x=-10.5,
                    refined_y=27.1,
                    refined_z=1.2,
                    refined_yaw=math.radians(12.0),
                ),
                (5.0, 6.0, 0.0, math.radians(20.0)): _FakeScoreResult(
                    fitness=float("inf"),
                    converged=False,
                    refined_x=99.0,
                    refined_y=99.0,
                    refined_z=99.0,
                    refined_yaw=math.radians(99.0),
                ),
            }
        )
        raw_candidates = [
            glq.GlobalLocalizationCandidate(
                x_m=-13.5,
                y_m=29.7,
                z_m=0.0,
                yaw_rad=math.radians(10.0),
                score=0.99,
                hit_count=100,
                point_count=512,
                bbs_score=0.99,
            ),
            glq.GlobalLocalizationCandidate(
                x_m=5.0,
                y_m=6.0,
                z_m=0.0,
                yaw_rad=math.radians(20.0),
                score=0.95,
                hit_count=90,
                point_count=512,
                bbs_score=0.95,
            ),
        ]
        ranked = engine._score_with_registration(
            np.zeros((8, 3), dtype=np.float64), raw_candidates
        )

        assert ranked[0].x_m == -10.5
        assert ranked[0].y_m == 27.1
        assert ranked[0].z_m == 1.2
        assert ranked[0].yaw_rad == math.radians(12.0)
        assert ranked[0].bbs_score == 0.99
        assert ranked[0].registration_fitness == 0.042

        unconverged = next(
            candidate
            for candidate in ranked
            if candidate.x_m == 5.0 and candidate.y_m == 6.0
        )
        assert unconverged.yaw_rad == math.radians(20.0)
        assert unconverged.z_m == 0.0
        assert not unconverged.registration_converged


def test_score_with_registration_keeps_raw_pose_by_default():
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        engine = glq.GlobalLocalizationEngine(
            glq.GlobalLocalizationConfig(), occupancy_yaml=yaml_path
        )
        engine.registration_scorer = _FakeRegistrationScorer(
            {
                (-13.5, 29.7, 0.0, math.radians(10.0)): _FakeScoreResult(
                    fitness=0.042,
                    converged=True,
                    refined_x=-10.5,
                    refined_y=27.1,
                    refined_z=1.2,
                    refined_yaw=math.radians(12.0),
                ),
            }
        )
        raw_candidates = [
            glq.GlobalLocalizationCandidate(
                x_m=-13.5,
                y_m=29.7,
                z_m=0.0,
                yaw_rad=math.radians(10.0),
                score=0.99,
                hit_count=100,
                point_count=512,
                bbs_score=0.99,
            ),
        ]
        ranked = engine._score_with_registration(
            np.zeros((8, 3), dtype=np.float64), raw_candidates
        )

        assert ranked[0].x_m == -13.5
        assert ranked[0].y_m == 29.7
        assert ranked[0].yaw_rad == math.radians(10.0)
        assert ranked[0].registration_fitness == 0.042
        # The 2D cell has no height; report the one registration converged to.
        assert ranked[0].z_m == 1.2


def test_score_with_registration_reports_scoring_height_when_unconverged():
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        engine = glq.GlobalLocalizationEngine(
            glq.GlobalLocalizationConfig(registration_seed_z_m=-11.4),
            occupancy_yaml=yaml_path,
        )
        engine.registration_scorer = _FakeRegistrationScorer(
            {
                (-13.5, 29.7, -11.4, 0.0): _FakeScoreResult(
                    fitness=float("nan"),
                    converged=False,
                    refined_x=99.0,
                    refined_y=99.0,
                    refined_z=99.0,
                    refined_yaw=99.0,
                ),
            }
        )
        raw_candidates = [
            glq.GlobalLocalizationCandidate(
                x_m=-13.5,
                y_m=29.7,
                z_m=0.0,
                yaw_rad=0.0,
                score=0.99,
                hit_count=100,
                point_count=512,
                bbs_score=0.99,
            ),
        ]
        ranked = engine._score_with_registration(
            np.zeros((8, 3), dtype=np.float64), raw_candidates
        )

        assert ranked[0].z_m == -11.4
        assert not ranked[0].registration_converged


def test_resolve_pclomp_search_method_mapping():
    assert glq.resolve_pclomp_search_method("kdtree") == 0
    assert glq.resolve_pclomp_search_method("direct26") == 1
    assert glq.resolve_pclomp_search_method("direct7") == 2
    assert glq.resolve_pclomp_search_method("DIRECT7") == 2
    assert glq.resolve_pclomp_search_method(" direct1 ") == 3

    try:
        glq.resolve_pclomp_search_method("octree")
        raise AssertionError("expected ValueError for an unknown search method")
    except ValueError:
        pass


def test_default_ndt_search_method_preserves_direct7_behavior():
    # The runtime default stays DIRECT7 so the G2 scorer behavior is unchanged
    # until the WP1 A/B explicitly selects KDTREE.
    assert glq.GlobalLocalizationConfig().ndt_search_method == "direct7"


def _write_reference_csv(path: Path, rows):
    import csv

    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "stamp_sec",
                "position_x",
                "position_y",
                "position_z",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def test_route_crop_generates_local_candidates():
    with tempfile.TemporaryDirectory() as tmp:
        reference_csv = Path(tmp) / "reference.csv"
        _write_reference_csv(
            reference_csv,
            [
                {
                    "stamp_sec": "100.0",
                    "position_x": "1.0",
                    "position_y": "2.0",
                    "position_z": "0.5",
                    "orientation_x": "0.0",
                    "orientation_y": "0.0",
                    "orientation_z": "0.0",
                    "orientation_w": "1.0",
                },
                {
                    "stamp_sec": "110.0",
                    "position_x": "11.0",
                    "position_y": "2.0",
                    "position_z": "0.5",
                    "orientation_x": "0.0",
                    "orientation_y": "0.0",
                    "orientation_z": "0.0",
                    "orientation_w": "1.0",
                },
            ],
        )
        config = glq.GlobalLocalizationConfig(
            candidate_source=glq.CANDIDATE_SOURCE_ROUTE_CROP,
            reference_csv=str(reference_csv),
            route_time_radius_sec=20.0,
            route_min_spacing_m=1.0,
            route_max_poses=8,
            route_yaw_offsets_deg="0",
            route_lateral_offsets_m="0",
            route_longitudinal_offsets_m="0",
            max_candidates=8,
        )
        engine = glq.GlobalLocalizationEngine(config)
        scan = np.array([[1.0, 0.0, 1.0], [2.0, 0.5, 1.0]], dtype=np.float64)
        result = engine.query(scan, scan_stamp_sec=105.0)

        assert result.candidate_source == glq.CANDIDATE_SOURCE_ROUTE_CROP
        assert result.candidates
        assert len(result.candidates) <= 8
        for candidate in result.candidates:
            assert math.hypot(candidate.x_m - 1.0, candidate.y_m - 2.0) <= 12.0
            assert math.hypot(candidate.x_m - 11.0, candidate.y_m - 2.0) <= 12.0


def test_route_crop_requires_scan_stamp():
    with tempfile.TemporaryDirectory() as tmp:
        reference_csv = Path(tmp) / "reference.csv"
        _write_reference_csv(
            reference_csv,
            [
                {
                    "stamp_sec": "100.0",
                    "position_x": "1.0",
                    "position_y": "2.0",
                    "position_z": "0.5",
                    "orientation_x": "0.0",
                    "orientation_y": "0.0",
                    "orientation_z": "0.0",
                    "orientation_w": "1.0",
                }
            ],
        )
        config = glq.GlobalLocalizationConfig(
            candidate_source=glq.CANDIDATE_SOURCE_ROUTE_CROP,
            reference_csv=str(reference_csv),
        )
        engine = glq.GlobalLocalizationEngine(config)
        scan = np.array([[1.0, 0.0, 1.0]], dtype=np.float64)
        result = engine.query(scan)

        assert result.candidates == []
        assert result.route_crop_error == "scan_stamp_sec required for route_crop"


def _route_rows(stamps):
    return [
        {
            "stamp_sec": str(stamp),
            "position_x": str(-1.0 + i),
            "position_y": "7.0",
            "position_z": "0.5",
            "orientation_x": "0.0",
            "orientation_y": "0.0",
            "orientation_z": "0.0",
            "orientation_w": "1.0",
        }
        for i, stamp in enumerate(stamps)
    ]


def test_route_crop_reports_scans_from_another_session():
    with tempfile.TemporaryDirectory() as tmp:
        reference_csv = Path(tmp) / "reference.csv"
        _write_reference_csv(reference_csv, _route_rows([100.0, 110.0]))
        config = glq.GlobalLocalizationConfig(
            candidate_source=glq.CANDIDATE_SOURCE_ROUTE_CROP,
            reference_csv=str(reference_csv),
            route_time_radius_sec=20.0,
        )
        engine = glq.GlobalLocalizationEngine(config)
        scan = np.array([[1.0, 0.0, 1.0]], dtype=np.float64)

        result = engine.query(scan, scan_stamp_sec=10_000.0)

        assert result.candidates == []
        assert "outside the reference trajectory" in result.route_crop_error


def test_route_crop_falls_back_to_bbs_outside_the_session():
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        reference_csv = Path(tmp) / "reference.csv"
        _write_reference_csv(reference_csv, _route_rows([100.0, 110.0]))
        config = glq.GlobalLocalizationConfig(
            candidate_source=glq.CANDIDATE_SOURCE_ROUTE_CROP,
            reference_csv=str(reference_csv),
            route_time_radius_sec=20.0,
            route_yaw_offsets_deg="0",
            route_lateral_offsets_m="0",
            route_longitudinal_offsets_m="0",
            angular_resolution_rad=math.radians(15.0),
            max_candidates=8,
            min_range_m=1.0,
        )
        engine = glq.GlobalLocalizationEngine(config, occupancy_yaml=yaml_path)
        true_x, true_y, true_yaw = -1.0, 7.0, math.radians(30.0)
        scan = make_scan(yaml_path, true_x, true_y, true_yaw)

        in_session = engine.query(scan, scan_stamp_sec=105.0)
        new_session = engine.query(scan, scan_stamp_sec=10_000.0)

        assert in_session.candidate_source == glq.CANDIDATE_SOURCE_ROUTE_CROP
        assert new_session.candidate_source == glq.CANDIDATE_SOURCE_BBS
        top = new_session.candidates[0]
        assert math.hypot(top.x_m - true_x, top.y_m - true_y) <= 1.0, (top.x_m, top.y_m)


def test_default_candidate_source_is_bbs():
    assert glq.GlobalLocalizationConfig().candidate_source == glq.CANDIDATE_SOURCE_BBS


def test_query_progress_callback_reports_phases():
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        true_x, true_y, true_yaw = -1.0, 7.0, math.radians(30.0)
        config = glq.GlobalLocalizationConfig(
            angular_resolution_rad=math.radians(15.0),
            max_candidates=8,
            min_range_m=1.0,
        )
        engine = glq.GlobalLocalizationEngine(config, occupancy_yaml=yaml_path)
        phases = []
        result = engine.query(
            make_scan(yaml_path, true_x, true_y, true_yaw),
            progress_callback=lambda phase, done, total: phases.append(phase),
        )
        assert result.candidates, "expected at least one candidate"
        assert "search" in phases
        assert phases[-1] == "done"


def _write_wall_map_pcd(path: Path) -> None:
    # Vertical walls along a 200 m strip, sampled every 0.2 m up to 3 m high.
    rng = np.random.default_rng(7)
    points = []
    for wall_y in (-6.0, 6.0):
        for x in np.arange(-100.0, 100.0, 0.2):
            for z in np.arange(0.0, 3.0, 0.2):
                points.append((x, wall_y + rng.normal(0.0, 0.01), z))
    for wall_x in np.arange(-95.0, 100.0, 15.0):
        for y in np.arange(-6.0, 6.0, 0.2):
            for z in np.arange(0.0, 3.0, 0.2):
                points.append((wall_x + 0.37 * abs(wall_x) % 3.0, y, z))
    lines = [
        "VERSION .7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    lines += [f"{x:.3f} {y:.3f} {z:.3f}" for x, y, z in points]
    path.write_text("\n".join(lines) + "\n")


def _zyx(yaw_deg, pitch_deg, roll_deg):
    y, p, r = np.radians([yaw_deg, pitch_deg, roll_deg])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    return rz @ ry @ rx


def test_level_attitude_levels_an_upside_down_or_tilted_scan():
    roll, pitch, level = glq.level_attitude(_zyx(90.0, 0.0, 180.0))
    assert math.isclose(abs(math.degrees(roll)), 180.0, abs_tol=1e-6)
    assert math.isclose(pitch, 0.0, abs_tol=1e-9)
    # A point above an upside-down sensor has negative z in its own frame.
    assert np.allclose(level @ np.array([1.0, 2.0, -3.0]), [1.0, -2.0, 3.0])

    tilted = _zyx(30.0, 7.0, -4.0)
    roll, pitch, level = glq.level_attitude(tilted)
    assert math.isclose(math.degrees(roll), -4.0, abs_tol=1e-6)
    assert math.isclose(math.degrees(pitch), 7.0, abs_tol=1e-6)
    # Levelling keeps the heading: tilted == Rz(30 deg) @ level.
    assert np.allclose(_zyx(30.0, 0.0, 0.0) @ level, tilted)


def test_quaternion_matrix_matches_the_zyx_rotation():
    # Roll 180 deg about x then yaw 90 deg: q = qz(90) * qx(180).
    s = math.sqrt(0.5)
    assert np.allclose(glq.quaternion_matrix(s, s, 0.0, 0.0), _zyx(90.0, 0.0, 180.0))


class _GroundScorer:
    """Unconverged results; the ground is at z = -10 for x > 0 and unknown elsewhere."""

    def __init__(self):
        self.poses = []

    def ground_z(self, x, y):
        return -10.0 if x > 0.0 else float("nan")

    def score_candidates(self, scan_xyz, poses):
        self.poses.extend(poses)
        return [
            _FakeScoreResult(
                fitness=float("nan"),
                converged=False,
                refined_x=0.0,
                refined_y=0.0,
                refined_z=0.0,
                refined_yaw=0.0,
            )
            for _ in poses
        ]


def test_flat_candidates_are_scored_at_the_ground_plus_the_sensor_height():
    def candidate(x_m, z_m=0.0):
        return glq.GlobalLocalizationCandidate(
            x_m=x_m,
            y_m=0.0,
            z_m=z_m,
            yaw_rad=0.0,
            score=0.9,
            hit_count=100,
            point_count=512,
            bbs_score=0.9,
        )

    candidates = [candidate(5.0), candidate(-5.0), candidate(5.0, z_m=-3.0)]
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        for sensor_height, expected in (
            (1.5, [-8.5, 0.4, -3.0]),
            (-1.0, [0.4, 0.4, -3.0]),
        ):
            engine = glq.GlobalLocalizationEngine(
                glq.GlobalLocalizationConfig(
                    registration_seed_z_m=0.4,
                    registration_sensor_height_m=sensor_height,
                ),
                occupancy_yaml=yaml_path,
            )
            engine.registration_scorer = _GroundScorer()
            ranked = engine._score_with_registration(
                np.zeros((8, 3), dtype=np.float64), candidates
            )
            # A candidate with a height keeps it; without ground, the seed height.
            assert [pose[2] for pose in engine.registration_scorer.poses] == expected
            assert sorted(c.z_m for c in ranked) == sorted(expected)


def test_registration_scorer_reports_the_ground_height():
    try:
        glq._append_module_dirs("g2_ndt_score")
        import g2_ndt_score
    except ImportError:
        print("skipping: g2_ndt_score is not built")
        return
    with tempfile.TemporaryDirectory() as tmp:
        # A ramp rising 1 m every 10 m, a wall on it, and one stray return 3 m
        # below the ground at the origin.
        points = [
            (x, y, 0.1 * x)
            for x in np.arange(-20.0, 20.0, 0.2)
            for y in np.arange(-5.0, 5.0, 0.2)
        ]
        points += [
            (x, 5.0, 0.1 * x + z)
            for x in np.arange(-20.0, 20.0, 0.2)
            for z in (1.0, 2.0)
        ]
        points.append((0.1, 0.1, -3.0))
        lines = [
            "VERSION .7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA ascii",
        ]
        lines += [f"{x:.3f} {y:.3f} {z:.3f}" for x, y, z in points]
        map_path = Path(tmp) / "ramp.pcd"
        map_path.write_text("\n".join(lines) + "\n")
        scorer = g2_ndt_score.MapNdtScorer(str(map_path))

        assert abs(scorer.ground_z(10.5, 0.5) - 1.0) < 0.15
        assert abs(scorer.ground_z(-10.5, 4.5) + 1.1) < 0.15
        assert abs(scorer.ground_z(0.5, 0.5)) < 0.15
        assert math.isnan(scorer.ground_z(500.0, 0.0))


def test_registration_scorer_shares_map_crops_without_changing_scores():
    try:
        glq._append_module_dirs("g2_ndt_score")
        import g2_ndt_score
    except ImportError:
        print("skipping: g2_ndt_score is not built")
        return
    with tempfile.TemporaryDirectory() as tmp:
        map_path = Path(tmp) / "walls.pcd"
        _write_wall_map_pcd(map_path)
        scorer = g2_ndt_score.MapNdtScorer(
            str(map_path),
            target_voxel_leaf_size=0.2,
            local_map_radius=60.0,
        )
        map_points = np.loadtxt(map_path, skiprows=10)
        sensor = np.array([40.0, 0.0, 0.0])
        offsets = map_points - sensor
        scan = offsets[np.hypot(offsets[:, 0], offsets[:, 1]) < 25.0]
        # Two candidates whose 25 m scan fits one 60 m crop (the second crop alone
        # would reach past the map's end at x = 100), and one far away.
        poses = [(40.0, 0.0, 0.0, 0.0), (48.0, 0.5, 0.0, 0.05), (-40.0, 0.0, 0.0, 0.0)]

        together = scorer.score_candidates(scan, poses)
        alone = [scorer.score_candidate(scan, *pose) for pose in poses]

        for shared, single in zip(together, alone, strict=True):
            assert shared.converged == single.converged
            assert math.isclose(shared.fitness, single.fitness, rel_tol=1e-6)
            assert math.isclose(shared.refined_x, single.refined_x, abs_tol=1e-6)
        assert together[0].fitness < together[2].fitness
        # The near candidate reused the first crop; the far one needed its own.
        assert together[1].target_point_count == together[0].target_point_count
        assert alone[1].target_point_count != alone[0].target_point_count
        assert together[2].target_point_count == alone[2].target_point_count


if __name__ == "__main__":
    test_query_recovers_known_pose()
    test_query_handles_empty_scan()
    test_bbs_cpp_search_path_includes_installed_lib_dir()
    test_score_with_registration_rewrites_converged_candidate_pose()
    test_score_with_registration_keeps_raw_pose_by_default()
    test_resolve_pclomp_search_method_mapping()
    test_default_ndt_search_method_preserves_direct7_behavior()
    test_route_crop_generates_local_candidates()
    test_route_crop_requires_scan_stamp()
    test_route_crop_reports_scans_from_another_session()
    test_route_crop_falls_back_to_bbs_outside_the_session()
    test_default_candidate_source_is_bbs()
    test_query_progress_callback_reports_phases()
    test_registration_scorer_shares_map_crops_without_changing_scores()
    print("test_global_localization_query: all tests passed")
