#!/usr/bin/env python3
"""tools/benchmark/dynamic_obstacles.py without ROS: ray casting, occlusion, the
obstacle share it reaches, determinism and the scenarios."""

from __future__ import annotations

import math
import struct
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "benchmark"))

import dynamic_obstacles as dyn


def _straight_trajectory(speed=1.0, seconds=20.0, height=1.0):
    """A sensor driving along +x at SPEED, HEIGHT m over a level ground at z = 0."""
    t = np.arange(0.0, seconds, 0.1)
    rows = np.zeros((len(t), 8))
    rows[:, 0] = 100.0 + t
    rows[:, 1] = speed * t
    rows[:, 3] = height
    rows[:, 7] = 1.0
    return dyn.Trajectory(rows)


def _ring_scan(radius=20.0, height=1.0):
    """A cylinder wall at RADIUS and the ground, seen from the origin HEIGHT m up."""
    az = np.radians(np.arange(-180.0, 180.0, 1.0))
    el = np.radians(np.arange(-15.0, 15.0, 1.0))
    a, e = (g.ravel() for g in np.meshgrid(az, el, indexing="ij"))
    dirs = np.column_stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    wall = radius / np.cos(e)
    ground = np.where(dirs[:, 2] < 0, height / np.maximum(-dirs[:, 2], 1e-9), np.inf)
    return dirs * np.minimum(wall, ground)[:, None]


def test_ray_hits_a_box_at_its_near_face():
    obstacle = dyn.Obstacle("car", 10.0, 0.0, 0.0, 0.0)
    box = dyn.box_in_sensor(obstacle, np.eye(3), np.array([0.0, 0.0, 1.0]))
    dirs = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    distance = dyn.ray_box_distance(dirs, box)
    # The car is 4.5 m long: its near face is 7.75 m ahead.
    assert math.isclose(distance[0], 7.75, abs_tol=1e-5)
    assert np.isinf(distance[1]) and np.isinf(distance[2])


def test_box_reach_bounds_the_box():
    obstacle = dyn.Obstacle("truck", 0.0, 4.0, 0.0, 0.0)
    box = dyn.box_in_sensor(obstacle, np.eye(3), np.array([0.0, 0.0, 2.0]))
    near, far, bottom, top = dyn.box_reach(box)
    # The truck's side is 2.75 m away; its roof is 1 m above the sensor.
    assert math.isclose(near, 2.75, abs_tol=1e-6)
    assert far > 6.0
    assert bottom < 0.0 < top


def test_azimuth_window_wraps_behind_the_sensor():
    obstacle = dyn.Obstacle("car", -10.0, 0.0, 0.0, 0.0)
    box = dyn.box_in_sensor(obstacle, np.eye(3), np.zeros(3))
    window = dyn.azimuth_window(box)
    assert window is not None
    assert dyn.in_window(np.array([math.pi, -math.pi + 0.01]), window).all()
    assert not dyn.in_window(np.array([0.0]), window).any()


def test_render_occludes_and_samples_the_obstacle():
    xyz = _ring_scan()
    obstacle = dyn.Obstacle("truck", 0.0, 5.0, -1.0, 0.0)
    box = dyn.box_in_sensor(obstacle, np.eye(3), np.zeros(3))
    pattern = dyn.RayPattern.parse("rings:-15:15:31:1")
    rendered = dyn.render(
        xyz, [box], pattern, np.random.default_rng(0), None, 0.0, 0.5, 100.0
    )
    hidden = ~rendered.keep
    # Wall points behind the truck are gone, the ones in front of the sensor stay.
    assert hidden.any()
    assert (xyz[hidden, 1] > 0).all()
    assert rendered.keep[(xyz[:, 1] < -1.0)].all()
    # The new points lie on the truck's near side, 3.75 m to the left.
    side = rendered.obstacle_points
    assert len(side) > 50
    assert np.allclose(side[:, 1], 3.75, atol=1e-3)
    assert 0.0 < rendered.fraction < 1.0
    assert rendered.shown == 1


def test_render_adds_obstacles_until_the_share_is_reached():
    xyz = _ring_scan()
    boxes = [
        dyn.box_in_sensor(
            dyn.Obstacle("pedestrian", 3.0 * math.cos(a), 3.0 * math.sin(a), -1.0, 0.0),
            np.eye(3),
            np.zeros(3),
        )
        for a in np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
    ]
    pattern = dyn.RayPattern.parse("rings:-15:15:31:1")
    shares = []
    for target in (0.0, 0.05, 0.15, 1.0):
        rendered = dyn.render(
            xyz, boxes, pattern, np.random.default_rng(0), target, 0.0, 0.5, 100.0
        )
        shares.append((rendered.shown, rendered.fraction))
    assert shares[0] == (0, 0.0)
    assert shares[1][1] >= 0.05 and shares[2][1] >= 0.15
    assert shares[1][0] < shares[2][0] < shares[3][0] == 24
    # Counted in 2 m voxels, near obstacles weigh less than their points.
    voxels = dyn.render(
        xyz, boxes, pattern, np.random.default_rng(0), None, 0.0, 0.5, 100.0, 2.0
    )
    assert voxels.fraction < voxels.point_fraction


def test_scenarios_are_deterministic_and_seeded():
    traj = _straight_trajectory(speed=5.0)
    for scenario in dyn.SCENARIOS:
        a = dyn.scenario_obstacles(scenario, 105.0, traj, 7, 1.0)
        b = dyn.scenario_obstacles(scenario, 105.0, traj, 7, 1.0)
        c = dyn.scenario_obstacles(scenario, 105.0, traj, 8, 1.0)
        assert a == b, scenario
        if scenario != "alongside":
            assert a != c, scenario
        assert all(o.ground_z == 0.0 for o in a)


def test_alongside_moves_with_the_sensor_and_crowd_walks():
    traj = _straight_trajectory(speed=5.0)
    first = dyn.scenario_obstacles("alongside", 102.0, traj, 0, 1.0)[0]
    later = dyn.scenario_obstacles("alongside", 104.0, traj, 0, 1.0)[0]
    # Static relative to the sensor: 10 m further after 2 s at 5 m/s.
    assert math.isclose(later.x - first.x, 10.0, abs_tol=1e-6)
    assert math.isclose(first.y, 3.5) and first.kind == "truck"
    walker = dyn.scenario_obstacles("crowd", 102.0, traj, 0, 1.0, count=1)[0]
    step = dyn.scenario_obstacles("crowd", 102.1, traj, 0, 1.0, count=1)[0]
    assert 0.04 <= math.hypot(step.x - walker.x, step.y - walker.y) <= 0.16
    # Parked obstacles do not move.
    assert dyn.scenario_obstacles(
        "parked", 101.0, traj, 0, 1.0
    ) == dyn.scenario_obstacles("parked", 110.0, traj, 0, 1.0)


def test_level_ground_overrides_the_sensor_height():
    traj = _straight_trajectory(height=2.0)
    obstacles = dyn.scenario_obstacles("startup", 100.0, traj, 0, ("z", 0.25))
    assert all(o.ground_z == 0.25 for o in obstacles)


def test_placement_skips_the_sensor_and_map_structure():
    sensor = np.array([0.0, 0.0])
    assert not dyn.placement_ok(dyn.Obstacle("car", 1.0, 0.0, 0.0, 0.0), sensor, None)
    assert dyn.placement_ok(dyn.Obstacle("car", 6.0, 0.0, 0.0, 0.0), sensor, None)
    wall = np.array(
        [[6.0, y, z] for y in np.arange(-3, 3, 0.1) for z in np.arange(0, 3, 0.1)]
    )
    occupancy = dyn.MapOccupancy(wall)
    assert not dyn.placement_ok(
        dyn.Obstacle("car", 6.0, 0.0, 0.0, 0.0), sensor, occupancy
    )
    assert dyn.placement_ok(dyn.Obstacle("car", -6.0, 0.0, 0.0, 0.0), sensor, occupancy)


def test_sensor_height_is_estimated_from_the_ground():
    xyz = _ring_scan(height=1.8)
    assert math.isclose(dyn.estimate_sensor_height(xyz, np.eye(3)), 1.8, abs_tol=0.05)


def test_trajectory_interpolates_and_reports_heading():
    rows = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, math.sin(0.25), math.cos(0.25)],
        ]
    )
    traj = dyn.Trajectory(rows)
    _, position = traj.pose(0.5)
    assert np.allclose(position, [1.0, 0.0, 0.0])
    assert math.isclose(traj.heading(1.0), 0.5, abs_tol=1e-9)
    assert traj.covers(0.5) and not traj.covers(5.0)
    assert math.isclose(traj.at_distance(1.5), 1.0)


def test_pattern_parsing():
    rings = dyn.RayPattern.parse("rings:-25:15:128:0.2")
    assert rings.kind == "rings" and rings.rings == 128
    random = dyn.RayPattern.parse("random:-7:52:20000")
    assert random.kind == "random" and random.count == 20000
    rays = random.rays(None, np.random.default_rng(0), 100)
    assert len(rays) == 20000
    assert np.allclose(np.linalg.norm(rays, axis=1), 1.0)
    for bad in ("rings:1:2", "spiral:0:1"):
        try:
            dyn.RayPattern.parse(bad)
        except ValueError:
            continue
        raise AssertionError(bad)


def _cloud(points, extra=b"\x07"):
    """A PointCloud2-like message of float32 x, y, z plus one more byte."""
    step = 13
    data = b"".join(struct.pack("<fff", *p) + extra for p in points)
    fields = [
        types.SimpleNamespace(name=n, offset=4 * i, datatype=7)
        for i, n in enumerate("xyz")
    ]
    return types.SimpleNamespace(
        fields=fields,
        width=len(points),
        height=1,
        point_step=step,
        row_step=step * len(points),
        data=data,
    )


def test_cloud_rows_keep_their_other_fields():
    msg = _cloud([(1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (-3.0, 0.0, 0.0)])
    raw, xyz, fields = dyn.cloud_arrays(msg)
    assert np.allclose(xyz[1], [0.0, 2.0, 0.0])
    keep = np.array([True, False, True])
    rows = dyn.compose_rows(raw, xyz, keep, np.array([[0.0, 5.0, 1.0]]), fields)
    assert rows.shape == (3, 13)
    added = struct.unpack("<fff", rows[2, :12].tobytes())
    assert np.allclose(added, [0.0, 5.0, 1.0])
    # The added point copies the extra byte of the real point nearest in azimuth.
    assert rows[2, 12] == 7


def test_injector_passes_scans_outside_the_trajectory_and_window():
    traj = _straight_trajectory()
    args = types.SimpleNamespace(
        pattern="rings:-15:15:31:1",
        sensor_height="1.0",
        ground_z=None,
        active="5:",
        scenario="alongside",
        seed=0,
        fraction=None,
        obstacle_leaf=0.0,
        min_range=0.5,
        max_range=100.0,
        count_leaf=0.0,
    )
    injector = dyn.Injector(args, traj, None)
    xyz = _ring_scan()
    assert injector.process(50.0, xyz) == (None, [])
    # The window counts from the first scan in the trajectory.
    assert injector.process(103.0, xyz) == (None, [])
    assert injector.process(107.0, xyz) == (None, [])
    rendered, obstacles = injector.process(110.0, xyz)
    assert rendered is not None and rendered.shown == len(obstacles) > 0
