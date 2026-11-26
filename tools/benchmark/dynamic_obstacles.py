#!/usr/bin/env python3
"""Inject synthetic dynamic obstacles into replayed LiDAR scans, online and seeded.

A stress test for localization against a prior map: people, cars and trucks that are
not in the map are rendered into real scans with ground truth, so occlusion and the
outlier load can be varied without recording new data. The tool subscribes to the
replayed cloud (play the bag with ``--remap TOPIC:=TOPIC_raw``), adds the obstacles and
republishes it on the original topic, so nothing else in the replay changes and no
rewritten bag is kept.

Obstacles are boxes standing on the ground:

  pedestrian 0.5 x 0.5 x 1.7 m, car 4.5 x 1.8 x 1.5 m, truck or bus 10 x 2.5 x 3 m

They are placed in the ground-truth (= map) frame from the ground-truth trajectory, and
rendered by ray casting from the sensor:
  * every real point whose ray hits an obstacle before it is removed (occlusion);
  * the obstacle's surface is sampled with the sensor's pattern: rings (``--pattern
    rings:EL_MIN:EL_MAX:RINGS:AZ_STEP_DEG``, e.g. a spinning Velodyne) or uniformly
    random rays (``--pattern random:EL_MIN:EL_MAX[:COUNT]``, a Livox MID-360; COUNT
    defaults to the scan's point count), thinned to ``--obstacle-leaf`` like the bag;
  * a synthetic return behind a real one in the same direction is dropped.

Scenarios (``--scenario``):

  crowd      pedestrians walking around the sensor, 1.5-10 m away
  crossing   cars (every fourth a truck) crossing 5-25 m ahead at 6-12 m/s
  alongside  trucks and cars that move with the sensor: static relative to it
  parked     cars, trucks and standing people along the route, fixed in the map
  startup    cars and people fixed around the start pose (global localization)

``--fraction F`` sets the load: obstacles are added in a fixed, seeded order until F of
the output points within ``--min-range``..``--max-range`` lie on obstacles (0 adds
none). Without it, every obstacle of the scenario is shown. Each obstacle's
placement is a function of the seed, the scenario and the time only, so a run is
reproducible whatever the scan rate or start. A placement that cuts the map (a wall)
or the sensor is skipped when ``--map`` is given.

``--log CSV`` writes one row per scan: the achieved share, the share after a voxel
filter of ``--count-leaf`` (the localizer's), the obstacles shown and the time taken.

usage: dynamic_obstacles.py --input T_raw --output T --gt GT.tum --scenario S
                            [--fraction F] [--seed N] [--pattern P] [--map MAP]
                            [--sensor-height H|auto] [--log CSV] [--active A:B]
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

KINDS = {
    "pedestrian": (0.5, 0.5, 1.7),
    "car": (4.5, 1.8, 1.5),
    "truck": (10.0, 2.5, 3.0),
}
SCENARIOS = ("crowd", "crossing", "alongside", "parked", "startup")
# A synthetic return this much behind a real one in the same direction is hidden.
HIDE_MARGIN_M = 0.5
# Obstacles farther than this from the sensor are left out: they add little.
OBSTACLE_RANGE_M = 45.0
# Angular cells of the real returns' range image, for hiding synthetic returns.
RANGE_IMAGE_CELL_DEG = 1.0


@dataclass(frozen=True)
class Obstacle:
    """A box standing on the ground, in the map frame."""

    kind: str
    x: float
    y: float
    ground_z: float
    yaw: float

    @property
    def size(self) -> tuple[float, float, float]:
        return KINDS[self.kind]


# ----------------------------------------------------------------------------------
# Trajectory


def quaternion_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class Trajectory:
    """Ground-truth sensor poses (TUM: t x y z qx qy qz qw), interpolated."""

    def __init__(self, rows: np.ndarray):
        rows = np.asarray(rows, dtype=float)
        rows = rows[np.argsort(rows[:, 0])]
        self.t = rows[:, 0]
        self.p = rows[:, 1:4]
        self.q = rows[:, 4:8] / np.linalg.norm(rows[:, 4:8], axis=1, keepdims=True)
        step = np.linalg.norm(np.diff(self.p[:, :2], axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(step)])

    @classmethod
    def load(cls, path) -> Trajectory:
        return cls(np.loadtxt(path, comments="#"))

    @property
    def start(self) -> float:
        return float(self.t[0])

    def covers(self, stamp: float, slack: float = 0.1) -> bool:
        return self.t[0] - slack <= stamp <= self.t[-1] + slack

    def pose(self, stamp: float) -> tuple[np.ndarray, np.ndarray]:
        """(R, p) of the sensor in the map frame at STAMP (clamped to the ends)."""
        i = int(np.clip(np.searchsorted(self.t, stamp), 1, len(self.t) - 1))
        t0, t1 = self.t[i - 1], self.t[i]
        a = 0.0 if t1 <= t0 else float(np.clip((stamp - t0) / (t1 - t0), 0.0, 1.0))
        q0, q1 = self.q[i - 1], self.q[i]
        if np.dot(q0, q1) < 0.0:
            q1 = -q1
        q = (1.0 - a) * q0 + a * q1
        return quaternion_matrix(q), (1.0 - a) * self.p[i - 1] + a * self.p[i]

    def heading(self, stamp: float) -> float:
        """The sensor's x axis projected on the ground, as a map-frame yaw."""
        rotation, _ = self.pose(stamp)
        return math.atan2(rotation[1, 0], rotation[0, 0])

    def at_distance(self, s: float) -> float:
        """The first stamp at which the route has covered S metres (clamped)."""
        i = int(np.clip(np.searchsorted(self.s, s), 0, len(self.t) - 1))
        return float(self.t[i])


# ----------------------------------------------------------------------------------
# Scenarios: each obstacle's placement is a function of (seed, scenario, index, time).


def _rng(seed: int, *keys: int) -> np.random.Generator:
    return np.random.default_rng([int(seed) & 0xFFFFFFFF, *[int(k) for k in keys]])


def _ground(traj: Trajectory, stamp: float, sensor_height) -> np.ndarray:
    """The ground under the sensor at STAMP. SENSOR_HEIGHT is the sensor's height over
    it, or ("z", Z) for a level ground at map height Z."""
    _, p = traj.pose(stamp)
    if isinstance(sensor_height, tuple):
        return np.array([p[0], p[1], sensor_height[1]])
    return np.array([p[0], p[1], p[2] - sensor_height])


def _episode(seed, salt, index, stamp, start, length):
    """(rng, spawn stamp) of obstacle INDEX's current episode of LENGTH seconds."""
    phase = _rng(seed, salt, index, 0).uniform(0.0, length)
    episode = math.floor((stamp - start - phase) / length)
    return _rng(
        seed, salt, index, episode + 1_000_000
    ), start + phase + episode * length


def scenario_obstacles(
    scenario: str,
    stamp: float,
    traj: Trajectory,
    seed: int,
    sensor_height: float,
    count: int | None = None,
) -> list[Obstacle]:
    """The scenario's obstacles at STAMP, in their fixed priority order."""
    start = traj.start
    if scenario == "crowd":
        length = 10.0
        out = []
        for i in range(count or 120):
            rng, spawn = _episode(seed, 1, i, stamp, start, length)
            # Centred on where the sensor is halfway through the episode, so a moving
            # sensor passes through the crowd.
            centre = _ground(traj, spawn + 0.5 * length, sensor_height)
            radius = rng.uniform(1.5, 10.0)
            bearing = rng.uniform(-math.pi, math.pi)
            walk = rng.uniform(-math.pi, math.pi)
            speed = rng.uniform(0.5, 1.5)
            dt = stamp - (spawn + 0.5 * length)
            out.append(
                Obstacle(
                    "pedestrian",
                    centre[0]
                    + radius * math.cos(bearing)
                    + speed * dt * math.cos(walk),
                    centre[1]
                    + radius * math.sin(bearing)
                    + speed * dt * math.sin(walk),
                    centre[2],
                    walk,
                )
            )
        return out
    if scenario == "crossing":
        # Platoons of three, 7 m apart (the last a truck in every other platoon).
        length = 4.0
        out = []
        for i in range(count or 48):
            platoon, member = divmod(i, 3)
            rng, spawn = _episode(seed, 2, platoon, stamp, start, length)
            middle = spawn + 0.5 * length
            centre = _ground(traj, middle, sensor_height)
            heading = traj.heading(middle)
            ahead = rng.uniform(5.0, 25.0)
            direction = heading + (math.pi / 2 if rng.uniform() < 0.5 else -math.pi / 2)
            speed = rng.uniform(6.0, 12.0)
            travelled = speed * (stamp - middle) - 7.0 * member
            out.append(
                Obstacle(
                    "truck" if member == 2 and platoon % 2 else "car",
                    centre[0]
                    + ahead * math.cos(heading)
                    + travelled * math.cos(direction),
                    centre[1]
                    + ahead * math.sin(heading)
                    + travelled * math.sin(direction),
                    centre[2],
                    direction,
                )
            )
        return out
    if scenario == "alongside":
        # (ahead, left, kind) in the sensor's heading frame: static relative to it.
        slots = [
            (0.0, 3.5, "truck"),
            (0.0, -3.5, "truck"),
            (11.0, 3.5, "car"),
            (-11.0, -3.5, "car"),
            (0.0, 7.0, "truck"),
            (0.0, -7.0, "truck"),
            (11.0, -3.5, "car"),
            (-11.0, 3.5, "car"),
            (13.0, 7.0, "truck"),
            (-13.0, -7.0, "truck"),
            (6.5, 0.0, "car"),
            (-6.5, 0.0, "car"),
            (0.0, 2.6, "car"),
            (0.0, -2.6, "car"),
            (-13.0, 7.0, "truck"),
            (13.0, -7.0, "truck"),
            (24.0, 3.5, "truck"),
            (-24.0, -3.5, "truck"),
            (24.0, -3.5, "truck"),
            (-24.0, 3.5, "truck"),
            (13.0, 0.0, "truck"),
            (-13.0, 0.0, "truck"),
            (24.0, 7.0, "truck"),
            (-24.0, -7.0, "truck"),
            (24.0, -7.0, "truck"),
            (-24.0, 7.0, "truck"),
        ]
        ground = _ground(traj, stamp, sensor_height)
        heading = traj.heading(stamp)
        c, s = math.cos(heading), math.sin(heading)
        return [
            Obstacle(
                kind,
                ground[0] + c * a - s * b,
                ground[1] + s * a + c * b,
                ground[2],
                heading,
            )
            for a, b, kind in slots[: count or len(slots)]
        ]
    if scenario == "parked":
        spacing = 3.0
        n = count or 2 * (int(traj.s[-1] / spacing) + 4)
        out = []
        for i in range(n):
            anchor, row = divmod(i, 2)
            rng = _rng(seed, 4, i)
            along = traj.at_distance(anchor * spacing + rng.uniform(0.0, spacing))
            base = _ground(traj, along, sensor_height)
            heading = traj.heading(along)
            roll = rng.uniform()
            kind = "car" if roll < 0.7 else ("truck" if roll < 0.85 else "pedestrian")
            side = 1.0 if row == 0 else -1.0
            lateral = side * rng.uniform(2.5, 5.0)
            ahead = rng.uniform(-2.0, 2.0)
            c, s = math.cos(heading), math.sin(heading)
            yaw = heading + rng.uniform(-0.15, 0.15)
            if kind == "car" and rng.uniform() < 0.2:
                yaw += math.pi / 2
            out.append(
                Obstacle(
                    kind,
                    base[0] + c * ahead - s * lateral,
                    base[1] + s * ahead + c * lateral,
                    base[2],
                    yaw,
                )
            )
        return out
    if scenario == "startup":
        base = _ground(traj, start, sensor_height)
        out = []
        for i in range(count or 48):
            rng = _rng(seed, 5, i)
            kind = "pedestrian" if i % 3 != 2 else "car"
            radius = (
                rng.uniform(1.5, 6.0)
                if kind == "pedestrian"
                else rng.uniform(3.5, 12.0)
            )
            bearing = rng.uniform(-math.pi, math.pi)
            out.append(
                Obstacle(
                    kind,
                    base[0] + radius * math.cos(bearing),
                    base[1] + radius * math.sin(bearing),
                    base[2],
                    rng.uniform(-math.pi, math.pi),
                )
            )
        return out
    raise ValueError(f"unknown scenario {scenario!r}; one of {', '.join(SCENARIOS)}")


# ----------------------------------------------------------------------------------
# Free space


class MapOccupancy:
    """Occupied voxels of the map, to skip obstacles placed inside structure."""

    def __init__(self, points: np.ndarray, leaf: float = 0.4):
        self.leaf = leaf
        self.keys = np.unique(self._keys(np.asarray(points, dtype=float)))

    def _keys(self, points: np.ndarray) -> np.ndarray:
        return voxel_keys(points, self.leaf)

    def occupied(self, points: np.ndarray) -> np.ndarray:
        keys = self._keys(points)
        index = np.clip(np.searchsorted(self.keys, keys), 0, len(self.keys) - 1)
        return self.keys[index] == keys


def box_samples(obstacle: Obstacle, step: float = 0.4, ground_clearance: float = 0.8):
    """Points inside the box, above GROUND_CLEARANCE, in the map frame."""
    length, width, height = obstacle.size
    axes = [
        np.arange(-length / 2 + step / 2, length / 2, step),
        np.arange(-width / 2 + step / 2, width / 2, step),
        np.arange(ground_clearance, height, step),
    ]
    a, b, z = (g.ravel() for g in np.meshgrid(*axes, indexing="ij"))
    c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
    return np.column_stack(
        [obstacle.x + c * a - s * b, obstacle.y + s * a + c * b, obstacle.ground_z + z]
    )


def placement_ok(
    obstacle: Obstacle,
    sensor_xy: np.ndarray,
    occupancy: MapOccupancy | None,
    clearance: float = 0.7,
    max_occupied_share: float = 0.03,
) -> bool:
    """Not around the sensor, and (with a map) not cutting map structure: at most
    MAX_OCCUPIED_SHARE of the box above 0.8 m (curbs, slopes and stray points aside)
    in occupied map voxels."""
    length, width, _ = obstacle.size
    rel = sensor_xy - np.array([obstacle.x, obstacle.y])
    c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
    along, across = c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]
    if abs(along) < length / 2 + clearance and abs(across) < width / 2 + clearance:
        return False
    if occupancy is None:
        return True
    return occupancy.occupied(box_samples(obstacle)).mean() <= max_occupied_share


# ----------------------------------------------------------------------------------
# Rendering


def box_in_sensor(obstacle: Obstacle, rotation: np.ndarray, position: np.ndarray):
    """(centre, rotation, half extents) of the box in the sensor frame."""
    length, width, height = obstacle.size
    centre = np.array([obstacle.x, obstacle.y, obstacle.ground_z + height / 2])
    c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
    yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return (
        rotation.T @ (centre - position),
        rotation.T @ yaw,
        np.array([length / 2, width / 2, height / 2]),
    )


def ray_box_distance(dirs: np.ndarray, box) -> np.ndarray:
    """Distance along each unit ray from the origin to the box; inf for a miss."""
    centre, rotation, half = box
    d = dirs.astype(np.float32) @ rotation.astype(np.float32)  # the rays, box frame
    d[np.abs(d) < 1e-9] = 1e-9
    o = -(centre @ rotation).astype(np.float32)  # the origin in the box frame
    half = half.astype(np.float32)
    inv = 1.0 / d
    t1 = (-half - o) * inv
    t2 = (half - o) * inv
    lo, hi = np.minimum(t1, t2), np.maximum(t1, t2)
    near = np.maximum(np.maximum(lo[:, 0], lo[:, 1]), lo[:, 2])
    far = np.minimum(np.minimum(hi[:, 0], hi[:, 1]), hi[:, 2])
    return np.where((far >= near) & (near > 0.0), near, np.inf)


def box_reach(box) -> tuple[float, float, float, float]:
    """(nearest distance, farthest distance, lowest and highest elevation) of the box
    from the sensor; the elevations conservatively."""
    centre, rotation, half = box
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    z = (centre + (signs * half) @ rotation.T)[:, 2]
    origin = -(centre @ rotation)  # the sensor in the box frame
    near = max(0.05, float(np.linalg.norm(origin - np.clip(origin, -half, half))))
    far = float(np.linalg.norm(centre)) + float(np.linalg.norm(half))

    def elevation(value, distance):
        return math.asin(max(-1.0, min(1.0, value / distance)))

    top = elevation(z.max(), near if z.max() >= 0.0 else far)
    bottom = elevation(z.min(), near if z.min() <= 0.0 else far)
    return near, far, bottom, top


def azimuth_window(box) -> tuple[float, float] | None:
    """(start, width) of the box's azimuth span seen from the sensor; None if it
    surrounds the sensor's z axis."""
    centre, rotation, half = box
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    corners = centre + (signs * half) @ rotation.T
    if np.hypot(centre[0], centre[1]) < np.linalg.norm(half[:2]):
        return None
    az = np.arctan2(corners[:, 1], corners[:, 0])
    mid = math.atan2(centre[1], centre[0])
    rel = np.angle(np.exp(1j * (az - mid)))
    return mid + rel.min(), rel.max() - rel.min()


def in_window(azimuth: np.ndarray, window) -> np.ndarray:
    if window is None:
        return np.ones(len(azimuth), dtype=bool)
    start, width = window
    return np.mod(azimuth - start, 2.0 * math.pi) <= width


@dataclass
class RayPattern:
    """Rings at fixed elevations, or uniformly random rays over an elevation band."""

    kind: str
    el_min: float
    el_max: float
    rings: int = 64
    az_step: float = math.radians(0.2)
    count: int = 0

    @classmethod
    def parse(cls, text: str) -> RayPattern:
        parts = text.split(":")
        if parts[0] == "rings" and len(parts) == 5:
            return cls(
                "rings",
                math.radians(float(parts[1])),
                math.radians(float(parts[2])),
                rings=int(parts[3]),
                az_step=math.radians(float(parts[4])),
            )
        if parts[0] == "random" and len(parts) in (3, 4):
            return cls(
                "random",
                math.radians(float(parts[1])),
                math.radians(float(parts[2])),
                count=int(parts[3]) if len(parts) == 4 else 0,
            )
        raise ValueError(
            f"bad --pattern {text!r}: rings:EL_MIN:EL_MAX:RINGS:AZ_STEP_DEG or "
            "random:EL_MIN:EL_MAX[:COUNT]"
        )

    def rays(
        self,
        window,
        rng: np.random.Generator,
        scan_points: int,
        min_spacing_rad: float = 0.0,
        band: tuple[float, float] = (-math.pi / 2, math.pi / 2),
    ) -> np.ndarray:
        """Unit rays within the azimuth WINDOW (None: all around) and the elevation
        BAND, no denser than MIN_SPACING_RAD apart (0: the pattern's own density)."""
        el_low, el_high = max(band[0], self.el_min), min(band[1], self.el_max)
        if el_high < el_low:
            return np.zeros((0, 3))
        span = 2.0 * math.pi if window is None else window[1]
        if self.kind == "rings":
            el_step = (self.el_max - self.el_min) / max(1, self.rings - 1)
            az_stride = max(1, int(min_spacing_rad / self.az_step))
            el_stride = max(1, int(min_spacing_rad / el_step)) if self.rings > 1 else 1
            az = np.arange(-math.pi, math.pi, self.az_step * az_stride)
            az = az[in_window(az, window)]
            el = np.linspace(self.el_min, self.el_max, self.rings)[::el_stride]
            el = el[(el >= el_low) & (el <= el_high)]
            az, el = (g.ravel() for g in np.meshgrid(az, el, indexing="ij"))
        else:
            count = self.count or scan_points
            low, high = math.sin(el_low), math.sin(el_high)
            # The pattern's density over the band's share of its solid angle.
            share = (high - low) / (math.sin(self.el_max) - math.sin(self.el_min))
            n = count * share * span / (2.0 * math.pi)
            if min_spacing_rad > 0.0:
                n = min(n, span * (el_high - el_low) / min_spacing_rad**2)
            n = int(n)
            start = -math.pi if window is None else window[0]
            az = start + span * rng.uniform(size=n)
            el = np.arcsin(low + (high - low) * rng.uniform(size=n))
        ce = np.cos(el)
        return np.column_stack([ce * np.cos(az), ce * np.sin(az), np.sin(el)])


def voxel_keys(points: np.ndarray, leaf: float) -> np.ndarray:
    """One int64 key per LEAF voxel (21 bits per axis)."""
    k = np.floor(points / leaf).astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def voxel_count(points: np.ndarray, leaf: float) -> int:
    if leaf <= 0.0 or len(points) == 0:
        return len(points)
    return len(np.unique(voxel_keys(points, leaf)))


def voxel_thin(points: np.ndarray, leaf: float) -> np.ndarray:
    """One point (the first) per LEAF voxel."""
    if leaf <= 0.0 or len(points) == 0:
        return points
    _, first = np.unique(voxel_keys(points, leaf), return_index=True)
    return points[np.sort(first)]


@dataclass
class Rendered:
    keep: np.ndarray  # mask over the input points: not occluded
    obstacle_points: np.ndarray  # (K, 3) in the sensor frame
    shown: int  # obstacles used, from the front of the list
    fraction: float  # obstacle share of the output within range, on the count leaf
    point_fraction: float  # the same share of the points themselves


class _AzimuthIndex:
    """Directions sorted by azimuth, to pick those in a window without a full pass."""

    def __init__(self, azimuth: np.ndarray):
        self.order = np.argsort(azimuth, kind="stable")
        self.sorted = azimuth[self.order]

    def select(self, window) -> np.ndarray:
        if window is None:
            return self.order
        start = (window[0] + math.pi) % (2.0 * math.pi) - math.pi
        end = start + window[1]
        lo = np.searchsorted(self.sorted, start)
        if end <= math.pi:
            return self.order[lo : np.searchsorted(self.sorted, end, side="right")]
        hi = np.searchsorted(self.sorted, end - 2.0 * math.pi, side="right")
        return np.concatenate([self.order[lo:], self.order[:hi]])


def render(
    xyz: np.ndarray,
    boxes: list,
    pattern: RayPattern,
    rng: np.random.Generator,
    fraction: float | None,
    obstacle_leaf: float,
    min_range: float,
    max_range: float,
    count_leaf: float = 0.0,
) -> Rendered:
    """Occlude XYZ by BOXES (front of the list first) and sample their surfaces, with
    as many boxes as make FRACTION of the output within range (all with None, none
    with 0). The share is counted in COUNT_LEAF voxels (0: points), as the localizer's
    voxel filter would see it."""
    ranges = np.linalg.norm(xyz, axis=1)
    in_range = (ranges >= min_range) & (ranges <= max_range)
    nothing = Rendered(np.ones(len(xyz), dtype=bool), np.zeros((0, 3)), 0, 0.0, 0.0)
    if not boxes or (fraction is not None and fraction <= 0.0):
        return nothing
    valid = np.isfinite(ranges) & (ranges > 0.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        dirs = xyz / ranges[:, None]
    azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
    elevation = np.arcsin(np.clip(np.nan_to_num(dirs[:, 2]), -1.0, 1.0))
    windows = [azimuth_window(box) for box in boxes]
    reaches = [box_reach(box) for box in boxes]
    # Each box's own rays, half the leaf apart at its centre or half its far side,
    # whichever is farther: its surface is thinned to that leaf anyway.
    ray_sets, owners = [], []
    for j, (box, window, reach) in enumerate(zip(boxes, windows, reaches)):
        typical = max(float(np.linalg.norm(box[0])), 0.5 * reach[1])
        spacing = 0.5 * obstacle_leaf / typical if obstacle_leaf > 0.0 else 0.0
        rays_j = pattern.rays(window, rng, len(xyz), spacing, (reach[2], reach[3]))
        ray_sets.append(rays_j)
        owners.append(np.full(len(rays_j), j))
    rays = np.vstack(ray_sets)
    owner = np.concatenate(owners)
    ray_az = np.arctan2(rays[:, 1], rays[:, 0])
    ray_el = np.arcsin(np.clip(rays[:, 2], -1.0, 1.0))
    # Range image of the real returns: the nearest per cell hides synthetic returns
    # behind it. Occluded returns lie behind the obstacle, so counting them changes
    # nothing.
    cell = math.radians(RANGE_IMAGE_CELL_DEG)
    n_az = math.ceil(2.0 * math.pi / cell)
    n_el = math.ceil(math.pi / cell) + 1

    def cells(az, el):
        a = np.clip(((az + math.pi) / cell).astype(np.int64), 0, n_az - 1)
        e = np.clip(((el + math.pi / 2) / cell).astype(np.int64), 0, n_el - 1)
        return a * n_el + e

    nearest = np.full(n_az * n_el, np.inf)
    real_cells = cells(azimuth[valid], elevation[valid])
    order = np.lexsort((ranges[valid], real_cells))
    sorted_cells = real_cells[order]
    first = np.concatenate([[True], sorted_cells[1:] != sorted_cells[:-1]])
    nearest[sorted_cells[first]] = ranges[valid][order][first]
    ray_limit = nearest[cells(ray_az, ray_el)] + HIDE_MARGIN_M
    # The real points within range by count voxel, for the share as boxes are added.
    counted = np.flatnonzero(in_range & valid)
    if count_leaf > 0.0 and len(counted):
        _, voxel_of = np.unique(
            voxel_keys(xyz[counted], count_leaf), return_inverse=True
        )
        voxel_of = voxel_of.ravel()
    else:
        voxel_of = np.arange(len(counted))
    n_voxels = int(voxel_of.max()) + 1 if len(voxel_of) else 0
    real_index = _AzimuthIndex(np.where(valid, azimuth, 10.0))
    ray_index = _AzimuthIndex(ray_az)

    def cast(box, window, reach):
        near, _, bottom, top = reach
        # Only returns beyond the box's nearest point, within its elevation band and
        # azimuth span, can be hidden by it.
        idx = real_index.select(window)
        idx = idx[valid[idx] & (ranges[idx] > near)]
        idx = idx[(elevation[idx] >= bottom) & (elevation[idx] <= top)]
        t = ray_box_distance(dirs[idx], box) if len(idx) else np.zeros(0)
        idx, t = idx[np.isfinite(t)], t[np.isfinite(t)]
        # Every ray, of this box or another, that this box may catch.
        ridx = ray_index.select(window)
        ridx = ridx[(ray_el[ridx] >= bottom) & (ray_el[ridx] <= top)]
        rt = ray_box_distance(rays[ridx], box) if len(ridx) else np.zeros(0)
        return (idx, t), (ridx[np.isfinite(rt)], rt[np.isfinite(rt)])

    real_hit = np.full(len(xyz), np.inf)
    ray_hit = np.full(len(rays), np.inf)
    added = 0

    def evaluate(k):
        nonlocal added
        for j in range(added, k):
            (idx, t), (ridx, rt) = cast(boxes[j], windows[j], reaches[j])
            real_hit[idx] = np.minimum(real_hit[idx], t)
            ray_hit[ridx] = np.minimum(ray_hit[ridx], rt)
        added = max(added, k)
        hidden = real_hit < ranges
        shown = (owner < k) & np.isfinite(ray_hit) & (ray_hit <= ray_limit)
        points = voxel_thin(rays[shown] * ray_hit[shown, None], obstacle_leaf)
        r_obs = np.linalg.norm(points, axis=1)
        mine = points[(r_obs >= min_range) & (r_obs <= max_range)]
        n_obs = voxel_count(mine, count_leaf)
        alive = ~hidden[counted]
        n_real = int(np.count_nonzero(np.bincount(voxel_of[alive], minlength=n_voxels)))
        n_points = int(alive.sum())
        return Rendered(
            ~hidden,
            points,
            k,
            n_obs / max(1, n_obs + n_real),
            len(mine) / max(1, len(mine) + n_points),
        )

    if fraction is None:
        return evaluate(len(boxes))
    # The first of 1, 2, ..., 8, then steps of about 1.2 times, that reaches the share.
    k = 1
    while True:
        result = evaluate(k)
        if result.fraction >= fraction or k == len(boxes):
            return result
        k = min(len(boxes), k + 1 if k < 8 else math.ceil(k * 1.2))


# ----------------------------------------------------------------------------------
# PointCloud2


FLOAT_TYPES = {7: np.float32, 8: np.float64}


def cloud_arrays(msg):
    """(raw rows (N, point_step) uint8, xyz (N, 3) float64, field offsets/types)."""
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = raw[: n * msg.point_step].reshape(n, msg.point_step)
    if msg.row_step != msg.width * msg.point_step and msg.height > 1:
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, -1)
        raw = raw[:, : msg.width * msg.point_step].reshape(n, msg.point_step)
    xyz = np.empty((n, 3))
    for i, name in enumerate("xyz"):
        offset, datatype = fields[name]
        dtype = FLOAT_TYPES[datatype]
        width = np.dtype(dtype).itemsize
        xyz[:, i] = raw[:, offset : offset + width].copy().view(dtype).ravel()
    return raw, xyz, fields


def compose_rows(raw, xyz, keep, obstacle_points, fields):
    """Kept rows, then one row per obstacle point copied from the real point nearest
    in azimuth (so time, ring and intensity stay plausible) with its xyz replaced."""
    kept = raw[keep]
    if not len(obstacle_points) or not len(kept):
        return np.ascontiguousarray(kept)
    az = np.arctan2(xyz[keep, 1], xyz[keep, 0])
    order = np.argsort(az)
    target = np.arctan2(obstacle_points[:, 1], obstacle_points[:, 0])
    pos = np.clip(np.searchsorted(az[order], target), 0, len(order) - 1)
    rows = kept[order[pos]].copy()
    for i, name in enumerate("xyz"):
        offset, datatype = fields[name]
        dtype = FLOAT_TYPES[datatype]
        width = np.dtype(dtype).itemsize
        rows[:, offset : offset + width] = (
            obstacle_points[:, i].astype(dtype).view(np.uint8).reshape(-1, width)
        )
    return np.ascontiguousarray(np.vstack([kept, rows]))


# ----------------------------------------------------------------------------------
# Map files, for --map and --sensor-height auto


def read_map_points(path) -> np.ndarray:
    """x, y, z of a binary float32 PLY or a binary PCD."""
    data = Path(path).read_bytes()
    if str(path).endswith(".ply"):
        end = data.index(b"end_header\n") + len(b"end_header\n")
        header = data[:end].decode(errors="replace").splitlines()
        count = int(
            next(h for h in header if h.startswith("element vertex")).split()[2]
        )
        props = [h.split()[2] for h in header if h.startswith("property float")]
        if len(props) != sum(h.startswith("property") for h in header):
            raise ValueError(f"{path}: only float PLY properties are supported")
        values = np.frombuffer(data, np.float32, count * len(props), end)
        values = values.reshape(count, len(props))
        return values[:, [props.index(c) for c in "xyz"]].astype(float)
    lines, pos = [], 0
    while True:
        end = data.index(b"\n", pos)
        lines.append(data[pos:end].decode(errors="replace"))
        pos = end + 1
        if lines[-1].startswith("DATA"):
            break
    header = {line.split()[0]: line.split()[1:] for line in lines if line.split()}
    if header["DATA"][0] != "binary" or set(header["SIZE"]) != {"4"}:
        raise ValueError(f"{path}: only binary float32 PCD is supported")
    fields = header["FIELDS"]
    count = int(header["POINTS"][0])
    values = np.frombuffer(data, np.float32, count * len(fields), pos)
    values = values.reshape(count, len(fields))
    return values[:, [fields.index(c) for c in "xyz"]].astype(float)


def estimate_sensor_height(xyz: np.ndarray, rotation: np.ndarray) -> float:
    """Height of the sensor over the ground from one scan: the median map-frame height
    of the lowest third of the returns 2-10 m away horizontally."""
    world = xyz @ rotation.T
    horizontal = np.hypot(world[:, 0], world[:, 1])
    near = np.sort(world[(horizontal > 2.0) & (horizontal < 10.0), 2])
    if len(near) < 50:
        return 0.0
    return float(-np.median(near[: len(near) // 3]))


# ----------------------------------------------------------------------------------
# The node


class Injector:
    """Everything but the ROS plumbing: one scan in, one scan out."""

    def __init__(self, args, traj: Trajectory, occupancy: MapOccupancy | None):
        self.args = args
        self.traj = traj
        self.occupancy = occupancy
        self.pattern = RayPattern.parse(args.pattern)
        if args.ground_z is not None:
            self.sensor_height = ("z", float(args.ground_z))
        elif args.sensor_height == "auto":
            self.sensor_height = None
        else:
            self.sensor_height = float(args.sensor_height)
        self.active = None
        self.first_stamp = None
        if args.active:
            a, _, b = args.active.partition(":")
            self.active = (float(a or "-inf"), float(b or "inf"))

    def process(self, stamp: float, xyz: np.ndarray):
        """(Rendered or None, the obstacles considered) for one scan."""
        if not self.traj.covers(stamp):
            return None, []
        rotation, position = self.traj.pose(stamp)
        if self.sensor_height is None:
            self.sensor_height = estimate_sensor_height(xyz, rotation)
            print(
                f"sensor height {self.sensor_height:.2f} m", file=sys.stderr, flush=True
            )
        if self.first_stamp is None:
            self.first_stamp = stamp
        elapsed = stamp - self.first_stamp
        if self.active and not self.active[0] <= elapsed <= self.active[1]:
            return None, []
        obstacles = [
            o
            for o in scenario_obstacles(
                self.args.scenario, stamp, self.traj, self.args.seed, self.sensor_height
            )
            if math.hypot(o.x - position[0], o.y - position[1]) < OBSTACLE_RANGE_M
            and placement_ok(o, position[:2], self.occupancy)
        ]
        boxes = [box_in_sensor(o, rotation, position) for o in obstacles]
        rng = _rng(self.args.seed, 99, round(stamp * 1000))
        rendered = render(
            xyz,
            boxes,
            self.pattern,
            rng,
            self.args.fraction,
            self.args.obstacle_leaf,
            self.args.min_range,
            self.args.max_range,
            self.args.count_leaf,
        )
        return rendered, obstacles


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", required=True, help="cloud topic to read")
    parser.add_argument("--output", required=True, help="cloud topic to publish")
    parser.add_argument(
        "--gt", required=True, type=Path, help="TUM trajectory, map frame"
    )
    parser.add_argument("--scenario", required=True, choices=SCENARIOS)
    parser.add_argument("--fraction", type=float, help="obstacle share of the points")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--pattern", default="rings:-25:15:128:0.2")
    parser.add_argument("--obstacle-leaf", type=float, default=0.0)
    parser.add_argument("--count-leaf", type=float, default=0.0)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=100.0)
    parser.add_argument(
        "--sensor-height", default="auto", help="over the ground, m (auto: from a scan)"
    )
    parser.add_argument("--ground-z", type=float, help="a level ground at this map z")
    parser.add_argument("--map", type=Path, help="skip obstacles that cut this map")
    parser.add_argument("--active", help="START:END seconds from the first scan")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--reliable", action="store_true", help="reliable subscription")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    traj = Trajectory.load(args.gt)
    occupancy = MapOccupancy(read_map_points(args.map)) if args.map else None
    injector = Injector(args, traj, occupancy)
    log = args.log.open("w", newline="", encoding="utf-8") if args.log else None
    writer = None
    if log:
        writer = csv.writer(log)
        writer.writerow(
            [
                "stamp",
                "points_in",
                "points_out",
                "obstacle_points",
                "point_fraction",
                "fraction",
                "obstacles_shown",
                "obstacles_considered",
                "ms",
            ]
        )

    rclpy.init()
    node = Node("dynamic_obstacles")
    publisher = node.create_publisher(
        PointCloud2,
        args.output,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )

    def on_cloud(msg):
        began = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw, xyz, fields = cloud_arrays(msg)
        finite = np.isfinite(xyz).all(axis=1)
        rendered, obstacles = injector.process(
            stamp, np.where(finite[:, None], xyz, 0.0)
        )
        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.is_dense = False
        if rendered is None:
            rows = raw
            stats = (0, 0.0, 0.0, 0)
        else:
            rows = compose_rows(
                raw, xyz, rendered.keep & finite, rendered.obstacle_points, fields
            )
            stats = (
                len(rendered.obstacle_points),
                rendered.point_fraction,
                rendered.fraction,
                rendered.shown,
            )
        out.height = 1
        out.width = len(rows)
        out.row_step = out.width * out.point_step
        out.data = rows.tobytes()
        publisher.publish(out)
        if writer:
            writer.writerow(
                [
                    f"{stamp:.6f}",
                    len(xyz),
                    len(rows),
                    stats[0],
                    f"{stats[1]:.4f}",
                    f"{stats[2]:.4f}",
                    stats[3],
                    len(obstacles),
                    f"{1000 * (time.perf_counter() - began):.1f}",
                ]
            )

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE
        if args.reliable
        else ReliabilityPolicy.BEST_EFFORT,
    )
    node.create_subscription(PointCloud2, args.input, on_cloud, qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
