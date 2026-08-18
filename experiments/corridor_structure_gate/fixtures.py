"""Synthetic north/south corridor alias fixture (WP2).

Two parallel corridors share an identical XY wall footprint, so a 2D occupancy
score cannot tell them apart. Only the 3D structure differs: the south corridor
has an overhead deck, the north corridor does not. This is the minimal shape of
the documented Koide outdoor_hard corridor alias: repetitive-road geometry with
different vertical structure.
"""

from __future__ import annotations

import numpy as np


def make_corridor_alias_map(wall_gap_m: float = 1.0, spacing_m: float = 1.0) -> np.ndarray:
    """World points for two parallel corridors and one overhead deck.

    Returns an (N, 3) float32 array. Both corridors run along x in [-10, 10].
    The south corridor (y ~ -3) carries an overhead deck; the north corridor
    (y ~ +150, matching the documented 140--160 m alias distance) does not.
    Wall and pillar positions are identical for the two corridors in the XY
    plane, so the 2D occupancy footprints match.
    """
    xs = np.linspace(-10.0, 10.0, 41)

    def corridor_walls(center_y: float):
        south_wall = np.column_stack(
            [xs, np.full_like(xs, center_y - wall_gap_m), np.zeros_like(xs)])
        north_wall = np.column_stack(
            [xs, np.full_like(xs, center_y + wall_gap_m), np.zeros_like(xs)])
        # Periodic pillars between the walls: same XY for both corridors.
        pillars = np.column_stack(
            [np.tile(xs[::4], 2), np.full(xs[::4].size * 2, center_y),
             np.zeros(xs[::4].size * 2)])
        return np.vstack([south_wall, north_wall, pillars])

    south = corridor_walls(-3.0)
    north = corridor_walls(150.0)

    deck_x = np.linspace(-8.0, 8.0, 17)
    deck_y = np.linspace(-4.0, -2.0, 5)
    deck_x, deck_y = np.meshgrid(deck_x, deck_y)
    deck = np.column_stack([deck_x.ravel(), deck_y.ravel(), np.full(deck_x.size, 2.0)])

    return np.vstack([south, north, deck]).astype(np.float32)


def make_scan(
    map_points: np.ndarray,
    pose_x: float,
    pose_y: float,
    pose_z: float,
    max_range_m: float = 12.0,
    step: int = 1,
) -> np.ndarray:
    """Deterministic scan: visible map points around a pose (noise-free)."""
    relative = map_points - np.array([pose_x, pose_y, pose_z])
    distances = np.hypot(relative[:, 0], relative[:, 1])
    visible = distances <= max_range_m
    return relative[visible][::step]


def alias_pose_south():
    return 0.0, -3.0, 0.0


def alias_pose_north():
    return 0.0, 150.0, 0.0
