from __future__ import annotations

import numpy as np

from experiments.corridor_structure_gate.interface import StructureGateInput
from experiments.corridor_structure_gate.interface import StructureGateResult


def height_histogram(points: np.ndarray, z_min: float, z_max: float, bins: int) -> np.ndarray:
    """Normalized vertical structure histogram of a point cloud.

    Bins range from z_min..z_max; a column with no points yields an all-zero
    histogram so the intersection similarity treats it as "nothing in common".
    """
    if len(points) == 0:
        return np.zeros(max(1, bins))
    hist, _ = np.histogram(points[:, 2], bins=bins, range=(z_min, z_max))
    total = float(hist.sum())
    if total <= 0.0:
        return np.zeros(bins)
    return hist / total


def elevated_histogram(
    points: np.ndarray,
    z_min: float,
    z_max: float,
    bins: int,
    ground_floor_m: float,
) -> np.ndarray:
    """Normalized histogram over the elevated bins only (z > ground_floor_m).

    Ground/wall structure at vehicle height is shared by every repetitive-road
    location and would dominate an intersection. Restricting to the elevated
    bins checks the structure that actually distinguishes aliases.
    """
    hist, edges = np.histogram(points[:, 2], bins=bins, range=(z_min, z_max))
    centers = 0.5 * (edges[:-1] + edges[1:])
    elevated = hist[centers > ground_floor_m]
    total = float(elevated.sum())
    if total <= 0.0:
        return np.zeros(elevated.size)
    return elevated / total


def histogram_intersection(left: np.ndarray, right: np.ndarray) -> float:
    return float(np.minimum(left, right).sum())


def crop_map(points: np.ndarray, x: float, y: float, radius_m: float) -> np.ndarray:
    if points is None or len(points) == 0:
        return np.empty((0, 3))
    distances = np.hypot(points[:, 0] - x, points[:, 1] - y)
    return points[distances <= radius_m]


class HeightHistogramGate:
    """Lightweight 3D structure gate for BBS top-K re-ranking.

    For each candidate the map is cropped to a local horizontal radius and its
    normalized *elevated* structure histogram is intersected with the scan's.
    Bins at or below `ground_floor_m` (shared by every corridor) are dropped
    from both sides, so the score reflects only the overhead/upper structure
    that distinguishes footprint-identical aliases.

    If the scan carries no elevated structure at all (below
    `min_elevated_fraction` of its points), the gate abstains and accepts:
    there is no discriminating structure to verify. Candidates whose similarity
    falls below `threshold` are rejected.

    Ground truth is never used at runtime; the map crop is the only prior.
    """

    name = "height_histogram"
    design = "elevated map-crop height-histogram intersection re-rank"

    def __init__(
        self,
        z_min: float = -3.0,
        z_max: float = 15.0,
        bins: int = 24,
        ground_floor_m: float = 0.5,
        min_elevated_fraction: float = 0.05,
        threshold: float = 0.5,
        local_radius_m: float = 60.0,
    ):
        self.z_min = z_min
        self.z_max = z_max
        self.bins = bins
        self.ground_floor_m = ground_floor_m
        self.min_elevated_fraction = min_elevated_fraction
        self.threshold = threshold
        self.local_radius_m = local_radius_m

    def apply(self, gate_input: StructureGateInput) -> StructureGateResult:
        scan_hist = height_histogram(
            gate_input.scan_xyz, self.z_min, self.z_max, self.bins)
        _, edges = np.histogram(
            np.array([0.0]), bins=self.bins, range=(self.z_min, self.z_max))
        centers = 0.5 * (edges[:-1] + edges[1:])
        elevated_fraction = float(scan_hist[centers > self.ground_floor_m].sum())
        scan_has_structure = elevated_fraction >= self.min_elevated_fraction
        scan_elevated = elevated_histogram(
            gate_input.scan_xyz, self.z_min, self.z_max, self.bins,
            self.ground_floor_m)
        similarities = []
        accepted = []
        for candidate in gate_input.candidates:
            if gate_input.map_points is None or len(gate_input.map_points) == 0:
                similarities.append(1.0)
                accepted.append(True)
                continue
            crop = crop_map(
                gate_input.map_points,
                candidate.x_m,
                candidate.y_m,
                self.local_radius_m,
            )
            map_elevated = elevated_histogram(
                crop, self.z_min, self.z_max, self.bins, self.ground_floor_m)
            if not scan_has_structure:
                # Nothing elevated to verify against; abstain on structure.
                similarities.append(1.0)
                accepted.append(True)
                continue
            similarity = histogram_intersection(scan_elevated, map_elevated)
            similarities.append(similarity)
            accepted.append(similarity >= self.threshold)
        return StructureGateResult(
            similarities=similarities,
            accepted=accepted,
            reason="reject candidates whose local map lacks the elevated "
            "structure the scan sees",
        )
