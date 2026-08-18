#!/usr/bin/env python3
"""WP2 corridor-alias structure gate experiment test (data-independent).

Builds the synthetic north/south corridor world, runs the baseline (accepts
everything) and the height-histogram gate, and asserts the gate rejects the
footprint-identical north alias while keeping the true south candidate.
"""

import numpy as np

from experiments.corridor_structure_gate.fixtures import alias_pose_north
from experiments.corridor_structure_gate.fixtures import alias_pose_south
from experiments.corridor_structure_gate.fixtures import make_corridor_alias_map
from experiments.corridor_structure_gate.fixtures import make_scan
from experiments.corridor_structure_gate.interface import StructureGateCandidate
from experiments.corridor_structure_gate.interface import StructureGateInput
from experiments.corridor_structure_gate.variants import HeightHistogramGate
from experiments.corridor_structure_gate.variants import ZScoreOnly


def make_input():
    world = make_corridor_alias_map()
    south_x, south_y, south_z = alias_pose_south()
    scan = make_scan(world, south_x, south_y, south_z)
    candidates = [
        StructureGateCandidate(south_x, south_y, south_z, 0.0),
        StructureGateCandidate(*alias_pose_north(), 0.0),
    ]
    return StructureGateInput(scan_xyz=scan, candidates=candidates, map_points=world)


def test_fixture_scan_is_2d_indistinguishable_but_3d_different():
    world = make_corridor_alias_map()
    # The deck adds points only near the south corridor.
    near_south = np.hypot(world[:, 0], world[:, 1] + 3.0) < 2.5
    near_north = np.hypot(world[:, 0], world[:, 1] - 150.0) < 2.5
    assert np.any(world[near_south][:, 2] > 1.0)
    assert not np.any(world[near_north][:, 2] > 1.0)


def test_baseline_accepts_everything_including_alias():
    result = ZScoreOnly().apply(make_input())
    assert all(result.accepted)
    assert all(sim == 1.0 for sim in result.similarities)


def test_height_histogram_gate_rejects_north_alias_and_keeps_south():
    result = HeightHistogramGate(
        z_min=-1.0, z_max=6.0, bins=14, threshold=0.5).apply(make_input())
    assert result.accepted[0], "true south candidate must stay"
    assert not result.accepted[1], "north alias must be rejected"
    assert result.similarities[0] > result.similarities[1]


def test_gate_abstains_when_scan_has_no_elevated_structure():
    # A scan taken in the plain north corridor has no overhead structure, so the
    # gate has nothing to verify and must not false-reject either corridor.
    world = make_corridor_alias_map()
    north_x, north_y, north_z = alias_pose_north()
    scan = make_scan(world, north_x, north_y, north_z)
    gate_input = StructureGateInput(
        scan_xyz=scan,
        candidates=[
            StructureGateCandidate(*alias_pose_south(), 0.0),
            StructureGateCandidate(north_x, north_y, north_z, 0.0),
        ],
        map_points=world,
    )
    result = HeightHistogramGate(
        z_min=-1.0, z_max=6.0, bins=14, threshold=0.5).apply(gate_input)
    assert all(result.accepted)


def test_gate_keeps_candidates_whose_elevated_structure_matches():
    world = make_corridor_alias_map()
    # A scan of the decked south corridor must also stay accepted at the true
    # south pose while the plain north pose is rejected.
    south_x, south_y, south_z = alias_pose_south()
    scan = make_scan(world, south_x, south_y, south_z)
    gate_input = StructureGateInput(
        scan_xyz=scan,
        candidates=[StructureGateCandidate(south_x, south_y, south_z, 0.0)],
        map_points=world,
    )
    result = HeightHistogramGate(
        z_min=-1.0, z_max=6.0, bins=14, threshold=0.5).apply(gate_input)
    assert result.accepted == [True]


if __name__ == "__main__":
    test_fixture_scan_is_2d_indistinguishable_but_3d_different()
    test_baseline_accepts_everything_including_alias()
    test_height_histogram_gate_rejects_north_alias_and_keeps_south()
    test_gate_abstains_when_scan_has_no_elevated_structure()
    test_gate_keeps_candidates_whose_elevated_structure_matches()
    print("test_corridor_structure_gate_experiment: all tests passed")
