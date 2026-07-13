#!/usr/bin/env python3

import numpy as np

from experiments.correspondence_localizability.interface import CorrespondenceGeometry
from experiments.correspondence_localizability.variants import HessianSpectrum
from experiments.correspondence_localizability.variants import LpicpPlaneLineContribution
from experiments.correspondence_localizability.variants import XicpDirectionContribution


def plane_geometry():
    x, y = np.meshgrid(np.linspace(-5.0, 5.0, 11), np.linspace(-5.0, 5.0, 11))
    points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    return CorrespondenceGeometry(
        points=points,
        plane_normals=np.tile([0.0, 0.0, 1.0], (len(points), 1)),
        plane_mask=np.ones(len(points), dtype=bool),
        line_directions=np.zeros_like(points),
        line_mask=np.zeros(len(points), dtype=bool),
    )


def test_single_plane_has_three_unobservable_directions():
    result = HessianSpectrum().analyze(plane_geometry())
    assert result.numerical_nullity == 3
    assert result.weak_ratio == 0.0


def test_xicp_variant_exposes_per_direction_support_without_changing_spectrum():
    baseline = HessianSpectrum().analyze(plane_geometry())
    xicp = XicpDirectionContribution().analyze(plane_geometry())
    np.testing.assert_allclose(xicp.eigenvalues, baseline.eigenvalues)
    assert np.count_nonzero(xicp.direction_effective_support) == 3


def test_line_features_supply_information_when_planes_are_absent():
    t = np.linspace(-5.0, 5.0, 51)
    points = np.vstack(
        (
            np.column_stack((t, np.ones_like(t), np.zeros_like(t))),
            np.column_stack((np.zeros_like(t), t, np.ones_like(t))),
            np.column_stack((np.ones_like(t), np.zeros_like(t), t)),
        )
    )
    directions = np.vstack(
        (
            np.tile([1.0, 0.0, 0.0], (len(t), 1)),
            np.tile([0.0, 1.0, 0.0], (len(t), 1)),
            np.tile([0.0, 0.0, 1.0], (len(t), 1)),
        )
    )
    geometry = CorrespondenceGeometry(
        points=points,
        plane_normals=np.zeros_like(points),
        plane_mask=np.zeros(len(points), dtype=bool),
        line_directions=directions,
        line_mask=np.ones(len(points), dtype=bool),
    )
    assert HessianSpectrum().analyze(geometry).correspondence_count == 0
    assert LpicpPlaneLineContribution().analyze(geometry).numerical_nullity == 0


if __name__ == "__main__":
    test_single_plane_has_three_unobservable_directions()
    test_xicp_variant_exposes_per_direction_support_without_changing_spectrum()
    test_line_features_supply_information_when_planes_are_absent()
