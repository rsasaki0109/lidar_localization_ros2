"""Hessian, X-ICP-like contribution, and LP-ICP-like mixed-feature variants.

These are diagnostic approximations for controlled comparison, not claims of a
verbatim reimplementation of either paper.
"""

from __future__ import annotations

import numpy as np

from .interface import CorrespondenceGeometry
from .interface import LocalizabilityResult


def _skew_batch(points: np.ndarray) -> np.ndarray:
    matrices = np.zeros((len(points), 3, 3), dtype=np.float64)
    x, y, z = points.T
    matrices[:, 0, 1] = -z
    matrices[:, 0, 2] = y
    matrices[:, 1, 0] = z
    matrices[:, 1, 2] = -x
    matrices[:, 2, 0] = -y
    matrices[:, 2, 1] = x
    return matrices


def _motion_jacobians(points: np.ndarray) -> np.ndarray:
    identity = np.broadcast_to(np.eye(3), (len(points), 3, 3))
    return np.concatenate((identity, -_skew_batch(points)), axis=2)


def _result(
    name: str, hessians: np.ndarray, include_direction_support: bool
) -> LocalizabilityResult:
    if not len(hessians):
        return LocalizabilityResult(
            name, 0, np.zeros(6), np.eye(6), 0.0, 6, np.zeros(6)
        )
    information = hessians.sum(axis=0) / len(hessians)
    eigenvalues, eigenvectors = np.linalg.eigh(0.5 * (information + information.T))
    strongest = max(float(eigenvalues[-1]), np.finfo(float).eps)
    support = np.zeros(6)
    if include_direction_support:
        contributions = np.einsum(
            "ki,nij,kj->nk", eigenvectors.T, hessians, eigenvectors.T
        )
        sums = contributions.sum(axis=0)
        squared_sums = np.square(contributions).sum(axis=0)
        support = np.divide(
            np.square(sums),
            len(hessians) * squared_sums,
            out=np.zeros(6),
            where=squared_sums > 0.0,
        )
    return LocalizabilityResult(
        name=name,
        correspondence_count=len(hessians),
        eigenvalues=eigenvalues,
        eigenvectors=eigenvectors,
        weak_ratio=max(0.0, float(eigenvalues[0]) / strongest),
        numerical_nullity=int(np.count_nonzero(eigenvalues <= strongest * 1.0e-4)),
        direction_effective_support=support,
    )


def _plane_hessians(geometry: CorrespondenceGeometry) -> np.ndarray:
    points = geometry.points[geometry.plane_mask]
    normals = geometry.plane_normals[geometry.plane_mask]
    if not len(points):
        return np.empty((0, 6, 6))
    jacobians = np.einsum("ni,nij->nj", normals, _motion_jacobians(points))
    return np.einsum("ni,nj->nij", jacobians, jacobians)


def _line_hessians(geometry: CorrespondenceGeometry) -> np.ndarray:
    points = geometry.points[geometry.line_mask]
    directions = geometry.line_directions[geometry.line_mask]
    if not len(points):
        return np.empty((0, 6, 6))
    motion = _motion_jacobians(points)
    projectors = np.eye(3)[None, :, :] - np.einsum(
        "ni,nj->nij", directions, directions
    )
    return np.einsum("nai,nab,nbj->nij", motion, projectors, motion)


class HessianSpectrum:
    name = "point_to_plane_hessian"

    def analyze(self, geometry: CorrespondenceGeometry) -> LocalizabilityResult:
        return _result(self.name, _plane_hessians(geometry), False)


class XicpDirectionContribution:
    name = "xicp_direction_contribution"

    def analyze(self, geometry: CorrespondenceGeometry) -> LocalizabilityResult:
        # The spectrum matches point-to-plane Hessian analysis; the additional
        # effective-support vector exposes redundant concentration per eigendirection.
        return _result(self.name, _plane_hessians(geometry), True)


class LpicpPlaneLineContribution:
    name = "lpicp_plane_line_contribution"

    def analyze(self, geometry: CorrespondenceGeometry) -> LocalizabilityResult:
        plane = _plane_hessians(geometry)
        line = _line_hessians(geometry)
        if not len(plane):
            mixed = line
        elif not len(line):
            mixed = plane
        else:
            # Normalize feature families so a large class cannot dominate only
            # because it supplied more correspondences.
            plane = plane * (0.5 / len(plane))
            line = line * (0.5 / len(line))
            mixed = np.concatenate((plane, line), axis=0)
            mixed = mixed * len(mixed)
        return _result(self.name, mixed, True)
