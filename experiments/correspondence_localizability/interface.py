"""Shared geometry and output contracts for localizability variants."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class CorrespondenceGeometry:
    points: np.ndarray
    plane_normals: np.ndarray
    plane_mask: np.ndarray
    line_directions: np.ndarray
    line_mask: np.ndarray


@dataclass(frozen=True)
class LocalizabilityResult:
    name: str
    correspondence_count: int
    eigenvalues: np.ndarray
    eigenvectors: np.ndarray
    weak_ratio: float
    numerical_nullity: int
    direction_effective_support: np.ndarray


class LocalizabilityVariant(Protocol):
    name: str

    def analyze(self, geometry: CorrespondenceGeometry) -> LocalizabilityResult:
        ...
