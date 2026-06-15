#!/usr/bin/env python3
"""Parity check: the optional C++ BBS backend (bbs_cpp) vs the Python reference.

The C++ search core is already proven bit-exact to the Python reference offline
by test/test_bbs_branch_and_bound.cpp (golden fixtures). This test exercises the
*pybind11 binding* -- the numpy<->struct plumbing in src/bbs_pybind.cpp -- by
running both backends through GlobalLocalizationEngine's exact call shape and
asserting identical candidates.

It is intentionally tolerant about availability: the compiled module only exists
when pybind11 was found at build time, so if `import bbs_cpp` fails the test
prints SKIP and exits 0. CMake passes BBS_CPP_MODULE_DIR pointing at the build
output directory; we add it to sys.path so the freshly built module is found
without installing.
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

# Let CMake point us at the build-tree module before it is installed.
_module_dir = os.environ.get("BBS_CPP_MODULE_DIR")
if _module_dir:
    sys.path.insert(0, _module_dir)

try:
    import bbs_cpp
except ImportError as exc:  # pragma: no cover - depends on build config
    print("SKIP: bbs_cpp not importable (%s); pybind11 backend not built" % exc)
    sys.exit(0)

import make_bbs_relocalization_attempts as bbs  # noqa: E402


def make_map(height, width, occupied_cells):
    occ = np.zeros((height, width), dtype=bool)
    for (y, x) in occupied_cells:
        occ[y, x] = True
    return occ


def cluster_map(height, width, seed, n_clusters, cluster_size):
    rng = np.random.default_rng(seed)
    occ = np.zeros((height, width), dtype=bool)
    for _ in range(n_clusters):
        cy = int(rng.integers(2, height - 2))
        cx = int(rng.integers(2, width - 2))
        for _ in range(cluster_size):
            y = min(height - 1, max(0, cy + int(rng.integers(-2, 3))))
            x = min(width - 1, max(0, cx + int(rng.integers(-2, 3))))
            occ[y, x] = True
    return occ


def scan_from_occupied(occ, resolution_m, true_xy, radius_m, max_points, seed):
    ys, xs = np.nonzero(occ)
    wx = (xs + 0.5) * resolution_m
    wy = (ys + 0.5) * resolution_m
    d = np.hypot(wx - true_xy[0], wy - true_xy[1])
    sel = np.nonzero(d < radius_m)[0]
    rng = np.random.default_rng(seed)
    if len(sel) > max_points:
        sel = rng.choice(sel, max_points, replace=False)
    return np.stack([wx[sel] - true_xy[0], wy[sel] - true_xy[1]], axis=1)


# (name, occ, scan, resolution_m, ares_rad, depth, max_c, nms) -- the same shapes
# the golden generator freezes, so the comparison spans single/multi-yaw + NMS.
def build_cases():
    cases = []
    cases.append((
        "six_by_six_single_yaw",
        make_map(6, 6, [(2, 2), (2, 3), (4, 4)]),
        np.array([[0.0, 0.0], [1.0, 0.0]], dtype=np.float64),
        1.0, 2.0 * math.pi, 2, 5, 0))

    occ2 = cluster_map(40, 48, seed=1, n_clusters=6, cluster_size=12)
    scan2 = scan_from_occupied(occ2, 1.0, (24.0, 20.0), 12.0, 60, seed=2)
    cases.append(("medium_4yaw_no_nms", occ2, scan2, 1.0,
                  math.radians(90.0), 3, 8, 0))
    cases.append(("medium_8yaw_nms3", occ2, scan2, 1.0,
                  math.radians(45.0), 3, 8, 3))

    occ4 = cluster_map(64, 64, seed=7, n_clusters=10, cluster_size=16)
    scan4 = scan_from_occupied(occ4, 0.5, (16.0, 16.0), 10.0, 80, seed=8)
    cases.append(("large_12yaw_depth4_nms2", occ4, scan4, 0.5,
                  math.radians(30.0), 4, 10, 2))
    return cases


def assert_parity(name, occ, scan, res, ares, depth, max_c, nms):
    py_cands = bbs.branch_and_bound_candidates(
        occ, scan, res, ares, depth, max_c, nms_radius_cells=nms)
    cpp_cands = bbs_cpp.branch_and_bound_candidates(
        occ, scan, res, ares, depth, max_c, nms_radius_cells=nms)

    assert len(cpp_cands) == len(py_cands), (
        "%s: count %d != %d" % (name, len(cpp_cands), len(py_cands)))
    for i, (c, p) in enumerate(zip(cpp_cands, py_cands)):
        assert (c.tx_cell, c.ty_cell, c.yaw_index, c.hit_count, c.point_count) \
            == (p.tx_cell, p.ty_cell, p.yaw_index, p.hit_count, p.point_count), (
                "%s[%d]: cell/yaw/hit mismatch cpp(%d,%d,%d,%d,%d) py(%d,%d,%d,%d,%d)"
                % (name, i, c.tx_cell, c.ty_cell, c.yaw_index, c.hit_count,
                   c.point_count, p.tx_cell, p.ty_cell, p.yaw_index, p.hit_count,
                   p.point_count))
        # score is the exact hit/point ratio in both backends.
        assert c.score == p.score, (
            "%s[%d]: score %r != %r" % (name, i, c.score, p.score))
        # yaw_rad comes from independent normalize implementations; allow ULPs.
        assert abs(c.yaw_rad - p.yaw_rad) < 1e-12, (
            "%s[%d]: yaw_rad %r != %r" % (name, i, c.yaw_rad, p.yaw_rad))
    print("  [%s] %d candidates bit-exact across backends" % (name, len(cpp_cands)))


def main():
    cases = build_cases()
    for case in cases:
        assert_parity(*case)
    print("bbs_cpp backend matches the Python reference on %d fixtures" % len(cases))


if __name__ == "__main__":
    main()
