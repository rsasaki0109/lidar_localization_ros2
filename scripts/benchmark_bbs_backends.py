#!/usr/bin/env python3
"""Wall-clock benchmark: the C++ ``bbs_cpp`` backend vs the Python BBS search.

Times the exact backend-dependent step ``GlobalLocalizationEngine.query()``
makes -- ``branch_and_bound_candidates`` on the *dilated* runtime matching grid --
on a real occupancy map, and asserts both backends return the identical top
candidate. Reports the median wall-clock per backend and records the 1-min load
average for every rep, flagging any rep taken at ``load >= 5`` as invalid: on a
shared machine, timing is only trustworthy when the box is quiet
(docs/g2_bbs_speedup.md).

Beyond the runtime default (5 deg yaw, 512 pts) it also times the C++ backend
with the coarser-yaw / fewer-points speedup levers -- the path toward a
sub-second query for live recovery (the dilated search is several seconds even
in C++ at the default settings).

The map defaults to the committed Koide ``outdoor_hard_bbs`` map; pass another
with ``--occupancy-yaml``. The scan is a seeded sample of occupied cells within a
radius of the chosen pose, so the run is deterministic and machine-independent in
shape (only the wall-clock numbers depend on the host).

Build the module first (needs pybind11), e.g.::

    g++ -O2 -std=c++17 -shared -fPIC -I include -I /usr/include/pybind11 \\
        $(python3-config --includes) -I "$(python3 -c 'import numpy;print(numpy.get_include())')" \\
        src/bbs_pybind.cpp -o /tmp/bbs/bbs_cpp$(python3-config --extension-suffix)

then point ``--module-dir`` (or ``$BBS_CPP_MODULE_DIR``) at it. When the package
is built with colcon, the module installs next to the node and is importable
directly, so no ``--module-dir`` is needed.
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import make_bbs_relocalization_attempts as bbs  # noqa: E402

DEFAULT_YAML = (ROOT / "data/public/koide_hard_localization/generated"
                / "occupancy_outdoor_hard_bbs/outdoor_hard_bbs.yaml")
LOAD_GATE = 5.0


def load1():
    return os.getloadavg()[0]


def median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def build_matching_grid(occ_map, dilate_cells):
    grid = occ_map.occupied
    for _ in range(max(0, dilate_cells)):
        grid = bbs._dilate_one_cell(grid)
    return grid


def seeded_scan_xy(occ_map, pose_xy, radius_m, max_points, seed):
    occ = occ_map.occupied
    ys, xs = np.nonzero(occ)
    wx = occ_map.origin_x_m + (xs + 0.5) * occ_map.resolution_m
    wy = occ_map.origin_y_m + (ys + 0.5) * occ_map.resolution_m
    d = np.hypot(wx - pose_xy[0], wy - pose_xy[1])
    sel = np.nonzero(d < radius_m)[0]
    rng = np.random.default_rng(seed)
    if len(sel) > max_points:
        sel = rng.choice(sel, max_points, replace=False)
    return np.stack([wx[sel] - pose_xy[0], wy[sel] - pose_xy[1]], axis=1)


def timed(fn, reps):
    times, loads = [], []
    out = None
    for _ in range(reps):
        lo = load1()
        t = time.perf_counter()
        out = fn()
        times.append(time.perf_counter() - t)
        loads.append(lo)
    return times, loads, out


def report(label, times, loads):
    med = median(times)
    flagged = any(l >= LOAD_GATE for l in loads)
    print("%-24s median %7.3f s  loads[%s]%s"
          % (label, med, ", ".join("%.1f" % l for l in loads),
             "  <-- load>=%.0f, INVALID" % LOAD_GATE if flagged else ""))
    return med, flagged


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--occupancy-yaml", type=Path, default=DEFAULT_YAML)
    ap.add_argument("--module-dir", default=os.environ.get("BBS_CPP_MODULE_DIR"),
                    help="directory holding the compiled bbs_cpp module")
    ap.add_argument("--resolution-m", type=float, default=0.2)
    ap.add_argument("--depth", type=int, default=4)
    ap.add_argument("--max-candidates", type=int, default=16)
    ap.add_argument("--nms-m", type=float, default=2.0)
    ap.add_argument("--dilate-cells", type=int, default=1)
    ap.add_argument("--scan-radius-m", type=float, default=30.0)
    ap.add_argument("--pose-x", type=float, default=0.0)
    ap.add_argument("--pose-y", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--py-reps", type=int, default=3)
    ap.add_argument("--cpp-reps", type=int, default=5)
    args = ap.parse_args()

    if args.module_dir:
        sys.path.insert(0, args.module_dir)
    try:
        import bbs_cpp
    except ImportError as exc:
        print("ERROR: bbs_cpp not importable (%s). Build it and pass --module-dir."
              % exc)
        return 2

    occ_map = bbs.load_occupancy_map(args.occupancy_yaml)
    grid = build_matching_grid(occ_map, args.dilate_cells)
    pose = (args.pose_x, args.pose_y)
    nms_cells = int(round(max(0.0, args.nms_m) / args.resolution_m))
    res, depth, max_c = args.resolution_m, args.depth, args.max_candidates
    print("map %dx%d occupied(dilated)=%d depth=%d nms_cells=%d  load_start=%.2f"
          % (grid.shape[0], grid.shape[1], int(grid.sum()), depth, nms_cells, load1()))

    scan512 = seeded_scan_xy(occ_map, pose, args.scan_radius_m, 512, args.seed)
    scan256 = seeded_scan_xy(occ_map, pose, args.scan_radius_m, 256, args.seed)
    a5, a10 = math.radians(5.0), math.radians(10.0)

    def call(mod, scan, ares):
        return mod.branch_and_bound_candidates(
            grid, scan, res, ares, depth, max_c, nms_radius_cells=nms_cells)

    py0 = call(bbs, scan512, a5)
    cpp0 = call(bbs_cpp, scan512, a5)
    same = len(py0) == len(cpp0) and bool(py0) and (
        (py0[0].tx_cell, py0[0].ty_cell, py0[0].yaw_index, py0[0].hit_count) ==
        (cpp0[0].tx_cell, cpp0[0].ty_cell, cpp0[0].yaw_index, cpp0[0].hit_count))

    pt, pl, _ = timed(lambda: call(bbs, scan512, a5), args.py_reps)
    ct, cl, _ = timed(lambda: call(bbs_cpp, scan512, a5), args.cpp_reps)
    c10t, c10l, _ = timed(lambda: call(bbs_cpp, scan512, a10), args.cpp_reps)
    c256t, c256l, _ = timed(lambda: call(bbs_cpp, scan256, a10), args.cpp_reps)

    print()
    py_med, py_bad = report("python  5deg 512pt", pt, pl)
    cpp_med, cpp_bad = report("bbs_cpp 5deg 512pt", ct, cl)
    report("bbs_cpp 10deg 512pt", c10t, c10l)
    report("bbs_cpp 10deg 256pt", c256t, c256l)
    print()
    print("speedup (5deg 512pt): %.1fx   top candidate identical: %s   candidates=%d"
          % (py_med / cpp_med, same, len(py0)))
    print("top: cell(%d,%d) yaw_index=%d hit=%d score=%.4f"
          % (cpp0[0].tx_cell, cpp0[0].ty_cell, cpp0[0].yaw_index,
             cpp0[0].hit_count, cpp0[0].score))
    print("load_end=%.2f   any_invalid=%s" % (load1(), py_bad or cpp_bad))
    if not same:
        print("ERROR: backends disagree on the top candidate")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
