#!/usr/bin/env python3
"""Smoke test for benchmark_g2_ndt_search_method.py (WP1 A/B runner).

The helper functions are always tested. The end-to-end smoke only runs when the
optional g2_ndt_score module is importable (pybind11 built), mirroring
test_bbs_cpp_backend_parity.py: it prints SKIP and exits 0 otherwise.
"""

import os
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import benchmark_g2_ndt_search_method as bench  # noqa: E402
import global_localization_query as glq  # noqa: E402

_module_dir = os.environ.get("G2_NDT_SCORE_MODULE_DIR")
if _module_dir:
    sys.path.insert(0, _module_dir)

try:
    import g2_ndt_score  # noqa: E402
    HAVE_MODULE = True
except ImportError as exc:  # pragma: no cover - depends on build config
    print("SKIP: g2_ndt_score not importable (%s); pybind11 backend not built" % exc)
    HAVE_MODULE = False


def write_occupancy_map(directory: Path) -> Path:
    grid = np.zeros((40, 40), dtype=bool)
    grid[10, 5:35] = True
    grid[10:30, 5] = True
    grid[20, 10:30] = True
    pixels = np.where(np.flipud(grid), 0, 254).astype(np.uint8)
    Image.fromarray(pixels, mode="L").save(directory / "map.pgm")
    yaml_path = directory / "map.yaml"
    yaml_path.write_text(
        'image: "map.pgm"\n'
        "resolution: 0.5\n"
        "origin: [-5.0, -5.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n")
    return yaml_path


def write_tiny_ply(path: Path, points: np.ndarray):
    with open(path, "wb") as stream:
        stream.write(
            f"ply\nformat binary_little_endian 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "end_header\n".encode())
        stream.write(points.astype("<f4").tobytes())


def test_helpers():
    assert glq.resolve_pclomp_search_method("kdtree") == 0
    assert glq.resolve_pclomp_search_method("direct7") == 2

    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = write_occupancy_map(Path(tmp))
        occ = glq.bbs_engine.load_occupancy_map(yaml_path)
        scan = bench.seeded_scan_xy(occ, (0.0, 0.0), 30.0, 128, seed=0)
        assert len(scan) > 0
        assert scan.shape[1] == 2
        poses = bench.seeded_candidate_poses(occ, (0.0, 0.0), 10.0, 8, seed=1)
        assert len(poses) == 8
        assert all(len(p) == 4 for p in poses)  # (x, y, z, yaw)


def test_end_to_end_smoke():
    if not HAVE_MODULE:
        return
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        occ_yaml = write_occupancy_map(root)
        map_path = root / "map.ply"
        x = np.linspace(-4.0, 4.0, 20)
        z = np.linspace(-1.0, 3.0, 10)
        xx, zz = np.meshgrid(x, z)
        pts = np.stack([xx.ravel(), np.zeros(xx.size), zz.ravel()], axis=1)
        write_tiny_ply(map_path, pts)

        out = root / "out"
        sys.argv = [
            "benchmark_g2_ndt_search_method.py",
            "--map", str(map_path),
            "--occupancy-yaml", str(occ_yaml),
            "--methods", "kdtree,direct7",
            "--threads", "1",
            "--repeats", "1",
            "--candidates", "4",
            "--center-x", "0.0",
            "--center-y", "2.0",
            "--output", str(out),
        ]
        bench.main()

        summary = __import__("json").loads((out / "summary.json").read_text())
        assert len(summary["runs"]) == 2
        assert {run["method"] for run in summary["runs"]} == {"kdtree", "direct7"}
        assert (out / "candidate_latency.csv").exists()
        csv_text = (out / "candidate_latency.csv").read_text()
        assert "kdtree,1," in csv_text
        assert "direct7,1," in csv_text


if __name__ == "__main__":
    test_helpers()
    test_end_to_end_smoke()
    print("test_benchmark_g2_ndt_search_method: all tests passed")
