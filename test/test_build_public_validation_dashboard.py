#!/usr/bin/env python3
"""Unit tests for the public validation dashboard (no ROS, no replay)."""

import importlib.util
import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "build_public_validation_dashboard",
    REPO_ROOT / "scripts" / "build_public_validation_dashboard.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


class TestDashboard(unittest.TestCase):
    def test_evidence_rows_appear_when_artifacts_exist(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            ws = Path(directory)
            write_json(
                ws / "artifacts/public/wp1_g2_ndt_search_method_ab_20260818/result.json",
                {"verdict": "keep_baseline_kdtree_1",
                 "result": {"per_candidate_ms": {"kdtree_t1": 761.3}}})
            write_json(
                ws / "artifacts/public/wp2_route_crop_ab_20260818/result.json",
                {"verdict": "route-crop wins WP2",
                 "result": {"route_crop_recall_le_5m": "10/10",
                            "route_crop_alias_free": "10/10"}})
            write_json(
                ws / "artifacts/public/koide_g3_recovery_regression/regression_result.json",
                {"overall_pass": False,
                 "recovery_health": {"recovery_confirmed_count": 0,
                                     "false_recovery_confirmed": False}})
            paths = {
                "demo_report_json": None,
                "release_summary_json": None,
                "public_summary_json": None,
                "demo_png": None,
                "wp1_result_json": ws / "artifacts/public/wp1_g2_ndt_search_method_ab_20260818/result.json",
                "wp2_result_json": ws / "artifacts/public/wp2_route_crop_ab_20260818/result.json",
                "koide_g3_result_json": ws / "artifacts/public/koide_g3_recovery_regression/regression_result.json",
                "koide_phase1_result_json": None,
            }
            data = MODULE.build_dashboard_data(paths)
            names = [row["experiment"] for row in data["evidence_rows"]]
            self.assertIn("WP1 G2 latency A/B", names)
            self.assertIn("WP2 route-crop A/B", names)
            self.assertIn("Koide G3 recovery regression", names)

    def test_render_contains_artifact_links(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            ws = Path(directory)
            paths = {
                "demo_report_json": None,
                "release_summary_json": None,
                "public_summary_json": None,
                "demo_png": None,
                "wp1_result_json": None,
                "wp2_result_json": None,
                "koide_g3_result_json": None,
                "koide_phase1_result_json": None,
            }
            data = MODULE.build_dashboard_data(paths)
            markdown = MODULE.render_markdown(data, ws)
            self.assertIn("Engineering Evidence", markdown)
            self.assertIn("../wp1_g2_ndt_search_method_ab_20260818/result.json", markdown)
            self.assertIn("koide_gif_gallery.md", markdown)
            html = MODULE.render_html(data)
            self.assertIn("Engineering Evidence", html)
            self.assertIn("../wp2_route_crop_ab_20260818/result.json", html)


if __name__ == "__main__":
    sys.exit(unittest.main())
