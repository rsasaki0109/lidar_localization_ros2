#!/usr/bin/env python3
"""Regression guards for the user-facing RViz configuration.

The RViz config is the primary visualization surface for quickstart and the
Nav2/standalone launches. These tests keep it parseable, keep the display names
semantic, and keep it installed with the package (quickstart.launch.py loads it
from the package share directory).
"""

import re
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
RViz_PATH = ROOT / "rviz" / "localization.rviz"
RVIZ_MID360_PATH = ROOT / "rviz" / "localization_mid360.rviz"
CMAKELISTS = ROOT / "CMakeLists.txt"
QUICKSTART_LAUNCH = ROOT / "launch" / "quickstart.launch.py"


class TestRvizConfig(unittest.TestCase):
    def test_config_is_valid_yaml(self):
        data = yaml.safe_load(RViz_PATH.read_text(encoding="utf-8"))
        manager = data["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        displays = manager["Displays"]
        names = [d.get("Name") for d in displays]
        self.assertGreaterEqual(len(displays), 6)
        for expected in (
            "Map (prior)",
            "Live Scan",
            "Local Map",
            "Trajectory (/path)",
            "Localized Pose (/pcl_pose)",
            "TF",
        ):
            self.assertIn(expected, names)

    def test_key_displays_are_enabled(self):
        data = yaml.safe_load(RViz_PATH.read_text(encoding="utf-8"))
        displays = {d.get("Name"): d for d in data["Visualization Manager"]["Displays"]}
        for name in ("Live Scan", "Localized Pose (/pcl_pose)", "Trajectory (/path)"):
            self.assertTrue(
                displays[name]["Enabled"], f"{name} must be visible by default"
            )

    def test_scan_display_uses_flat_color(self):
        data = yaml.safe_load(RViz_PATH.read_text(encoding="utf-8"))
        scan = next(
            d
            for d in data["Visualization Manager"]["Displays"]
            if d.get("Name") == "Live Scan"
        )
        self.assertEqual(scan["Color Transformer"], "FlatColor")
        self.assertFalse(scan["Use rainbow"])

    def test_rviz_dir_is_installed_with_package(self):
        cmake = CMAKELISTS.read_text(encoding="utf-8")
        match = re.search(
            r"install\(DIRECTORY\s*(.*?)\s*DESTINATION share/\$\{PROJECT_NAME\}",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(match, "install(DIRECTORY ... share) block not found")
        self.assertIn("rviz", match.group(1))

    def test_quickstart_loads_config_from_package_share(self):
        source = QUICKSTART_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"rviz", rviz_file', source)
        self.assertIn("localization_mid360.rviz", source)
        self.assertIn("localization.rviz", source)

    def test_mid360_config_uses_livox_scan_topic(self):
        data = yaml.safe_load(RVIZ_MID360_PATH.read_text(encoding="utf-8"))
        scan = next(
            d
            for d in data["Visualization Manager"]["Displays"]
            if d.get("Name") == "Live Scan"
        )
        self.assertEqual(scan["Topic"]["Value"], "/livox/points")

    def test_both_configs_show_global_candidates(self):
        for path in (RViz_PATH, RVIZ_MID360_PATH):
            with self.subTest(path=path.name):
                data = yaml.safe_load(path.read_text(encoding="utf-8"))
                displays = {
                    d.get("Name"): d for d in data["Visualization Manager"]["Displays"]
                }
                self.assertIn("Global Candidates", displays)
                candidates = displays["Global Candidates"]
                self.assertEqual(
                    candidates["Topic"]["Value"], "/global_localization_node/candidates"
                )


if __name__ == "__main__":
    unittest.main()
