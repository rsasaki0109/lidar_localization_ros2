#!/usr/bin/env python3

import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
import unittest


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "benchmark_from_manifest"
LOADER = SourceFileLoader("benchmark_from_manifest", str(SCRIPT))
SPEC = importlib.util.spec_from_loader("benchmark_from_manifest", LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class SystemCommandManifestTest(unittest.TestCase):
    def test_external_system_command_does_not_require_localizer_yaml(self):
        manifest = {"system": {"command": "ros2 launch kiss_icp odometry.launch.py"}}
        command = MODULE.build_system_command(manifest, REPO, None)
        self.assertEqual(command, "ros2 launch kiss_icp odometry.launch.py")

    def test_invalid_external_system_section_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "non-empty command"):
            MODULE.build_system_command({"system": {}}, REPO, None)


if __name__ == "__main__":
    unittest.main()
