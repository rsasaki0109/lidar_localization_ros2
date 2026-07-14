#!/usr/bin/env python3

from pathlib import Path
import os
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "setup_local_env.sh"


class SetupLocalEnvTest(unittest.TestCase):
    def run_setup(self, workspace: Path, explicit: Path | None = None) -> str:
        env = os.environ.copy()
        env["LIDAR_LOCALIZATION_WS_ROOT"] = str(workspace)
        env["LIDAR_LOCALIZATION_ROS_DISTRO"] = os.environ.get("ROS_DISTRO", "jazzy")
        env.pop("LIDAR_LOCALIZATION_OVERLAY", None)
        if explicit is not None:
            env["LIDAR_LOCALIZATION_OVERLAY"] = str(explicit)
        result = subprocess.run(
            ["bash", "-c", f'source "{SCRIPT}" && printf %s "$LIDAR_TEST_OVERLAY_SOURCED"'],
            check=True, capture_output=True, text=True, env=env)
        return result.stdout

    def test_conventional_workspace_install_is_sourced(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            setup = workspace / "install" / "setup.bash"
            setup.parent.mkdir(parents=True)
            setup.write_text("export LIDAR_TEST_OVERLAY_SOURCED=conventional\n")
            self.assertEqual(self.run_setup(workspace), "conventional")

    def test_explicit_overlay_has_priority(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            conventional = workspace / "install" / "setup.bash"
            explicit = workspace / "explicit_setup.bash"
            conventional.parent.mkdir(parents=True)
            conventional.write_text("export LIDAR_TEST_OVERLAY_SOURCED=conventional\n")
            explicit.write_text("export LIDAR_TEST_OVERLAY_SOURCED=explicit\n")
            self.assertEqual(self.run_setup(workspace, explicit), "explicit")


if __name__ == "__main__":
    unittest.main()
