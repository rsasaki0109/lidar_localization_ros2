#!/usr/bin/env python3

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "setup_local_env.sh"


class SetupLocalEnvTest(unittest.TestCase):
    def run_setup(
        self,
        workspace: Path | None,
        explicit: Path | None = None,
        script: Path = SCRIPT,
    ) -> str:
        env = os.environ.copy()
        env.pop("LIDAR_LOCALIZATION_WS_ROOT", None)
        env.pop("LIDAR_LOCALIZATION_LOCAL_PREFIX", None)
        env.pop("LIDAR_TEST_OVERLAY_SOURCED", None)
        if workspace is not None:
            env["LIDAR_LOCALIZATION_WS_ROOT"] = str(workspace)
        env["LIDAR_LOCALIZATION_ROS_DISTRO"] = os.environ.get("ROS_DISTRO", "jazzy")
        env.pop("LIDAR_LOCALIZATION_OVERLAY", None)
        if explicit is not None:
            env["LIDAR_LOCALIZATION_OVERLAY"] = str(explicit)
        result = subprocess.run(
            [
                "bash",
                "-c",
                f'source "{script}" && printf %s "$LIDAR_TEST_OVERLAY_SOURCED"',
            ],
            check=True,
            capture_output=True,
            text=True,
            env=env,
        )
        return result.stdout

    def test_explicit_root_with_nested_overlay_is_honored(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            setup = workspace / "build_ws/install/setup.bash"
            setup.parent.mkdir(parents=True)
            setup.write_text(
                'export LIDAR_TEST_OVERLAY_SOURCED="$LIDAR_LOCALIZATION_LOCAL_PREFIX"\n'
            )
            self.assertEqual(self.run_setup(workspace), str(workspace / "local_prefix"))

    def test_default_root_for_supported_repository_layouts(self):
        for relative in ["repo", "src/repo", "worktrees/repo"]:
            with self.subTest(layout=relative), tempfile.TemporaryDirectory() as tmp:
                workspace = Path(tmp)
                script = workspace / relative / "scripts/setup_local_env.sh"
                script.parent.mkdir(parents=True)
                script.write_bytes(SCRIPT.read_bytes())
                setup = workspace / "build_ws/install/setup.bash"
                setup.parent.mkdir(parents=True)
                setup.write_text(
                    'export LIDAR_TEST_OVERLAY_SOURCED="$LIDAR_LOCALIZATION_LOCAL_PREFIX"\n'
                )
                self.assertEqual(
                    self.run_setup(None, script=script), str(workspace / "local_prefix")
                )

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
