#!/usr/bin/env python3

import os
import re
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


ROS_RUN_SCRIPTS = {
    "scripts/check_lidar_localization_bringup.py",
    "scripts/analyze_koide_imu_consistency.py",
    "scripts/create_lidar_localization_config.py",
    "scripts/quickstart.py",
    "scripts/startup_initialization_node.py",
    "scripts/prepare_koide_hard_relocalization_assets.sh",
    "scripts/run_koide_hard_imu_deskew_smoke.sh",
    "scripts/run_koide_corner_deskew_ab.sh",
    "scripts/run_lidar_localization_imu_comparison.py",
    "scripts/summarize_koide_deskew_ab.py",
    "scripts/summarize_registration_localizability.py",
    "scripts/validate_lidar_localization_imu.py",
}
BOOTSTRAP_SCRIPT = "scripts/bootstrap_colcon_workspace.sh"


def cmake_script_groups(text: str) -> dict[str, list[str]]:
    groups = {}
    pattern = re.compile(
        r"set\((?P<name>LIDAR_LOCALIZATION_[A-Z_]+_SCRIPTS)\s+(?P<body>.*?)\)",
        re.DOTALL,
    )
    for match in pattern.finditer(text):
        groups[match.group("name")] = re.findall(r"scripts/[^\s)]+", match.group("body"))
    return groups


def cmake_install_programs() -> set[str]:
    text = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    groups = cmake_script_groups(text)
    match = re.search(
        r"install\(PROGRAMS(?P<body>.*?)DESTINATION\s+lib/\$\{PROJECT_NAME\}\)",
        text,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError("install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME}) not found")
    body = match.group("body")
    programs = set(re.findall(r"scripts/[^\s)]+", body))
    for variable in re.findall(r"\$\{([A-Z_]+)\}", body):
        if variable not in groups:
            raise AssertionError(f"unknown install(PROGRAMS) variable: {variable}")
        programs.update(groups[variable])
    return programs


class TestPackageInstallContract(unittest.TestCase):
    def test_user_facing_ros_run_scripts_are_installed(self):
        installed = cmake_install_programs()

        self.assertFalse(
            ROS_RUN_SCRIPTS - installed,
            f"missing install(PROGRAMS) entries: {sorted(ROS_RUN_SCRIPTS - installed)}",
        )

    def test_user_facing_ros_run_scripts_are_executable(self):
        for relative_path in sorted(ROS_RUN_SCRIPTS):
            with self.subTest(relative_path=relative_path):
                path = REPO_ROOT / relative_path

                self.assertTrue(path.exists())
                self.assertTrue(os.access(path, os.X_OK), f"{relative_path} is not executable")

    def test_grouped_install_scripts_exist_and_are_unique(self):
        cmake_text = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        groups = cmake_script_groups(cmake_text)
        grouped_scripts = [script for values in groups.values() for script in values]

        self.assertTrue(groups, "no grouped script lists found")
        self.assertEqual(len(grouped_scripts), len(set(grouped_scripts)))
        self.assertEqual(cmake_install_programs(), set(grouped_scripts))
        for relative_path in sorted(grouped_scripts):
            with self.subTest(relative_path=relative_path):
                self.assertTrue((REPO_ROOT / relative_path).is_file())

    def test_lidar_localization_mid360_helper_package_is_installed_as_directory(self):
        cmake_text = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

        self.assertIn("scripts/lidar_localization_mid360", cmake_text)
        self.assertTrue((REPO_ROOT / "scripts/lidar_localization_mid360/__init__.py").exists())
        self.assertTrue((REPO_ROOT / "scripts/lidar_localization_mid360/validation_model.py").exists())

    def test_colcon_bootstrap_script_is_executable(self):
        path = REPO_ROOT / BOOTSTRAP_SCRIPT

        self.assertTrue(path.exists())
        self.assertTrue(os.access(path, os.X_OK), f"{BOOTSTRAP_SCRIPT} is not executable")

    def test_dependencies_repos_lists_required_ndt_dependency(self):
        repos_text = (REPO_ROOT / "dependencies.repos").read_text(encoding="utf-8")

        self.assertIn("ndt_omp_ros2:", repos_text)
        self.assertIn("https://github.com/rsasaki0109/ndt_omp_ros2.git", repos_text)


if __name__ == "__main__":
    sys.exit(unittest.main())
