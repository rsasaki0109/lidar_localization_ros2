#!/usr/bin/env python3

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PACKAGE_NAME = "lidar_localization_ros2"


def resolve_repo_root() -> Path:
    script_path = Path(__file__).resolve()
    candidates = [
        script_path.parents[1],
        script_path.parents[3] / "share" / PACKAGE_NAME,
    ]
    for candidate in candidates:
        if (candidate / "experiments").is_dir() and (candidate / "docs").is_dir():
            return candidate
    raise RuntimeError(f"Could not resolve repo root from {script_path}")


def main() -> int:
    repo_root = resolve_repo_root()
    scripts = [
        repo_root / "scripts" / "run_borderline_gate_experiments.py",
        repo_root / "scripts" / "run_imu_guard_experiments.py",
        repo_root / "scripts" / "run_recovery_action_experiments.py",
        repo_root / "scripts" / "run_reinit_trigger_experiments.py",
    ]
    for script in scripts:
        subprocess.run([sys.executable, str(script)], check=True, cwd=repo_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
