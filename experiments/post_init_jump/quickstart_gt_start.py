#!/usr/bin/env python3
"""Run quickstart with a ground-truth initial pose at a chosen bag time.

For run_benchmark.py --quickstart: strips --gt-start-file/--gt-start-bag/
--gt-start-sec from the arguments, starts gt_initial_pose.py with them, and runs
quickstart (with LL_PARAM_OVERRIDES, see quickstart_overrides.py) on the rest.
Pass --no-auto-initialize in the case's quickstart_args so that only this pose
starts tracking.
"""

import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import quickstart_overrides  # noqa: E402  (applies LL_PARAM_OVERRIDES)


def main() -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--gt-start-file", required=True)
    parser.add_argument("--gt-start-bag", required=True)
    parser.add_argument("--gt-start-sec", required=True)
    parser.add_argument("--gt-start-offset-m", default="0")
    own, rest = parser.parse_known_args()
    injector = subprocess.Popen(
        [
            sys.executable,
            str(HERE / "gt_initial_pose.py"),
            "--ground-truth",
            own.gt_start_file,
            "--bag",
            own.gt_start_bag,
            "--start-sec",
            own.gt_start_sec,
            "--offset-m",
            own.gt_start_offset_m,
        ]
    )
    try:
        return quickstart_overrides.quickstart.main(rest)
    finally:
        injector.terminate()
        injector.wait(timeout=10)


if __name__ == "__main__":
    sys.exit(main())
