#!/usr/bin/env python3
"""Run quickstart with localizer parameters overridden from LL_PARAM_OVERRIDES (JSON).

For replay comparisons only: run_benchmark.py --quickstart "python3 <this file>"
lets a suite vary localizer parameters that quickstart does not expose, without
touching the runtime defaults.
"""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import create_lidar_localization_config as config_tool
import quickstart

OVERRIDES = json.loads(os.environ.get("LL_PARAM_OVERRIDES", "{}"))
_make_params = config_tool.make_params


def make_params(args):
    params = _make_params(args)
    unknown = sorted(set(OVERRIDES) - set(params))
    if unknown:
        raise SystemExit(f"LL_PARAM_OVERRIDES: unknown parameters {unknown}")
    params.update(OVERRIDES)
    return params


config_tool.make_params = make_params
quickstart.config_tool.make_params = make_params

if __name__ == "__main__":
    sys.exit(quickstart.main())
