#!/usr/bin/env python3
"""Tabulate eval_global_search.py results and replay them through the startup logic.

Per result file: queries, success (top within --success-m and --success-yaw-deg),
top on the wrong floor (|dz| > 1.5 m), median / p95 query time, peak RSS, and a
startup replay: every query is used as a start, and the following answers go
through quickstart_model.decide_startup (the shipped gates, including the
refined-fix agreement and nearby-alternative checks), ground truth as odometry.
A published pose more than 1 m or 20 degrees off is a wrong start.

usage: compare.py RESULT.json... [--success-m 1.0] [--success-yaw-deg 5] [--raw]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))

import quickstart_model as model  # noqa: E402

PARAMS = model.StartupParams()
OBSERVATION_FIELDS = {f.name for f in fields(model.StartupObservation)}


def observation(query: dict) -> model.StartupObservation:
    candidates = [
        {
            "x": c["x"],
            "y": c["y"],
            "yaw_deg": c["yaw_deg"],
            "registration_fitness": c["fitness"],
            "registration_converged": c["fitness"] is not None
            and math.isfinite(c["fitness"]),
            "registration_refined": c["fitness"] is not None,
        }
        for c in query["candidates"]
    ]
    top = query["candidates"][0]
    refined = top["fitness"] is not None
    extra = {}
    if "query_refined_alternative_registration_fitness" in OBSERVATION_FIELDS:
        extra["query_refined_alternative_registration_fitness"] = (
            model.refined_alternative_registration_fitness(
                candidates,
                PARAMS.registration_refined_alternative_min_separation_m,
                PARAMS.registration_refined_alternative_min_yaw_deg,
            )
            if refined
            else None
        )
        extra["query_top_refined"] = refined
    return model.StartupObservation(
        now_sec=query["stamp"],
        scan_ready=True,
        saved_pose_available=False,
        global_available=True,
        query_candidate_scores=tuple(float(c["score"]) for c in query["candidates"]),
        query_candidate_age_sec=1.0,
        query_top_pose=(top["x"], top["y"], math.radians(top["yaw_deg"])),
        query_scan_stamp_sec=query["stamp"],
        query_top_registration_fitness=top["fitness"],
        query_alternative_registration_fitness=model.alternative_registration_fitness(
            candidates, PARAMS.registration_alternative_min_separation_m
        ),
        query_odom_pose=tuple(query["odom"]),
        **extra,
    )


def startup_from(queries: list, start: int) -> str:
    state = model.decide_startup(
        PARAMS,
        model.StartupState(name=model.STATE_WAITING_FOR_SCAN),
        model.StartupObservation(
            now_sec=queries[start]["stamp"],
            scan_ready=True,
            saved_pose_available=False,
            global_available=True,
        ),
    ).state
    for query in queries[start:]:
        if not query["candidates"]:
            continue
        decision = model.decide_startup(PARAMS, state, observation(query))
        state = decision.state
        if decision.action == model.ACTION_PUBLISH_GLOBAL:
            c = query["candidates"][decision.candidate_index]
            return (
                "wrong" if c["error_m"] > 1.0 or c["yaw_error_deg"] > 20.0 else "right"
            )
        if decision.action == model.ACTION_NEEDS_OPERATOR:
            return "manual"
    return "unfinished"


def load_queries(path: Path, raw: bool) -> list:
    """The queries; with raw, 3D-BBS's own top (before NDT) as the only candidate."""
    queries = json.loads(path.read_text())["queries"]
    if raw:
        queries = [
            {**q, "candidates": [q["raw_top"]] if q.get("raw_top") else []}
            for q in queries
        ]
    return queries


def row(path: Path, max_error_m: float, max_yaw_deg: float, raw: bool) -> dict:
    data = json.loads(path.read_text())
    queries = load_queries(path, raw)
    n = len(queries)
    tops = [q["candidates"][0] if q["candidates"] else None for q in queries]
    ok = sum(
        t is not None
        and t["error_m"] <= max_error_m
        and t["yaw_error_deg"] <= max_yaw_deg
        for t in tops
    )
    floor = sum(t is not None and abs(t.get("z_error_m", 0.0)) > 1.5 for t in tops)
    times = np.array([q["search_sec"] if raw else q["runtime_sec"] for q in queries])
    starts = {"wrong": 0, "right": 0, "manual": 0, "unfinished": 0}
    for start in range(max(0, n - 12)):
        starts[startup_from(queries, start)] += 1
    return {
        "name": path.stem,
        "queries": n,
        "success": ok,
        "wrong_floor": floor,
        "median_s": float(np.median(times)) if n else float("nan"),
        "p95_s": float(np.percentile(times, 95)) if n else float("nan"),
        "max_s": float(times.max()) if n else float("nan"),
        "peak_rss_mb": data.get("info", {}).get("peak_rss_mb", float("nan")),
        "starts": starts,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("results", nargs="+", type=Path)
    parser.add_argument("--success-m", type=float, default=1.0)
    parser.add_argument("--success-yaw-deg", type=float, default=5.0)
    parser.add_argument(
        "--raw", action="store_true", help="3D-BBS results: score its top before NDT"
    )
    args = parser.parse_args(argv)
    print(
        "| pair | queries | success | wrong floor | median s | p95 s | max s "
        "| peak RSS MB | starts wrong / right / manual |"
    )
    print("|---|---|---|---|---|---|---|---|---|")
    total = {"queries": 0, "success": 0, "wrong_floor": 0}
    starts = {"wrong": 0, "right": 0, "manual": 0, "unfinished": 0}
    times = []
    for path in args.results:
        r = row(path, args.success_m, args.success_yaw_deg, args.raw)
        for key in total:
            total[key] += r[key]
        for key in starts:
            starts[key] += r["starts"][key]
        queries = json.loads(path.read_text())["queries"]
        times.extend(q["search_sec"] if args.raw else q["runtime_sec"] for q in queries)
        s = r["starts"]
        print(
            f"| {r['name']} | {r['queries']} | {r['success']} | {r['wrong_floor']} "
            f"| {r['median_s']:.2f} | {r['p95_s']:.2f} | {r['max_s']:.1f} "
            f"| {r['peak_rss_mb']:.0f} | {s['wrong']} / {s['right']} / {s['manual']} |"
        )
    if times:
        print(
            f"| **total** | {total['queries']} | {total['success']} "
            f"({100.0 * total['success'] / max(1, total['queries']):.0f}%) "
            f"| {total['wrong_floor']} | {np.median(times):.2f} "
            f"| {np.percentile(times, 95):.2f} | {max(times):.1f} | "
            f"| {starts['wrong']} / {starts['right']} / {starts['manual']} |"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
