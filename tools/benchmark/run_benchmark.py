#!/usr/bin/env python3
"""Replay a benchmark suite through the quickstart and score it against ground truth.

Each run starts the suite's odometry, plays the bag, starts
``quickstart.py --map <map>`` with a fresh pose state, records ``/pcl_pose``
and scores it with benchmark_eval.py in the map (= ground-truth) frame.

usage: run_benchmark.py SUITE.yaml --out DIR [--repeats N] [--case NAME ...]
       run_benchmark.py SUITE.yaml --out DIR --evaluate-only
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import benchmark_eval

TOOL_DIR = Path(__file__).resolve().parent


def expand(value, suite_dir: Path):
    """Expand ${VAR} and ${SUITE_DIR} in strings, recursively."""
    if isinstance(value, str):
        return os.path.expandvars(value.replace("${SUITE_DIR}", str(suite_dir)))
    if isinstance(value, list):
        return [expand(item, suite_dir) for item in value]
    if isinstance(value, dict):
        return {key: expand(item, suite_dir) for key, item in value.items()}
    return value


def load_suite(path: Path) -> list[dict]:
    """Cases of a suite, each merged over the suite defaults."""
    suite = yaml.safe_load(path.read_text(encoding="utf-8"))
    defaults = suite.get("defaults", {})
    cases = []
    for case in suite["cases"]:
        merged = {**defaults, **case}
        merged["odometry_args"] = list(defaults.get("odometry_args", [])) + list(
            case.get("odometry_args", [])
        )
        merged["quickstart_args"] = list(defaults.get("quickstart_args", [])) + list(
            case.get("quickstart_args", [])
        )
        cases.append(expand(merged, path.resolve().parent))
    return cases


def bag_start_and_duration(bag: Path) -> tuple[float, float]:
    info = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))[
        "rosbag2_bagfile_information"
    ]
    return (
        info["starting_time"]["nanoseconds_since_epoch"] * 1.0e-9,
        info["duration"]["nanoseconds"] * 1.0e-9,
    )


def start(command: list[str], log: Path, env: dict) -> subprocess.Popen:
    handle = log.open("w", encoding="utf-8")
    return subprocess.Popen(
        command,
        stdout=handle,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )


def stop(process: subprocess.Popen | None, grace_sec: float = 10.0) -> None:
    """SIGINT the process group, then TERM, then KILL."""
    if process is None or process.poll() is not None:
        return
    for sig, wait_sec in ((signal.SIGINT, grace_sec), (signal.SIGTERM, 5.0)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait_sec)
            return
        except subprocess.TimeoutExpired:
            continue
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait()


def run_once(
    case: dict, run_dir: Path, domain_id: int, quickstart_cmd: list[str]
) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    bag = Path(case["bag"])
    _, duration = bag_start_and_duration(bag)
    offset = float(case.get("start_offset_sec", 0.0))
    rate = float(case.get("rate", 1.0))

    recorder = start(
        [sys.executable, str(TOOL_DIR / "record_pose.py"), str(run_dir)],
        run_dir / "recorder.log",
        env,
    )
    odometry = None
    if case.get("odometry_launch"):
        odometry = start(
            [
                "ros2",
                "launch",
                *shlex.split(case["odometry_launch"]),
                "use_sim_time:=true",
                *case["odometry_args"],
            ],
            run_dir / "odometry.log",
            env,
        )
    time.sleep(3.0)
    play_cmd = ["ros2", "bag", "play", str(bag), "--clock", "--rate", str(rate)]
    if offset > 0.0:
        play_cmd += ["--start-offset", str(offset)]
    play_cmd += ["--topics", *case["topics"]]
    play = start(play_cmd, run_dir / "play.log", env)
    time.sleep(3.0)
    quickstart = start(
        [
            *quickstart_cmd,
            "--map",
            case["map"],
            "--no-rviz",
            "--state-file",
            str(run_dir / "pose.json"),
            "--output",
            str(run_dir / "quickstart.yaml"),
            *case["quickstart_args"],
        ],
        run_dir / "quickstart.log",
        env,
    )
    try:
        play.wait(timeout=(duration - offset) / rate + 120.0)
    except subprocess.TimeoutExpired:
        stop(play)
    time.sleep(5.0)
    for process in (quickstart, odometry, recorder):
        stop(process)


def score(case: dict, run_dir: Path) -> benchmark_eval.RunScore:
    bag_start, duration = bag_start_and_duration(Path(case["bag"]))
    start_sec = bag_start + float(case.get("start_offset_sec", 0.0))
    result = benchmark_eval.score_run(
        benchmark_eval.load_tum(run_dir / "est.tum"),
        benchmark_eval.load_tum(case["gt"]),
        start_sec,
        benchmark_eval.load_alignment_levels(run_dir / "alignment.jsonl"),
        end_sec=bag_start + duration,
    )
    (run_dir / "score.json").write_text(json.dumps(result.as_dict(), indent=2) + "\n")
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("suite", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repeats", type=int)
    parser.add_argument("--case", action="append", help="run only these cases")
    parser.add_argument("--evaluate-only", action="store_true")
    parser.add_argument(
        "--quickstart",
        default="ros2 run lidar_localization_ros2 quickstart.py",
        help="command that starts the quickstart (e.g. a source checkout's script)",
    )
    parser.add_argument("--domain-id", type=int, default=120)
    args = parser.parse_args(argv)

    cases = [
        case
        for case in load_suite(args.suite)
        if not args.case or case["name"] in args.case
    ]
    results: dict[str, list[benchmark_eval.RunScore]] = {}
    for case in cases:
        repeats = args.repeats or int(case.get("repeats", 1))
        scores = []
        for repeat in range(repeats):
            run_dir = args.out / case["name"] / f"run{repeat + 1}"
            if not args.evaluate_only:
                print(
                    f"[benchmark] {case['name']} run {repeat + 1}/{repeats}", flush=True
                )
                run_once(case, run_dir, args.domain_id, shlex.split(args.quickstart))
            if (run_dir / "est.tum").exists():
                scores.append(score(case, run_dir))
        results[case["name"]] = scores
        print(
            benchmark_eval.summary_table({case["name"]: scores}).splitlines()[-1],
            flush=True,
        )

    table = (
        benchmark_eval.summary_table(results)
        + "\n\n"
        + benchmark_eval.health_table(results)
    )
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "summary.md").write_text(table + "\n", encoding="utf-8")
    (args.out / "results.json").write_text(
        json.dumps(
            {name: [s.as_dict() for s in runs] for name, runs in results.items()},
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(table)
    return 0


if __name__ == "__main__":
    sys.exit(main())
