#!/usr/bin/env python3
"""Replay a benchmark suite through the quickstart and score it against ground truth.

Each run starts the suite's odometry, plays the bag, starts
``quickstart.py --map <map>`` with a fresh pose state, records the pose
(``/pcl_pose``; ``/localization/pose_with_covariance`` with ``--profile mid360``
or ``nav2``) and scores it with benchmark_eval.py in the map (= ground-truth)
frame.

With odometry, quickstart starts only once the odom TF is on /tf, gets the
odometry settings explicitly (``--odom-tf-prediction --base-frame <child of
odom>``) instead of discovering them, and a run whose configuration still has
odometry off is marked invalid (``invalid.txt``), left out of the scores and
makes the exit code 1.

usage: run_benchmark.py SUITE.yaml --out DIR [--repeats N] [--case NAME ...]
       run_benchmark.py SUITE.yaml --out DIR --evaluate-only
"""

from __future__ import annotations

import argparse
import dataclasses
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

# How long a run waits for the odometry's first odom TF after the bag starts.
ODOMETRY_TF_TIMEOUT_SEC = 30.0
# Quickstart starts this long after the bag (or later, once the odom TF is seen).
QUICKSTART_DELAY_SEC = 3.0
INVALID_FILE = "invalid.txt"


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


def expects_odometry(case: dict) -> bool:
    """Runs of this case must start with odometry (default: when it launches one)."""
    return bool(case.get("expect_odometry", bool(case.get("odometry_launch"))))


def odometry_quickstart_args(quickstart_args: list[str], base_frame: str) -> list[str]:
    """quickstart_args plus the odometry settings, so they do not depend on discovery.

    Settings the case already gives are kept.
    """
    flags = [arg.split("=", 1)[0] for arg in quickstart_args]
    extra = []
    if not {"--odom-tf-prediction", "--no-odom-tf-prediction"} & set(flags):
        extra.append("--odom-tf-prediction")
    if "--base-frame" not in flags:
        extra.append(f"--base-frame={base_frame}")
    return list(quickstart_args) + extra


def quickstart_profile(quickstart_args: list[str]) -> str:
    """The --profile the quickstart runs with (standalone when not given)."""
    profile = "standalone"
    for index, arg in enumerate(quickstart_args):
        if arg.startswith("--profile="):
            profile = arg.split("=", 1)[1]
        elif arg == "--profile" and index + 1 < len(quickstart_args):
            profile = quickstart_args[index + 1]
    return profile


def pose_topic(case: dict) -> str:
    """The pose topic to record: the case's, else the one of its quickstart profile."""
    if case.get("pose_topic"):
        return str(case["pose_topic"])
    if quickstart_profile(list(case.get("quickstart_args", []))) in {"nav2", "mid360"}:
        return "/localization/pose_with_covariance"
    return "/pcl_pose"


def quickstart_uses_odometry(config_text: str) -> bool | None:
    """use_odom_tf_prediction of a quickstart configuration, None if not there."""
    try:
        params = yaml.safe_load(config_text)
    except yaml.YAMLError:
        return None
    stack = [params]
    while stack:
        item = stack.pop()
        if isinstance(item, dict):
            if "use_odom_tf_prediction" in item:
                return bool(item["use_odom_tf_prediction"])
            stack.extend(item.values())
    return None


def invalid_reason(run_dir: Path) -> str | None:
    path = run_dir / INVALID_FILE
    if not path.exists():
        return None
    return path.read_text(encoding="utf-8").strip() or "invalid"


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


def wait_for_odometry_tf(case: dict, env: dict) -> tuple[str | None, str]:
    """(child frame of the first odom TF, or None; the waiter's message)."""
    timeout = float(case.get("odometry_tf_timeout_sec", ODOMETRY_TF_TIMEOUT_SEC))
    try:
        result = subprocess.run(
            [
                sys.executable,
                str(TOOL_DIR / "wait_for_odometry.py"),
                "--odom-frame",
                str(case.get("odom_frame", "odom")),
                "--timeout",
                str(timeout),
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout + 30.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return None, f"odometry TF wait did not return within {timeout + 30.0:g} s"
    frame = result.stdout.strip().splitlines()
    if result.returncode == 0 and frame:
        return frame[-1], ""
    return None, (result.stderr or result.stdout).strip()


def run_once(
    case: dict, run_dir: Path, domain_id: int, quickstart_cmd: list[str]
) -> str | None:
    """Run the case once; return why the run is invalid, or None."""
    run_dir.mkdir(parents=True, exist_ok=True)
    for stale in (INVALID_FILE, "quickstart.yaml"):
        (run_dir / stale).unlink(missing_ok=True)
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    bag = Path(case["bag"])
    _, duration = bag_start_and_duration(bag)
    offset = float(case.get("start_offset_sec", 0.0))
    rate = float(case.get("rate", 1.0))

    recorder = start(
        [
            sys.executable,
            str(TOOL_DIR / "record_pose.py"),
            str(run_dir),
            pose_topic(case),
        ],
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
    play_started = time.monotonic()
    quickstart_args = list(case["quickstart_args"])
    expect_odometry = expects_odometry(case)
    invalid = None
    if expect_odometry:
        # Quickstart's own discovery can miss an odometry node DDS has not
        # reported to it yet; the benchmark must not depend on that.
        base_frame, message = wait_for_odometry_tf(case, env)
        if base_frame is None:
            invalid = f"no odometry: {message}"
        else:
            quickstart_args = odometry_quickstart_args(quickstart_args, base_frame)
    quickstart = None
    if invalid is None:
        time.sleep(max(0.0, play_started + QUICKSTART_DELAY_SEC - time.monotonic()))
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
                *quickstart_args,
            ],
            run_dir / "quickstart.log",
            env,
        )
        invalid = supervise(
            play,
            quickstart,
            run_dir / "quickstart.yaml",
            expect_odometry,
            (duration - offset) / rate + 120.0,
        )
    if invalid is None:
        time.sleep(5.0)
    for process in (play, quickstart, odometry, recorder):
        stop(process)
    if invalid:
        (run_dir / INVALID_FILE).write_text(invalid + "\n", encoding="utf-8")
    return invalid


def supervise(
    play: subprocess.Popen,
    quickstart: subprocess.Popen,
    config: Path,
    expect_odometry: bool,
    timeout_sec: float,
) -> str | None:
    """Wait for the replay to end; stop early (and return why) if the run is invalid."""
    deadline = time.monotonic() + timeout_sec
    checked = not expect_odometry
    while play.poll() is None and time.monotonic() < deadline:
        if not checked and config.exists():
            uses_odometry = quickstart_uses_odometry(config.read_text(encoding="utf-8"))
            if uses_odometry is False:
                return (
                    "no odometry: quickstart configured use_odom_tf_prediction: false"
                )
            checked = uses_odometry is not None
        if not checked and quickstart.poll() is not None:
            return f"quickstart exited with {quickstart.returncode} before configuring"
        time.sleep(0.5)
    if not checked:
        return "no odometry: quickstart wrote no configuration"
    return None


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
    odometry_log = run_dir / "odometry.log"
    if odometry_log.exists():
        result = dataclasses.replace(
            result,
            odometry_worst_loop_sec=benchmark_eval.odometry_worst_loop_sec(
                odometry_log.read_text(errors="replace")
            ),
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
    invalid: list[str] = []
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
            reason = invalid_reason(run_dir)
            if reason:
                invalid.append(f"{case['name']} run {repeat + 1}: {reason}")
                print(f"[benchmark] INVALID {invalid[-1]}", flush=True)
                continue
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
    note = benchmark_eval.stalled_odometry_note(results)
    if note:
        table += "\n\n" + note
    if invalid:
        table += "\n\nInvalid runs, not scored (re-run them):\n" + "\n".join(
            f"- {line}" for line in invalid
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
    return 1 if invalid else 0


if __name__ == "__main__":
    sys.exit(main())
