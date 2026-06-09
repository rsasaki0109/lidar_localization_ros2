#!/usr/bin/env python3

import argparse
import re
from pathlib import Path
from typing import Iterable
from typing import List
from typing import Tuple


FORBIDDEN_PATTERNS = [
    re.compile(r"runtime relocalization solved", re.IGNORECASE),
    re.compile(r"automatic relocalization", re.IGNORECASE),
    re.compile(r"automatic recovery works", re.IGNORECASE),
    re.compile(r"production-ready global relocalization", re.IGNORECASE),
    re.compile(r"production-grade recovery", re.IGNORECASE),
    re.compile(r"global relocalization solved", re.IGNORECASE),
]

SCAN_FILES = [
    "README.md",
    "CHANGELOG.md",
    "docs/v1_status.md",
    "docs/benchmarking.md",
    "docs/public_validation_log.md",
]

EXCLUDED_PATHS = {
    "docs/v1_1_relocalization.md",
    "docs/competitive_roadmap.md",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fail if user-facing docs contain unsafe v1.1 relocalization claims."
    )
    parser.add_argument("--repo-root", required=True)
    return parser.parse_args()


def iter_target_files(repo_root: Path) -> Iterable[Path]:
    for relative in SCAN_FILES:
        path = repo_root / relative
        if path.exists():
            yield path


NEGATION_MARKERS = (
    "do not claim",
    "does not claim",
    "not claim",
    "out of scope",
    "avoid claiming",
    "do not treat",
    "is not ",
    "are not ",
    "not a ",
    "not an ",
    "not production",
    "not automatic",
)


def in_do_not_claim_section(lines: List[str], line_index: int) -> bool:
    for idx in range(line_index, -1, -1):
        stripped = lines[idx].strip().lower()
        if stripped.startswith("## "):
            return "do not claim" in stripped
    return False


def scan_file(path: Path) -> List[Tuple[int, str, str]]:
    hits: List[Tuple[int, str, str]] = []
    lines = path.read_text(encoding="utf-8").splitlines()
    for line_no, line in enumerate(lines, start=1):
        lowered = line.lower()
        if in_do_not_claim_section(lines, line_no - 1):
            continue
        if any(marker in lowered for marker in NEGATION_MARKERS):
            continue
        for pattern in FORBIDDEN_PATTERNS:
            if pattern.search(line):
                hits.append((line_no, pattern.pattern, line.strip()))
    return hits


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    all_hits: List[Tuple[str, int, str, str]] = []

    for path in iter_target_files(repo_root):
        if path.relative_to(repo_root).as_posix() in EXCLUDED_PATHS:
            continue
        for line_no, pattern, line in scan_file(path):
            all_hits.append((str(path.relative_to(repo_root)), line_no, pattern, line))

    if all_hits:
        print("Unsafe v1.1 claim language found:", flush=True)
        for rel_path, line_no, pattern, line in all_hits:
            print(f"- {rel_path}:{line_no} [{pattern}] {line}", flush=True)
        return 1

    print("claim-language check: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
