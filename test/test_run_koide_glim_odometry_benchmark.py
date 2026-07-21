import csv
import json
import runpy
from pathlib import Path

import pytest


RUNNER = runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "scripts/run_koide_glim_odometry_benchmark.py"),
    run_name="koide_glim_runner_test",
)


def test_parse_docker_memory_units():
    parse = RUNNER["parse_size_bytes"]
    assert parse("128MiB / 8GiB") == 128 * 1024**2
    assert parse("1.5 GB / 8 GB") == 1_500_000_000
    assert parse("512 kB / 8 GB") == 512_000
    with pytest.raises(ValueError):
        parse("not available")


def test_percentile_uses_nearest_ordered_sample():
    percentile = RUNNER["percentile"]
    assert percentile([], 0.95) == 0.0
    assert percentile([4.0, 1.0, 3.0, 2.0], 0.5) == 3.0
    assert percentile([4.0, 1.0, 3.0, 2.0], 0.95) == 4.0


def test_resource_evidence_records_cpu_memory_and_rss(tmp_path):
    samples = [
        {"elapsed_sec": 1.0, "cpu_percent": 75.0,
         "memory_bytes": 1000, "rss_bytes": 700},
        {"elapsed_sec": 2.0, "cpu_percent": 125.0,
         "memory_bytes": 2000, "rss_bytes": 1500},
    ]
    summary = RUNNER["write_resource_evidence"](tmp_path, samples)
    assert summary["sample_count"] == 2
    assert summary["cpu_percent_mean"] == 100.0
    assert summary["cpu_percent_p95"] == 125.0
    assert summary["memory_peak_bytes"] == 2000
    assert summary["rss_peak_bytes"] == 1500

    with (tmp_path / "resource_trace.csv").open(newline="") as stream:
        assert list(csv.DictReader(stream))[-1]["rss_bytes"] == "1500"
    persisted = json.loads((tmp_path / "resource_summary.json").read_text())
    assert persisted == summary
