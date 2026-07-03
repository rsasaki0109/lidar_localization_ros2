#!/usr/bin/env python3

import csv
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import check_koide_recovery_health as health  # noqa: E402


def _write_csv(path: Path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)


def test_passes_when_stable_window_present():
    with tempfile.TemporaryDirectory() as tmp:
        events = Path(tmp) / "events.csv"
        _write_csv(
            events,
            ["event"],
            [
                {"event": "recovery_confirmed"},
                {"event": "stable_recovered_request_window"},
            ],
        )
        summary = health.summarize_recovery_health(events)
        assert summary.ok
        assert summary.stable_recovered_request_windows == 1


def test_fails_when_only_recovery_confirmed():
    with tempfile.TemporaryDirectory() as tmp:
        events = Path(tmp) / "events.csv"
        _write_csv(events, ["event"], [{"event": "recovery_confirmed"}])
        summary = health.summarize_recovery_health(events)
        assert not summary.ok
        assert summary.false_recovery_confirmed


def test_cli_json_output():
    with tempfile.TemporaryDirectory() as tmp:
        events = Path(tmp) / "events.csv"
        out = Path(tmp) / "summary.json"
        _write_csv(
            events,
            ["event"],
            [
                {"event": "recovery_confirmed"},
                {"event": "stable_recovered_request_window"},
            ],
        )
        assert health.main([
            "--supervisor-events-csv", str(events),
            "--output-json", str(out),
        ]) == 0
        payload = json.loads(out.read_text(encoding="utf-8"))
        assert payload["ok"] is True


def test_alignment_values_json_parsing():
    with tempfile.TemporaryDirectory() as tmp:
        events = Path(tmp) / "events.csv"
        alignment = Path(tmp) / "alignment.csv"
        _write_csv(events, ["event"], [
            {"event": "recovery_confirmed"},
            {"event": "stable_recovered_request_window"},
        ])
        _write_csv(
            alignment,
            ["stamp_sec", "message", "values_json"],
            [
                {
                    "stamp_sec": "100.0",
                    "message": "ok",
                    "values_json": json.dumps({
                        "reinitialization_requested": "true",
                        "fitness_score": "2.5",
                        "tracking_ok": "false",
                    }),
                },
                {
                    "stamp_sec": "101.0",
                    "message": "ok",
                    "values_json": json.dumps({
                        "reinitialization_requested": "false",
                        "fitness_score": "0.8",
                        "failure_category": "healthy",
                    }),
                },
            ],
        )
        summary = health.summarize_recovery_health(events, alignment)
        assert summary.reinitialization_requested_rows == 1
        assert summary.ok


if __name__ == "__main__":
    test_passes_when_stable_window_present()
    test_fails_when_only_recovery_confirmed()
    test_cli_json_output()
    test_alignment_values_json_parsing()
    print("test_check_koide_recovery_health: all tests passed")
