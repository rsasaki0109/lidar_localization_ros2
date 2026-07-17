#!/usr/bin/env python3

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import summarize_localization_health as health  # noqa: E402


def test_empty_rows_summary_has_full_key_set():
    empty = health.summarize_alignment([], stable_ok_rows_required=5, recovery_window_sec=10.0)
    assert empty["row_count"] == 0
    assert empty["reinitialization_recovered_count"] == 0
    assert empty["reinitialization_unrecovered_count"] == 0
    assert empty["ok_rate_percent"] is None
    assert empty["failure_like_rate_percent"] is None
    assert empty["request_reason_counts"] == {}


def test_empty_rows_summary_renders_markdown(tmp_path):
    alignment_csv = tmp_path / "alignment_status.csv"
    alignment_csv.write_text(
        "message_index,stamp_sec,status_index,level,name,message,hardware_id,values_json\n",
        encoding="utf-8",
    )
    summary = health.build_summary(
        alignment_csv=alignment_csv,
        trajectory_eval_json=None,
        stable_ok_rows_required=5,
        recovery_window_sec=10.0,
        false_recovery_rmse_threshold_m=5.0,
    )
    markdown = health.build_markdown(summary)
    assert "rows: `0`" in markdown
