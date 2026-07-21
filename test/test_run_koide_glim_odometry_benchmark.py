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


def test_recovery_safety_gate_subset_ignores_expected_accuracy_failures():
    gates = {name: True for name in RUNNER["RECOVERY_SAFETY_GATES"]}
    gates.update({"coverage": False, "translation_ate": False, "rpe_translation": False})
    assert RUNNER["failed_recovery_safety_gates"]({"gates": gates}) == []
    gates["bounded_queue"] = False
    assert RUNNER["failed_recovery_safety_gates"]({"gates": gates}) == ["bounded_queue"]


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


def test_indoor_sensor_profile_sets_topics_extrinsic_and_voxels(tmp_path):
    configs = {
        "config_ros.json": {"glim_ros": {}},
        "config_sensors.json": {"sensors": {}},
        "config_preprocess.json": {"preprocess": {}},
        "config_odometry_cpu.json": {"odometry_estimation": {}},
    }
    for name, value in configs.items():
        (tmp_path / name).write_text(json.dumps(value))

    RUNNER["apply_sensor_profile"](tmp_path, "indoor_azure_kinect")
    ros = json.loads((tmp_path / "config_ros.json").read_text())["glim_ros"]
    assert ros["imu_topic"] == "/imu"
    assert ros["points_topic"] == "/points2/decompressed"
    sensors = json.loads(
        (tmp_path / "config_sensors.json").read_text())["sensors"]
    assert sensors["global_shutter_lidar"] is True
    assert sensors["T_lidar_imu"] == RUNNER["INDOOR_AZURE_KINECT_T_LIDAR_IMU"]
    preprocess = json.loads(
        (tmp_path / "config_preprocess.json").read_text())["preprocess"]
    assert preprocess["downsample_resolution"] == 0.25
    odometry = json.loads(
        (tmp_path / "config_odometry_cpu.json").read_text())["odometry_estimation"]
    assert odometry["ivox_resolution"] == 0.5


def test_recovery_sidecar_command_is_gt_free_bounded_and_self_cleaning():
    command = RUNNER["recovery_sidecar_command"](
        reset_z_m=-11.3,
        max_scan_points=256,
        angular_resolution_deg=10.0,
        max_candidates=8,
        max_attempts=3,
        max_walk_candidates=1,
    )
    assert "/glil/recovery_points" in command
    assert "request_clear_confirms_recovery:=true" in command
    assert "max_attempts:=3" in command
    assert "max_walk_candidates:=1" in command
    assert "angular_resolution_deg:=10.0" in command
    assert "reset_default_z_m:=-11.3" in command
    assert "recovery_g2_pid=$!" in command
    assert "recovery_supervisor_pid=$!" in command
