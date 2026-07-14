#!/usr/bin/env python3
"""Run the offline IMU/LiDAR consistency analysis on every Koide sequence."""

import argparse
import json
from pathlib import Path
import statistics
import subprocess
import sys


SEQUENCES = {
    "indoor_easy_01": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_easy_02": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_hard_01": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_kidnap_01": ("/imu", "/points2/decompressed", "indoor"),
    "indoor_kidnap_02": ("/imu", "/points2/decompressed", "indoor"),
    "outdoor_hard_01a": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_hard_01b": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_hard_02a": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_hard_02b": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_kidnap_a": ("/livox/imu", "/livox/points", "outdoor"),
    "outdoor_kidnap_b": ("/livox/imu", "/livox/points", "outdoor"),
}


def compact_sequence_result(name, family, report):
    alignment = report["between_cloud_alignment"]
    accel = report["accelerometer"]
    extrinsic = report["static_imu_to_cloud_extrinsic"]
    return {
        "sequence": name,
        "family": family,
        "imu_frame_ids": report["imu_frame_ids"],
        "cloud_frame_ids": report["cloud_frame_ids"],
        "imu_sample_count": report["imu_sample_count"],
        "cloud_scan_count": report["cloud_scan_count"],
        "imu_period_median_sec": report["imu_timing"]["median_period_sec"],
        "cloud_period_median_sec": report["cloud_timing"]["median_period_sec"],
        "imu_header_minus_record_median_sec": report["imu_timing"][
            "median_header_minus_record_sec"],
        "cloud_header_minus_record_median_sec": report["cloud_timing"][
            "median_header_minus_record_sec"],
        "imu_minus_reference_time_offset_sec": alignment[
            "imu_minus_reference_time_offset_sec"],
        "identity_rate_rmse_rad_s": alignment["identity"]["rate_rmse_rad_s"],
        "signed_axis_rate_rmse_rad_s": alignment[
            "best_right_handed_signed_axis_mapping"]["rate_rmse_rad_s"],
        "wahba_rate_rmse_rad_s": alignment["wahba"]["rate_rmse_rad_s"],
        "signed_axis_mapping": alignment[
            "best_right_handed_signed_axis_mapping"]["matrix"],
        "imu_to_reference_rotation": alignment["wahba"][
            "imu_to_reference_rotation"],
        "rotation_from_identity_deg": alignment["wahba"][
            "rotation_from_identity_deg"],
        "gyro_bias_sensor_rad_s": alignment["wahba"]["gyro_bias_sensor_rad_s"],
        "static_extrinsic_difference_deg": (
            extrinsic["fitted_rotation_difference_deg"] if extrinsic else None),
        "accel_units": accel["input_units"],
        "accel_scale_to_m_s2": accel["scale_to_m_s2"],
        "gravity_world_sign": accel.get("gravity_world_sign"),
        "gravity_direction_sensor": accel.get("gravity_direction_sensor"),
        "apparent_accel_bias_sensor_m_s2": accel.get(
            "apparent_accel_bias_sensor_m_s2"),
        "accel_quiet_residual_rmse_m_s2": accel.get(
            "quiet_corrected_residual_rmse_m_s2"),
        "point_time_available": bool(report["point_time_hypotheses"]),
    }


def aggregate_results(results):
    aggregate = {}
    for family in sorted({result["family"] for result in results}):
        selected = [result for result in results if result["family"] == family]
        variants = {}
        for variant, field in (
            ("identity", "identity_rate_rmse_rad_s"),
            ("signed_axis", "signed_axis_rate_rmse_rad_s"),
            ("wahba", "wahba_rate_rmse_rad_s"),
        ):
            values = [result[field] for result in selected]
            variants[variant] = {
                "median_rate_rmse_rad_s": statistics.median(values),
                "worst_rate_rmse_rad_s": max(values),
            }
        aggregate[family] = {
            "sequence_count": len(selected),
            "variants": variants,
            "max_abs_time_offset_sec": max(
                abs(result["imu_minus_reference_time_offset_sec"])
                for result in selected),
            "accel_units": sorted({result["accel_units"] for result in selected}),
            "gravity_world_signs": sorted({
                result["gravity_world_sign"] for result in selected
                if result["gravity_world_sign"] is not None}),
            "max_static_extrinsic_difference_deg": max(
                (result["static_extrinsic_difference_deg"] for result in selected
                 if result["static_extrinsic_difference_deg"] is not None),
                default=None),
        }
    return aggregate


def main():
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-scans", type=int, default=0,
                        help="0 analyzes every cloud scan")
    parser.add_argument("--sequence", action="append", choices=SEQUENCES)
    parser.add_argument(
        "--analyzer", type=Path,
        default=repo / "scripts" / "analyze_koide_imu_consistency.py")
    args = parser.parse_args()

    names = args.sequence or list(SEQUENCES)
    assets = args.data_root / "generated" / "localization_gif_benchmarks" / "assets"
    details = args.output_dir / "sequences"
    details.mkdir(parents=True, exist_ok=True)
    compact = []
    for name in names:
        imu_topic, cloud_topic, family = SEQUENCES[name]
        output = details / f"{name}.json"
        command = [
            sys.executable, str(args.analyzer),
            "--bag", str(args.data_root / "sequences" / name),
            "--reference", str(assets / f"{name}_reference.csv"),
            "--output", str(output),
            "--imu-topic", imu_topic,
            "--cloud-topic", cloud_topic,
            "--max-scans", str(args.max_scans),
        ]
        print(f"Analyzing {name}...", flush=True)
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        compact.append(compact_sequence_result(name, family, json.loads(output.read_text())))

    summary = {
        "schema_version": 1,
        "purpose": "Koide local yaw prediction calibration; no global localization",
        "sequence_count": len(compact),
        "all_expected_sequences_analyzed": set(names) == set(SEQUENCES),
        "sequences": compact,
        "families": aggregate_results(compact),
    }
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(summary_path)


if __name__ == "__main__":
    main()
