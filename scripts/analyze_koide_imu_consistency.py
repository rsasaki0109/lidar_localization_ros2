#!/usr/bin/env python3
"""Diagnose Koide IMU axes/extrinsic/bias and cloud timestamp convention.

This is an offline, read-only analyzer.  It compares scan-interval gyro
integrals with rotations interpolated from a pose reference and writes a JSON
report suitable for repeated-run comparison.
"""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_normalize(q):
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid quaternion")
    return q / norm


def quat_slerp(q0, q1, fraction):
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return quat_normalize(q0 + fraction * (q1 - q0))
    angle = math.acos(dot)
    return (math.sin((1.0 - fraction) * angle) * q0 +
            math.sin(fraction * angle) * q1) / math.sin(angle)


def quat_to_matrix(q):
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def rotation_log(rotation):
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    vee = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    if angle < 1e-8:
        return 0.5 * vee
    return angle * vee / (2.0 * math.sin(angle))


def interpolate_quaternion(times, quaternions, stamp):
    index = int(np.searchsorted(times, stamp))
    if index == 0 or index >= len(times):
        return None
    left = index - 1
    dt = times[index] - times[left]
    if dt <= 0.0:
        return None
    return quat_slerp(quaternions[left], quaternions[index], (stamp-times[left])/dt)


def integrate_gyro(times, gyro, start, end):
    if end <= start or start < times[0] or end > times[-1]:
        return None
    inner = np.flatnonzero((times > start) & (times < end))
    sample_times = np.concatenate(([start], times[inner], [end]))
    samples = np.vstack([
        np.interp(sample_times, times, gyro[:, axis]) for axis in range(3)
    ]).T
    return np.trapezoid(samples, sample_times, axis=0)


def wahba(source, target):
    covariance = target.T @ source
    u, _, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def fit_rotation_and_bias(source_integrals, target_rotations, durations):
    rotation = np.eye(3)
    bias = np.zeros(3)
    for _ in range(10):
        corrected = source_integrals - durations[:, None] * bias
        rotation = wahba(corrected, target_rotations)
        residual_sensor_rate = (
            source_integrals - (rotation.T @ target_rotations.T).T
        ) / durations[:, None]
        next_bias = np.median(residual_sensor_rate, axis=0)
        if np.linalg.norm(next_bias - bias) < 1e-10:
            bias = next_bias
            break
        bias = next_bias
    return rotation, bias


def residual_summary(rotation, bias, source, target, durations):
    prediction = (rotation @ (source - durations[:, None] * bias).T).T
    error = prediction - target
    norms = np.linalg.norm(error / durations[:, None], axis=1)
    dots = np.sum(prediction * target, axis=1)
    denominators = np.linalg.norm(prediction, axis=1) * np.linalg.norm(target, axis=1)
    valid = denominators > 1e-8
    angular = np.degrees(np.arccos(np.clip(dots[valid] / denominators[valid], -1, 1)))
    return {
        "rate_rmse_rad_s": float(np.sqrt(np.mean(norms * norms))),
        "rate_median_rad_s": float(np.median(norms)),
        "direction_median_deg": float(np.median(angular)) if angular.size else None,
        "direction_p95_deg": float(np.percentile(angular, 95)) if angular.size else None,
    }


def right_handed_axis_permutations():
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            matrix = np.zeros((3, 3))
            for row, column in enumerate(permutation):
                matrix[row, column] = signs[row]
            if np.linalg.det(matrix) > 0.5:
                yield matrix


def rotation_angle_deg(rotation):
    return math.degrees(math.acos(float(np.clip((np.trace(rotation)-1)*0.5, -1, 1))))


def load_reference(path):
    times, quaternions = [], []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            times.append(float(row["stamp_sec"]))
            quaternions.append([
                float(row["orientation_x"]), float(row["orientation_y"]),
                float(row["orientation_z"]), float(row["orientation_w"]),
            ])
    return np.asarray(times), np.asarray(quaternions)


def point_time_duration(message):
    field = next((item for item in message.fields if item.name in ("t", "time", "timestamp")), None)
    if field is None or message.width * message.height == 0:
        return None
    formats = {2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    if field.datatype not in formats:
        return None
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": ["time"], "formats": [endian + formats[field.datatype]],
                      "offsets": [field.offset], "itemsize": message.point_step})
    values = np.frombuffer(message.data, dtype=dtype, count=message.width * message.height)["time"]
    values = values[np.isfinite(values)]
    if not values.size:
        return None
    scale = 1e-9 if field.name in ("t", "timestamp") and np.max(np.abs(values)) > 1e5 else 1.0
    return float((np.max(values) - np.min(values)) * scale)


def read_bag(uri, imu_topic, cloud_topic, max_scans):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {imu_topic, cloud_topic}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    classes = {topic: get_message(types[topic]) for topic in wanted}
    imu_times, gyro, scans = [], [], []
    imu_frames, cloud_frames = set(), set()
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        message = deserialize_message(raw, classes[topic])
        stamp = float(message.header.stamp.sec) + 1e-9 * message.header.stamp.nanosec
        if topic == imu_topic:
            imu_frames.add(message.header.frame_id)
            imu_times.append(stamp)
            gyro.append([message.angular_velocity.x, message.angular_velocity.y,
                         message.angular_velocity.z])
        elif len(scans) < max_scans:
            cloud_frames.add(message.header.frame_id)
            duration = point_time_duration(message)
            if duration is not None and duration > 0.0:
                scans.append((stamp, duration, message.header.frame_id))
    return np.asarray(imu_times), np.asarray(gyro), scans, sorted(imu_frames), sorted(cloud_frames)


def evaluate_hypothesis(name, scans, imu_times, gyro, ref_times, ref_quats):
    source, target, durations = [], [], []
    for stamp, duration, _ in scans:
        start, end = ((stamp, stamp + duration) if name == "start" else
                      (stamp - duration, stamp))
        q0 = interpolate_quaternion(ref_times, ref_quats, start)
        q1 = interpolate_quaternion(ref_times, ref_quats, end)
        integral = integrate_gyro(imu_times, gyro, start, end)
        if q0 is None or q1 is None or integral is None:
            continue
        source.append(integral)
        target.append(rotation_log(quat_to_matrix(q0).T @ quat_to_matrix(q1)))
        durations.append(end - start)
    source, target, durations = map(np.asarray, (source, target, durations))
    if len(source) < 10:
        raise RuntimeError(f"only {len(source)} usable scan intervals for {name}")
    rotation, bias = fit_rotation_and_bias(source, target, durations)
    identity = residual_summary(np.eye(3), np.zeros(3), source, target, durations)
    fitted = residual_summary(rotation, bias, source, target, durations)
    permutations = []
    for matrix in right_handed_axis_permutations():
        permutation_bias = np.median(
            (source - (matrix.T @ target.T).T) / durations[:, None], axis=0)
        summary = residual_summary(matrix, permutation_bias, source, target, durations)
        permutations.append((summary["rate_rmse_rad_s"], matrix, permutation_bias, summary))
    _, axis_matrix, axis_bias, axis_summary = min(permutations, key=lambda item: item[0])
    return {
        "usable_scan_count": int(len(source)),
        "identity": identity,
        "wahba": {
            **fitted,
            "imu_to_reference_rotation": rotation.tolist(),
            "rotation_from_identity_deg": rotation_angle_deg(rotation),
            "gyro_bias_sensor_rad_s": bias.tolist(),
        },
        "best_right_handed_signed_axis_mapping": {
            **axis_summary,
            "matrix": axis_matrix.tolist(),
            "gyro_bias_sensor_rad_s": axis_bias.tolist(),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--cloud-topic", default="/livox/points")
    parser.add_argument("--max-scans", type=int, default=2000)
    args = parser.parse_args()
    ref_times, ref_quats = load_reference(args.reference)
    imu_times, gyro, scans, imu_frames, cloud_frames = read_bag(
        args.bag, args.imu_topic, args.cloud_topic, args.max_scans)
    report = {
        "schema_version": 1,
        "bag": str(args.bag.resolve()),
        "reference": str(args.reference.resolve()),
        "imu_topic": args.imu_topic,
        "cloud_topic": args.cloud_topic,
        "imu_sample_count": int(len(imu_times)),
        "cloud_scan_count": len(scans),
        "imu_frame_ids": imu_frames,
        "cloud_frame_ids": cloud_frames,
        "imu_frame_ids_match_cloud": imu_frames == cloud_frames,
        "hypotheses": {
            name: evaluate_hypothesis(name, scans, imu_times, gyro, ref_times, ref_quats)
            for name in ("start", "end")
        },
    }
    report["preferred_cloud_stamp_reference"] = min(
        report["hypotheses"],
        key=lambda name: report["hypotheses"][name]["wahba"]["rate_rmse_rad_s"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
