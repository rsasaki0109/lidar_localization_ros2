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


def cumulative_trapezoid(times, values):
    """Return an integral lookup table with the same length as the samples."""
    if len(times) != len(values) or len(times) < 2:
        raise ValueError("at least two timestamped samples are required")
    dt = np.diff(times)
    if np.any(dt <= 0.0):
        raise ValueError("sample timestamps must be strictly increasing")
    cumulative = np.zeros_like(values, dtype=float)
    cumulative[1:] = np.cumsum(
        0.5 * (values[:-1] + values[1:]) * dt[:, None], axis=0)
    return cumulative


def interpolate_vectors(times, values, query):
    query = np.asarray(query)
    return np.column_stack([
        np.interp(query, times, values[:, axis]) for axis in range(values.shape[1])
    ])


def integrate_from_cumulative(times, cumulative, starts, ends):
    starts = np.asarray(starts)
    ends = np.asarray(ends)
    valid = ((ends > starts) & (starts >= times[0]) & (ends <= times[-1]))
    result = np.full((len(starts), cumulative.shape[1]), np.nan)
    if np.any(valid):
        result[valid] = (
            interpolate_vectors(times, cumulative, ends[valid]) -
            interpolate_vectors(times, cumulative, starts[valid]))
    return result, valid


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
    missing = [topic for topic in (imu_topic, cloud_topic) if topic not in types]
    if missing:
        raise RuntimeError(f"bag is missing required topics: {', '.join(missing)}")
    wanted = {imu_topic, cloud_topic}
    if "/tf_static" in types:
        wanted.add("/tf_static")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    classes = {topic: get_message(types[topic]) for topic in wanted}
    imu_times, gyro, accel, scans = [], [], [], []
    imu_header_minus_record, cloud_header_minus_record = [], []
    imu_frames, cloud_frames = set(), set()
    static_transforms = []
    while reader.has_next():
        topic, raw, record_stamp_ns = reader.read_next()
        message = deserialize_message(raw, classes[topic])
        if topic == "/tf_static":
            for transform in message.transforms:
                static_transforms.append({
                    "parent": transform.header.frame_id,
                    "child": transform.child_frame_id,
                    "translation": [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ],
                    "rotation_xyzw": [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ],
                })
            continue
        stamp = float(message.header.stamp.sec) + 1e-9 * message.header.stamp.nanosec
        if topic == imu_topic:
            imu_frames.add(message.header.frame_id)
            imu_times.append(stamp)
            gyro.append([message.angular_velocity.x, message.angular_velocity.y,
                         message.angular_velocity.z])
            accel.append([message.linear_acceleration.x, message.linear_acceleration.y,
                          message.linear_acceleration.z])
            imu_header_minus_record.append(stamp - 1e-9 * record_stamp_ns)
        elif max_scans <= 0 or len(scans) < max_scans:
            cloud_frames.add(message.header.frame_id)
            duration = point_time_duration(message)
            scans.append((stamp, duration, message.header.frame_id))
            cloud_header_minus_record.append(stamp - 1e-9 * record_stamp_ns)
    return {
        "imu_times": np.asarray(imu_times),
        "gyro": np.asarray(gyro),
        "accel": np.asarray(accel),
        "scans": scans,
        "imu_frames": sorted(imu_frames),
        "cloud_frames": sorted(cloud_frames),
        "imu_header_minus_record": np.asarray(imu_header_minus_record),
        "cloud_header_minus_record": np.asarray(cloud_header_minus_record),
        "static_transforms": static_transforms,
    }


def timing_summary(times, header_minus_record):
    delta = np.diff(times)
    positive_delta = delta[delta > 0.0]
    return {
        "sample_count": int(len(times)),
        "median_period_sec": float(np.median(positive_delta)) if positive_delta.size else None,
        "p95_period_sec": float(np.percentile(positive_delta, 95)) if positive_delta.size else None,
        "non_monotonic_count": int(np.count_nonzero(delta <= 0.0)),
        "median_header_minus_record_sec": (
            float(np.median(header_minus_record)) if header_minus_record.size else None),
        "p95_abs_header_minus_record_sec": (
            float(np.percentile(np.abs(header_minus_record), 95))
            if header_minus_record.size else None),
    }


def accelerometer_unit_scale(accel):
    median_norm = float(np.median(np.linalg.norm(accel, axis=1)))
    if 0.5 <= median_norm <= 1.5:
        return "g", 9.80665, median_norm
    if 5.0 <= median_norm <= 15.0:
        return "m_s2", 1.0, median_norm
    return "unknown", None, median_norm


def accelerometer_summary(
        imu_times, gyro, accel, ref_times, ref_quats,
        imu_to_reference_rotation, imu_minus_reference_offset_sec):
    units, scale, median_norm = accelerometer_unit_scale(accel)
    summary = {
        "input_units": units,
        "scale_to_m_s2": scale,
        "median_norm_input_units": median_norm,
    }
    if scale is None:
        summary["status"] = "unknown_acceleration_units"
        return summary

    count = min(len(imu_times), 5000)
    indices = np.linspace(0, len(imu_times) - 1, count, dtype=int)
    sample_times = imu_times[indices] - imu_minus_reference_offset_sec
    sample_accel = accel[indices] * scale
    sample_gyro = gyro[indices]
    quaternions = []
    valid_indices = []
    for local_index, stamp in enumerate(sample_times):
        quaternion = interpolate_quaternion(ref_times, ref_quats, stamp)
        if quaternion is not None:
            quaternions.append(quaternion)
            valid_indices.append(local_index)
    if len(valid_indices) < 100:
        summary["status"] = "insufficient_reference_overlap"
        summary["usable_sample_count"] = len(valid_indices)
        return summary

    sample_accel = sample_accel[valid_indices]
    sample_gyro = sample_gyro[valid_indices]
    reference_rotations = np.asarray([quat_to_matrix(q) for q in quaternions])
    sensor_to_reference = np.asarray(imu_to_reference_rotation)
    measured_world = np.einsum(
        "nij,nj->ni", reference_rotations,
        (sensor_to_reference @ sample_accel.T).T)

    gyro_norm = np.linalg.norm(sample_gyro, axis=1)
    accel_norm_error = np.abs(np.linalg.norm(sample_accel, axis=1) - 9.80665)
    quiet_metric = gyro_norm + 0.1 * accel_norm_error
    quiet_count = max(100, len(quiet_metric) // 5)
    quiet = np.argsort(quiet_metric)[:quiet_count]
    gravity_sign = 1.0 if np.median(measured_world[quiet, 2]) >= 0.0 else -1.0
    expected_world = np.array([0.0, 0.0, gravity_sign * 9.80665])
    expected_reference = np.einsum(
        "nji,j->ni", reference_rotations, expected_world)
    expected_sensor = (sensor_to_reference.T @ expected_reference.T).T
    residual_sensor = sample_accel - expected_sensor
    apparent_bias = np.median(residual_sensor[quiet], axis=0)
    corrected_residual = residual_sensor - apparent_bias

    summary.update({
        "status": "ok",
        "usable_sample_count": len(valid_indices),
        "quiet_sample_count": int(quiet_count),
        "gravity_world_sign": "+Z" if gravity_sign > 0.0 else "-Z",
        "gravity_direction_sensor": (
            np.median(expected_sensor[quiet], axis=0) / 9.80665).tolist(),
        "apparent_accel_bias_sensor_m_s2": apparent_bias.tolist(),
        "quiet_corrected_residual_rmse_m_s2": float(np.sqrt(np.mean(
            np.sum(corrected_residual[quiet] ** 2, axis=1)))),
    })
    return summary


def static_imu_to_cloud_transform(static_transforms, imu_frames, cloud_frames):
    for transform in static_transforms:
        if transform["parent"] in cloud_frames and transform["child"] in imu_frames:
            return {
                **transform,
                "imu_to_cloud_rotation": quat_to_matrix(
                    transform["rotation_xyzw"]).tolist(),
            }
        if transform["parent"] in imu_frames and transform["child"] in cloud_frames:
            rotation = quat_to_matrix(transform["rotation_xyzw"])
            return {
                **transform,
                "imu_to_cloud_rotation": rotation.T.tolist(),
                "inverted": True,
            }
    return None


def evaluate_hypothesis(name, scans, imu_times, gyro, ref_times, ref_quats):
    source, target, durations = [], [], []
    for stamp, duration, _ in scans:
        if duration is None or duration <= 0.0:
            continue
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


def evaluate_between_cloud_stamps(
        scans, imu_times, gyro, ref_times, ref_quats,
        offset_min_sec=-0.25, offset_max_sec=0.25, offset_step_sec=0.005):
    """Fit gyro axes/bias and IMU-reference time offset between cloud stamps.

    Unlike the per-point scan-time hypotheses, this works for clouds without a
    time field and directly measures the rotation seed needed by local tracking.
    A positive offset means the IMU timestamps are later than the matching
    LiDAR/reference timestamps.
    """
    stamps = np.asarray([scan[0] for scan in scans])
    starts, ends, target, durations = [], [], [], []
    for start, end in zip(stamps[:-1], stamps[1:]):
        duration = end - start
        if duration <= 0.0 or duration > 0.5:
            continue
        q0 = interpolate_quaternion(ref_times, ref_quats, start)
        q1 = interpolate_quaternion(ref_times, ref_quats, end)
        if q0 is None or q1 is None:
            continue
        starts.append(start)
        ends.append(end)
        target.append(rotation_log(quat_to_matrix(q0).T @ quat_to_matrix(q1)))
        durations.append(duration)
    starts = np.asarray(starts)
    ends = np.asarray(ends)
    target = np.asarray(target)
    durations = np.asarray(durations)
    if len(starts) < 10:
        raise RuntimeError(f"only {len(starts)} usable between-cloud intervals")

    cumulative = cumulative_trapezoid(imu_times, gyro)
    candidates = []
    offsets = np.arange(
        offset_min_sec, offset_max_sec + 0.5 * offset_step_sec, offset_step_sec)
    for offset in offsets:
        source, valid = integrate_from_cumulative(
            imu_times, cumulative, starts + offset, ends + offset)
        if np.count_nonzero(valid) < 10:
            continue
        candidate_source = source[valid]
        candidate_target = target[valid]
        candidate_durations = durations[valid]
        rotation, bias = fit_rotation_and_bias(
            candidate_source, candidate_target, candidate_durations)
        residual = residual_summary(
            rotation, bias, candidate_source, candidate_target, candidate_durations)
        candidates.append((
            residual["rate_rmse_rad_s"], float(offset), candidate_source,
            candidate_target, candidate_durations, rotation, bias, residual))
    if not candidates:
        raise RuntimeError("no time-offset candidate overlaps the IMU stream")

    _, offset, source, target, durations, rotation, bias, fitted = min(
        candidates, key=lambda item: item[0])
    identity = residual_summary(np.eye(3), np.zeros(3), source, target, durations)
    permutations = []
    for matrix in right_handed_axis_permutations():
        permutation_bias = np.median(
            (source - (matrix.T @ target.T).T) / durations[:, None], axis=0)
        summary = residual_summary(matrix, permutation_bias, source, target, durations)
        permutations.append((summary["rate_rmse_rad_s"], matrix, permutation_bias, summary))
    _, axis_matrix, axis_bias, axis_summary = min(permutations, key=lambda item: item[0])
    return {
        "usable_interval_count": int(len(source)),
        "imu_minus_reference_time_offset_sec": offset,
        "offset_search": {
            "min_sec": offset_min_sec,
            "max_sec": offset_max_sec,
            "step_sec": offset_step_sec,
            "candidate_count": len(candidates),
        },
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
    parser.add_argument("--time-offset-min-sec", type=float, default=-0.25)
    parser.add_argument("--time-offset-max-sec", type=float, default=0.25)
    parser.add_argument("--time-offset-step-sec", type=float, default=0.005)
    args = parser.parse_args()
    ref_times, ref_quats = load_reference(args.reference)
    bag = read_bag(
        args.bag, args.imu_topic, args.cloud_topic, args.max_scans)
    interval_alignment = evaluate_between_cloud_stamps(
        bag["scans"], bag["imu_times"], bag["gyro"], ref_times, ref_quats,
        args.time_offset_min_sec, args.time_offset_max_sec, args.time_offset_step_sec)
    point_time_hypotheses = {}
    if sum(scan[1] is not None and scan[1] > 0.0 for scan in bag["scans"]) >= 10:
        point_time_hypotheses = {
            name: evaluate_hypothesis(
                name, bag["scans"], bag["imu_times"], bag["gyro"],
                ref_times, ref_quats)
            for name in ("start", "end")
        }
    imu_offset = interval_alignment["imu_minus_reference_time_offset_sec"]
    accel = accelerometer_summary(
        bag["imu_times"], bag["gyro"], bag["accel"], ref_times, ref_quats,
        interval_alignment["wahba"]["imu_to_reference_rotation"], imu_offset)
    static_extrinsic = static_imu_to_cloud_transform(
        bag["static_transforms"], bag["imu_frames"], bag["cloud_frames"])
    if static_extrinsic is not None:
        fitted_rotation = np.asarray(
            interval_alignment["wahba"]["imu_to_reference_rotation"])
        static_rotation = np.asarray(static_extrinsic["imu_to_cloud_rotation"])
        static_extrinsic["fitted_rotation_difference_deg"] = rotation_angle_deg(
            static_rotation.T @ fitted_rotation)
    cloud_times = np.asarray([scan[0] for scan in bag["scans"]])
    report = {
        "schema_version": 2,
        "bag": str(args.bag.resolve()),
        "reference": str(args.reference.resolve()),
        "imu_topic": args.imu_topic,
        "cloud_topic": args.cloud_topic,
        "imu_sample_count": int(len(bag["imu_times"])),
        "cloud_scan_count": len(bag["scans"]),
        "imu_frame_ids": bag["imu_frames"],
        "cloud_frame_ids": bag["cloud_frames"],
        "imu_frame_ids_match_cloud": bag["imu_frames"] == bag["cloud_frames"],
        "static_imu_to_cloud_extrinsic": static_extrinsic,
        "imu_timing": timing_summary(
            bag["imu_times"], bag["imu_header_minus_record"]),
        "cloud_timing": timing_summary(
            cloud_times, bag["cloud_header_minus_record"]),
        "between_cloud_alignment": interval_alignment,
        "accelerometer": accel,
        "point_time_hypotheses": point_time_hypotheses,
    }
    report["preferred_cloud_stamp_reference"] = (
        min(point_time_hypotheses,
            key=lambda name: point_time_hypotheses[name]["wahba"]["rate_rmse_rad_s"])
        if point_time_hypotheses else "unavailable_no_point_time_field")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
