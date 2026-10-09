"""Preparation and map-frame scoring for the official Koide public bag demo."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import urllib.request
import zipfile
from pathlib import Path

SOURCE = "https://zenodo.org/records/10122133"
FILES = {
    "outdoor_hard_02b.zip": "267377a88402f825ab70a161cc48983c",
    "map_outdoor_hard.ply": "562b607124bc31bbfc4c96eaaf47bbfb",
    "gt.zip": "ab2a80dc1d06767b7c3ce1893f3020b0",
}
POINTS = "/livox/points"
IMU = "/livox/imu"
G = 9.80665


def checksum(path: Path, algorithm="md5") -> str:
    with path.open("rb") as stream:
        return stream_checksum(stream, algorithm)


def stream_checksum(stream, algorithm="sha256") -> str:
    digest = hashlib.new(algorithm)
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def extract_archive(archive: Path, destination: Path) -> None:
    with zipfile.ZipFile(archive) as zipped:
        for name in zipped.namelist():
            target = (destination / name).resolve()
            if not target.is_relative_to(destination.resolve()):
                raise ValueError(
                    f"archive member is outside the data directory: {name}"
                )
        zipped.extractall(destination)


def prepare_assets(data: Path, download: bool) -> tuple[Path, Path, Path]:
    data.mkdir(parents=True, exist_ok=True)
    for name, expected in FILES.items():
        path = data / name
        if not path.is_file():
            if not download:
                raise ValueError(
                    f"missing {path}; use --download to fetch official data"
                )
            print(f"Downloading {name} from {SOURCE}", flush=True)
            partial = path.with_name(name + ".partial")
            with (
                urllib.request.urlopen(
                    f"{SOURCE}/files/{name}?download=1", timeout=60
                ) as response,
                partial.open("wb") as stream,
            ):
                shutil.copyfileobj(response, stream)
            if checksum(partial) != expected:
                partial.unlink()
                raise ValueError(
                    f"checksum mismatch for downloaded {name}; retry --download"
                )
            partial.replace(path)
        if checksum(path) != expected:
            raise ValueError(
                f"checksum mismatch for {path}; move it aside and retry --download"
            )
    raw = data / "outdoor_hard_02b"
    reference = data / "gt" / "traj_lidar_outdoor_hard_02.txt"
    if not (raw / "metadata.yaml").is_file():
        extract_archive(data / "outdoor_hard_02b.zip", data)
    if not reference.is_file():
        extract_archive(data / "gt.zip", data)
    # Cached extracted files must still match the verified official archives.
    for archive in (data / "outdoor_hard_02b.zip", data / "gt.zip"):
        with zipfile.ZipFile(archive) as zipped:
            for name in zipped.namelist():
                if name.endswith("/") or (
                    archive.name == "gt.zip"
                    and name != "gt/traj_lidar_outdoor_hard_02.txt"
                ):
                    continue
                with zipped.open(name) as stream:
                    expected = stream_checksum(stream)
                if checksum(data / name, "sha256") != expected:
                    raise ValueError(
                        f"extracted file differs from official archive: {data / name}; move it aside and retry"
                    )
    return raw, data / "map_outdoor_hard.ply", reference


def scale_acceleration(message) -> None:
    """Koide stores acceleration in g; covariances scale by g squared."""
    a = message.linear_acceleration
    a.x, a.y, a.z = a.x * G, a.y * G, a.z * G
    if message.linear_acceleration_covariance[0] != -1.0:
        message.linear_acceleration_covariance = [
            value * G * G for value in message.linear_acceleration_covariance
        ]


def prepare_si_bag(raw: Path, destination: Path) -> Path:
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Imu

    source_hash = checksum(raw / "rosbag2_2023_09_13-00_53_37_0.db3", "sha256")
    provenance = {
        "source_db_sha256": source_hash,
        "imu_scale": G,
        "topics": [POINTS, IMU],
        "tf_messages": 0,
    }
    marker = destination / "preparation.json"
    if destination.exists():
        if marker.is_file() and (destination / "metadata.yaml").is_file():
            cached = json.loads(marker.read_text())
            if (
                all(cached.get(key) == value for key, value in provenance.items())
                and cached.get("prepared_files")
                and all(
                    checksum(destination / name, "sha256") == digest
                    for name, digest in cached["prepared_files"].items()
                )
            ):
                return destination
        raise ValueError(
            f"unrecognized prepared bag {destination}; move it aside before retrying"
        )
    partial = destination.with_name(destination.name + ".partial")
    if partial.exists():
        raise ValueError(
            f"incomplete conversion at {partial}; move it aside before retrying"
        )
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(raw), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[POINTS, IMU]))
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(partial), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in reader.get_all_topics_and_types():
        if topic.name in (POINTS, IMU):
            writer.create_topic(topic)
    print(
        "Preparing SI IMU bag (PointCloud2 bytes are preserved; no TF added)",
        flush=True,
    )
    while reader.has_next():
        topic, raw_message, timestamp = reader.read_next()
        if topic == IMU:
            message = deserialize_message(raw_message, Imu)
            scale_acceleration(message)
            raw_message = serialize_message(message)
        writer.write(topic, raw_message, timestamp)
    del writer
    provenance["prepared_files"] = {
        path.name: checksum(path, "sha256")
        for path in [partial / "metadata.yaml", *partial.glob("*.db3")]
    }
    (partial / "preparation.json").write_text(json.dumps(provenance, indent=2) + "\n")
    partial.replace(destination)
    return destination


def bag_cloud_times(bag: Path) -> tuple[int, dict[int, int]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    info = rosbag2_py.Info().read_metadata(str(bag), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[POINTS]))
    times = {}
    while reader.has_next():
        _, raw, received = reader.read_next()
        cloud = deserialize_message(raw, PointCloud2)
        times[stamp_ns(cloud.header.stamp)] = received
    return info.starting_time.nanoseconds, times


def stamp_ns(stamp) -> int:
    return stamp.sec * 10**9 + stamp.nanosec


def evaluate(poses, reference, expected_stamps, diagnostic_stamps) -> dict:
    import numpy as np

    rows = np.asarray(poses, dtype=float)
    gt = np.asarray(reference, dtype=float)
    if len(rows) < 2 or len(gt) < 2 or not expected_stamps:
        raise ValueError("insufficient poses, ground truth or input clouds to evaluate")
    if (
        rows.ndim != 2
        or gt.ndim != 2
        or rows.shape[1] != 8
        or gt.shape[1] != 8
        or not np.isfinite(rows).all()
        or not np.isfinite(gt).all()
    ):
        raise ValueError("poses and ground truth must contain finite TUM rows")
    gt = gt[np.argsort(gt[:, 0])]
    rows = rows[np.argsort(rows[:, 0])]
    # No spatial alignment: both pose and GT are already in the map frame.
    index = np.clip(np.searchsorted(gt[:, 0], rows[:, 0]), 1, len(gt) - 1)
    earlier = abs(gt[index - 1, 0] - rows[:, 0]) < abs(gt[index, 0] - rows[:, 0])
    index = np.where(earlier, index - 1, index)
    matched = abs(gt[index, 0] - rows[:, 0]) < 0.15
    if not matched.any():
        raise ValueError("no poses match ground truth within 0.15 s")
    errors = np.linalg.norm(rows[matched, 1:4] - gt[index[matched], 1:4], axis=1)
    coverage = len(set(diagnostic_stamps) & set(expected_stamps)) / len(expected_stamps)
    result = {
        "poses": len(rows),
        "matched_poses": int(matched.sum()),
        "unique_matched_pose_stamps": len(np.unique(rows[matched, 0])),
        "rmse_m": float(np.sqrt(np.mean(errors**2))),
        "p95_error_m": float(np.percentile(errors, 95)),
        "max_output_gap_sec": float(np.diff(rows[:, 0]).max()),
        "diagnostic_clouds": len(set(diagnostic_stamps) & set(expected_stamps)),
        "expected_clouds": len(expected_stamps),
        "diagnostic_coverage": coverage,
    }
    result["passed"] = (
        math.isfinite(result["rmse_m"])
        and result["rmse_m"] <= 0.35
        and coverage >= 0.97
        and result["unique_matched_pose_stamps"] >= len(expected_stamps) * 0.95
    )
    return result
