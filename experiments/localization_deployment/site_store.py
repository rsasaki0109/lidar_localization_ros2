#!/usr/bin/env python3
"""Stage validated map packages and switch versions by replacing one state file."""

import argparse
import fcntl
import hashlib
import json
import math
import re
import shutil
import tempfile
from pathlib import Path

import yaml


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_occupancy(root, occupancy_path, files):
    metadata = yaml.safe_load((root / occupancy_path).read_text())
    if not isinstance(metadata, dict) or not isinstance(metadata.get("image"), str):
        raise ValueError("Expected occupancy YAML with an image path")
    image = Path(metadata["image"])
    if image.is_absolute() or (occupancy_path.parent / image).as_posix() not in files:
        raise ValueError("The occupancy image must be a declared relative asset")
    origin = metadata.get("origin")
    resolution = metadata.get("resolution")
    if not finite_number(resolution) or resolution <= 0:
        raise ValueError("The occupancy resolution must be positive and finite")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(finite_number(value) for value in origin)
    ):
        raise ValueError("The occupancy origin must contain finite x, y and yaw values")
    free = metadata.get("free_thresh", 0.196)
    occupied = metadata.get("occupied_thresh", 0.65)
    if not (
        finite_number(free) and finite_number(occupied) and 0 <= free < occupied <= 1
    ):
        raise ValueError("Expected ordered occupancy thresholds between zero and one")


def ros_parameter_sections(document):
    if not isinstance(document, dict) or not document:
        raise ValueError("Expected ROS node mappings with ros__parameters")
    for name, node in document.items():
        if name == "ros__parameters":
            if not isinstance(node, dict):
                raise ValueError("Expected a mapping under ros__parameters")
            yield node
        else:
            yield from ros_parameter_sections(node)


def validate(root):
    definition = root / "site.json"
    if definition.resolve() != definition or not definition.is_file():
        raise ValueError("Expected a regular site.json inside the package")
    manifest = json.loads(definition.read_text())
    if not isinstance(manifest, dict) or manifest.get("schema") != 1:
        raise ValueError("Expected site manifest schema 1")
    roles = manifest["roles"]
    frames = manifest["frames"]
    if set(roles) != {"pointcloud", "occupancy", "parameters"}:
        raise ValueError("Expected pointcloud, occupancy and parameters roles")
    if (
        set(frames) != {"map", "odom", "base"}
        or any(
            not isinstance(frame, str) or not frame.strip() for frame in frames.values()
        )
        or len(set(frames.values())) != 3
    ):
        raise ValueError("Expected three distinct map, odom and base frames")
    files = manifest["sha256"]
    if "site.json" in files or not set(roles.values()) <= set(files):
        raise ValueError("The roles must refer to declared assets")
    for relative, checksum in files.items():
        path = Path(relative)
        asset = root / path
        if (
            not path.parts
            or path.is_absolute()
            or ".." in path.parts
            or path.as_posix() != relative
            or asset.resolve() != asset
            or not asset.is_file()
            or digest(asset) != checksum
        ):
            raise ValueError(f"Invalid asset or checksum: {relative}")
    if Path(roles["pointcloud"]).suffix.lower() not in {".pcd", ".ply"}:
        raise ValueError("Expected a PCD or PLY pointcloud")
    validate_occupancy(root, Path(roles["occupancy"]), files)
    parameters = yaml.safe_load((root / roles["parameters"]).read_text())
    if not isinstance(parameters, dict) or "ros__parameters" in parameters:
        raise ValueError("Expected ROS node mappings with ros__parameters")
    list(ros_parameter_sections(parameters))
    return manifest


def version_path(store, version):
    if not isinstance(version, str) or not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9._-]*", version
    ):
        raise ValueError("Invalid version name")
    target = store / "versions" / version
    if target.resolve() != target:
        raise ValueError("Version must be inside the store")
    return target


def stage(store, version, source):
    target = version_path(store, version)
    if target.exists():
        raise FileExistsError(f"Version already staged: {version}")
    manifest = validate(source)
    target.parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=target.parent) as directory:
        staging = Path(directory) / "site"
        staging.mkdir()
        for relative in ["site.json", *manifest["sha256"]]:
            path = staging / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source / relative, path)
        validate(staging)
        staging.rename(target)


def select(store, state_path, requested, rollback=False):
    state = json.loads(state_path.read_text()) if state_path.exists() else {}
    current = state.get("current")
    target = state.get("previous") if rollback else requested
    path = version_path(store, target)
    validate(path)
    if target != current:
        state = {"current": target, "previous": current}
        with tempfile.NamedTemporaryFile(mode="w", dir=store, delete=False) as stream:
            temp = Path(stream.name)
            try:
                json.dump(state, stream)
                stream.flush()
                temp.replace(state_path)
            finally:
                temp.unlink(missing_ok=True)
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--store", type=Path, required=True)
    commands = parser.add_subparsers(dest="command", required=True)
    stage_command = commands.add_parser("stage")
    stage_command.add_argument("version")
    stage_command.add_argument("source", type=Path)
    activate = commands.add_parser("activate")
    activate.add_argument("version")
    commands.add_parser("rollback")
    current = commands.add_parser("current")
    current.add_argument("--with-parameters", action="store_true")
    args = parser.parse_args()

    store = args.store.resolve()
    store.mkdir(parents=True, exist_ok=True)
    state_path = store / "state.json"
    try:
        with (store / ".lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if args.command == "stage":
                stage(store, args.version, args.source.resolve())
            elif args.command in {"activate", "rollback"}:
                state = select(
                    store,
                    state_path,
                    getattr(args, "version", None),
                    args.command == "rollback",
                )
                print(json.dumps(state))
            else:
                version = json.loads(state_path.read_text())["current"]
                asset_dir = version_path(store, version)
                manifest = validate(asset_dir)
                roles = {
                    key: str(asset_dir / val) for key, val in manifest["roles"].items()
                }
                selection = {
                    "version": version,
                    "roles": roles,
                    "frames": manifest["frames"],
                }
                if args.with_parameters:
                    parameters = yaml.safe_load(Path(roles["parameters"]).read_text())
                    for section in ros_parameter_sections(parameters):
                        if "map_path" in section:
                            section["map_path"] = roles["pointcloud"]
                    parameters.setdefault("/**", {}).setdefault("ros__parameters", {})[
                        "map_path"
                    ] = roles["pointcloud"]
                    selection["parameters_yaml"] = yaml.safe_dump(
                        parameters, sort_keys=False
                    )
                print(json.dumps(selection))
    except (
        KeyError,
        OSError,
        OverflowError,
        RecursionError,
        TypeError,
        ValueError,
        yaml.YAMLError,
    ) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
