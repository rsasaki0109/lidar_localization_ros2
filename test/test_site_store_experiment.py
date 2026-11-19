"""Exercise the experimental map-package CLI through its public commands."""

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

CLI = (
    Path(__file__).resolve().parents[1]
    / "experiments"
    / "localization_deployment"
    / "site_store.py"
)


@pytest.fixture
def package(tmp_path):
    src = tmp_path / "package"
    src.mkdir()
    assets = {
        "cloud.ply": b"ply\nformat ascii 1.0\nend_header\n",
        "map.pgm": b"P5\n1 1\n255\n\x00",
        "grid.yaml": b"image: map.pgm\nresolution: 0.2\norigin: [0, 0, 0]\n",
        "parameters.yaml": b"robot:\n  localizer:\n    ros__parameters:\n      threads: 2\n",
    }
    for name, content in assets.items():
        (src / name).write_bytes(content)
    manifest = {
        "schema": 1,
        "roles": {
            "pointcloud": "cloud.ply",
            "occupancy": "grid.yaml",
            "parameters": "parameters.yaml",
        },
        "frames": {"map": "map", "odom": "odom", "base": "base_link"},
        "sha256": {
            name: hashlib.sha256(content).hexdigest()
            for name, content in assets.items()
        },
    }
    (src / "site.json").write_text(json.dumps(manifest))
    return src


def command(store, *args, check=True):
    return subprocess.run(
        [sys.executable, str(CLI), "--store", str(store), *map(str, args)],
        capture_output=True,
        text=True,
        check=check,
    )


def rewrite_asset(package, name, content):
    (package / name).write_bytes(content)
    file = package / "site.json"
    manifest = json.loads(file.read_text())
    manifest["sha256"][name] = hashlib.sha256(content).hexdigest()
    file.write_text(json.dumps(manifest))


@pytest.mark.parametrize(
    "name,content,expected",
    [
        ("cloud.ply", b"truncated", "Invalid asset or checksum"),
        (
            "grid.yaml",
            b"image: map.pgm\nresolution: 0\norigin: [0, 0, 0]\n",
            "occupancy resolution",
        ),
        ("grid.yaml", b"image: ../map.pgm\n", "declared relative asset"),
        ("parameters.yaml", b"localizer: {threads: 2}\n", "ROS node mappings"),
    ],
)
def test_bad_package_leaves_current_version_untouched(
    tmp_path, package, name, content, expected
):
    store = tmp_path / "store"
    command(store, "stage", "v1", package)
    command(store, "activate", "v1")
    original_state = (store / "state.json").read_bytes()
    if name == "cloud.ply":
        (package / name).write_bytes(content)
    else:
        rewrite_asset(package, name, content)

    error = command(store, "stage", "v2", package, check=False)
    assert error.returncode == 2
    assert expected in error.stderr
    assert (store / "state.json").read_bytes() == original_state
    assert not (store / "versions" / "v2").exists()
    assert json.loads(command(store, "current").stdout)["version"] == "v1"


def test_activate_and_rollback_keep_the_last_valid_selection(tmp_path, package):
    store = tmp_path / "store"
    command(store, "stage", "v1", package)
    command(store, "activate", "v1")
    rewrite_asset(
        package,
        "parameters.yaml",
        b"/**:\n  ros__parameters:\n    threads: 4\n",
    )
    command(store, "stage", "v2", package)
    selected = json.loads(command(store, "activate", "v2").stdout)
    assert selected == {"current": "v2", "previous": "v1"}
    assert json.loads(command(store, "activate", "v2").stdout) == selected

    old_params = store / "versions/v1/parameters.yaml"
    original = old_params.read_bytes()
    old_params.write_bytes(b"broken")
    assert command(store, "rollback", check=False).returncode == 2
    assert json.loads(command(store, "current").stdout)["version"] == "v2"
    old_params.write_bytes(original)
    assert json.loads(command(store, "rollback").stdout) == {
        "current": "v1",
        "previous": "v2",
    }
    assert (
        Path(
            json.loads(command(store, "current").stdout)["roles"]["parameters"]
        ).read_bytes()
        == original
    )


def test_rendered_parameters_point_to_selected_package(tmp_path, package):
    store = tmp_path / "store"
    original = b"/**:\n  ros__parameters:\n    threads: 2\n    map_path: /old/map.ply\n"
    rewrite_asset(package, "parameters.yaml", original)
    command(store, "stage", "v1", package)
    command(store, "activate", "v1")
    selected = json.loads(command(store, "current", "--with-parameters").stdout)
    params = yaml.safe_load(selected["parameters_yaml"])
    assert params["/**"]["ros__parameters"] == {
        "threads": 2,
        "map_path": selected["roles"]["pointcloud"],
    }
    assert Path(selected["roles"]["parameters"]).read_bytes() == original
