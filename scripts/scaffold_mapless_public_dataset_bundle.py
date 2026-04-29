#!/usr/bin/env python3

import argparse
import json
import os
import shlex
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Optional
from typing import Tuple

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a reusable bundle for a public LiDAR+IMU dataset that has no packaged pointcloud map. "
            "The bundle separates mapping and localization runs."
        )
    )
    parser.add_argument("--spec", required=True, help="Path to the bundle spec YAML")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional override for the generated bundle directory",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="Print resolved outputs without writing files",
    )
    return parser.parse_args()


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected a YAML mapping in {path}")
    return data


def write_text(path: Path, content: str, print_only: bool) -> None:
    if print_only:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def write_yaml(path: Path, data: Dict[str, Any], print_only: bool) -> None:
    if print_only:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)


def resolve_path(raw: Optional[str], base_dir: Path) -> Optional[Path]:
    if raw is None:
        return None
    raw_str = os.path.expandvars(os.path.expanduser(str(raw)))
    if raw_str.startswith("repo://"):
        expanded = (REPO_ROOT / raw_str[len("repo://") :]).resolve()
    elif raw_str.startswith("manifest://"):
        expanded = (base_dir / raw_str[len("manifest://") :]).resolve()
    else:
        expanded = Path(raw_str)
    if not expanded.is_absolute():
        expanded = (base_dir / expanded).resolve()
    return expanded


def normalize_repo_or_absolute(raw: str, base_dir: Path) -> str:
    raw_str = str(raw)
    if raw_str.startswith("repo://"):
        return raw_str
    resolved = resolve_path(raw_str, base_dir)
    assert resolved is not None
    return str(resolved)


def bundle_path_value(
    raw: Optional[str],
    default_relpath: str,
    spec_dir: Path,
    bundle_dir: Path,
) -> Tuple[Path, str]:
    if raw:
        raw_str = os.path.expandvars(os.path.expanduser(str(raw)))
        if raw_str.startswith("repo://"):
            resolved = (REPO_ROOT / raw_str[len("repo://") :]).resolve()
        elif raw_str.startswith("manifest://"):
            resolved = (bundle_dir / raw_str[len("manifest://") :]).resolve()
        else:
            candidate = Path(raw_str)
            if candidate.is_absolute():
                resolved = candidate.resolve()
            else:
                resolved = (bundle_dir / candidate).resolve()
    else:
        resolved = (bundle_dir / default_relpath).resolve()
    assert resolved is not None

    try:
        relative = resolved.relative_to(bundle_dir)
    except ValueError:
        return resolved, str(resolved)
    return resolved, f"manifest://{relative.as_posix()}"


def require_path(section: Dict[str, Any], key: str, base_dir: Path) -> Path:
    raw = section.get(key)
    if not raw:
        raise RuntimeError(f"Missing required field: {key}")
    resolved = resolve_path(str(raw), base_dir)
    assert resolved is not None
    return resolved


def shell_quote_or_empty(value: Optional[str]) -> str:
    return shlex.quote(value or "")


def render_mapping_command(
    template: str,
    values: Dict[str, str],
) -> str:
    formatted_values = {key: shell_quote_or_empty(val) for key, val in values.items()}
    return template.format(**formatted_values)


def build_localization_manifest(
    spec: Dict[str, Any],
    spec_dir: Path,
    bundle_dir: Path,
    generated_map_value: str,
    localization_output_value: str,
) -> Dict[str, Any]:
    dataset = spec.get("dataset", {})
    bundle = spec.get("bundle", {})
    localizer = spec.get("localizer", {})
    benchmark = spec.get("benchmark", {})
    reference = spec.get("reference", {})

    manifest: Dict[str, Any] = {
        "dataset": {
            "bag_path": str(require_path(dataset, "localization_bag_path", spec_dir)),
            "map_path": generated_map_value,
            "cloud_topic": str(dataset.get("cloud_topic", "/velodyne_points")),
        },
        "localizer": {
            "base_param_yaml": normalize_repo_or_absolute(
                str(bundle.get("base_param_yaml", "repo://param/nav2_ndt_urban.yaml")),
                spec_dir,
            ),
        },
        "benchmark": {
            "output_dir": localization_output_value,
            "target_process_pattern": str(
                benchmark.get("target_process_pattern", "lidar_localization_node")
            ),
            "record_topic": str(benchmark.get("record_topic", "/pcl_pose")),
            "diagnostic_topic": str(benchmark.get("diagnostic_topic", "/alignment_status")),
            "bag_duration": float(benchmark.get("bag_duration", 120.0)),
            "bag_start_offset": float(benchmark.get("bag_start_offset", 0.0)),
            "settle_seconds": float(benchmark.get("settle_seconds", 5.0)),
            "post_roll_seconds": float(benchmark.get("post_roll_seconds", 1.0)),
            "record_qos_reliability": str(
                benchmark.get("record_qos_reliability", "reliable")
            ),
            "record_qos_durability": str(
                benchmark.get("record_qos_durability", "volatile")
            ),
        },
    }

    generated_reference_csv: Optional[str] = None
    generated_initial_pose_yaml: Optional[str] = None
    if reference:
        _, generated_reference_csv = bundle_path_value(
            bundle.get("reference_csv"),
            "generated/reference_pose.csv",
            spec_dir,
            bundle_dir,
        )
        _, generated_initial_pose_yaml = bundle_path_value(
            bundle.get("initial_pose_yaml"),
            "generated/initial_pose.yaml",
            spec_dir,
            bundle_dir,
        )

    reference_csv = dataset.get("reference_csv")
    if generated_reference_csv:
        manifest["dataset"]["reference_csv"] = generated_reference_csv
        manifest["benchmark"]["max_time_diff"] = float(
            reference.get("max_match_diff", benchmark.get("max_time_diff", 0.05))
        )
    elif reference_csv:
        manifest["dataset"]["reference_csv"] = str(require_path(dataset, "reference_csv", spec_dir))
        manifest["benchmark"]["max_time_diff"] = float(benchmark.get("max_time_diff", 0.05))

    initial_pose_yaml = dataset.get("initial_pose_yaml")
    if generated_initial_pose_yaml:
        manifest["dataset"]["initial_pose_yaml"] = generated_initial_pose_yaml
    elif initial_pose_yaml:
        manifest["dataset"]["initial_pose_yaml"] = str(
            require_path(dataset, "initial_pose_yaml", spec_dir)
        )

    ros_domain_id = benchmark.get("ros_domain_id")
    if ros_domain_id is not None:
        manifest["benchmark"]["ros_domain_id"] = int(ros_domain_id)

    twist_topic = str(dataset.get("twist_topic", "") or "")
    if twist_topic:
        manifest["dataset"]["twist_topic"] = twist_topic

    imu_topic = str(dataset.get("imu_topic", "") or "")
    if imu_topic:
        manifest["dataset"]["imu_topic"] = imu_topic

    launch_package = localizer.get("launch_package")
    if launch_package:
        manifest["localizer"]["launch_package"] = str(launch_package)
    launch_file = localizer.get("launch_file")
    if launch_file:
        manifest["localizer"]["launch_file"] = str(launch_file)
    launch_args = localizer.get("launch_args")
    if launch_args:
        manifest["localizer"]["launch_args"] = list(launch_args)
    param_overrides = localizer.get("param_overrides")
    if param_overrides:
        manifest["localizer"]["param_overrides"] = dict(param_overrides)

    return manifest


def build_bundle_readme(
    bundle_name: str,
    bundle_dir: Path,
    mapping_bag_path: Path,
    localization_bag_path: Path,
    generated_map_path: Path,
    localization_manifest_path: Path,
    nav2_script_path: Path,
    reference_csv: Optional[Path],
    initial_pose_yaml: Optional[Path],
    reference_script_path: Optional[Path],
) -> str:
    lines = [
        f"# {bundle_name}",
        "",
        "This bundle separates map creation from localization evaluation.",
        "",
        "Workflow:",
        "1. Build a pointcloud map from the mapping run.",
        "2. Extract a reference CSV and initial pose from the localization run if reference extraction is configured.",
        "3. Keep the localization run separate.",
        "4. Benchmark localization against the generated map.",
        "",
        "Resolved inputs:",
        f"- mapping_bag_path: `{mapping_bag_path}`",
        f"- localization_bag_path: `{localization_bag_path}`",
        f"- generated_map_path: `{generated_map_path}`",
    ]
    if reference_csv is not None:
        lines.append(f"- reference_csv: `{reference_csv}`")
    else:
        lines.append("- reference_csv: `not set`")
    if initial_pose_yaml is not None:
        lines.append(f"- initial_pose_yaml: `{initial_pose_yaml}`")
    else:
        lines.append("- initial_pose_yaml: `not set`")
    lines.extend(
        [
            "",
            "Generated files:",
            f"- mapping runner: `{bundle_dir / 'run_mapping.sh'}`",
        ]
    )
    if reference_script_path is not None:
        lines.append(f"- reference extraction runner: `{reference_script_path}`")
    lines.extend(
        [
            f"- localization manifest: `{localization_manifest_path}`",
            f"- localization benchmark runner: `{bundle_dir / 'run_localization_benchmark.sh'}`",
            f"- optional Nav2 map helper: `{nav2_script_path}`",
            "",
            "Important rule:",
            "- do not build the map and evaluate localization on the same run if you want a meaningful result",
        ]
    )
    return "\n".join(lines) + "\n"


def build_mapping_script(
    repo_root: Path,
    setup_script: Path,
    mapping_bag_path: Path,
    generated_map_path: Path,
    values: Dict[str, str],
    command_template: str,
) -> str:
    if command_template:
        mapper_command = render_mapping_command(command_template, values)
        mapper_hint = ""
    else:
        mapper_command = 'echo "Edit MAPPER_COMMAND in this script or provide mapping.command_template in the bundle spec." >&2; exit 2'
        mapper_hint = (
            "# Example template:\n"
            "# MAPPER_COMMAND='ros2 launch lidarslam_ros2 mapping.launch.py "
            "points_topic:=/velodyne_points imu_topic:=/imu/data "
            "bag_path:=/path/to/mapping_run map_path:=/path/to/generated_map.pcd'\n"
        )

    return f"""#!/usr/bin/env bash
set -euo pipefail

print_only=0
if [[ "${{1:-}}" == "--print-only" ]]; then
  print_only=1
  shift
fi

set +u
source {shlex.quote(str(setup_script))}
set -u

MAPPING_BAG_PATH={shlex.quote(str(mapping_bag_path))}
GENERATED_MAP_PATH={shlex.quote(str(generated_map_path))}
mkdir -p "$(dirname "${{GENERATED_MAP_PATH}}")"

{mapper_hint}MAPPER_COMMAND=${{MAPPER_COMMAND:-{shlex.quote(mapper_command)}}}

printf 'mapping_bag_path: %s\\n' "${{MAPPING_BAG_PATH}}"
printf 'generated_map_path: %s\\n' "${{GENERATED_MAP_PATH}}"
printf 'mapper_command: %s\\n' "${{MAPPER_COMMAND}}"

if [[ "${{print_only}}" == "1" ]]; then
  exit 0
fi

eval "${{MAPPER_COMMAND}}"
"""


def build_localization_script(
    setup_script: Path,
    localization_manifest_path: Path,
) -> str:
    return f"""#!/usr/bin/env bash
set -euo pipefail

set +u
source {shlex.quote(str(setup_script))}
set -u

ros2 run lidar_localization_ros2 benchmark_from_manifest \\
  --manifest {shlex.quote(str(localization_manifest_path))} \\
  "$@"
"""


def build_reference_script(
    setup_script: Path,
    bag_path: Path,
    pose_topic: str,
    sample_topic: str,
    output_csv_path: Path,
    output_initial_pose_yaml_path: Path,
    bag_duration: float,
    bag_start_offset: float,
    initial_pose_skip_sec: float,
    max_match_diff: float,
    output_stamp_source: str,
) -> str:
    sample_topic_arg = ""
    if sample_topic:
        sample_topic_arg = f"  --sample-topic {shlex.quote(sample_topic)} \\\n"
    return f"""#!/usr/bin/env bash
set -euo pipefail

print_only=0
if [[ "${{1:-}}" == "--print-only" ]]; then
  print_only=1
  shift
fi

set +u
source {shlex.quote(str(setup_script))}
set -u

BAG_PATH={shlex.quote(str(bag_path))}
POSE_TOPIC={shlex.quote(pose_topic)}
OUTPUT_CSV={shlex.quote(str(output_csv_path))}
OUTPUT_INITIAL_POSE_YAML={shlex.quote(str(output_initial_pose_yaml_path))}
mkdir -p "$(dirname "${{OUTPUT_CSV}}")"
mkdir -p "$(dirname "${{OUTPUT_INITIAL_POSE_YAML}}")"

CMD=$(cat <<'EOF'
ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 \\
  --bag-path {shlex.quote(str(bag_path))} \\
  --pose-topic {shlex.quote(pose_topic)} \\
{sample_topic_arg}  --output-csv {shlex.quote(str(output_csv_path))} \\
  --output-initial-pose-yaml {shlex.quote(str(output_initial_pose_yaml_path))} \\
  --bag-duration {bag_duration:.9f} \\
  --bag-start-offset {bag_start_offset:.9f} \\
  --initial-pose-skip-sec {initial_pose_skip_sec:.9f} \\
  --max-match-diff {max_match_diff:.9f} \\
  --output-stamp-source {shlex.quote(output_stamp_source)} \\
  "$@"
EOF
)

printf 'bag_path: %s\\n' "${{BAG_PATH}}"
printf 'pose_topic: %s\\n' "${{POSE_TOPIC}}"
printf 'output_csv: %s\\n' "${{OUTPUT_CSV}}"
printf 'output_initial_pose_yaml: %s\\n' "${{OUTPUT_INITIAL_POSE_YAML}}"

if [[ "${{print_only}}" == "1" ]]; then
  printf '%s\\n' "${{CMD}}"
  exit 0
fi

eval "${{CMD}}"
"""


def build_nav2_map_script(
    setup_script: Path,
    generated_map_path: Path,
    nav2_output_dir: Path,
    nav2_map_name: str,
    reference_csv: Optional[Path],
) -> str:
    reference_arg = ""
    if reference_csv is not None:
        reference_arg = f"  --reference-csv {shlex.quote(str(reference_csv))} \\\n"
    return f"""#!/usr/bin/env bash
set -euo pipefail

print_only=0
if [[ "${{1:-}}" == "--print-only" ]]; then
  print_only=1
  shift
fi

set +u
source {shlex.quote(str(setup_script))}
set -u

PCD_PATH={shlex.quote(str(generated_map_path))}
OUTPUT_DIR={shlex.quote(str(nav2_output_dir))}
MAP_NAME={shlex.quote(nav2_map_name)}
mkdir -p "${{OUTPUT_DIR}}"

CMD=$(cat <<'EOF'
ros2 run lidar_localization_ros2 generate_occupancy_map_from_pcd.py \\
  --pcd {shlex.quote(str(generated_map_path))} \\
  --output-dir {shlex.quote(str(nav2_output_dir))} \\
  --map-name {shlex.quote(nav2_map_name)} \\
{reference_arg}  "$@"
EOF
)

printf 'pcd_path: %s\\n' "${{PCD_PATH}}"
printf 'output_dir: %s\\n' "${{OUTPUT_DIR}}"

if [[ "${{print_only}}" == "1" ]]; then
  printf '%s\\n' "${{CMD}}"
  exit 0
fi

eval "${{CMD}}"
"""


def main() -> int:
    args = parse_args()
    spec_path = Path(args.spec).expanduser().resolve()
    spec = load_yaml(spec_path)
    spec_dir = spec_path.parent

    serialized = json.dumps(spec, ensure_ascii=False)
    if "/absolute/path/to/" in serialized and not args.print_only:
        raise RuntimeError(
            "Spec still contains /absolute/path/to/ placeholders. Replace them before execution."
        )

    dataset = spec.get("dataset", {})
    bundle = spec.get("bundle", {})
    mapping = spec.get("mapping", {})
    reference = spec.get("reference", {})
    if (
        not isinstance(dataset, dict)
        or not isinstance(bundle, dict)
        or not isinstance(mapping, dict)
        or not isinstance(reference, dict)
    ):
        raise RuntimeError("dataset, bundle, mapping, and reference sections must be YAML mappings")

    bundle_name = str(spec.get("name", spec_path.stem))

    if args.output_dir:
        bundle_dir = Path(args.output_dir).expanduser().resolve()
    else:
        output_dir_raw = bundle.get("output_dir")
        if not output_dir_raw:
            raise RuntimeError("Missing bundle.output_dir and no --output-dir override was provided")
        bundle_dir = resolve_path(str(output_dir_raw), spec_dir)
        assert bundle_dir is not None

    mapping_bag_path = require_path(dataset, "mapping_bag_path", spec_dir)
    localization_bag_path = require_path(dataset, "localization_bag_path", spec_dir)
    reference_csv_raw = dataset.get("reference_csv")
    initial_pose_yaml_raw = dataset.get("initial_pose_yaml")
    reference_csv = resolve_path(str(reference_csv_raw), spec_dir) if reference_csv_raw else None
    initial_pose_yaml = (
        resolve_path(str(initial_pose_yaml_raw), spec_dir) if initial_pose_yaml_raw else None
    )

    reference_script_path: Optional[Path] = None
    reference_pose_topic = str(reference.get("pose_topic", "") or "")
    reference_enabled = bool(reference)
    if reference_enabled and not reference_pose_topic:
        raise RuntimeError("reference.pose_topic is required when the reference section is present")

    generated_map_path, generated_map_value = bundle_path_value(
        bundle.get("generated_map_path"),
        "generated/mapping_map.pcd",
        spec_dir,
        bundle_dir,
    )
    localization_output_dir, localization_output_value = bundle_path_value(
        bundle.get("localization_output_dir"),
        "output/localization_benchmark",
        spec_dir,
        bundle_dir,
    )
    nav2_output_dir, _ = bundle_path_value(
        bundle.get("nav2_map_output_dir"),
        "generated/nav2_map",
        spec_dir,
        bundle_dir,
    )
    generated_reference_csv_path: Optional[Path] = None
    generated_initial_pose_yaml_path: Optional[Path] = None
    if reference_enabled:
        generated_reference_csv_path, _ = bundle_path_value(
            bundle.get("reference_csv"),
            "generated/reference_pose.csv",
            spec_dir,
            bundle_dir,
        )
        generated_initial_pose_yaml_path, _ = bundle_path_value(
            bundle.get("initial_pose_yaml"),
            "generated/initial_pose.yaml",
            spec_dir,
            bundle_dir,
        )
        reference_csv = generated_reference_csv_path
        initial_pose_yaml = generated_initial_pose_yaml_path

    localization_manifest_path = bundle_dir / "localization_run_manifest.yaml"
    mapping_script_path = bundle_dir / "run_mapping.sh"
    reference_script_path = bundle_dir / "run_extract_reference.sh" if reference_enabled else None
    localization_script_path = bundle_dir / "run_localization_benchmark.sh"
    nav2_script_path = bundle_dir / "run_generate_nav2_map.sh"
    copied_spec_path = bundle_dir / "bundle_spec.yaml"
    readme_path = bundle_dir / "README.md"
    setup_script = REPO_ROOT / "scripts" / "setup_local_env.sh"

    values = {
        "bundle_dir": str(bundle_dir),
        "mapping_bag_path": str(mapping_bag_path),
        "localization_bag_path": str(localization_bag_path),
        "reference_csv": "" if reference_csv is None else str(reference_csv),
        "generated_map_path": str(generated_map_path),
        "cloud_topic": str(dataset.get("cloud_topic", "/velodyne_points")),
        "imu_topic": str(dataset.get("imu_topic", "") or ""),
        "twist_topic": str(dataset.get("twist_topic", "") or ""),
        "dataset_name": bundle_name,
    }

    localization_manifest = build_localization_manifest(
        spec,
        spec_dir,
        bundle_dir,
        generated_map_value,
        localization_output_value,
    )

    mapping_script = build_mapping_script(
        REPO_ROOT,
        setup_script,
        mapping_bag_path,
        generated_map_path,
        values,
        str(mapping.get("command_template", "") or ""),
    )
    reference_script = None
    if reference_enabled:
        reference_script = build_reference_script(
            setup_script,
            require_path(
                {"bag_path": reference.get("bag_path", str(localization_bag_path))},
                "bag_path",
                spec_dir,
            ),
            reference_pose_topic,
            str(reference.get("sample_topic", dataset.get("cloud_topic", "/velodyne_points")) or ""),
            generated_reference_csv_path,
            generated_initial_pose_yaml_path,
            float(reference.get("bag_duration", spec.get("benchmark", {}).get("bag_duration", 120.0))),
            float(
                reference.get("bag_start_offset", spec.get("benchmark", {}).get("bag_start_offset", 0.0))
            ),
            float(reference.get("initial_pose_skip_sec", 0.05)),
            float(reference.get("max_match_diff", spec.get("benchmark", {}).get("max_time_diff", 0.05))),
            str(reference.get("output_stamp_source", "sample_header")),
        )
    localization_script = build_localization_script(setup_script, localization_manifest_path)
    nav2_script = build_nav2_map_script(
        setup_script,
        generated_map_path,
        nav2_output_dir,
        str(bundle.get("nav2_map_name", "nav2_generated_map")),
        reference_csv,
    )
    readme = build_bundle_readme(
        bundle_name,
        bundle_dir,
        mapping_bag_path,
        localization_bag_path,
        generated_map_path,
        localization_manifest_path,
        nav2_script_path,
        reference_csv,
        initial_pose_yaml,
        reference_script_path,
    )

    if args.print_only:
        print(f"bundle_dir: {bundle_dir}")
        print(f"mapping_script: {mapping_script_path}")
        if reference_script_path is not None:
            print(f"reference_script: {reference_script_path}")
        print(f"localization_manifest: {localization_manifest_path}")
        print(f"localization_script: {localization_script_path}")
        print(f"nav2_script: {nav2_script_path}")
        print(f"generated_map_path: {generated_map_path}")
        if reference_csv is not None:
            print(f"reference_csv: {reference_csv}")
        if initial_pose_yaml is not None:
            print(f"initial_pose_yaml: {initial_pose_yaml}")
        print(f"localization_output_dir: {localization_output_dir}")
        return 0

    bundle_dir.mkdir(parents=True, exist_ok=True)
    write_yaml(localization_manifest_path, localization_manifest, False)
    write_yaml(copied_spec_path, spec, False)
    write_text(mapping_script_path, mapping_script, False)
    if reference_script_path is not None and reference_script is not None:
        write_text(reference_script_path, reference_script, False)
    write_text(localization_script_path, localization_script, False)
    write_text(nav2_script_path, nav2_script, False)
    write_text(readme_path, readme, False)
    mapping_script_path.chmod(0o755)
    if reference_script_path is not None:
        reference_script_path.chmod(0o755)
    localization_script_path.chmod(0o755)
    nav2_script_path.chmod(0o755)

    print(f"bundle_dir: {bundle_dir}")
    print(f"mapping_script: {mapping_script_path}")
    if reference_script_path is not None:
        print(f"reference_script: {reference_script_path}")
    print(f"localization_manifest: {localization_manifest_path}")
    print(f"localization_script: {localization_script_path}")
    print(f"nav2_script: {nav2_script_path}")
    print(f"generated_map_path: {generated_map_path}")
    if reference_csv is not None:
        print(f"reference_csv: {reference_csv}")
    if initial_pose_yaml is not None:
        print(f"initial_pose_yaml: {initial_pose_yaml}")
    print(f"localization_output_dir: {localization_output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
