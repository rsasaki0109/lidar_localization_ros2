#!/usr/bin/env bash
set -euo pipefail

PATH="${HOME}/.local/bin:${PATH}"

MAP_ID="1WPWmFCjV7eQee4kyBpmGNlX7awerCPxc"
BAG_ID="1yEB5j74gPLLbkkf87cuCxUgHXTkgSZbn"

usage() {
  cat <<'EOF'
Fetch the official Autoware Istanbul localization dataset and prepare a ROS 2 bag directory.

Usage:
  scripts/fetch_official_autoware_istanbul_dataset.sh [--output-dir DIR] [--skip-reindex] [--force-reindex]

Options:
  --output-dir DIR  Destination directory. Default: ./data/official/autoware_istanbul
  --skip-reindex    Download files, but skip `ros2 bag reindex`.
  --force-reindex   Recreate metadata.yaml even if it already exists.
  -h, --help        Show this help.
EOF
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Missing required command: ${cmd}" >&2
    exit 1
  fi
}

download_file() {
  local url="$1"
  local dst="$2"

  if [[ -f "${dst}" ]]; then
    echo "Using existing file: ${dst}"
    return 0
  fi

  echo "Downloading ${url}"
  gdown "${url}" -O "${dst}"
}

output_dir="$(pwd)/data/official/autoware_istanbul"
skip_reindex=0
force_reindex=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      shift
      output_dir="$1"
      ;;
    --skip-reindex)
      skip_reindex=1
      ;;
    --force-reindex)
      force_reindex=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

require_cmd gdown
require_cmd ros2

mkdir -p "${output_dir}"

map_path="${output_dir}/pointcloud_map.pcd"
bag_dir="${output_dir}/localization_rosbag"
db3_path="${bag_dir}/autoware_istanbul_localization_0.db3"
metadata_path="${bag_dir}/metadata.yaml"

mkdir -p "${bag_dir}"

download_file "https://drive.google.com/uc?id=${MAP_ID}" "${map_path}"
download_file "https://drive.google.com/uc?id=${BAG_ID}" "${db3_path}"

if [[ "${skip_reindex}" -eq 0 ]]; then
  if [[ -f "${metadata_path}" && "${force_reindex}" -eq 0 ]]; then
    echo "Using existing file: ${metadata_path}"
  else
    rm -f "${metadata_path}"
    echo "Reindexing ${bag_dir}"
    ros2 bag reindex "${bag_dir}"
  fi
fi

cat <<EOF
Prepared official Autoware Istanbul localization dataset:
  Map PCD:        ${map_path}
  ROS2 bag dir:   ${bag_dir}
  ROS2 db3 file:  ${db3_path}
  Metadata:       ${metadata_path}

Observed bag topics from the official localization-only database:
  /localization/util/downsample/pointcloud
  /sensing/gnss/pose
  /sensing/gnss/pose_with_covariance
  /localization/twist_estimator/twist_with_covariance
  /clock
  /tf_static

Notes:
  - This dataset is intended for map-based localization evaluation.
  - Use /sensing/gnss/pose_with_covariance as the reference trajectory source.
  - This package only needs pointcloud_map.pcd. The lanelet map note in the Autoware docs
    does not apply to lidar_localization_ros2 itself.
EOF
