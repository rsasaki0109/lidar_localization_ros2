#!/usr/bin/env bash
set -euo pipefail

PATH="${HOME}/.local/bin:${PATH}"

BAG_URL="https://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz"
MAP_URL="https://raw.githubusercontent.com/koide3/hdl_localization/master/data/map.pcd"

usage() {
  cat <<'EOF'
Fetch the official hdl_localization sample dataset and prepare a ROS 2 bag.

Usage:
  scripts/fetch_official_hdl_localization_sample.sh [--output-dir DIR] [--skip-convert] [--force-convert]

Options:
  --output-dir DIR  Destination directory. Default: ./data/official/hdl_localization
  --skip-convert    Download the ROS1 bag and map, but skip rosbag2 conversion.
  --force-convert   Recreate the rosbag2 output directory if it already exists.
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
  curl -L --fail --continue-at - --output "${dst}" "${url}"
}

patch_metadata() {
  local metadata_path="$1"

  python3 - "${metadata_path}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
patched = text.replace('offered_qos_profiles: []', 'offered_qos_profiles: ""')
path.write_text(patched)
PY
}

output_dir="$(pwd)/data/official/hdl_localization"
skip_convert=0
force_convert=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      shift
      output_dir="$1"
      ;;
    --skip-convert)
      skip_convert=1
      ;;
    --force-convert)
      force_convert=1
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

require_cmd curl
require_cmd tar
require_cmd python3

mkdir -p "${output_dir}"

tar_path="${output_dir}/hdl_400.bag.tar.gz"
bag_path="${output_dir}/hdl_400.bag"
map_path="${output_dir}/map.pcd"
ros2_dir="${output_dir}/hdl_400_ros2"

download_file "${BAG_URL}" "${tar_path}"
download_file "${MAP_URL}" "${map_path}"

if [[ ! -f "${bag_path}" ]]; then
  echo "Extracting ${tar_path}"
  tar -xf "${tar_path}" -C "${output_dir}"
else
  echo "Using existing file: ${bag_path}"
fi

if [[ "${skip_convert}" -eq 0 ]]; then
  require_cmd rosbags-convert

  if [[ -d "${ros2_dir}" ]]; then
    if [[ "${force_convert}" -eq 1 ]]; then
      rm -rf "${ros2_dir}"
    else
      echo "Using existing directory: ${ros2_dir}"
    fi
  fi

  if [[ ! -d "${ros2_dir}" ]]; then
    echo "Converting ${bag_path} to rosbag2"
    rosbags-convert \
      --src "${bag_path}" \
      --dst "${ros2_dir}" \
      --include-topic /velodyne_points /gpsimu_driver/imu_data /gpsimu_driver/gpstime \
      --dst-typestore ros2_humble
    patch_metadata "${ros2_dir}/metadata.yaml"
  fi
fi

cat <<EOF
Prepared official hdl_localization sample:
  ROS1 bag:  ${bag_path}
  Map PCD:   ${map_path}
  ROS2 bag:  ${ros2_dir}

Notes:
  - The ROS2 bag contains only standard ROS message types:
    /velodyne_points, /gpsimu_driver/imu_data, /gpsimu_driver/gpstime
  - hdl_localization's original demo relies on relocalization.
    This repository still needs a good initial pose for a fair comparison on this dataset.
EOF
