#!/usr/bin/env bash
# Download Kenji Koide's "Hard Point Cloud Localization Dataset" from Zenodo (10.5281/zenodo.10122133)
# for use with lidar_localization_ros2 benchmarks. Maps are PLY; bags are ROS 2 inside per-sequence zips.
set -euo pipefail

ZENODO_BASE="https://zenodo.org/records/10122133/files"

usage() {
  cat <<'EOF'
Fetch Koide Hard Point Cloud Localization Dataset assets from Zenodo.

Usage:
  scripts/fetch_koide_hard_pointcloud_localization_dataset.sh [--output-dir DIR] [options]

Options:
  --output-dir DIR     Root directory. Default: ./data/public/koide_hard_localization
  --with-maps          Download all map_*.ply files (~38 MiB total)
  --with-gt            Download gt.zip and unpack traj_lidar_*.txt (~1 MiB)
  --sequence NAME      Also download and unzip NAME.zip (e.g. indoor_easy_01). Large.
  --force-sequences    Re-download / re-unzip sequence archives when they already exist
  -h, --help           Show this help.

Examples:
  # Reference trajectories + maps only (small):
  scripts/fetch_koide_hard_pointcloud_localization_dataset.sh --with-maps --with-gt

  # Full indoor_easy_01 bag (~1.7 GiB download):
  scripts/fetch_koide_hard_pointcloud_localization_dataset.sh --with-maps --with-gt --sequence indoor_easy_01

Sequence zip names (Zenodo): indoor_easy_01, indoor_easy_02, indoor_hard_01,
  indoor_kidnap_01, indoor_kidnap_02, outdoor_hard_01a, outdoor_hard_01b,
  outdoor_hard_02a, outdoor_hard_02b, outdoor_kidnap_a, outdoor_kidnap_b

Indoor ROS topics: /points2/decompressed (PointCloud2), /imu (Imu)
Outdoor ROS topics: /livox/points (PointCloud2), /livox/imu (Imu)

Convert GT (after --with-gt) to benchmark CSV + initial pose, e.g.:
  ros2 run lidar_localization_ros2 tum_trajectory_to_pose_reference_csv.py \\
    --input  DIR/gt/traj_lidar_indoor_easy_01.txt \\
    --output-csv DIR/generated/indoor_easy_01_reference.csv \\
    --output-initial-pose-yaml DIR/generated/indoor_easy_01_initial_pose.yaml \\
    --initial-pose-skip-sec 0.05

Zenodo: https://zenodo.org/records/10122133
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

output_dir="$(pwd)/data/public/koide_hard_localization"
with_maps=0
with_gt=0
force_sequences=0
declare -a sequences=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      shift
      output_dir="$1"
      ;;
    --with-maps)
      with_maps=1
      ;;
    --with-gt)
      with_gt=1
      ;;
    --sequence)
      shift
      sequences+=("$1")
      ;;
    --force-sequences)
      force_sequences=1
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
require_cmd unzip

mkdir -p "${output_dir}"

if [[ "${with_maps}" -eq 1 ]]; then
  for f in \
    map_indoor_easy.ply \
    map_indoor_hard.ply \
    map_outdoor_hard.ply \
    map_outdoor_kidnap.ply
  do
    download_file "${ZENODO_BASE}/${f}?download=1" "${output_dir}/${f}"
  done
fi

if [[ "${with_gt}" -eq 1 ]]; then
  gt_zip="${output_dir}/gt.zip"
  download_file "${ZENODO_BASE}/gt.zip?download=1" "${gt_zip}"
  if [[ ! -d "${output_dir}/gt" ]] || [[ "${force_sequences}" -eq 1 ]]; then
    rm -rf "${output_dir}/gt"
    echo "Unpacking ${gt_zip}"
    unzip -q -o "${gt_zip}" -d "${output_dir}"
  else
    echo "Using existing directory: ${output_dir}/gt"
  fi
fi

for seq in "${sequences[@]}"; do
  zip_name="${seq}.zip"
  zip_path="${output_dir}/${zip_name}"
  dest="${output_dir}/sequences/${seq}"
  download_file "${ZENODO_BASE}/${zip_name}?download=1" "${zip_path}"
  if [[ -d "${dest}" ]] && [[ "${force_sequences}" -eq 0 ]]; then
    echo "Using existing directory: ${dest}"
    continue
  fi
  rm -rf "${dest}"
  mkdir -p "${dest}"
  echo "Unpacking ${zip_path} -> ${dest}"
  unzip -q -o "${zip_path}" -d "${dest}"
done

map_hint=""
if [[ "${with_maps}" -eq 1 ]]; then
  map_hint=$'\n'"Maps are under: ${output_dir}/map_*.ply"
fi

cat <<EOF
Koide Hard Point Cloud Localization Dataset stage directory: ${output_dir}
${map_hint}

Find ROS 2 bags with: find ${output_dir} -name metadata.yaml

Example manifest: param/benchmark/koide_hard_localization_run_manifest.example.yaml
Citation: Kenji Koide, Hard Point Cloud Localization Dataset, Zenodo, 2023. https://doi.org/10.5281/zenodo.10122133
EOF
