#!/usr/bin/env bash
# Render one ground-truth route overview GIF for every Koide evaluation trajectory.
set -euo pipefail

usage() {
  cat <<'EOF'
Render the eight Koide Hard Point Cloud Localization Dataset reference routes.

Usage:
  scripts/render_koide_dataset_gallery.sh --data-dir DIR [options]

Options:
  --data-dir DIR    Dataset root containing gt/ and map_*.ply (required)
  --output-dir DIR  GIF destination (default: REPO/images/koide)
  --frames N        Frames per GIF (default: 64)
  --fps N           GIF frame rate (default: 10)
  --force           Regenerate reference CSVs and occupancy maps
  -h, --help        Show this help

The output is a dataset route overview, not a localization accuracy result.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
data_dir=""
output_dir="${repo_root}/images/koide"
frames=64
fps=10
force=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --data-dir) shift; data_dir="$1" ;;
    --output-dir) shift; output_dir="$1" ;;
    --frames) shift; frames="$1" ;;
    --fps) shift; fps="$1" ;;
    --force) force=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ -z "${data_dir}" ]]; then
  echo "--data-dir is required" >&2
  exit 2
fi

data_dir="$(realpath "${data_dir}")"
mkdir -p "${output_dir}"
work_dir="${data_dir}/generated/gif_gallery"
reference_dir="${work_dir}/reference"
occupancy_dir="${work_dir}/occupancy"
mkdir -p "${reference_dir}" "${occupancy_dir}"

sequences=(
  indoor_easy_01
  indoor_easy_02
  indoor_hard_01
  indoor_kidnap_01
  indoor_kidnap_02
  outdoor_hard_01
  outdoor_hard_02
  outdoor_kidnap
)
declare -A map_key=(
  [indoor_easy_01]=indoor_easy
  [indoor_easy_02]=indoor_easy
  [indoor_hard_01]=indoor_hard
  [indoor_kidnap_01]=indoor_hard
  [indoor_kidnap_02]=indoor_hard
  [outdoor_hard_01]=outdoor_hard
  [outdoor_hard_02]=outdoor_hard
  [outdoor_kidnap]=outdoor_kidnap
)

maps=(indoor_easy indoor_hard outdoor_hard outdoor_kidnap)
for map in "${maps[@]}"; do
  source_map="${data_dir}/map_${map}.ply"
  map_dir="${occupancy_dir}/${map}"
  map_yaml="${map_dir}/map.yaml"
  if [[ ! -f "${source_map}" ]]; then
    echo "Missing map: ${source_map}" >&2
    exit 1
  fi
  if [[ "${force}" -eq 1 || ! -f "${map_yaml}" ]]; then
    python3 "${script_dir}/generate_occupancy_map_from_pcd.py" \
      --pcd "${source_map}" \
      --output-dir "${map_dir}" \
      --map-name map \
      --resolution 0.2 \
      --inflate-radius-m 0.4
  fi
done

for sequence in "${sequences[@]}"; do
  tum="${data_dir}/gt/traj_lidar_${sequence}.txt"
  reference="${reference_dir}/${sequence}.csv"
  initial_pose="${reference_dir}/${sequence}_initial_pose.yaml"
  if [[ ! -f "${tum}" ]]; then
    echo "Missing reference trajectory: ${tum}" >&2
    exit 1
  fi
  if [[ "${force}" -eq 1 || ! -f "${reference}" || ! -f "${initial_pose}" ]]; then
    python3 "${script_dir}/tum_trajectory_to_pose_reference_csv.py" \
      --input "${tum}" \
      --output-csv "${reference}" \
      --output-initial-pose-yaml "${initial_pose}" \
      --initial-pose-skip-sec 0.05
  fi

  python3 "${script_dir}/render_koide_localization_gif.py" \
    --occupancy-yaml "${occupancy_dir}/${map_key[${sequence}]}/map.yaml" \
    --reference-csv "${reference}" \
    --reference-only \
    --output-gif "${output_dir}/${sequence}.gif" \
    --frames "${frames}" \
    --fps "${fps}" \
    --sequence-label "Koide ${sequence}" \
    --estimate-label "Traversed GT route" \
    --title "Koide dataset reference route"
done

echo "Koide GIF gallery: ${output_dir}"
