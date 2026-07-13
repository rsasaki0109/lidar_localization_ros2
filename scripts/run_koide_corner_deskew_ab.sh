#!/usr/bin/env bash
# Repeat the controlled Koide corner no-deskew vs current-deskew experiment.
set -euo pipefail

usage() {
  cat <<'EOF'
Run the Koide outdoor_hard_01a corner deskew A/B gate.

Usage:
  scripts/run_koide_corner_deskew_ab.sh [options]

Options:
  --download             Download missing Koide assets on the first repeat.
  --data-dir DIR         Dataset root passed to the Koide smoke runner.
  --output-dir DIR       Default: /tmp/lidarloc_koide_corner_deskew_ab
  --start-offset SEC     Replay start; default 85 (warm-up before the corner).
  --duration SEC         Replay duration; default 27 (ends near bag 112 s).
  --repeats N            Default: 3.
  --ros-domain-id ID     First repeat domain ID; default 205.
  --print-only           Generate configs and commands without ROS execution.
  -h, --help             Show this help.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
smoke="${script_dir}/run_koide_hard_imu_deskew_smoke.sh"
summarizer="${script_dir}/summarize_koide_deskew_ab.py"

data_dir=""
output_dir="/tmp/lidarloc_koide_corner_deskew_ab"
start_offset="85"
duration="27"
repeats="3"
ros_domain_id="205"
download=0
print_only=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --download) download=1 ;;
    --data-dir) shift; data_dir="$1" ;;
    --output-dir) shift; output_dir="$1" ;;
    --start-offset) shift; start_offset="$1" ;;
    --duration) shift; duration="$1" ;;
    --repeats) shift; repeats="$1" ;;
    --ros-domain-id) shift; ros_domain_id="$1" ;;
    --print-only) print_only=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if ! [[ "${repeats}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--repeats must be a positive integer" >&2
  exit 2
fi

mkdir -p "${output_dir}"
failed=0
for ((i = 1; i <= repeats; ++i)); do
  repeat_dir="${output_dir}/repeat_$(printf '%02d' "${i}")"
  args=(
    --output-dir "${repeat_dir}"
    --start-offset "${start_offset}"
    --duration "${duration}"
    --ros-domain-id "$((ros_domain_id + i - 1))"
    --mode lidar_only
    --mode deskew
    --mode imu_pose_history
    --mode lidar_constant_velocity
    --mode localizability_guard
  )
  if [[ -n "${data_dir}" ]]; then
    args+=(--data-dir "${data_dir}")
  fi
  if [[ "${download}" -eq 1 && "${i}" -eq 1 ]]; then
    args+=(--download)
  fi
  if [[ "${print_only}" -eq 1 ]]; then
    args+=(--print-only)
  fi
  echo "== Koide corner deskew A/B repeat ${i}/${repeats} =="
  if ! "${smoke}" "${args[@]}"; then
    failed=1
  fi
done

if [[ "${print_only}" -eq 1 ]]; then
  echo "Generated ${repeats} repeat plans under ${output_dir}"
  exit 0
fi

if ! "${summarizer}" \
  --output-dir "${output_dir}" \
  --expected-repeats "${repeats}"; then
  failed=1
fi
exit "${failed}"
