#!/usr/bin/env bash
# Fetch, verify, and unpack every Koide sequence with bounded parallelism.
set -uo pipefail

usage() {
  cat <<'EOF'
Fetch all eleven Koide ROS 2 bag archives.

Usage:
  scripts/fetch_all_koide_sequences.sh --output-dir DIR [--jobs N]

Options:
  --output-dir DIR  Dataset root on a large disk (required)
  --jobs N          Concurrent downloads (default: 3)
  --keep-zips       Keep verified ZIP archives after extraction
  -h, --help        Show this help

Completed bags with metadata.yaml are skipped. Partial downloads use .zip.part and resume.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
output_dir=""
jobs=3
keep_zips=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir) shift; output_dir="$1" ;;
    --jobs) shift; jobs="$1" ;;
    --keep-zips) keep_zips=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ -z "${output_dir}" ]]; then
  echo "--output-dir is required" >&2
  exit 2
fi
if ! [[ "${jobs}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--jobs must be a positive integer" >&2
  exit 2
fi

mkdir -p "${output_dir}/download_logs"
output_dir="$(realpath "${output_dir}")"

sequences=(
  indoor_easy_01
  indoor_easy_02
  indoor_hard_01
  indoor_kidnap_01
  indoor_kidnap_02
  outdoor_hard_01a
  outdoor_hard_01b
  outdoor_hard_02a
  outdoor_hard_02b
  outdoor_kidnap_a
  outdoor_kidnap_b
)
declare -A expected_md5=(
  [indoor_easy_01]=046c1de5047a88dcbbf14e98df6874b1
  [indoor_easy_02]=ecf01901f0e581c81d20e2bf86867925
  [indoor_hard_01]=c5eca239e8635656af1ec5c5bb6532b7
  [indoor_kidnap_01]=c1ce1dcab51ee6a92a8d9afd4b349e21
  [indoor_kidnap_02]=8b68cd8d90bf623181e9353b1c94df35
  [outdoor_hard_01a]=d126024c8f310c2c48239f136d7e0ed0
  [outdoor_hard_01b]=d397c28cca763d34844189cb73de8e02
  [outdoor_hard_02a]=8f56fdfa93f4fa456e234f58e193a08c
  [outdoor_hard_02b]=267377a88402f825ab70a161cc48983c
  [outdoor_kidnap_a]=3c6941b8c70ca41c79dae83758632625
  [outdoor_kidnap_b]=893ed4a732c5c0c9dc38d069e7056a69
)

fetch_one() {
  local sequence="$1"
  local metadata="${output_dir}/sequences/${sequence}/metadata.yaml"
  local archive="${output_dir}/${sequence}.zip"
  local log="${output_dir}/download_logs/${sequence}.log"

  if [[ -f "${metadata}" ]]; then
    if [[ -f "${archive}" ]]; then
      local staged_md5
      staged_md5="$(md5sum "${archive}" | awk '{print $1}')"
      if [[ "${staged_md5}" != "${expected_md5[${sequence}]}" ]]; then
        echo "${sequence}: existing archive MD5 mismatch (${staged_md5})" >&2
        return 1
      fi
      if [[ "${keep_zips}" -eq 0 ]]; then
        rm -f "${archive}"
      fi
    fi
    echo "${sequence}: already unpacked"
    return 0
  fi

  if ! "${script_dir}/fetch_koide_hard_pointcloud_localization_dataset.sh" \
      --sequence "${sequence}" --output-dir "${output_dir}" >"${log}" 2>&1; then
    echo "${sequence}: fetch failed (see ${log})" >&2
    return 1
  fi
  if [[ ! -f "${metadata}" ]]; then
    echo "${sequence}: missing metadata after extraction" >&2
    return 1
  fi

  local actual_md5
  actual_md5="$(md5sum "${archive}" | awk '{print $1}')"
  if [[ "${actual_md5}" != "${expected_md5[${sequence}]}" ]]; then
    echo "${sequence}: MD5 mismatch (${actual_md5})" >&2
    return 1
  fi
  if [[ "${keep_zips}" -eq 0 ]]; then
    rm -f "${archive}"
  fi
  echo "${sequence}: unpacked and verified"
}

failed=0
for sequence in "${sequences[@]}"; do
  while (( $(jobs -rp | wc -l) >= jobs )); do
    wait -n || failed=1
  done
  fetch_one "${sequence}" &
done
while (( $(jobs -rp | wc -l) > 0 )); do
  wait -n || failed=1
done

if [[ "${failed}" -ne 0 ]]; then
  echo "One or more Koide sequence downloads failed" >&2
  exit 1
fi
echo "All Koide sequences are available under ${output_dir}/sequences"
