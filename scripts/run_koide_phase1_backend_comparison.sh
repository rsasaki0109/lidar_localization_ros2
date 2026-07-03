#!/usr/bin/env bash
# Phase 1: Koide outdoor_hard_01a 60 s smoke — NDT_OMP vs SMALL_GICP vs SMALL_VGICP (voxel 0.5).
set -euo pipefail

usage() {
  cat <<'EOF'
Run the Koide Phase 1 registration-backend comparison (60 s downsampled smoke).

Requires:
  - Koide dataset under data/public/koide_hard_localization/
  - idle machine (load < 5 recommended)
  - small_gicp installed + lidar_localization_ros2 rebuilt for GICP backends

Usage:
  scripts/run_koide_phase1_backend_comparison.sh [options]

Options:
  --report-dir DIR       Output root. Default: artifacts/public/koide_phase1_backend_comparison
  --max-load N           Warn when 1-min load >= N. Default: 5
  --skip-gicp            Run NDT_OMP only (when small_gicp is not built).
  --resume               Reuse completed run directories.
  --print-only           Print commands without executing.
  -h, --help             Show this help.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
report_dir_default="${repo_root}/../../artifacts/public/koide_phase1_backend_comparison"
max_load="5"
skip_gicp=0
resume=0
print_only=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --report-dir) shift; report_dir="$1" ;;
    --max-load) shift; max_load="$1" ;;
    --skip-gicp) skip_gicp=1 ;;
    --resume) resume=1 ;;
    --print-only) print_only=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

report_dir="${report_dir:-${report_dir_default}}"

data_meta="${repo_root}/data/public/koide_hard_localization/sequences/outdoor_hard_01a/metadata.yaml"
if [[ ! -f "${data_meta}" ]]; then
  echo "Koide dataset missing: ${data_meta}" >&2
  exit 2
fi

load_avg="$(awk '{print $1}' /proc/loadavg)"
load_int="$(awk -v load="${load_avg}" 'BEGIN { printf "%d", load + 0.5 }')"
if (( load_int >= max_load )); then
  echo "warning: 1-min load ${load_avg} >= ${max_load}; alignment timing may be invalid" >&2
fi

# shellcheck source=/dev/null
source "${repo_root}/scripts/setup_local_env.sh"
ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/../.." && pwd)}"
# shellcheck source=/dev/null
source "${ws_root}/install/setup.bash" 2>/dev/null || true
export LD_LIBRARY_PATH="${LIDAR_LOCALIZATION_LOCAL_PREFIX:-${ws_root}/local_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

mkdir -p "${report_dir}"

declare -A manifests=(
  [ndt]="param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60.yaml"
  [small_gicp_ds]="param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60_small_gicp_ds.yaml"
  [small_vgicp_ds]="param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60_small_vgicp_ds.yaml"
)
declare -A run_dirs=(
  [ndt]="/tmp/lidarloc_koide_outdoor_hard_01a_smoke60"
  [small_gicp_ds]="/tmp/lidarloc_koide_01a_smoke60_small_gicp_ds"
  [small_vgicp_ds]="/tmp/lidarloc_koide_01a_smoke60_small_vgicp_ds"
)

have_small_gicp() {
  local flags="${ws_root}/build/lidar_localization_ros2/CMakeFiles/lidar_localization_component.dir/flags.make"
  [[ -f "${flags}" ]] && grep -q "LIDAR_LOCALIZATION_HAVE_SMALL_GICP=1" "${flags}" && return 0
  return 1
}

run_manifest() {
  local key="$1"
  local manifest="${repo_root}/${manifests[$key]}"
  local out_dir="${run_dirs[$key]}"
  if [[ "${resume}" -eq 1 && -f "${out_dir}/health_summary.json" && -f "${out_dir}/trajectory_eval.json" ]]; then
    echo "Skipping ${key}: existing results in ${out_dir}"
    return 0
  fi
  pkill -9 -f "lidar_localization_ros2/lib/lidar_localization_ros2/" 2>/dev/null || true
  sleep 2
  echo "Running ${key} via ${manifest}..."
  if [[ "${print_only}" -eq 1 ]]; then
    python3 "${repo_root}/scripts/benchmark_from_manifest" --manifest "${manifest}" --print-only
    return 0
  fi
  python3 "${repo_root}/scripts/benchmark_from_manifest" --manifest "${manifest}"
}

if [[ "${print_only}" -eq 1 ]]; then
  for key in ndt small_gicp_ds small_vgicp_ds; do
    run_manifest "${key}"
  done
  exit 0
fi

run_manifest ndt

gicp_available=0
if [[ "${skip_gicp}" -eq 0 ]] && have_small_gicp; then
  gicp_available=1
  run_manifest small_gicp_ds
  run_manifest small_vgicp_ds
else
  echo "Skipping SMALL_GICP / SMALL_VGICP (small_gicp not built or --skip-gicp)." >&2
  echo "Install: clone https://github.com/koide3/small_gicp into local_prefix, rebuild." >&2
fi

summary_json="${report_dir}/comparison_summary.json"
python3 "${repo_root}/scripts/summarize_koide_phase1_backend_comparison.py" \
  --ndt-dir "${run_dirs[ndt]}" \
  --small-gicp-dir "${run_dirs[small_gicp_ds]}" \
  --small-vgicp-dir "${run_dirs[small_vgicp_ds]}" \
  --output-json "${summary_json}"

cp -f "${summary_json}" "${report_dir}/regression_result.json" 2>/dev/null || true
echo "comparison_summary: ${summary_json}"
