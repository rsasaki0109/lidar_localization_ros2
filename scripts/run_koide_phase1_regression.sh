#!/usr/bin/env bash
# Release-style wrapper for the Koide Phase 1 backend comparison (smoke60).
set -euo pipefail

usage() {
  cat <<'EOF'
Run the Koide Phase 1 backend comparison regression (60 s smoke, NDT vs GICP backends).

Writes regression_result.json with overall_pass when NDT_OMP completes with
alignment diagnostics and any completed GICP backends also pass the audit.

Usage:
  scripts/run_koide_phase1_regression.sh [options]

Options:
  --output-dir DIR       Artifact directory. Default: artifacts/public/koide_phase1_backend_comparison
  --max-load N           Warn when 1-min load average exceeds N. Default: 5
  --skip-gicp            Run NDT_OMP only (when small_gicp is not built).
  --resume               Reuse completed run directories and an existing pass summary.
  -h, --help             Show this help.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/../.." && pwd)}"
else
  package_prefix="$(cd "${script_dir}/../.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${package_prefix}/../../.." && pwd)}"
  repo_root="${LIDAR_LOCALIZATION_REPO_ROOT:-${ws_root}/repo}"
fi

output_dir="${ws_root}/artifacts/public/koide_phase1_backend_comparison"
max_load="5"
skip_gicp=0
resume=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir) shift; output_dir="$1" ;;
    --max-load) shift; max_load="$1" ;;
    --skip-gicp) skip_gicp=1 ;;
    --resume) resume=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

data_meta="${repo_root}/data/public/koide_hard_localization/sequences/outdoor_hard_01a/metadata.yaml"
summary_json="${output_dir}/comparison_summary.json"
result_json="${output_dir}/regression_result.json"

if [[ ! -f "${data_meta}" ]]; then
  cat >"${result_json}" <<EOF
{
  "overall_pass": false,
  "skipped": true,
  "reason": "koide dataset missing (${data_meta})",
  "output_dir": "${output_dir}"
}
EOF
  echo "Koide Phase 1 regression skipped: dataset not present" >&2
  exit 0
fi

load_avg="$(awk '{print $1}' /proc/loadavg)"
load_int="$(awk -v load="${load_avg}" 'BEGIN { printf "%d", load + 0.5 }')"
if (( load_int >= max_load )); then
  echo "warning: 1-min load ${load_avg} >= ${max_load}; alignment timing may be invalid" >&2
fi

mkdir -p "${output_dir}"

if [[ "${resume}" -eq 1 && -f "${summary_json}" ]]; then
  if python3 - "${summary_json}" <<'PY'
import json, sys
payload = json.loads(open(sys.argv[1], encoding="utf-8").read())
sys.exit(0 if payload.get("overall_pass") else 1)
PY
  then
    cp -f "${summary_json}" "${result_json}"
    echo "Koide Phase 1 regression resumed from existing pass: ${summary_json}"
    exit 0
  fi
fi

comparison_args=(--report-dir "${output_dir}" --max-load "${max_load}")
if [[ "${resume}" -eq 1 ]]; then
  comparison_args+=(--resume)
fi
if [[ "${skip_gicp}" -eq 1 ]]; then
  comparison_args+=(--skip-gicp)
fi

comparison_status=0
if ! "${script_dir}/run_koide_phase1_backend_comparison.sh" "${comparison_args[@]}"; then
  comparison_status=$?
fi

if [[ ! -f "${summary_json}" ]]; then
  cat >"${result_json}" <<EOF
{
  "overall_pass": false,
  "comparison_exit_code": ${comparison_status},
  "reason": "comparison_summary.json missing",
  "output_dir": "${output_dir}"
}
EOF
  exit 1
fi

python3 - "${summary_json}" "${result_json}" "${comparison_status}" "${output_dir}" <<'PY'
import json
import sys
from pathlib import Path

summary_path = Path(sys.argv[1])
result_path = Path(sys.argv[2])
comparison_status = int(sys.argv[3])
output_dir = sys.argv[4]
summary = json.loads(summary_path.read_text(encoding="utf-8"))
overall_pass = bool(summary.get("overall_pass")) and comparison_status == 0
payload = {
    "overall_pass": overall_pass,
    "comparison_exit_code": comparison_status,
    "comparison_summary": summary,
    "comparison_summary_json": str(summary_path),
    "output_dir": output_dir,
    "recommended_default": summary.get("recommended_default"),
    "completed_backends": summary.get("completed_backends"),
}
result_path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")
print(json.dumps(payload, indent=2, sort_keys=False))
sys.exit(0 if overall_pass else 1)
PY
