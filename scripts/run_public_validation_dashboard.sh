#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Build the public validation dashboard from demo and regression summaries.

Usage:
  scripts/run_public_validation_dashboard.sh [options]

Options:
  --workspace-root DIR   lidarloc_ws root. Default: parent of repo root.
  --output-dir DIR       Dashboard output directory.
                         Default: <workspace>/artifacts/public/dashboard
  --demo-report-json PATH
  --release-summary-json PATH
  --public-summary-json PATH
  -h, --help             Show this help.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/.." && pwd)}"
else
  package_prefix="$(cd "${script_dir}/../.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${package_prefix}/../../.." && pwd)}"
  repo_root="${LIDAR_LOCALIZATION_REPO_ROOT:-${ws_root}/repo}"
fi

output_dir="${ws_root}/artifacts/public/dashboard"
extra_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace-root)
      shift
      ws_root="$1"
      extra_args+=(--workspace-root "${ws_root}")
      ;;
    --output-dir)
      shift
      output_dir="$1"
      extra_args+=(--output-dir "${output_dir}")
      ;;
    --demo-report-json|--release-summary-json|--public-summary-json)
      extra_args+=("$1")
      shift
      extra_args+=("$1")
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

python3 "${repo_root}/scripts/build_public_validation_dashboard.py" \
  --workspace-root "${ws_root}" \
  --output-dir "${output_dir}" \
  "${extra_args[@]}"
