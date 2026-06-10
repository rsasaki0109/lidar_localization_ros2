#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Smoke-check the v1.1 relocalization MVP boundary.

Checks:
  1. user-facing docs do not claim runtime relocalization is solved
  2. the v1.1 dry-run endpoint example manifest expands the full no-publish chain
  3. optional recorded artifact validation JSON still shows published_count=0

Usage:
  scripts/run_v1_1_relocalization_smoke.sh [options]

Options:
  --artifact-dir DIR   Directory with relocalization_reset_commands_validation.json
                       Default: <workspace>/artifacts/public/v1_1_boreas_localizer_only_120_current
  --skip-artifacts     Skip recorded artifact validation.
  -h, --help           Show this help.
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

artifact_dir="${ws_root}/artifacts/public/v1_1_boreas_localizer_only_120_current"
skip_artifacts=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --artifact-dir)
      shift
      artifact_dir="$1"
      ;;
    --skip-artifacts)
      skip_artifacts=1
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

echo "== v1.1 claim-language check =="
python3 "${repo_root}/scripts/validate_v1_1_claim_language.py" --repo-root "${repo_root}"

echo "== v1.1 dry-run endpoint manifest hook expansion =="
example_manifest="${repo_root}/param/benchmark/v1_1_boreas_dry_run_endpoint.example.yaml"
hook_log="$(mktemp)"
set +e
source "${repo_root}/scripts/setup_local_env.sh" >/dev/null 2>&1
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest "${example_manifest}" \
  --print-only >"${hook_log}" 2>&1
hook_rc=$?
set -e
if [[ "${hook_rc}" -ne 0 ]]; then
  cat "${hook_log}" >&2
  exit "${hook_rc}"
fi

for required_hook in \
  "validate_relocalization_reset_candidate_plan.py" \
  "make_relocalization_reset_commands.py" \
  "validate_relocalization_reset_commands.py"
do
  if ! grep -q "${required_hook}" "${hook_log}"; then
    echo "Missing expected v1.1 hook in --print-only output: ${required_hook}" >&2
    cat "${hook_log}" >&2
    rm -f "${hook_log}"
    exit 1
  fi
done
rm -f "${hook_log}"
echo "hook expansion: ok"

if [[ "${skip_artifacts}" -eq 1 ]]; then
  echo "artifact validation: skipped"
  exit 0
fi

validation_json="${artifact_dir}/relocalization_reset_commands_validation.json"
if [[ ! -f "${validation_json}" ]]; then
  echo "artifact validation: skipped (missing ${validation_json})"
  exit 0
fi

python3 - "${validation_json}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
data = json.loads(path.read_text(encoding="utf-8"))
assert data.get("validation_passed") is True, data
assert int(data.get("summary_published_count", -1)) == 0, data
assert int(data.get("dry_run_generated_count", 0)) >= 1, data
print(f"artifact validation: ok ({path})")
PY

echo "v1.1 relocalization smoke: pass"
