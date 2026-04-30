#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the full lidar_localization release regression suite.

This suite combines:
  - the public regression suite
  - the long Nav2 reinitialization-supervisor regression

Usage:
  run_release_regression_suite.sh [options]

Options:
  --work-dir DIR                Working directory for generated run outputs.
                                Default: /tmp/lidarloc_release_regression_suite
  --report-dir DIR              Directory for final summary artifacts.
                                Default: <workspace>/artifacts/public/release_regression_suite
  --public-ros-domain-base N    Base ROS domain id for the public suite. Default: 200
  --nav2-ros-domain-base N      Base ROS domain id for the Nav2 regression. Default: 210
  --hdl-repeat-count N          HDL repeat count passed to the public suite. Default: 2
  --resume                      Reuse existing completed outputs when possible.
  -h, --help                    Show this help.
EOF
}

script_dir="$(cd "$(dirname "$0")" && pwd)"
if [[ -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/.." && pwd)}"
else
  package_prefix="$(cd "${script_dir}/../.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${package_prefix}/../../.." && pwd)}"
  repo_root="${LIDAR_LOCALIZATION_REPO_ROOT:-${ws_root}/repo}"
fi

if [[ ! -f "${repo_root}/scripts/setup_local_env.sh" ]]; then
  echo "repo root does not look valid: ${repo_root}" >&2
  exit 1
fi

work_dir="/tmp/lidarloc_release_regression_suite"
report_dir="${ws_root}/artifacts/public/release_regression_suite"
public_ros_domain_base=200
nav2_ros_domain_base=210
hdl_repeat_count=2
resume=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      shift
      work_dir="$1"
      ;;
    --report-dir)
      shift
      report_dir="$1"
      ;;
    --public-ros-domain-base)
      shift
      public_ros_domain_base="$1"
      ;;
    --nav2-ros-domain-base)
      shift
      nav2_ros_domain_base="$1"
      ;;
    --hdl-repeat-count)
      shift
      hdl_repeat_count="$1"
      ;;
    --resume)
      resume=1
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

if ! [[ "${hdl_repeat_count}" =~ ^[0-9]+$ ]] || (( hdl_repeat_count < 1 )); then
  echo "--hdl-repeat-count must be a positive integer" >&2
  exit 1
fi

public_max_domain=$((public_ros_domain_base + (2 * hdl_repeat_count)))
nav2_max_domain=$((nav2_ros_domain_base + 1))
if (( public_max_domain > 232 || nav2_max_domain > 232 )); then
  echo "ROS domain range exceeds Fast DDS limit 232" >&2
  exit 1
fi

public_output_dir="${work_dir}/public"
public_report_dir="${report_dir}/public_regression_suite"
nav2_output_dir="${report_dir}/nav2_reinit_supervisor_regression_150"
summary_json="${report_dir}/summary.json"
summary_md="${report_dir}/summary.md"

mkdir -p "${work_dir}" "${report_dir}"

set +u
source "${repo_root}/scripts/setup_local_env.sh"
set -u

public_cmd=(
  "${repo_root}/scripts/run_public_regression_suite.sh"
  --output-dir "${public_output_dir}"
  --report-dir "${public_report_dir}"
  --ros-domain-base "${public_ros_domain_base}"
  --hdl-repeat-count "${hdl_repeat_count}"
)
nav2_cmd=(
  "${repo_root}/scripts/run_nav2_reinit_supervisor_regression.sh"
  --output-dir "${nav2_output_dir}"
  --ros-domain-base "${nav2_ros_domain_base}"
)

if [[ "${resume}" == "1" ]]; then
  public_cmd+=(--resume)
  nav2_cmd+=(--resume)
fi

"${public_cmd[@]}"
"${nav2_cmd[@]}"

python3 - "${public_report_dir}/summary.json" "${nav2_output_dir}/regression_result.json" "${summary_json}" "${summary_md}" <<'PY'
import json
import sys
from pathlib import Path

public_summary_path = Path(sys.argv[1])
nav2_summary_path = Path(sys.argv[2])
summary_json_path = Path(sys.argv[3])
summary_md_path = Path(sys.argv[4])

public_summary = json.loads(public_summary_path.read_text(encoding="utf-8"))
nav2_summary = json.loads(nav2_summary_path.read_text(encoding="utf-8"))

result = {
    "overall_pass": bool(public_summary.get("overall_pass")) and bool(nav2_summary.get("overall_pass")),
    "public_regression_suite": {
        "overall_pass": bool(public_summary.get("overall_pass")),
        "summary_json": str(public_summary_path),
        "istanbul_pass": bool(public_summary.get("istanbul", {}).get("pass")),
        "hdl_pass": bool(public_summary.get("hdl", {}).get("pass")),
    },
    "nav2_reinitialization_supervisor_regression": {
        "overall_pass": bool(nav2_summary.get("overall_pass")),
        "summary_json": str(nav2_summary_path),
        "recommended_run": nav2_summary.get("recommended_run"),
        "candidate_goal_status": nav2_summary.get("candidate", {}).get("goal_status"),
        "candidate_ok_rows_after_first_trigger": nav2_summary.get("candidate", {}).get("alignment", {}).get("ok_rows_after_first_trigger"),
        "baseline_requested_rows": nav2_summary.get("baseline", {}).get("alignment", {}).get("reinitialization_requested_rows"),
        "candidate_requested_rows": nav2_summary.get("candidate", {}).get("alignment", {}).get("reinitialization_requested_rows"),
    },
}

summary_json_path.parent.mkdir(parents=True, exist_ok=True)
summary_json_path.write_text(json.dumps(result, indent=2, sort_keys=False) + "\n", encoding="utf-8")

summary_md_path.write_text(
    "\n".join(
        [
            "# Release Regression Suite",
            "",
            f"- overall_pass: `{result['overall_pass']}`",
            "",
            "## Public Regression Suite",
            f"- overall_pass: `{result['public_regression_suite']['overall_pass']}`",
            f"- istanbul_pass: `{result['public_regression_suite']['istanbul_pass']}`",
            f"- hdl_pass: `{result['public_regression_suite']['hdl_pass']}`",
            f"- summary_json: `{result['public_regression_suite']['summary_json']}`",
            "",
            "## Nav2 Reinitialization Supervisor Regression",
            f"- overall_pass: `{result['nav2_reinitialization_supervisor_regression']['overall_pass']}`",
            f"- recommended_run: `{result['nav2_reinitialization_supervisor_regression']['recommended_run']}`",
            f"- candidate_goal_status: `{result['nav2_reinitialization_supervisor_regression']['candidate_goal_status']}`",
            f"- candidate_ok_rows_after_first_trigger: `{result['nav2_reinitialization_supervisor_regression']['candidate_ok_rows_after_first_trigger']}`",
            f"- requested_rows: `{result['nav2_reinitialization_supervisor_regression']['baseline_requested_rows']} -> {result['nav2_reinitialization_supervisor_regression']['candidate_requested_rows']}`",
            f"- summary_json: `{result['nav2_reinitialization_supervisor_regression']['summary_json']}`",
            "",
            f"- combined_summary_json: `{summary_json_path}`",
        ]
    )
    + "\n",
    encoding="utf-8",
)

print(json.dumps(result, indent=2, sort_keys=False))
sys.exit(0 if result["overall_pass"] else 1)
PY

echo "summary_json: ${summary_json}"
echo "summary_md: ${summary_md}"
