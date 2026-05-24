# Benchmarking

This guide is the command entry point for replay, trajectory evaluation, and public regression checks.
Use it when you want to compare localization changes under repeatable rosbag playback.

## Goal

- replay the same bag with the same launch command
- record pose, diagnostics, CPU, and memory
- compare output against a reference trajectory when one exists
- keep public, private, and experimental benchmark claims clearly separated

## Quick Choice

| Goal | Command |
|---|---|
| Re-check the public validation boundary | `scripts/run_public_regression_suite.sh` |
| Run the release gate | `ros2 run lidar_localization_ros2 run_release_regression_suite.sh` |
| Run one dataset from a manifest | `ros2 run lidar_localization_ros2 benchmark_from_manifest --manifest ...` |
| Evaluate a pose CSV | `ros2 run lidar_localization_ros2 benchmark_eval_trajectory ...` |
| Compare multiple run directories | `ros2 run lidar_localization_ros2 benchmark_compare_runs ...` |
| Generate a Nav2 occupancy map from PCD | `ros2 run lidar_localization_ros2 generate_occupancy_map_from_pcd.py ...` |

Start every workflow from the repository root:

```bash
source scripts/setup_local_env.sh
```

## Build

```bash
cd ../build_ws
colcon build --symlink-install --packages-up-to lidar_localization_ros2
cd ../repo
source scripts/setup_local_env.sh
```

See [local_build.md](local_build.md) for the no-sudo local-prefix setup.

## Map Format Notes

- Runtime map paths accept `.ply` and `.pcd`.
- Public benchmark/runtime validation should prefer binary little-endian float32 `.ply` maps.
- Avoid Open3D's default double-precision PLY output for runtime benchmarking.
- Generated `.pcd` maps are useful for inspection, but are not the preferred benchmark path.
- Do not use a map built from the same evaluation run for external performance claims.

## Run a benchmark

### Run the public regression suite

Use this when you want one command that re-checks the current public validation boundary.

```bash
source scripts/setup_local_env.sh
scripts/run_public_regression_suite.sh
```

It currently checks:

- Autoware Istanbul `60 s` default-on no-IMU safety
- HDL `60 s` IMU safety / throughput regression, repeated twice by default

Outputs:

- `artifacts/public/public_regression_suite/summary.json`
- `artifacts/public/public_regression_suite/summary.md`

Latest recorded public validation snapshot:

- [public_validation_log.md](public_validation_log.md)
- `2026-05-22`, commit `2a5f11f`, release regression `overall_pass=true`
- Istanbul `60 s` no-IMU safety check: `translation_rmse_m=1.176`, `rotation_rmse_deg=0.393`
- HDL `60 s` IMU safety check: median pose rows `558.5 -> 553.5`
- Nav2 reinitialization supervisor `150 s`: requested rows `944 -> 7`

This is public replay and controlled Nav2 regression validation, not Jetson + MID-360 hardware
validation.

## Dataset Roles

Use the public datasets for different jobs. Do not treat Istanbul as the main benchmark for all
localization claims.

| Dataset | Role | Do not use it for |
|---|---|---|
| Autoware Istanbul | no-IMU urban replay safety, Nav2 replay regression, GNSS-referenced smoke | IMU preintegration claims, production long-horizon robustness claims |
| HDL sample | IMU pipeline smoke, throughput safety, direct `hdl_localization` sample compatibility | final backend ranking without a fair reference/initialization policy |
| Boreas | public `LiDAR + IMU + GT` candidate; current localizer-only manifests are diagnostic until prediction/map-split behavior is fixed | quick plug-and-play map-based claims or backend ranking before the workflow is validated |
| Koide hard localization | next controlled public benchmark candidate; indoor and outdoor `60 s` smoke runs are staged | IMU/preintegration claims before calibration and extrinsics are controlled |

The next strong benchmark track should promote Boreas or Koide-style public data, not deeper
Istanbul-only tuning.

### Koide smoke manifests

After staging Koide maps, GT, and sequence bags under `data/public/koide_hard_localization`, use:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_indoor_easy_01_smoke60.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_120.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_180_reinit090.yaml
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_180_reinit090_route_proximity_relocalization_artifacts.yaml
```

Latest local Koide snapshot:

- `2026-05-24`, `indoor_easy_01_smoke60`: `translation_rmse_m=0.079`, `rotation_rmse_deg=1.639`, `ok_rows=773/773`
- `2026-05-24`, `outdoor_hard_01a_smoke60`: `translation_rmse_m=0.209`, `rotation_rmse_deg=2.241`, `ok_rows=223/225`
- `2026-05-25`, `outdoor_hard_01a_120`: `translation_rmse_m=0.224`, `rotation_rmse_deg=2.083`, `ok_rows=482/499`
- `2026-05-25`, `outdoor_hard_01a_180`: `translation_rmse_m=1.197`, `rotation_rmse_deg=10.337`, `ok_rows=466/679`
- `2026-05-25`, `outdoor_hard_01a_180_reinit090`: `reinitialization_requested_rows=264`, `recovered_request_windows=0`

The bags contain IMU topics, but the current smoke configs keep IMU preintegration disabled where
the dataset calibration path is not controlled.

The current outdoor boundary is between 120 s and 180 s. In the `180 s` run, pose output stopped
after about `126.2 s`; the first fitness score above `100` appeared at stamp `1694532947.800806284`,
and the run ended with `179` consecutive rejected updates. Treat this as the next recovery and
relocalization target, not as a passing benchmark.

Lowering `reinitialization_trigger_threshold` to `0.90` raises `/reinitialization_requested`, but it
does not recover by itself. The first request appeared at stamp `1694532923.600409269` with reason
`fitness_exploded_reinit_requested`; the request window remained unrecovered. The existing
`enable_rejected_seed_update` diagnostic also does not clear the `180 s` boundary: it still stops
pose output at about `126.2 s` and ends near `translation_rmse_m=1.100`,
`rotation_rmse_deg=10.368`.

The `180_reinit090` request window is not empty. A route-grid diagnostic on `2026-05-25` generated
one attempt and `90` candidates. Reference-oracle scoring found candidate `49` at `0.000 m`
translation error. `candidate_index` ordering over the same top-32 scored rows selected candidate
`6` first, with oracle score `26.293 m`, so plain candidate order is unsafe as a reset source.
`route_proximity` ordering fixes this artifact failure without using `oracle_rank`: it sorts by
request-time proximity and route-center offsets, selected candidate `49` first, scored it with
NDT_OMP score `0.984596`, and produced a validated dry-run `/initialpose` command with
`published_count=0`. This is still not an automatic runtime recovery claim because reset
publication remains disabled and the route-grid corridor is an offline evaluation artifact.

### Run a manifest with health summary

Use `benchmark_from_manifest` for repeatable single-run or sweep benchmarks. It resolves paths,
writes a generated localization YAML into the output directory, copies the manifest for
reproducibility, records diagnostics, and evaluates the trajectory when `reference_csv` is present.

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest /absolute/path/to/run_manifest.yaml
```

Useful manifest templates:

- `param/benchmark/private_dataset_run_manifest.example.yaml`
- `param/benchmark/private_dataset_sweep_manifest.example.yaml`
- `param/benchmark/koide_hard_localization_run_manifest.example.yaml`
- `param/benchmark/v1_1_boreas_localizer_only.example.yaml`
- `param/benchmark/v1_1_koide_outdoor01_public_map_failure_boundary.example.yaml`

Common path prefixes:

- `repo://...`: resolve from this repository root
- `manifest://...`: resolve from the manifest directory
- normal relative paths: resolve from the manifest directory

Health summary is enabled by default for single-run manifests and writes:

- `health_summary.json`
- `health_summary.md`

Disable it only when the run intentionally has no `/alignment_status` recording:

```yaml
benchmark:
  write_health_summary: false
```

### Fetch a real benchmark dataset from hdl_localization

Use this for a reproducible public LiDAR bag from `hdl_localization`.

```bash
source scripts/setup_local_env.sh
scripts/fetch_official_hdl_localization_sample.sh
```

Prepared assets:

- `data/official/hdl_localization/hdl_400.bag`
- `data/official/hdl_localization/map.pcd`
- `data/official/hdl_localization/hdl_400_ros2`

Use this official sample for externally shared `hdl_localization`-style results. Cite the upstream
repository and dataset URL instead of presenting local field logs as open benchmark data.

### Fetch the Autoware Istanbul no-IMU regression dataset

Use the official Autoware Istanbul localization assets for no-IMU urban replay and Nav2 regression.
It is useful, but it should not be the main dataset for IMU or long-horizon robustness claims.

```bash
source scripts/setup_local_env.sh
scripts/fetch_official_autoware_istanbul_dataset.sh
```

Prepared assets:

- `data/official/autoware_istanbul/pointcloud_map.pcd`
- `data/official/autoware_istanbul/localization_rosbag`

Recommended topics:

- cloud: `/localization/util/downsample/pointcloud`
- reference pose: `/sensing/gnss/pose_with_covariance`
- twist: `/localization/twist_estimator/twist_with_covariance`

Extract a reference CSV and initial-pose YAML directly from the bag:

```bash
ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 \
  --bag-path data/official/autoware_istanbul/localization_rosbag \
  --pose-topic /sensing/gnss/pose_with_covariance \
  --sample-topic /localization/util/downsample/pointcloud \
  --bag-duration 60 \
  --initial-pose-skip-sec 0.05 \
  --output-csv /tmp/autoware_istanbul_reference_60s.csv \
  --output-initial-pose-yaml /tmp/autoware_istanbul_initial_pose_60s.yaml
```

Use a unique `ROS_DOMAIN_ID` for replay so unrelated ROS 2 traffic cannot leak into the benchmark.

### Run a private or NC dataset without committing raw paths

Keep private bag, map, and ground-truth paths outside the repository. Drive the run through a manifest:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest /absolute/path/to/private_dataset_run_manifest.yaml
```

Rules:

- commit only example manifests and public benchmark configs
- keep real manifests outside the repo or under ignored names such as `*.local.yaml`
- do not publish NC/private results as open benchmark artifacts

### Run a public LiDAR+IMU dataset that has no packaged map

When a public dataset has LiDAR, IMU, and GT but no packaged pointcloud map, keep mapping and
localization as separate runs.

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 scaffold_mapless_public_dataset_bundle.py \
  --spec /absolute/path/to/mapless_public_dataset_bundle.yaml
```

Start from:

- `param/benchmark/mapless_public_dataset_bundle.example.yaml`
- `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml`

The generated bundle contains mapping, reference extraction, localization benchmark, and Nav2 map
generation scripts. See [mapless_public_dataset_workflow.md](mapless_public_dataset_workflow.md).

### Boreas starter

Convert a Boreas raw sequence to rosbag2, then scaffold the split mapping/localization bundle.

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 convert_boreas_sequence_to_rosbag2.py \
  --sequence-dir /absolute/path/to/boreas-sequence \
  --bag-dir /tmp/boreas_sequence_rosbag2 \
  --force
```

Then use:

- `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml`
- [boreas_mapless_public_dataset_workflow.md](boreas_mapless_public_dataset_workflow.md)

### Generate a first synthetic dataset from graph PCDs

Use this only for smoke/debug when no real rosbag2 dataset is available locally.

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_make_graph_dataset \
  --graph-dir /path/to/graph \
  --bag-dir /tmp/lidarloc_first_result/bag \
  --map-pcd /tmp/lidarloc_first_result/synthetic_map.pcd \
  --param-yaml /tmp/lidarloc_first_result/benchmark_ndt_omp.yaml \
  --reference-csv /tmp/lidarloc_first_result/reference_pose.csv \
  --sequence-start 0 \
  --sequence-count 20 \
  --map-point-stride 8 \
  --cloud-point-stride 1 \
  --registration-method NDT_OMP \
  --force
```

Do not use graph-derived synthetic results as public open-data claims.

### Benchmark this package

Use `benchmark_runner` directly when you already have a bag and launch command.

```bash
ros2 run lidar_localization_ros2 benchmark_runner \
  --bag-path /path/to/rosbag2 \
  --output-dir /tmp/lidarloc_benchmark/run_001 \
  --ros-domain-id 92 \
  --bag-duration 60 \
  --system-command "ros2 launch lidar_localization_ros2 lidar_localization.launch.py" \
  --target-process-pattern lidar_localization_node \
  --record-topic /pcl_pose \
  --diagnostic-topic /alignment_status
```

`--bag-duration` is enforced by the runner itself, not by `ros2 bag play`, so the command works on
ROS 2 Humble.

### Benchmark another system with the same harness

```bash
ros2 run lidar_localization_ros2 benchmark_runner \
  --bag-path /path/to/rosbag2 \
  --output-dir /tmp/lidarloc_benchmark/other_system \
  --system-command "ros2 launch some_other_package some_launch.py" \
  --target-process-pattern some_localizer_executable \
  --record-topic /localization/pose_with_covariance
```

Keep bag path, reference CSV, output directory naming, and evaluation command identical when comparing
two systems.

## Artifacts

Each direct `benchmark_runner` run stores:

- `pose_trace.csv`
- `alignment_status.csv`
- `resource_trace.csv`
- `summary.json`
- process logs for the system, recorders, and bag playback

`summary.json`, `trajectory_eval.json`, `health_summary.json`, and comparison JSON files are the
preferred artifacts to commit or attach to reports. Keep raw bags and large generated outputs outside
the repository.

## Evaluate against reference trajectory

```bash
ros2 run lidar_localization_ros2 benchmark_eval_trajectory \
  --estimated-csv /tmp/lidarloc_benchmark/run_001/pose_trace.csv \
  --reference-csv /path/to/reference_pose.csv \
  --output-json /tmp/lidarloc_benchmark/run_001/trajectory_eval.json
```

For `evo_ape` output:

```bash
ros2 run lidar_localization_ros2 benchmark_eval_evo_ape \
  --estimated-csv /tmp/lidarloc_benchmark/run_001/pose_trace.csv \
  --reference-csv /path/to/reference_pose.csv \
  --output-json /tmp/lidarloc_benchmark/run_001/evo_ape_translation.json \
  --save-results-zip /tmp/lidarloc_benchmark/run_001/evo_ape_translation.zip
```

For split rosbag2 sequences with a TUM GT file:

```bash
ros2 run lidar_localization_ros2 tum_trajectory_to_pose_reference_csv_for_rosbag2.py \
  --input /path/to/gt.tum \
  --bag-path /path/to/rosbag2 \
  --output-csv /tmp/reference.csv \
  --output-initial-pose-yaml /tmp/initial_pose.yaml \
  --initial-pose-skip-sec 0.05
```

To build a GT-aligned diagnostic map:

```bash
ros2 run lidar_localization_ros2 build_gt_aligned_map_from_reference_csv.py \
  --bag-path /path/to/rosbag2 \
  --reference-csv /tmp/reference.csv \
  --cloud-topic /livox/points \
  --point-stride 10 \
  --voxel-size 0.5 \
  --output-map /tmp/gt_aligned_map.ply
```

This is an engineering check for map coverage, not an external benchmark map policy.

## Compare multiple runs

```bash
ros2 run lidar_localization_ros2 benchmark_compare_runs \
  --run-dir /tmp/lidarloc_benchmark/run_ndt_omp \
  --run-dir /tmp/lidarloc_benchmark/run_small_gicp \
  --output-json /tmp/lidarloc_benchmark/comparison.json
```

For Nav2 replay supervisor-policy regression:

```bash
ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_regression.sh
```

For the combined release-style boundary:

```bash
ros2 run lidar_localization_ros2 run_release_regression_suite.sh
```

Release outputs:

- `artifacts/public/release_regression_suite/summary.json`
- `artifacts/public/release_regression_suite/summary.md`

## Relocalization Artifacts

The v1.1 relocalization path is artifact-first. The public endpoint is a validated dry-run
`/initialpose` command artifact, not automatic reset publication.

Typical advanced sequence:

1. Summarize alignment health with `summarize_localization_health.py`.
2. Generate candidate attempts with `make_route_grid_relocalization_attempts.py`.
3. Create and resolve registration jobs with `make_registration_relocalization_jobs.py` and
   `resolve_registration_relocalization_scans.py`.
4. Score bounded jobs with `relocalization_ndt_score_jobs`.
5. Create and validate a dry-run reset plan with `select_relocalization_reset_candidates.py`,
   `make_relocalization_reset_commands.py`, and `validate_relocalization_reset_commands.py`.

Use each script's `--help` output for the required CSV paths and thresholds.

Guarded publication is experimental and requires explicit `--execute`:

```bash
ros2 run lidar_localization_ros2 publish_relocalization_reset_commands.py \
  --commands-csv /path/to/relocalization_reset_commands.csv \
  --validation-json /path/to/relocalization_reset_commands_validation.json \
  --output-csv /path/to/relocalization_reset_command_execution.csv
```

Without `--execute`, it only writes a no-publish execution report.

## Provided tools

Core run and evaluation:

- `benchmark_runner`
- `benchmark_from_manifest`
- `benchmark_pose_recorder`
- `benchmark_diagnostic_recorder`
- `benchmark_eval_trajectory`
- `benchmark_eval_evo_ape`
- `benchmark_compare_runs`
- `benchmark_sweep_localizer`

Dataset preparation:

- `fetch_official_hdl_localization_sample.sh`
- `fetch_official_autoware_istanbul_dataset.sh`
- `fetch_koide_hard_pointcloud_localization_dataset.sh`
- `convert_boreas_sequence_to_rosbag2.py`
- `scaffold_mapless_public_dataset_bundle.py`
- `benchmark_make_graph_dataset`
- `generate_occupancy_map_from_pcd.py`
- `benchmark_extract_pose_reference_from_rosbag2`
- `tum_trajectory_to_pose_reference_csv_for_rosbag2.py`
- `build_gt_aligned_map_from_reference_csv.py`

Regression and Nav2 comparison:

- `run_public_regression_suite.sh`
- `run_nav2_replay_smoke`
- `run_nav2_reinit_supervisor_regression.sh`
- `run_nav2_reinit_supervisor_sweep.sh`
- `run_release_regression_suite.sh`
- `compare_nav2_reinit_supervisor_runs.py`

Relocalization artifact pipeline:

- `summarize_localization_health.py`
- `summarize_relocalization_attempts.py`
- `make_disabled_relocalization_attempts.py`
- `make_route_grid_relocalization_attempts.py`
- `score_relocalization_candidates_with_reference.py`
- `make_registration_relocalization_jobs.py`
- `resolve_registration_relocalization_scans.py`
- `relocalization_ndt_score_jobs`
- `summarize_registration_relocalization_scores.py`
- `compare_registration_relocalization_score_summaries.py`
- `run_registration_ordering_comparison.py`
- `select_relocalization_reset_candidates.py`
- `validate_relocalization_reset_candidate_plan.py`
- `make_relocalization_reset_commands.py`
- `validate_relocalization_reset_commands.py`
- `publish_relocalization_reset_commands.py`
- `observe_relocalization_reset_execution.py`
- `build_relocalization_demo_report.py`

## Recommended metrics

At minimum, compare:

- translation RMSE
- rotation RMSE
- matched sample count
- lost-track or failure-like rows
- reinitialization request rows and request windows
- median, p95, and p99 alignment time
- median and peak CPU usage
- median and peak memory usage

## Current limitations

- `benchmark_runner` monitors one target process selected by substring match.
- Trajectory evaluation uses nearest-timestamp association and assumes compatible clocks.
- Relocalization success rate still depends on task-specific scoring outside simple RMSE.
- The official `hdl_localization` sample still needs a fair initial pose policy before final backend
  ranking.
