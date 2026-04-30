# Benchmarking

## Goal

This repository now includes a small benchmark harness so that localization changes can be evaluated under repeatable rosbag playback.

## Provided tools

- `benchmark_runner`
  - Starts a system under test
  - Records a pose topic into CSV
  - Replays a rosbag
  - Samples CPU and memory usage for the target process
  - Saves logs and a machine-readable `summary.json`
- `benchmark_pose_recorder`
  - Records `geometry_msgs/msg/PoseWithCovarianceStamped` into CSV
- `benchmark_diagnostic_recorder`
  - Records `diagnostic_msgs/msg/DiagnosticArray` into CSV
- `benchmark_eval_trajectory`
  - Compares an estimated trajectory CSV against a reference trajectory CSV
- `benchmark_eval_evo_ape`
  - Converts the benchmark CSVs to TUM format and evaluates them with `evo_ape`
  - Useful when you need a paper-style `ATE` workflow instead of only the built-in RMSE JSON
- `tum_trajectory_to_pose_reference_csv_for_rosbag2.py`
  - Crops a TUM trajectory to the time window of a rosbag2 bag and exports benchmark `reference.csv`
  - Useful for split datasets such as Koide `outdoor_hard_01a` / `01b`, where one GT trajectory spans multiple bag files
- `build_gt_aligned_map_from_reference_csv.py`
  - Builds a diagnostic map by transforming PointCloud2 scans with benchmark `reference.csv` poses
  - Writes float32 binary `.ply` output by default so the runtime stack sees `property float x/y/z`
  - Useful for proving whether a failure is due to map coverage or to registration itself
- `benchmark_compare_runs`
  - Aggregates `summary.json`, `trajectory_eval.json`, and `alignment_status.csv`
  - Prints a quick human-readable comparison across multiple run directories
  - Includes reinitialization-request observability from `alignment_status.csv`, such as
    `reinitialization_requested_rows`, `reinitialization_request_score_max`, and the first trigger reason
  - Optionally writes a machine-readable comparison JSON
- `compare_nav2_reinit_supervisor_runs.py`
  - Aggregates one or more `run_nav2_replay_smoke` log directories
  - Compares reinitialization-request windows and post-trigger `ok` recovery
  - Writes a compact JSON/Markdown summary for supervisor-policy comparisons
- `run_nav2_reinit_supervisor_regression.sh`
  - Runs a fixed `120 s` Istanbul replay pair: `no_supervisor` vs `configured_initial_pose_count1`
  - Uses the compare helper and enforces pass/fail checks for the current recommended policy
- `run_release_regression_suite.sh`
  - Runs the short public regression suite and the long Nav2 supervisor regression
  - Writes one combined pass/fail summary for release-style validation
- `benchmark_make_graph_dataset`
  - Builds a small synthetic map from graph PCDs
  - Writes a rosbag2 input with `/velodyne_points` and `/initialpose`
  - Writes a matching reference trajectory CSV
  - Writes a dataset-specific parameter YAML for `lidar_localization_ros2`

## Build

```bash
colcon build --packages-select lidar_localization_ros2
source install/setup.bash
```

## Map Format Notes

- Runtime manifests accept `.ply` and `.pcd` paths, but current benchmark validation in this repo
  should prefer `.ply`.
- For generated maps, use binary little-endian float32 PLY fields (`property float x/y/z`).
- Avoid Open3D's default double-precision PLY output for runtime benchmarking.
- Generated `.pcd` maps are still useful for inspection tools, but they are not the recommended
  benchmark/runtime path in this repo today.

## Run a benchmark

### Run the public regression suite

Use this when you want one command that re-checks the repository's public validation boundary.

```bash
source scripts/setup_local_env.sh
scripts/run_public_regression_suite.sh
```

This runs:

- Autoware Istanbul `60 s` default-on run to verify that the recommended IMU-capable default does not break a no-IMU dataset
- HDL `60 s` with `use_imu_preintegration=false/true`, repeated twice by default, to verify IMU safety and throughput

Outputs:

- working runs under `/tmp/lidarloc_public_regression_suite` by default
- final summary at `artifacts/public/public_regression_suite/summary.json`
- human-readable snapshot at `artifacts/public/public_regression_suite/summary.md`

Current pass criteria:

- Istanbul keeps `imu_prediction_active_rows=0`, stays alive during replay, and clears broad `pose_rows` / `matched_sample_count` / RMSE sanity bounds
- HDL keeps all repeated runs alive during replay, keeps both repeated groups above a broad `pose_rows` floor, requires the IMU-enabled group median to stay under a broad median alignment-time ceiling, uses IMU prediction at least once, and allows at most one automatic IMU disable fallback

Interpretation:

- Istanbul has no accelerometer stream, so it is only a default-on no-IMU safety check, not an IMU benefit benchmark
- HDL has no strong public ground truth and single-run pose-row ratios are noisy, so the suite uses broad smoke bounds there rather than a strict acceptance benchmark

### Fetch a real benchmark dataset from hdl_localization

Use the helper script when you want a reproducible real LiDAR bag instead of the graph-derived synthetic smoke test.

```bash
source scripts/setup_local_env.sh
scripts/fetch_official_hdl_localization_sample.sh
```

The script downloads the official `hdl_localization` example assets and prepares:

- `data/official/hdl_localization/hdl_400.bag`
- `data/official/hdl_localization/map.pcd`
- `data/official/hdl_localization/hdl_400_ros2`

Upstream sources:

- `https://github.com/koide3/hdl_localization`
- `https://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz`
- `https://raw.githubusercontent.com/koide3/hdl_localization/master/data/map.pcd`

The generated `rosbag2` dataset keeps only standard ROS message types:

- `/velodyne_points`
- `/gpsimu_driver/imu_data`
- `/gpsimu_driver/gpstime`

Important caveat:

- `hdl_localization`'s original demo expects a relocalization step
- `lidar_localization_ros2` does not have equivalent automatic global relocalization yet
- use this dataset once you have either a good initial pose or an explicit relocalization experiment plan

Publication rule:

- use this official `hdl_400` sample for any benchmark result that will be published or shared outside the local workspace
- do not present local field-recorded bags or graph-derived synthetic bags as open benchmark data
- cite the upstream `hdl_localization` repository and dataset URL in any published report
- prefer linking to the upstream bag download instead of rehosting the raw bag, unless dataset redistribution terms are clarified separately

### Fetch a stronger public map-based localization dataset from Autoware

Use the official Autoware Istanbul localization assets when you want a harder public benchmark with a
pointcloud map and a localization-only ROS 2 bag.

```bash
source scripts/setup_local_env.sh
scripts/fetch_official_autoware_istanbul_dataset.sh
```

The script prepares:

- `data/official/autoware_istanbul/pointcloud_map.pcd`
- `data/official/autoware_istanbul/localization_rosbag`

The official localization-only bag contains:

- `/localization/util/downsample/pointcloud`
- `/sensing/gnss/pose`
- `/sensing/gnss/pose_with_covariance`
- `/localization/twist_estimator/twist_with_covariance`
- `/clock`
- `/tf_static`

Recommended use:

- use `/localization/util/downsample/pointcloud` as the input cloud topic
- use `/sensing/gnss/pose_with_covariance` as the reference trajectory source
- keep `hdl_400` as the direct `hdl_localization` baseline, and use Istanbul as the harder public benchmark

Official sources:

- `https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/datasets/index.md`
- `https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/simulation-evaluation/components_evaluation/localization_evaluation/urban-environment-evaluation.md`
- `https://github.com/orgs/autowarefoundation/discussions/5135`

Example benchmark command:

```bash
ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 \
  --bag-path data/official/autoware_istanbul/localization_rosbag \
  --pose-topic /sensing/gnss/pose_with_covariance \
  --sample-topic /localization/util/downsample/pointcloud \
  --bag-duration 30 \
  --initial-pose-skip-sec 0.05 \
  --output-csv /tmp/autoware_istanbul_reference_30s.csv \
  --output-initial-pose-yaml /tmp/autoware_istanbul_initial_pose_30s.yaml
```

Use a unique ROS domain when replaying the bag to avoid unrelated ROS 2 traffic leaking into the benchmark:

```bash
ros2 run lidar_localization_ros2 benchmark_runner \
  --bag-path data/official/autoware_istanbul/localization_rosbag \
  --output-dir /tmp/lidarloc_istanbul_ndt_30s \
  --ros-domain-id 92 \
  --bag-duration 30 \
  --system-command "ros2 launch lidar_localization_ros2 lidar_localization.launch.py localization_param_dir:=/tmp/autoware_istanbul_localization.yaml cloud_topic:=/localization/util/downsample/pointcloud" \
  --target-process-pattern lidar_localization_node \
  --record-topic /pcl_pose \
  --diagnostic-topic /alignment_status
```

The temporary `localization_param_dir` should contain:

- the Istanbul `map_path`
- a fair initial pose, typically taken from `--output-initial-pose-yaml`
- for Istanbul specifically, skip the very first GNSS sample because its yaw can be unstable at replay start
- any dataset-specific tuning, such as a relaxed `score_threshold`

Clean Istanbul 30-second snapshot:

- reference extraction: direct bag read with `benchmark_extract_pose_reference_from_rosbag2`
- stable-init threshold sweep summary CSV: `/tmp/lidarloc_istanbul_stable_threshold_sweep/summary.csv`
- twist-linear sweep summary CSV: `/tmp/lidarloc_istanbul_twist_linear_threshold_sweep/summary.csv`
- best current run directory: `/tmp/lidarloc_istanbul_twist_linear_threshold_sweep/twist_linear_thr6`
- report: `artifacts/public/autoware_istanbul_ndt_30s/index.html`
- matched samples: `109 / 238`
- translation RMSE vs GNSS reference: `1.163 m`
- rotation RMSE vs GNSS reference: `0.383 deg`

Interpretation:

- the Istanbul benchmark wiring is now stable enough for public reporting
- the first exported GNSS sample can produce a bad initial yaw on this dataset
- skipping that unstable sample and then tightening `score_threshold` to `5-10` improves the result dramatically
- adding linear-only twist prediction improves the current `NDT_OMP` public 30-second result further
- remaining work is in localization quality, not in reference extraction or ROS graph hygiene

Quick sweep command:

```bash
python3 scripts/benchmark_sweep_localizer \
  --bag-path data/official/autoware_istanbul/localization_rosbag \
  --reference-csv /tmp/lidarloc_istanbul_ref_direct_30s.csv \
  --base-param-yaml /tmp/lidarloc_istanbul_ndt_30s_direct.yaml \
  --config-json param/benchmark/autoware_istanbul_ndt_quick_sweep.json \
  --output-dir /tmp/lidarloc_istanbul_ndt_quick_sweep \
  --map-path data/official/autoware_istanbul/pointcloud_map.pcd \
  --cloud-topic /localization/util/downsample/pointcloud \
  --bag-duration 30 \
  --settle-seconds 5 \
  --post-roll-seconds 1
```

Stable-init threshold sweep command:

```bash
python3 scripts/benchmark_sweep_localizer \
  --bag-path data/official/autoware_istanbul/localization_rosbag \
  --reference-csv /tmp/lidarloc_istanbul_ref_direct_30s.csv \
  --base-param-yaml /tmp/lidarloc_istanbul_ndt_30s_stable_base.yaml \
  --config-json /tmp/lidarloc_istanbul_stable_threshold_sweep.json \
  --output-dir /tmp/lidarloc_istanbul_stable_threshold_sweep \
  --map-path data/official/autoware_istanbul/pointcloud_map.pcd \
  --cloud-topic /localization/util/downsample/pointcloud \
  --bag-duration 30 \
  --settle-seconds 5 \
  --post-roll-seconds 1
```

Twist-linear threshold sweep command:

```bash
python3 scripts/benchmark_sweep_localizer \
  --bag-path data/official/autoware_istanbul/localization_rosbag \
  --reference-csv /tmp/lidarloc_autoware_istanbul_reference_30s_stable.csv \
  --base-param-yaml /tmp/lidarloc_istanbul_ndt_30s_stable_base.yaml \
  --config-json param/benchmark/autoware_istanbul_ndt_twist_linear_sweep.json \
  --output-dir /tmp/lidarloc_istanbul_twist_linear_threshold_sweep \
  --cloud-topic /localization/util/downsample/pointcloud \
  --twist-topic /localization/twist_estimator/twist_with_covariance \
  --bag-duration 30 \
  --settle-seconds 5 \
  --post-roll-seconds 1
```

Twist-linear sweep snapshot:

- `thr5`: translation `1.740 m`, rotation `0.115 deg`, matched `105`
- `thr6`: translation `1.163 m`, rotation `0.383 deg`, matched `109`
- `thr7`: translation `1.970 m`, rotation `0.238 deg`, matched `108`
- `thr8`: translation `1.378 m`, rotation `0.586 deg`, matched `110`
- `thr10`: translation `2.895 m`, rotation `0.704 deg`, matched `116`

At the moment `twist_linear_thr6` is the best balanced public setting on the first `30 s`.

Istanbul `60 s` predictor comparison:

```bash
source scripts/setup_local_env.sh
scripts/run_autoware_istanbul_60s_predictor_compare.sh
```

This produces:

- a stable-init `60 s` reference CSV and initial pose YAML
- two run directories under `/tmp/lidarloc_autoware_istanbul_60s_predictor_compare/runs`
- a comparison JSON under `artifacts/public/istanbul_60s_predictor_compare/comparison.json`
- one drift-correlation HTML and JSON pair for each run

### Run a private or NC dataset without committing raw paths

When the real acceptance bag cannot be published, keep the raw bag, map, and ground-truth files
outside the repository and drive the benchmark through a manifest file instead.

Single-run template:

- `param/benchmark/private_dataset_run_manifest.example.yaml`

Sweep template:

- `param/benchmark/private_dataset_sweep_manifest.example.yaml`

The manifest keeps:

- dataset paths such as `bag_path`, `map_path`, and optional `reference_csv`
- topic names such as `cloud_topic`, `twist_topic`, and `imu_topic`
- the base parameter YAML plus any dataset-specific parameter overrides
- benchmark timing and ROS domain settings

Path rules:

- normal relative paths are resolved relative to the manifest file
- `repo://...` resolves relative to this repository root
- `manifest://...` explicitly resolves relative to the manifest file

Run one benchmark:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest /absolute/path/to/private_dataset_run_manifest.yaml
```

Run a sweep:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 benchmark_from_manifest \
  --manifest /absolute/path/to/private_dataset_sweep_manifest.yaml
```

What the wrapper does:

- resolves all relative paths relative to the manifest file itself
- writes a generated localization YAML into the benchmark output directory
- copies the manifest into the output directory for reproducibility
- runs `benchmark_runner` or `benchmark_sweep_localizer`
- runs `benchmark_eval_trajectory` automatically when `reference_csv` is present in single-run mode

Operational rule:

- keep the real manifest outside the repository or under a user-specific ignored path
- if you keep it inside the repo, prefer `param/benchmark/*.local.yaml` or `param/benchmark/private/`
- commit only example manifests and public benchmark configs
- do not publish NC dataset results as if they were open benchmark artifacts

### Run a public LiDAR+IMU dataset that has no packaged map

When the public dataset includes LiDAR, IMU, and GT but no packaged pointcloud map, keep mapping and
localization as separate runs.

Use the scaffold helper:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 scaffold_mapless_public_dataset_bundle.py \
  --spec /absolute/path/to/mapless_public_dataset_bundle.yaml
```

Start from:

- `param/benchmark/mapless_public_dataset_bundle.example.yaml`
- `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml` for the Boreas starter path

The generated bundle contains:

- `run_mapping.sh`
- `run_extract_reference.sh` when the spec includes `reference:`
- `localization_run_manifest.yaml`
- `run_localization_benchmark.sh`
- `run_generate_nav2_map.sh`
- `README.md`

Design rule:

- use one run to create the map
- use a different run to evaluate localization against that map

If the dataset carries GT in the localization bag, prefer using the bundle `reference:` section.
That adds a bundle-local reference CSV and initial-pose YAML extraction step before the benchmark.

This lets you use an external mapper such as `lidarslam_ros2` without baking mapper-specific logic into
`benchmark_from_manifest`.

### Boreas starter

For a first public `LiDAR + IMU + GT` dataset with this workflow, start from Boreas:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 convert_boreas_sequence_to_rosbag2.py \
  --sequence-dir /absolute/path/to/boreas-sequence \
  --bag-dir /tmp/boreas_sequence_rosbag2 \
  --force
```

Then scaffold the split mapping/localization bundle from:

- `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml`

Reference:

- [docs/boreas_mapless_public_dataset_workflow.md](boreas_mapless_public_dataset_workflow.md)

### Generate a first synthetic dataset from graph PCDs

This is the fastest way to produce a first result when no rosbag2 file is available locally.

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

The generated parameter file expects the standard launch file and the generated bag includes:

- `/velodyne_points`
- `/initialpose`

### Benchmark this package

```bash
ros2 run lidar_localization_ros2 benchmark_runner \
  --bag-path /path/to/rosbag2 \
  --output-dir /tmp/lidarloc_benchmark/run_001 \
  --system-command "ros2 launch lidar_localization_ros2 lidar_localization.launch.py" \
  --target-process-pattern lidar_localization_node \
  --record-topic /pcl_pose \
  --diagnostic-topic /alignment_status
```

`benchmark_runner --bag-duration` is enforced by the runner itself, not by `ros2 bag play`.
This keeps the same command line working on ROS 2 Humble, where `ros2 bag play` does not accept `--duration`.

### Benchmark another system with the same harness

```bash
ros2 run lidar_localization_ros2 benchmark_runner \
  --bag-path /path/to/rosbag2 \
  --output-dir /tmp/lidarloc_benchmark/autoware_ndt \
  --system-command "ros2 launch some_other_package some_launch.py" \
  --target-process-pattern some_localizer_executable \
  --record-topic /localization/pose_with_covariance
```

## Artifacts

Each run stores:

- `pose_trace.csv`
- `alignment_status.csv`
- `resource_trace.csv`
- `summary.json`
- `system.stdout.log`
- `system.stderr.log`
- `pose_recorder.stdout.log`
- `pose_recorder.stderr.log`
- `diagnostic_recorder.stdout.log`
- `diagnostic_recorder.stderr.log`
- `bag_play.stdout.log`
- `bag_play.stderr.log`

`summary.json` is intended to be committed or attached to reports.

If the localizer emits reinitialization diagnostics, `alignment_status.csv` also captures:

- `reinitialization_requested`
- `reinitialization_request_reason`
- `reinitialization_request_score`

## Evaluate against reference trajectory

The evaluator expects the same CSV column names produced by `benchmark_pose_recorder`.

```bash
ros2 run lidar_localization_ros2 benchmark_eval_trajectory \
  --estimated-csv /tmp/lidarloc_benchmark/run_001/pose_trace.csv \
  --reference-csv /path/to/reference_pose.csv \
  --output-json /tmp/lidarloc_benchmark/run_001/trajectory_eval.json
```

To run the same pair through `evo_ape`:

```bash
ros2 run lidar_localization_ros2 benchmark_eval_evo_ape \
  --estimated-csv /tmp/lidarloc_benchmark/run_001/pose_trace.csv \
  --reference-csv /path/to/reference_pose.csv \
  --output-json /tmp/lidarloc_benchmark/run_001/evo_ape_translation.json \
  --save-results-zip /tmp/lidarloc_benchmark/run_001/evo_ape_translation.zip
```

Notes:

- default `--pose-relation trans_part` corresponds to translation APE in meters
- for datasets already expressed in the same map frame, keep the default unaligned evaluation
- if you need paper-by-paper comparability, make sure the sequence unit and initialization policy also match

To crop a TUM GT trajectory to the time span of a split rosbag2 sequence:

```bash
ros2 run lidar_localization_ros2 tum_trajectory_to_pose_reference_csv_for_rosbag2.py \
  --input data/public/koide_hard_localization/gt/traj_lidar_outdoor_hard_01.txt \
  --bag-path data/public/koide_hard_localization/sequences/outdoor_hard_01b/outdoor_hard_01b \
  --output-csv data/public/koide_hard_localization/benchmark/outdoor_hard_01b/reference.csv \
  --output-initial-pose-yaml data/public/koide_hard_localization/benchmark/outdoor_hard_01b/initial_pose.yaml \
  --initial-pose-skip-sec 0.05
```

To build a GT-aligned diagnostic map from a PointCloud2 rosbag2 sequence and a
reference CSV:

```bash
ros2 run lidar_localization_ros2 build_gt_aligned_map_from_reference_csv.py \
  --bag-path data/public/koide_hard_localization/sequences/outdoor_hard_01 \
  --reference-csv data/public/koide_hard_localization/benchmark/outdoor_hard_01/reference.csv \
  --cloud-topic /livox/points \
  --point-stride 10 \
  --voxel-size 0.5 \
  --output-map /tmp/koide_outdoor01_gt_map_stride10_voxel05.ply
```

This is useful as an engineering check when you suspect map-coverage mismatch.
When writing `.ply`, the helper emits a binary little-endian float32 PLY (`property float x/y/z`)
because the runtime stack does not behave reliably with Open3D's default double-precision PLY fields.
Do not use a map built from the same evaluation run for external performance claims.

## Compare multiple runs

```bash
ros2 run lidar_localization_ros2 benchmark_compare_runs \
  --run-dir /tmp/lidarloc_benchmark/run_ndt_omp \
  --run-dir /tmp/lidarloc_benchmark/run_small_gicp \
  --output-json /tmp/lidarloc_benchmark/comparison.json
```

For Nav2 replay supervisor-policy comparisons:

```bash
ros2 run lidar_localization_ros2 compare_nav2_reinit_supervisor_runs.py \
  --run configured_initial_pose=/tmp/lidarloc_replay_supervisor_120 \
  --run latest_pose=/tmp/lidarloc_replay_latest_pose_supervisor_120 \
  --run no_supervisor=/tmp/lidarloc_replay_no_supervisor_120 \
  --output-json /tmp/lidarloc_nav2_reinit_compare/comparison.json \
  --output-md /tmp/lidarloc_nav2_reinit_compare/summary.md
```

To sweep configured-initial-pose burst counts on the same `120 s` replay:

```bash
ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_sweep.sh
```

This writes:

- `artifacts/public/nav2_reinit_supervisor_burst_sweep_120/comparison.json`
- `artifacts/public/nav2_reinit_supervisor_burst_sweep_120/summary.md`

Current recommendation from that sweep: keep `reinitialization_supervisor_publish_count:=1`.

To run the current long replay regression for the recommended supervisor policy:

```bash
ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_regression.sh
```

This writes:

- `artifacts/public/nav2_reinit_supervisor_regression_120/comparison.json`
- `artifacts/public/nav2_reinit_supervisor_regression_120/summary.md`
- `artifacts/public/nav2_reinit_supervisor_regression_120/regression_result.json`

Current pass criteria:

- both runs finish `navigate_to_pose` with `SUCCEEDED`
- `no_supervisor` shows at least one reinitialization request
- `configured_initial_pose_count1` keeps `reinitialization_requested_rows <= 20`
- `configured_initial_pose_count1` reduces `reinitialization_requested_rows` by at least `10x` vs `no_supervisor`
- the compare helper recommends `configured_initial_pose_count1`

To run the combined release-style regression boundary:

```bash
ros2 run lidar_localization_ros2 run_release_regression_suite.sh
```

This writes:

- `artifacts/public/release_regression_suite/summary.json`
- `artifacts/public/release_regression_suite/summary.md`

and keeps the underlying per-suite artifacts under:

- `artifacts/public/release_regression_suite/public_regression_suite/`
- `artifacts/public/release_regression_suite/nav2_reinit_supervisor_regression_120/`

## Recommended metrics

At minimum, compare these metrics across systems and commits:

- translation RMSE
- rotation RMSE
- relocalization success rate
- relocalization latency
- lost-track rate
- median, p95, and p99 alignment time
- median and peak CPU usage
- median and peak memory usage

## Current limitations

- `benchmark_runner` monitors one target process chosen by a command-line substring match
- trajectory evaluation uses nearest-timestamp association and assumes synchronized clocks
- relocalization success rate and lost-track rate still need higher-level task-specific scoring logic
- the official `hdl_localization` sample still needs a fair initial pose policy before it can be used for final backend ranking
