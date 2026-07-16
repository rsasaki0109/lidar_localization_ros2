# Script catalog

The script directory is intentionally flat. Existing filenames are public CLI
and documentation contracts, and installed scripts must remain discoverable by
`ros2 run lidar_localization_ros2 <name>`. Use the categories below instead of
moving files into subdirectories.

## Common workflows

| Task | Entry point |
| --- | --- |
| Generate a site preset | `create_lidar_localization_config.py` |
| Check generic bringup | `check_lidar_localization_bringup.py` |
| Check MID-360 bringup | `check_mid360_legged_bringup.py` |
| Run public demo | `run_public_demo.sh` |
| Run public regression | `run_public_regression_suite.sh` |
| Run release regression | `run_release_regression_suite.sh` |
| Run manifest benchmark | `benchmark_from_manifest` |
| Compare benchmark runs | `benchmark_compare_runs` |
| Build odometry runtime evidence | `build_odometry_runtime_evidence.py` |
| Run fixed GLIM Koide odometry benchmark | `run_koide_glim_odometry_benchmark.py` |
| Check Koide 380 s odometry goal | `check_koide_odometry_completion.py` |
| Require Koide completion repeats | `summarize_koide_odometry_completion.py` |

## Categories

- `benchmark_*`: benchmark execution, recording, conversion, scoring, and
  comparison primitives.
- `fetch_*`, `convert_*`, `generate_*`, `build_*`, `prepare_*`, `scaffold_*`:
  dataset, map, manifest, and report preparation.
- `run_*`: end-to-end demos, experiments, replays, smoke tests, and regression
  suites. Dataset-specific runners should state their data prerequisites in
  `--help` output or their companion documentation.
- `analyze_*`, `diagnose_*`, `summarize_*`, `compare_*`, `validate_*`,
  `check_*`: offline analysis and acceptance checks.
- `publish_*`, `relay_*`, `republish_*`, `record_*`, `send_*`, `inject_*`:
  small ROS graph adapters used by launches and experiments.
- `make_*`, `select_*`, `score_*`, `resolve_*`, `observe_*`: global
  localization and relocalization planning stages.
- `global_localization_*`, `reinitialization_supervisor_*`: runtime G2/G3
  localization and recovery components.
- `render_*`: GIF and gallery rendering.
- `setup_local_env.sh`, `bootstrap_colcon_workspace.sh`: local developer
  environment setup; source or run these from the repository checkout.

## Installed versus development-only

The grouped lists in `CMakeLists.txt` are the installation source of truth.
Installed entries are callable from the package prefix. Scripts omitted from
those lists are repository-development tools and may rely on checkout-relative
paths.

Current development-only helpers are:

- environment: `setup_local_env.sh`, `bootstrap_colcon_workspace.sh`;
- Koide acquisition/rendering: `fetch_all_koide_sequences.sh`,
  `render_koide_dataset_gallery.sh`, `render_koide_localization_gif.py`,
  `prepare_koide_localization_gif_benchmarks.py`;
- focused research/reporting: `analyze_pose_covariance_calibration.py`,
  `diagnose_local_map_crop_coverage.py`, `demo_pose_arbitration.py`,
  `render_global_localization_demo_gif.py`,
  `summarize_koide_phase1_backend_comparison.py`;
- legacy/direct regression wrappers: `run_hdl_g3_recovery_regression.sh`,
  `run_koide_g3_recovery_regression.sh`,
  `run_koide_phase1_backend_comparison.sh`,
  `run_koide_phase1_regression.sh`;
- lower-level benchmark utilities: `benchmark_convert_traj_refined`,
  `benchmark_drift_correlation_report`.

`scripts/lidar_localization_mid360/` is the small shared Python package used by
MID-360 configuration and validation commands. Pure policy modules may be
installed beside executable scripts because runtime nodes import them from the
same directory.

## Adding a script

1. Choose an existing action prefix and a narrow, descriptive name.
2. Add a shebang and executable bit for a command; omit command-line behavior
   for a pure import module.
3. Add user-facing commands to the appropriate grouped CMake list.
4. Add a focused test for argument, output, or cleanup contracts.
5. Keep generated outputs outside the repository by default.
