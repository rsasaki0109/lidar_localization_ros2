# Agent Notes

## Project Scope

- This repository is the ROS 2 package `lidar_localization_ros2`.
- Runtime code lives in `src/`, `include/`, `launch/`, `param/`, and `scripts/`.
- Experimental alternatives live under `experiments/` and are intentionally discardable until a variant wins the shared comparison.
- The local workspace layout is expected to be:

```text
lidarloc_ws/
  build_ws/
  local_prefix/
  repo/
```

## Environment

- Use the no-sudo local environment from the repository root:

```bash
source scripts/setup_local_env.sh
```

- The setup script sources ROS 2 Humble, adds `../local_prefix`, and then sources `../build_ws/install/setup.bash` when present.
- Do not replace the local-prefix workflow with system-wide dependency installs unless explicitly requested.
- Build from the overlay workspace, not from this repository directory:

```bash
cd ../build_ws
colcon build --symlink-install --packages-up-to lidar_localization_ros2
```

## Validation

- For C++/launch/package changes, run at least:

```bash
source scripts/setup_local_env.sh
cd ../build_ws
colcon build --symlink-install --packages-up-to lidar_localization_ros2
```

- For localization behavior, recovery logic, or parameter-default changes, also run the focused experiment suite after the build:

```bash
ros2 run lidar_localization_ros2 run_experiment_suite.py
```

- For public-smoke validation use:

```bash
scripts/run_public_regression_suite.sh
```

- For release-style validation use:

```bash
ros2 run lidar_localization_ros2 run_release_regression_suite.sh
```

- If required public datasets are absent or a regression run is too heavy for the current task, state exactly which validation was skipped.

## Development Rules

- New behavior should first be introduced as multiple comparable variants under `experiments/`.
- Promote only the winning behavior into runtime code after the shared fixture/rubric comparison.
- `docs/interfaces.md`, `docs/experiments.md`, and `docs/decisions.md` are generated from experiment results; do not hand-edit them as source-of-truth documents.
- Keep `param/nav2_ndt_urban.yaml` conservative. Long-horizon urban replay is still a known robustness boundary, not a solved production claim.
- If adding a user-facing script, add it to the `install(PROGRAMS ...)` list in `CMakeLists.txt`.
- If adding a parameter, keep declarations, YAML presets, README/docs, and diagnostics aligned.
- Keep `small_gicp` optional behind the existing CMake/config guards.
- Preserve ROS topic and frame contracts unless the task explicitly changes them.

## Benchmark And Dataset Rules

- Prefer binary little-endian float32 PLY maps for benchmark/runtime validation.
- Generated PCD maps are acceptable for inspection, but not the preferred benchmark/runtime path.
- Use a unique `ROS_DOMAIN_ID` for rosbag replay benchmarks to avoid unrelated ROS 2 graph traffic.
- Istanbul localization-only public runs are default-on no-IMU safety checks; do not describe them as IMU benefit benchmarks.
- Publishable benchmark claims should use official public datasets such as Autoware Istanbul or the official `hdl_localization` sample, with upstream sources cited.
- Do not present local field-recorded bags or graph-derived synthetic bags as open benchmark data.

## Generated Files

- Avoid editing or committing generated/heavy local outputs unless the task specifically asks for them.
- Treat these paths as generated or local workspace state: `build/`, `install/`, `log/`, `data/official/`, `artifacts/`, `../build_ws/`, `../local_prefix/`, `../third_party_build/`, and `../third_party_debs/`.
- Prefer `/tmp` for ad hoc benchmark output directories.
