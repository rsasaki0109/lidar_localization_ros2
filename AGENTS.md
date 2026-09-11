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

- The setup script sources ROS 2 Humble for this workspace overlay, adds `../local_prefix`, and then sources `../build_ws/install/setup.bash` when present.
- Upstream README/CI also track Jazzy builds; local overlay development here remains Humble-based unless explicitly migrated.
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

- For localization behavior, recovery logic, or parameter-default changes, also run the focused Python test suite after the build:

```bash
python3 -m pytest test/
```

- For bringup acceptance use:

```bash
ros2 run lidar_localization_ros2 check_lidar_localization_bringup.py --help
ros2 run lidar_localization_ros2 check_mid360_legged_bringup.py --help
```

- If required datasets or a ROS graph are absent and a check is too heavy for the current task, state exactly which validation was skipped.

## Development Rules

- New runtime behavior starts as a focused test or a `experiments/` C++ candidate before it graduates into `src/`/`include/`.
- Promote only the winning behavior into runtime code after the shared fixture/rubric comparison.
- Keep `param/nav2_ndt_urban.yaml` conservative. Long-horizon urban replay is still a known robustness boundary, not a solved production claim.
- If adding a user-facing script, add it to the `install(PROGRAMS ...)` list in `CMakeLists.txt`.
- If adding a parameter, keep declarations, YAML presets, README/docs, and diagnostics aligned.
- Keep `small_gicp` optional behind the existing CMake/config guards.
- Preserve ROS topic and frame contracts unless the task explicitly changes them.

## Benchmark And Dataset Rules

- Prefer binary little-endian float32 PLY maps for runtime validation.
- Generated PCD maps are acceptable for inspection, but not the preferred runtime path.
- Publishable claims should cite official public datasets with upstream sources.
- Do not present local field-recorded bags or graph-derived synthetic bags as open benchmark data.

## Generated Files

- Avoid editing or committing generated/heavy local outputs unless the task specifically asks for them.
- Treat these paths as generated or local workspace state: `build/`, `install/`, `log/`, `data/official/`, `artifacts/`, `../build_ws/`, `../local_prefix/`, `../third_party_build/`, and `../third_party_debs/`.
- Prefer `/tmp` for ad hoc benchmark output directories.
