# Script catalog

The script directory is intentionally flat. Existing filenames are public CLI
contracts, and installed scripts must remain discoverable by
`ros2 run lidar_localization_ros2 <name>`. Use the categories below instead of
moving files into subdirectories.

## Common workflows

| Task | Entry point |
| --- | --- |
| One-command guarded bringup | `quickstart.py` |
| Generate a site preset | `create_lidar_localization_config.py` |
| Check generic bringup | `check_lidar_localization_bringup.py` |
| Check MID-360 bringup | `check_mid360_legged_bringup.py` |
| Run global localization query | `global_localization_query.py` |

## Categories

- `check_*`: bringup acceptance checks and generated-command validation.
- `quickstart.py`, `quickstart_model.py`, `startup_initialization_node.py`:
  first-bringup orchestration, ROS-free startup policy/persistence, and its ROS I/O node.
- `create_lidar_localization_config.py`: site preset generation.
- `global_localization_node.py`, `global_localization_query.py`,
  `g2_candidate_registration_rank_policy.py`: runtime G2/G3 localization and
  candidate ranking.
- `reinitialization_supervisor_node.py`, `reinitialization_supervisor_policy.py`:
  runtime recovery supervision.
- `make_bbs_relocalization_attempts.py`, `make_route_grid_relocalization_attempts.py`:
  BBS/route candidate generation. The BBS search engine is also the runtime
  Python fallback used by the global-localization nodes.
- `publish_*`, `relay_*`, `republish_*`, `record_*`, `send_*`, `inject_*`:
  small ROS graph adapters used by launches and recovery handling.
- `watch_alignment.py`, `watch_startup.py`: status watchers.
- `tum_trajectory_to_pose_reference_csv*.py`, `augment_pointcloud_intensity.py`:
  dataset preparation.
- `setup_local_env.sh`, `bootstrap_colcon_workspace.sh`: local developer
  environment setup; source or run these from the repository checkout.

## Installed versus development-only

The grouped lists in `CMakeLists.txt` are the installation source of truth.
Installed entries are callable from the package prefix. Scripts omitted from
those lists are repository-development tools and may rely on checkout-relative
paths.

Current development-only helpers are:

- environment: `setup_local_env.sh`, `bootstrap_colcon_workspace.sh`;
- dataset preparation: `augment_pointcloud_intensity.py`,
  `tum_trajectory_to_pose_reference_csv.py`,
  `tum_trajectory_to_pose_reference_csv_for_rosbag2.py`.

`scripts/lidar_localization_mid360/` is the small shared Python package used by
MID-360 configuration and validation commands. Pure policy modules may be
installed beside executable scripts because runtime nodes import them from the
same directory.

`relocalization_attempt_common.py` is a shared helper module installed the same
way: it owns the `relocalization_attempts.csv` fieldname contract used by the
candidate generators and the runtime global-localization engine.

`experiments/glim_prior_map_localizer/` and
`experiments/imu_yaw_prediction/` hold the remaining C++ experiment sources.
They are discardable and only promoted into `src/`/`include/` after a shared
fixture comparison.

## Adding a script

1. Choose an existing action prefix and a narrow, descriptive name.
2. Add a shebang and executable bit for a command; omit command-line behavior
   for a pure import module.
3. Add user-facing commands to the appropriate grouped CMake list.
4. Add a focused test for argument, output, or cleanup contracts.
5. Keep generated outputs outside the repository by default.
