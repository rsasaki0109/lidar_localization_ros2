# v1 Status

This document defines what `v1.0.0` means for this repository.

## Included

- ROS 2 LiDAR localization node with lifecycle support
- Nav2 integration launch files for localization-only and full navigation bringup
- helper scripts for occupancy-map generation, replay restamping, odom fallback, and goal dispatch
- rosbag benchmark tooling and trajectory comparison helpers
- mapless-public dataset scaffolding for split mapping/localization workflows
- Boreas raw-sequence to rosbag2 conversion for the first public `LiDAR + IMU + GT` starter path
- a recommended Nav2 preset: `param/nav2_ndt_urban.yaml`
- a runtime-facing reinitialization-request signal on `/reinitialization_requested`
- recovery-supervisor diagnostics in `/alignment_status` with explicit state/action fields
- an optional external supervisor that republishes `/initialpose` when `/reinitialization_requested` goes true

## Validated scope

The following paths were validated in this workspace before the `v1.0.0` packaging pass:

- `colcon build --packages-up-to lidar_localization_ros2 --symlink-install`
- localizer launch and lifecycle activation
- Nav2 wrapper launch argument resolution
- demo `navigate_to_pose` smoke using the built-in odom/localization demo path
- replay `navigate_to_pose` smoke using the real localizer path and helper orchestration
- shutdown behavior after the destructor-path crash fix
- public regression suite entry point: `scripts/run_public_regression_suite.sh`
- reinitialization supervisor regression entry point: `ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_regression.sh`
- experiment suite entry point: `ros2 run lidar_localization_ros2 run_experiment_suite.py`
- mapless-public dataset scaffolding:
  - `ros2 run lidar_localization_ros2 scaffold_mapless_public_dataset_bundle.py --spec /absolute/path/to/spec.yaml`
  - generated `run_extract_reference.sh` path for bundles that need GT reference / initial-pose extraction
- experiment-first comparison entry point: `ros2 run lidar_localization_ros2 run_imu_guard_experiments.py`

## Recommended entry points

- localization only:
  - `ros2 launch lidar_localization_ros2 nav2_lidar_localization.launch.py`
- full Nav2 wrapper:
  - `ros2 launch lidar_localization_ros2 nav2_navigation.launch.py map_yaml:=/absolute/path/to/map.yaml`
- self-contained smoke:
  - `ros2 run lidar_localization_ros2 run_nav2_demo_smoke --map-yaml /absolute/path/to/map.yaml --initial-pose-x 0.0 --initial-pose-y 0.0 --goal-x 1.0 --goal-y 0.0`
- replay smoke:
  - `ros2 run lidar_localization_ros2 run_nav2_replay_smoke --map-yaml /absolute/path/to/map.yaml --pcd-map-path /absolute/path/to/map.pcd --bag-path /absolute/path/to/bag`
- reinitialization supervisor regression:
  - `ros2 run lidar_localization_ros2 run_nav2_reinit_supervisor_regression.sh`
- public regression:
  - `scripts/run_public_regression_suite.sh`
  - Istanbul `60 s` default-on no-IMU safety run
  - HDL `60 s` IMU safety / throughput compare
- release regression:
  - `ros2 run lidar_localization_ros2 run_release_regression_suite.sh`
  - combines the public suite and the long Nav2 reinitialization-supervisor regression
- experiment comparison:
  - `ros2 run lidar_localization_ros2 run_experiment_suite.py`
  - current problems: IMU correction guard, borderline measurement gate, recovery action selection, reinitialization trigger
  - generated docs: `docs/interfaces.md`, `docs/experiments.md`, `docs/decisions.md`
  - problem-specific runners remain available for narrower iteration
  - `ros2 run lidar_localization_ros2 run_imu_guard_experiments.py`
  - `ros2 run lidar_localization_ros2 run_borderline_gate_experiments.py`
  - `ros2 run lidar_localization_ros2 run_recovery_action_experiments.py`
  - `ros2 run lidar_localization_ros2 run_reinit_trigger_experiments.py`
- localization health summary:
  - `ros2 run lidar_localization_ros2 summarize_localization_health.py --alignment-csv /absolute/path/to/alignment_status.csv --trajectory-eval-json /absolute/path/to/trajectory_eval.json`
  - current role: artifact-first recovery/reinitialization metrics for v1.1 dataset expansion
  - next target: [v1.1 relocalization plan](v1_1_relocalization.md)
- relocalization attempt summary:
  - `ros2 run lidar_localization_ros2 summarize_relocalization_attempts.py --alignment-csv /absolute/path/to/alignment_status.csv --attempts-csv /absolute/path/to/relocalization_attempts.csv --allow-missing-attempts`
  - current role: fixed artifact schema for candidate generation, dry-run reset artifacts, and explicit zero-attempt baselines
- disabled relocalization attempt baseline:
  - `ros2 run lidar_localization_ros2 make_disabled_relocalization_attempts.py --alignment-csv /absolute/path/to/alignment_status.csv --output-csv /absolute/path/to/relocalization_attempts.csv`
  - current role: converts request windows into rejected `candidate_generator_disabled` attempts before a real candidate generator exists
- route-grid relocalization candidate artifact:
  - `ros2 run lidar_localization_ros2 make_route_grid_relocalization_attempts.py --alignment-csv /absolute/path/to/alignment_status.csv --reference-csv /absolute/path/to/reference.csv --output-csv /absolute/path/to/relocalization_attempts.csv`
  - current role: converts request windows into non-zero route-corridor candidates while keeping attempts rejected until scoring/refinement/reset are implemented
- reference-oracle candidate scoring:
  - `ros2 run lidar_localization_ros2 score_relocalization_candidates_with_reference.py --attempts-csv /absolute/path/to/relocalization_attempts.csv --candidates-csv /absolute/path/to/relocalization_candidates.csv --reference-csv /absolute/path/to/reference.csv --output-attempts-csv /absolute/path/to/relocalization_attempts.csv`
  - current role: fills candidate score fields from reference trajectory as an offline upper-bound diagnostic; still no registration scoring or pose reset
- registration relocalization job artifact:
  - `ros2 run lidar_localization_ros2 make_registration_relocalization_jobs.py --attempts-csv /absolute/path/to/relocalization_attempts.csv --candidates-csv /absolute/path/to/relocalization_candidate_scores.csv --bag-path /absolute/path/to/bag --map-path /absolute/path/to/map.pcd --cloud-topic /velodyne_points --output-csv /absolute/path/to/relocalization_registration_jobs.csv`
  - current role: freezes the bag/map/topic/candidate-pose contract for the next registration scorer; still no registration execution or reset
- registration scan-resolution artifact:
  - `ros2 run lidar_localization_ros2 resolve_registration_relocalization_scans.py --jobs-csv /absolute/path/to/relocalization_registration_jobs.csv --output-csv /absolute/path/to/relocalization_registration_scores.csv`
  - current role: resolves nearest PointCloud2 scan, timing delta, point counts, and scan bounds for each registration job; still no registration execution or reset
- NDT relocalization job scoring:
  - `ros2 run lidar_localization_ros2 relocalization_ndt_score_jobs --input-csv /absolute/path/to/relocalization_registration_scores.csv --output-csv /absolute/path/to/relocalization_registration_scores_ndt.csv --max-jobs 1`
  - current role: first real NDT_OMP candidate scorer; fills registration score/convergence/refinement fields while keeping reset disabled
- registration score summary:
  - `ros2 run lidar_localization_ros2 summarize_registration_relocalization_scores.py --scores-csv /absolute/path/to/relocalization_registration_scores_ndt.csv --output-json /absolute/path/to/relocalization_registration_score_summary.json`
  - current role: scorer coverage/gate diagnostics for NDT relocalization artifacts; still no reset acceptance
- registration score comparison:
  - `ros2 run lidar_localization_ros2 compare_registration_relocalization_score_summaries.py --summary candidate_index:/absolute/path/to/summary_a.json --summary oracle_rank:/absolute/path/to/summary_b.json`
  - current role: compares candidate ordering diagnostics without promoting oracle order or reset acceptance
- registration ordering comparison runner:
  - `ros2 run lidar_localization_ros2 run_registration_ordering_comparison.py --attempts-csv /absolute/path/to/relocalization_attempts.csv --candidates-csv /absolute/path/to/relocalization_candidate_scores.csv --bag-path /absolute/path/to/bag --map-path /absolute/path/to/map.pcd --cloud-topic /velodyne_points --output-dir /absolute/path/to/ordering_comparison --top-k 5`
  - current role: runs the bounded jobs -> scan resolution -> NDT scoring -> summary -> comparison path for `candidate_index` and `oracle_rank`; still no reset acceptance
- manifest ordering comparison hook:
  - `benchmark.write_registration_ordering_comparison: true`
  - current role: opt-in wrapper around the ordering comparison runner for bounded public artifacts; default examples keep it disabled because it runs NDT scoring
- dry-run reset candidate plan:
  - `ros2 run lidar_localization_ros2 select_relocalization_reset_candidates.py --scores-csv /absolute/path/to/relocalization_registration_scores_ndt.csv --output-csv /absolute/path/to/relocalization_reset_candidate_plan.csv`
  - current role: selects one registration-gate-passing candidate per attempt for a dry-run reset plan; still writes `accepted=false`
- manifest reset candidate plan hook:
  - `benchmark.write_relocalization_reset_candidate_plan: true`
  - current role: opt-in reset-plan artifact generation from either explicit NDT score CSV or `candidate_index_topK` ordering-comparison scores
- reset candidate plan validation:
  - `ros2 run lidar_localization_ros2 validate_relocalization_reset_candidate_plan.py --plan-csv /absolute/path/to/relocalization_reset_candidate_plan.csv --scores-csv /absolute/path/to/relocalization_registration_scores_ndt.csv`
  - current role: contract check that keeps dry-run plans from silently claiming accepted resets or oracle-based runtime selection
- reset command dry-run artifact:
  - `ros2 run lidar_localization_ros2 make_relocalization_reset_commands.py --plan-csv /absolute/path/to/relocalization_reset_candidate_plan.csv --output-csv /absolute/path/to/relocalization_reset_commands.csv`
  - current role: converts selected reset-plan rows into `/initialpose` publish-intent artifacts without publishing
- reset command validation:
  - `ros2 run lidar_localization_ros2 validate_relocalization_reset_commands.py --commands-csv /absolute/path/to/relocalization_reset_commands.csv --summary-json /absolute/path/to/relocalization_reset_commands.json`
  - current role: verifies the v1.1 MVP endpoint: a validated dry-run `/initialpose` command artifact
- guarded reset command publisher:
  - `ros2 run lidar_localization_ros2 publish_relocalization_reset_commands.py --commands-csv /absolute/path/to/relocalization_reset_commands.csv --validation-json /absolute/path/to/relocalization_reset_commands_validation.json --output-csv /absolute/path/to/relocalization_reset_command_execution.csv`
  - current role: experimental guarded utility; writes a no-publish execution report by default, and actual `/initialpose` publish requires explicit `--execute`
  - v1.1 claim status: not part of the default public benchmark path or MVP endpoint
- post-reset execution observation:
  - `ros2 run lidar_localization_ros2 observe_relocalization_reset_execution.py --execution-csv /absolute/path/to/relocalization_reset_command_execution.csv --commands-csv /absolute/path/to/relocalization_reset_commands.csv --alignment-csv /absolute/path/to/alignment_status.csv --output-csv /absolute/path/to/relocalization_reset_execution_observation.csv`
  - current role: future controlled execution evaluator for published command rows and post-reset alignment status
  - v1.1 claim status: not the public MVP endpoint

## Recommended preset

Use `param/nav2_ndt_urban.yaml` unless you are intentionally running one of the experimental presets.

Current rationale:

- it is the most conservative branch among the tested Nav2-facing presets
- more aggressive recovery branches improved some intermediate diagnostics but regressed long-horizon translation in later replay tests
- when enabling the external reinitialization supervisor, keep `reinitialization_supervisor_use_latest_pose:=false`
  and start with `reinitialization_supervisor_publish_count:=1`
- `/reinitialization_requested` stays latched after a trigger until a new `/initialpose` clears it
- the current long replay regression boundary for that policy is `run_nav2_reinit_supervisor_regression.sh`
- the current one-command release boundary is `run_release_regression_suite.sh`

## Known limits

- long-horizon urban replay remains the main unresolved robustness gap
- experimental recovery presets are kept for research, not as defaults
- occupancy-map generation from PCD is route-crop oriented and is not intended as a universal mapper
- smoke success does not replace real-robot validation with real odom, real TF, and moving goals

## Non-goals for v1

- claiming universal recovery from prolonged reject streaks
- claiming production-grade performance for every dense urban replay segment
- replacing a full SLAM or mapping stack
- claiming production-grade global relocalization; v1.1 starts this as an experimental artifact-first path
