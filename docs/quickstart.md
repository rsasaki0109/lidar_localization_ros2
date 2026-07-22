# Quickstart and Automatic Initialization

`quickstart.py` is the user-facing entry point for first bringup. It generates a normal
package parameter file and launches the existing localizer; the localization algorithm
and topic/frame contracts are unchanged.

## Start

With a known pose, pass it explicitly:

```bash
ros2 run lidar_localization_ros2 quickstart.py \
  --profile standalone \
  --map /absolute/path/to/map.pcd \
  --initial-pose X Y Z QX QY QZ QW
```

Without a known pose, enable map-wide search by supplying the 2D occupancy map used by
the existing G2 BBS_2D engine:

```bash
ros2 run lidar_localization_ros2 quickstart.py \
  --profile mid360 \
  --map /absolute/path/to/map.pcd \
  --occupancy-map /absolute/path/to/map.yaml
```

Use `--dry-run` to generate the configuration and inspect both the launch and bringup
check commands without starting ROS nodes. A five-second topic/TF check runs after
launch by default; disable it with `--no-bringup-check`. Use `--no-rviz` on a headless
robot and `--use-sim-time` for rosbag replay.

When exactly one live `sensor_msgs/msg/PointCloud2` or `sensor_msgs/msg/Imu` topic is
visible, quickstart selects it. If several exist, it keeps the selected profile's safe
default and prints `ambiguous`; specify `--cloud-topic` or `--imu-topic` rather than
guessing. Frame defaults also come from the profile and can be overridden with
`--lidar-frame`, `--imu-frame`, `--base-frame`, `--odom-frame`, and `--global-frame`.

## Initialization order

The startup manager uses this fixed order:

1. an explicit `--initial-pose`, when supplied;
2. the last verified pose saved for the exact same pointcloud map contents;
3. guarded BBS_2D global search when `--occupancy-map` is supplied;
4. an operator pose from RViz **2D Pose Estimate**.

There is no implicit `(0, 0, 0)` fallback. An explicit pose disables both saved-pose
publication and global search for that start; the startup manager monitors localizer
diagnostics without publishing the explicit pose a second time.

The saved state defaults to
`~/.local/state/lidar_localization_ros2/<map-name>.json`. Its map identity is the full
SHA-256 and byte size of the `.pcd` or `.ply`; renaming an unchanged map is safe, while
changing any map content rejects the old pose. Before publication, the current scan must
also converge from the stored pose under the NDT score gate. If the optional scorer is
unavailable, quickstart skips automatic restore instead of trusting the pose. Writes are
atomic. A pose is saved only after fresh diagnostics report stable tracking and acceptable
fitness for the configured number of consecutive samples. `--no-restore-saved-pose`
disables restore without disabling future verified saves, and
`--saved-pose-max-age-sec` can impose an age limit.

## Global-search safety gates

Global initialization reuses `global_localization_node.py`; it does not duplicate the
BBS implementation. The startup-only state machine is separate from the G3 lost-tracking
supervisor because cold start has no previously trusted tracking episode.

A candidate is published only when:

- G2 confirms that 3D NDT registration scoring is active (required by default);
- the G2 score is at least `--min-candidate-score`;
- the score lead over candidate 2 is at least `--min-score-margin`;
- `candidate_age_sec` is present and no greater than `--max-candidate-age-sec`;
- at least `--global-consensus-samples` results from distinct scan timestamps agree
  within the configured translation and yaw bounds;
- the localizer subsequently reports acceptable fitness and stable tracking for
  `--verification-samples` fresh diagnostic messages.

Failed saved poses fall through to global search. Weak, ambiguous, stale, inconsistent,
timed-out, or unverified global candidates consume a bounded query attempt; after
`--max-global-attempts`, the node publishes nothing and requests RViz input. Status is
available as JSON on `/startup_initialization/status`.

The compiled G2 backend and 3D scoring are enabled by default. If scorer loading or
scoring fails, quickstart rejects the 2D-only result instead of weakening the policy.
Use `--global-seed-z` for maps whose sensor height is not near zero. HDL-style maps may
need `--refine-global-candidates`; refinement remains opt-in because repeated geometry
can make several BBS hypotheses converge to the same local optimum. Keep the robot
stationary during cold-start search: the candidate age and query timeout defaults are
30 seconds, and an over-time in-flight query falls back directly to the operator instead
of issuing duplicate work. Quickstart uses 256 scan points, 10-degree yaw sampling,
eight candidates, and 3 m BBS non-maximum suppression to keep the guarded query bounded;
all are available as `--global-*` overrides for measured site tuning. The
`--no-require-global-registration-scoring` escape hatch is intended only for replay
experiments, not unattended startup.

The occupancy grid must represent the same physical map as the 3D map. The package does
not infer this relationship and cannot make a mismatched pair safe. Create a grid with
`generate_occupancy_map_from_pcd.py` when its route-crop behavior fits the site, then
inspect the result before use.

## Profiles

| Profile | Default cloud | Default IMU | Pose output | TF expectation |
| --- | --- | --- | --- | --- |
| `standalone` | `/velodyne_points` | `/imu` | `/pcl_pose` | localizer publishes `map -> base_link` |
| `nav2` | `/velodyne_points` | `/imu/data` | `/localization/pose_with_covariance` | external `odom -> base_link` required |
| `mid360` | `/livox/points` | `/livox/imu` | `/localization/pose_with_covariance` | external `odom -> base_link` required |

Do not enable quickstart's static TF publishers when `robot_state_publisher`, the sensor
driver, or a bag already owns the same edge. Use `--no-publish-lidar-tf` and leave IMU TF
publication off as appropriate.

## Troubleshooting

- `no_safe_automatic_source`: no matching saved pose and no occupancy map; use RViz or
  restart with `--occupancy-map`.
- `map_mismatch`: the stored pose belongs to different map contents and was ignored.
- `ambiguous_candidate_retry`: similar places are not distinguishable at the configured
  margin; do not loosen the margin without replay evidence.
- `global_attempts_exhausted`: automatic publication stopped; set the pose in RViz.
- no detected sensor topic: start the driver or bag first, or pass the topic explicitly.

Run the printed `check_lidar_localization_bringup.py` command when data or pose output is
missing. See [frame contract](frame_contract.md), [troubleshooting](troubleshooting.md),
[global localization](global_localization.md), and the
[Phase 4 validation record](quickstart_validation.md) for lower-level details and the
measured dataset boundary.
