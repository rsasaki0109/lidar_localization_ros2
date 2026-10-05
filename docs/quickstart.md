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

Without a known pose, quickstart searches the whole map. It generates the 2D occupancy
grid that the G2 BBS_2D engine needs from the point cloud map (points from 0.4 m to
2.0 m above the ground are obstacles, so ceilings and tree canopy are not) and caches it
under `~/.cache/lidar_localization_ros2/occupancy/` by map contents:

```bash
ros2 run lidar_localization_ros2 quickstart.py \
  --profile mid360 \
  --map /absolute/path/to/map.pcd
```

Pass `--occupancy-map /absolute/path/to/map.yaml` to use your own grid, or
`--no-auto-occupancy-map` to start from RViz instead.

Replays of a recorded mapping run (route-crop needs the run's timestamps; see below):

```bash
ros2 run lidar_localization_ros2 quickstart.py \
  --profile mid360 \
  --map /absolute/path/to/map.pcd \
  --reference-csv /absolute/path/to/mapping_run/reference.csv
```

Route-crop picks reference poses by the scan's timestamp, so it only re-localizes
within the recorded session (replays and benchmarks of the mapping run). A live robot
on a later day is outside that time span: route-crop then reports
`scan is ... s outside the reference trajectory`. Supply `--occupancy-map` as well and
G2 uses route-crop inside the session and map-wide BBS outside it.

For a full repeat-route workflow (mapping CSV → one-command bringup → status checks),
see [site_setup.md](site_setup.md).

`quickstart.py --help` lists the options a bringup needs; `--help-all` adds the global
search and recovery tuning options, which keep their defaults unless replay evidence
says otherwise. Use `--dry-run` to generate the configuration and inspect both the launch
and bringup check commands without starting ROS nodes. A five-second topic/TF check runs after
launch by default; disable it with `--no-bringup-check`. Use `--no-rviz` on a headless
robot.

ROS time follows `/clock` when quickstart sees one, as during `ros2 bag play --clock`
or in a simulator, and the wall clock otherwise; the choice is printed as
`clock=sim` or `clock=wall`. Start the bag before quickstart, or pass `--use-sim-time`
(`--no-use-sim-time` forces the wall clock).

When exactly one live `sensor_msgs/msg/PointCloud2` or `sensor_msgs/msg/Imu` topic is
visible, quickstart selects it. If several exist, it keeps the selected profile's safe
default and prints `ambiguous`; specify `--cloud-topic` or `--imu-topic` rather than
guessing. Frame defaults also come from the profile and can be overridden with
`--lidar-frame`, `--imu-frame`, `--base-frame`, `--odom-frame`, and `--global-frame`.

If the robot already runs odometry that publishes `odom -> base_frame` (a LIO front end
such as RKO-LIO, or wheel or leg odometry), add `--odom-tf-prediction`. Scan matching is
then seeded from that odometry, `map -> odom` is published, and the odometry-bridged pose
keeps flowing while scans are rejected. Handheld or other fast motion needs it: on the
Koide `outdoor_hard_02b` handheld sequence at 1x, seeded at the true start pose, the
standalone configuration lost track after 23 s without it and stayed within 1.4 m (median
0.07 m) for the whole 298 s with it.

`--odom-tf-prediction` also enables the seed correction guard: once tracking has settled,
a scan match that moves the pose more than 0.3 m or 15 degrees from the odometry
prediction is rejected, since with good odometry such a jump is a local minimum, not
motion. On a Unitree Go2 indoor aisle (JEPLO `EIL_Box`), objects missing from the map
made NDT jump 1-2 m along the aisle with excellent fitness; the guard took the median
error from 0.71 m to 0.11 m on a lidar_slam_ros2 map. It waits for five accepted scans
in a row within those limits after any reset or odometry gap (more than 1 s between
odometry seeds), so it neither holds an imprecise global-search pose nor blocks the
correction after the robot was carried. It gives way after 30 rejections in a row.

## Localizing on a lidar_slam_ros2 map

A map built with [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2)
(`lidarslam-map start <bag>`) can be used directly. Its frame starts at the first
mapping pose, so a robot that starts where mapping started is near `(0, 0, 0)`, and
the same package's RKO-LIO front end provides the odometry. For a Livox MID-360:

```bash
# 1. Odometry: RKO-LIO from lidar_slam_ros2, publishing odom -> livox_frame
ros2 run rko_lio online_node --ros-args \
  -p lidar_topic:=/livox/lidar -p imu_topic:=/livox/imu -p base_frame:=livox_frame \
  -p initialization_phase:=true \
  -p "extrinsic_lidar2base_quat_xyzw_xyz:=[0.0,0.0,0.0,1.0,0.0,0.0,0.0]" \
  -p "extrinsic_imu2base_quat_xyzw_xyz:=[0.0,0.0,0.0,1.0,0.0,0.0,0.0]"

# 2. Localization with automatic global initialization
ros2 run lidar_localization_ros2 quickstart.py --profile standalone \
  --map /path/to/output/my_map/map.pcd \
  --global-seed-z 0 --odom-tf-prediction \
  --cloud-topic /livox/lidar --imu-topic /livox/imu \
  --lidar-frame livox_frame --base-frame livox_frame --no-publish-lidar-tf
```

Give the extrinsics explicitly. Without them RKO-LIO looks them up in TF, and with no
`livox_frame` in the tree it publishes no odometry. `--global-seed-z` is the sensor
height in the map frame; it is about 0 near the mapping start. With a known start pose,
use `--initial-pose 0 0 0 0 0 0 1` instead of global search. For a bag, add `-p
use_sim_time:=true` to RKO-LIO; quickstart picks up `/clock` itself once the bag plays.

Measured on the lidar_slam_ros2 MID-360 demo bag (a 1 km drive at up to 6 m/s), with a
map built from the same bag and its SLAM trajectory as the reference:

| setup | result |
| --- | --- |
| above, global initialization | initialized from 2 G2 answers, tracking 12 s into the bag; within 1.0 m (median 0.30 m) to the end |
| `--initial-pose` without odometry | lost when the car reached 6 m/s, after 100-125 s |

## Initialization order

The startup manager uses this fixed order:

1. an explicit `--initial-pose`, when supplied;
2. the last verified pose saved for the exact same pointcloud map contents;
3. guarded global search over an occupancy grid (`--occupancy-map`, or one generated
   from the map), or guarded route-crop search when `--reference-csv` is supplied;
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
BBS implementation. When `--reference-csv` is supplied (or `--occupancy-map` for BBS),
quickstart also launches the guarded G3 `reinitialization_supervisor_node` by default
(`--g3-recovery`, disable with `--no-g3-recovery`) so lost tracking can re-query G2 and
re-seed `/initialpose` after startup.

The startup-only state machine is separate from the G3 lost-tracking
supervisor because cold start has no previously trusted tracking episode.

The G3 supervisor applies the startup distinctiveness gate to its G2 answers
(`max_registration_fitness_ratio`), including the reply that verifies a reset: a
verify reply that disagrees with the reset reseeds from its own candidates only when
they pass. While `odom_bridge_pose` is available (for example
with `--odom-tf-prediction`), it also does not reset the bridged pose on one answer: an
answer must match an earlier one, from another scan within
`odometry_confirmation_window_sec` (120 s), moved forward by the bridged motion
(`require_odometry_confirmation`). On the Koide `outdoor_hard_02b` replay this stopped
resets onto aliased places 160-260 m away during a stretch where NDT fails but odometry
holds the pose.

Odometry that stops for more than `odometry_max_gap_sec` (2 s; for example a LIO front
end blinded while the sensor is covered) cannot vouch for motion across the gap, and a
robot carried meanwhile keeps a pose that is metres off. For
`odometry_confirmation_window_sec` after such a dropout, the supervisor:

- accepts a single gated answer without odometry confirmation;
- queries G2 on its own once scans have failed for `query_after_odometry_dropout_sec`
  (5 s; 0 disables). The localizer's own request needs 30 s without an accepted scan,
  and one accepted scan at the wrong pose restarts that wait. If this episode gives
  up, the dropout requests no more queries, and the localizer's request takes over.

On the Koide `outdoor_kidnap_b` replay (sensor covered and carried several times), this
raised the share of time within 3 m after the first carry from 0.11-0.34 to 0.57-0.62,
with every reset within 2 m.

A candidate is published only when:

- G2 confirms that 3D NDT registration scoring is active (required by default);
- the G2 score is at least `--min-candidate-score`;
- the score lead over candidate 2 is at least `--min-score-margin`;
- `candidate_age_sec` is present and no greater than `--max-candidate-age-sec`;
- at least `--global-consensus-samples` results from distinct scan timestamps agree
  within the configured translation and yaw bounds (with odometry, after moving each
  earlier result to the new scan time; see below);
- the localizer subsequently reports acceptable fitness and stable tracking for
  `--verification-samples` fresh diagnostic messages.

Failed saved poses fall through to global search. Weak, ambiguous, stale, inconsistent,
timed-out, or unverified global candidates consume a bounded query attempt; after
`--max-global-attempts`, the node publishes nothing and requests RViz input. Status is
available as JSON on `/startup_initialization/status`.

The compiled G2 backend and 3D scoring are enabled by default. If scorer loading or
scoring fails, quickstart rejects the 2D-only result instead of weakening the policy.
Global candidates have no height of their own, and NDT scores them at a seed height.
`--global-seed-z` gives one map-frame z for the whole map. `--sensor-height` gives the
sensor's height above the ground instead, and each candidate is scored at the lowest
map points under it plus that height, which also holds on maps with hills or ramps. On
the lidar_slam_ros2 MID-360 demo map, whose drive spans 12 m of height, global
initialization started 125 s into the bag (11 m below the map origin) took 29 s and 12
retries with the default seed z of 0, and the first query with `--sensor-height 1.66`
(or `1.0`). On Koide `outdoor_hard_02b`, `--sensor-height 1.33` (or `1.0`) replaces
`--global-seed-z -11.4` with the same result (median 0.09 m). HDL-style maps may
need `--refine-global-candidates`; refinement remains opt-in because repeated geometry
can make several BBS hypotheses converge to the same local optimum. The candidate age
and query timeout defaults are 30 seconds, and an over-time in-flight query falls back
directly to the operator instead of issuing duplicate work. Quickstart uses 256 scan
points, 5-degree yaw sampling, eight candidates, and 3 m BBS non-maximum suppression to keep the guarded query bounded;
all are available as `--global-*` overrides for measured site tuning. The
`--no-require-global-registration-scoring` escape hatch is intended only for replay
experiments, not unattended startup.

The occupancy grid must represent the same physical map as the 3D map. The package does
not infer this relationship and cannot make a mismatched pair safe. Create a grid with
`generate_occupancy_map_from_pcd` when its route-crop behavior fits the site, then
inspect the result before use.

When you generate a grid yourself indoors, pass `--max-obstacle-height-m` (for example
`1.5`; quickstart's own grids use 2.0). Without it a cell is occupied when its points
span 0.4 m in height, and floor plus ceiling does that
everywhere, so the whole room becomes occupied and global search has no free space to
match. On a Unitree Go2 map of an indoor aisle (lidar_slam_ros2, JEPLO `EIL_Mix`), 1.5 m
left the walls as outlines with free floor inside. Global initialization on another
session then succeeded where the default grid failed (`ambiguous_candidate_retry`).

### Starting while moving

A G2 answer describes the scan it was computed from, which is 10-20 s old when it
arrives. Without odometry, keep the robot stationary during cold-start search. When an
`odom_frame_id -> base_frame_id` TF is available (the `mid360` profile requires one),
the startup node uses it automatically:

- each result is remembered as the map -> odom transform it implies (the last
  `global_consensus_history`, default 5), including results that failed the score,
  margin, or distinctiveness gates;
- a result is confirmed when it matches an earlier result from another scan, moved to
  the new scan time by odometry, and at least one of the two passed all gates. The
  match allows 2 m plus `global_consensus_translation_per_odom_m` (5%) of the distance
  travelled, for the 5-degree heading quantization of each result;
- the published `/initialpose` is moved from the queried scan to the latest odometry,
  and the log reports how far (`moved the global candidate by odometry ...`).
- a result whose scan is at least `global_attempt_refund_travel_m` (5 m) of odometry
  away from the previous result's gives its attempt back: a moving robot keeps
  searching new places, while a stationary one still stops after
  `--max-global-attempts`. `max_global_queries` (30) bounds the total.

The node logs `odometry at the queried scan` or `no odometry at the queried scan` for
every result. It waits up to 2 s of scans for odometry before the first query, so a
fix is not lost to a TF listener that started after the scan. Disable all of this with
the startup node's `enable_odom_motion_compensation:=false`.

On the Koide `outdoor_hard_02b` handheld sequence (about 1 m/s), only 9 of 37 G2
answers 8 s apart had a correct top candidate. Replaying those answers, odometry let 12
of 37 start points initialize within six answers (0 without odometry) and 35 of 37
with an unlimited budget, with no accepted pose more than 3 m wrong.

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

- `no_safe_automatic_source`: no matching saved pose and no occupancy map or reference
  CSV (for example `--no-auto-occupancy-map`, or the grid could not be generated); use
  RViz or restart with `--occupancy-map` / `--reference-csv`.
- `map_mismatch`: the stored pose belongs to different map contents and was ignored.
- `ambiguous_candidate_retry`: similar places are not distinguishable at the configured
  margin; do not loosen the margin without replay evidence. With `--reference-csv` and
  3D registration scoring, a top `registration_fitness` at or below **0.5** bypasses this
  gate and publishes after localizer verification instead.
- `ambiguous_registration_retry`: the top candidate's `registration_fitness` is not at
  most half of the best fitness at another place (>= 5 m away). Aliased areas score many
  similar, mediocre poses; on the Koide outdoor map they scored 0.8–2.0 with a runner-up
  close behind, while the true pose scored 0.04 against 0.82 elsewhere. Tune with the
  startup node's `max_registration_fitness_ratio` (0 disables) and
  `registration_alternative_min_separation_m`.
- `global_attempts_exhausted`: automatic publication stopped; set the pose in RViz.
- no detected sensor topic: start the driver or bag first, or pass the topic explicitly.

Run the printed `check_lidar_localization_bringup.py` command when data or pose output is
missing. See [frame contract](frame_contract.md), [troubleshooting](troubleshooting.md),
[global localization](global_localization.md), and the
[Phase 4 validation record](quickstart_validation.md) for lower-level details and the
measured dataset boundary.
