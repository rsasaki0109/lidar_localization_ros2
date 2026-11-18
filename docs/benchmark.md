# Localization Benchmark

`tools/benchmark` replays public datasets through the same two commands a user runs,
odometry plus `quickstart.py --map <map>`, and scores the published `/pcl_pose`
against ground truth.

## What is measured

Every run starts from nothing: no saved pose and no initial pose, so global
initialization has to find the robot. Scores are taken **in the map frame without
any alignment**. The maps are in the ground-truth frame, and aligning the estimate to
the truth first would hide the worst failure: a wrong initialization that is then
tracked consistently. On a Unitree Go2, such runs were 1.8 m off but looked like
0.2 m after alignment.

| column | meaning |
| --- | --- |
| initialized | runs that published a pose |
| wrong init | runs whose first 2 s of poses were more than 1 m from the truth |
| time to first pose | seconds from the start of the replay |
| median / p95 / max xy | horizontal error of the matched poses |
| >1 m | share of poses more than 1 m off (median over runs) |
| tracked | share of the ground-truth time from the first pose to the end of the replay that has a matched pose (median over runs); a localizer that stops publishing loses it |

Runs also record the localizer's per-scan `/alignment_status`. A second table scores it
as a health signal: the share of poses more than 1 m off that it did not report OK
(the higher the better), and of poses within 0.3 m that it did not report OK (false
alarms, the lower the better). A pose with no status in the last second counts as
flagged.

The "lost" columns count only the scans where the localizer requests
reinitialization (or stopped reporting). A scan that NDT rejects while the odometry
bridge carries the pose is WARN, but not lost. The results below were measured
before the "tracked" and "lost" columns existed. See
[experiments/alignment_health](../experiments/alignment_health/README.md) for why
neither column alone separates lost poses from good ones.

## Results

### Summary

| dataset | runs | initialized | wrong init | median xy (m) | p95 xy (m) |
|---|---|---|---|---|---|
| Koide outdoor_hard (01a-02b) | 12 | 12 | 0 | 0.08-0.12 | 0.21-0.91 |
| Koide outdoor_kidnap (a, b) | 6 | 6 | 0 | 0.04-0.12 | 15.85-60.50 |
| JEPLO EIL_*, lidar_slam_ros2 map | 12 | 12 | 0 | 0.03 | 0.09-0.13 |
| JEPLO wide maps and EIL_*, rebuilt inputs, low-sensor settings | 18 | 18 | 0 | 0.02-0.03 | - |

Ranges are over cases. The kidnap p95 comes from the poses while the sensor is
carried, before recovery. The sections below have the per-case tables and what went wrong.

### Per case

Measured 2026-10-06 on `main` (Jazzy, replay at 1x, three runs per case unless noted).

**Handheld MID-360 outdoors (Koide):** every run initialized, none wrongly.

| case | runs | initialized | wrong init | time to first pose (s) | median xy (m) | p95 xy (m) | max xy (m) | >1 m |
|---|---|---|---|---|---|---|---|---|
| outdoor_hard_01a | 3 | 3 | 0 | 15.8 | 0.12 | 0.91 | 1.82 | 0.032 |
| outdoor_hard_01b | 3 | 3 | 0 | 22.2 | 0.08 | 0.21 | 0.87 | 0.000 |
| outdoor_hard_02a | 3 | 3 | 0 | 15.3 | 0.10 | 0.68 | 1.45 | 0.023 |
| outdoor_hard_02b | 3 | 3 | 0 | 12.3 | 0.09 | 0.37 | 128.81 | 0.008 |
| outdoor_kidnap_a | 3 | 3 | 0 | 14.5 | 0.04 | 60.50 | 180.96 | 0.073 |
| outdoor_kidnap_b | 3 | 3 | 0 | 18.6 | 0.12 | 15.85 | 150.50 | 0.328 |

The kidnap cases are far off while the sensor is carried, until recovery. In one
outdoor_hard_02b run the RKO-LIO odometry dropped scans with too few ICP keypoints;
the recovery supervisor then reset on a single unconfirmed answer 128 m away
(the confirmation is waived while odometry is out).

How `/alignment_status` tracked the error on the Koide runs (two per case, with #201):

| case | runs with >1 m poses | flagged when >1 m off | flagged when <0.3 m |
|---|---|---|---|
| outdoor_hard_01a | 2 | 0.98 | 0.13 |
| outdoor_hard_01b | 2 | 0.60 | 0.33 |
| outdoor_hard_02a | 2 | 0.93 | 0.41 |
| outdoor_hard_02b | 2 | 0.41 | 0.20 |
| outdoor_kidnap_a | 2 | 0.61 | 0.06 |
| outdoor_kidnap_b | 2 | 0.17 | 0.02 |

It catches slow loss of tracking (01a, 02a), but it reports OK on most poses after a
wrong reset (01b, kidnap_b: a confident match at the wrong place), and it raises
false alarms on 20-41% of good poses in 01b-02b.

**Unitree Go2 (JEPLO), lidar_slam_ros2 map of EIL_Mix:** every run initializes by
itself and none at a wrong place, and tracking stays within 3 cm (median). Before the
NDT score could judge ambiguity on its own (#201), one run in three ran out of global
attempts on ambiguous answers in the symmetric capture room.

| case | runs | initialized | wrong init | time to first pose (s) | median xy (m) | p95 xy (m) | max xy (m) | >1 m |
|---|---|---|---|---|---|---|---|---|
| EIL_Box | 3 | 3 | 0 | 17.1 | 0.03 | 0.09 | 4.61 | 0.001 |
| EIL_Stairs | 3 | 3 | 0 | 10.9 | 0.03 | 0.13 | 0.94 | 0.000 |
| EIL_Mask1 | 3 | 3 | 0 | 9.8 | 0.03 | 0.10 | 0.94 | 0.000 |
| EIL_Mask2 | 3 | 3 | 0 | 9.8 | 0.03 | 0.13 | 1.19 | 0.000 |
| Long_Stairs_self_map | 3 | 0 | 0 | - | - | - | - | - |
| Outdoor1_self_map | 1 | 0 | 0 | - | - | - | - | - |
| Outdoor2_self_map | 1 | 0 | 0 | - | - | - | - | - |

On the large Long_Stairs and Outdoor maps, global search did not find the Go2 with the
2026-10-06 defaults (before the sparse-band search became the quickstart default in #225).
The cause, measured below: the 2D search saturates. An upside-down MID-360 0.45 m
above the ground leaves about 100 points in the default height band. On a grid
inflated by 0.6 m they fit perfectly at many places, and branch-and-bound returns
eight of those places in search order. The 1.7-2.3 m offsets seen then are corners
of that perfect-fit area around the robot.

**Unitree Go2 wide maps, rebuilt inputs (2026-10-11).** The bags and maps were
rebuilt from the raw bags as above. The EIL cases use an EIL_Mix map accumulated the
same way, not the lidar_slam_ros2 map, so they differ from the table above. Runs:
baseline (defaults) and the [Large maps](go2_mid360_quickstart.md#large-maps)
settings (`go2_jeplo_low_sensor.yaml`), one 4-core machine, Jazzy:

| case | defaults: runs / initialized / wrong / lost | low-sensor settings: runs / initialized / wrong / lost | first pose (s) | median xy (m) |
|---|---|---|---|---|
| Long_Stairs_self_map | 1 / 0 / 0 / - | 2 / 2 / 0 / 0 | 12.3-12.6 | 0.02 |
| Outdoor1_self_map | 1 / 0 / 0 / - | 2 / 2 / 0 / 0 | 11.1-12.6 | 0.02 |
| Outdoor2_self_map | 1 / 0 / 0 / - | 2 / 2 / 0 / 0 | 11.1-14.6 | 0.02 |
| EIL_Box | 4 / 4 / 1 / 1 | 3 / 3 / 0 / 0 | 9.1-10.1 | 0.03 |
| EIL_Stairs | 4 / 4 / 0 / 0 | 3 / 3 / 0 / 0 | 9.4 | 0.02 |
| EIL_Mask1 | 4 / 4 / 3 / 2 | 3 / 3 / 0 / 0 | 8.8-10.1 | 0.02 |
| EIL_Mask2 | 4 / 4 / 1 / 1 | 3 / 3 / 0 / 0 | 9.3-9.5 | 0.02 |

"Lost" counts runs whose median error is above 1 m. The first-pose times and medians are
for the low-sensor runs. On this EIL map the defaults started wrongly in 5 of 16 runs,
EIL_Mask1 three times 1.1-2.0 m off, EIL_Mask2 once turned 180 degrees, and EIL_Box once
2 m off, after which tracking settled 4.5 m along the aisle.

See [experiments/go2_wide_map_global_init](../experiments/go2_wide_map_global_init/README.md)
for the diagnosis, the compared settings, and the EIL_Box start that falls on the
Go2 stepping off a box.

## Running it

Build lidar_slam_ros2 (for `rko_lio_odometry.launch.py`) and this package, then:

```bash
export JEPLO_ROOT=/data/jeplo                       # Unitree Go2, upside-down MID-360
export KOIDE_ROOT=/data/koide_hard_localization     # handheld MID-360 outdoors
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/go2_jeplo.yaml --out results/go2
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/koide_outdoor.yaml --out results/koide
```

Each case runs three times by default (`--repeats`, `--case` to select). The run
directories keep every log, the recorded trajectory (`est.tum`), the startup status
stream, and `score.json`. `summary.md` and `results.json` collect the cases.
`--evaluate-only` re-scores existing runs, and `--quickstart` runs a source checkout
instead of the installed package. Runs whose RKO-LIO odometry stalled (a registration
loop over 10 s in its exit profile, e.g. on a cold disk after a restart) are listed
under the tables: the startup then lacks odometry, so re-run them before comparing.

Datasets:

- JEPLO (Unitree Go2): <https://huggingface.co/datasets/ASIG-Lab/JEPLO-Dataset>
  (formerly `ASIG-X/JEPLO`, which returned 404 on 2026-10-11). The suite uses
  `bags_pc2_merged`, `gt` and, for Long_Stairs and Outdoor*, maps built from the same
  sequence (`maps_loc`). Ground truth covers the 10 m x 4 m capture area. The
  dataset now ships only the raw bags (`bags/`, Livox `CustomMsg` at 20 Hz) and `gt/`;
  rebuild the other two as below.
- Koide et al., hard localization: the outdoor MID-360 sequences with
  `map_outdoor_hard.ply` / `map_outdoor_kidnap.ply`. The kidnap sequences cover the
  sensor and carry it elsewhere.

### Rebuilding the JEPLO bags and self maps

`jeplo_to_pc2_bag.py` writes `PointCloud2` bags at 10 Hz, two sweeps per cloud,
stamped mid-sweep, with absolute per-point times (which `rko_lio_jeplo.yaml`
expects). The self maps accumulate the bag along RKO-LIO's own trajectory and move
it into the ground-truth frame. RKO-LIO's error on returning to the capture room
after 250-700 s is 2-4 cm on these sequences.

```bash
python3 tools/benchmark/jeplo_to_pc2_bag.py $RAW/bags/Long_Stairs $JEPLO_ROOT/bags_pc2_merged/Long_Stairs
# rko_lio_jeplo.yaml plus lidar_topic, imu_topic, base_frame: livox_frame,
# bag_path, dump_results: true, results_dir, under /**: ros__parameters
ros2 run rko_lio offline_node --ros-args --params-file long_stairs_rko.yaml
python3 tools/benchmark/build_map_from_odometry.py --bag $JEPLO_ROOT/bags_pc2_merged/Long_Stairs \
  --trajectory <results_dir>/Long_Stairs_0/Long_Stairs_tum_0.txt --out /tmp/Long_Stairs_odom.pcd
python3 tools/benchmark/align_map_to_ground_truth.py --map /tmp/Long_Stairs_odom.pcd \
  --trajectory <results_dir>/Long_Stairs_0/Long_Stairs_tum_0.txt \
  --ground-truth $JEPLO_ROOT/gt/Long_Stairs.txt --out $JEPLO_ROOT/maps_loc/Long_Stairs/map.pcd
```

These maps are not the ones the 2026-10-06 results above used.

### The Go2 map

The EIL_* cases localize the way a user would: on a lidar_slam_ros2 map of another
run, EIL_Mix, moved into the ground-truth frame by fitting its trajectory to the
ground truth (residual 0.09 m, the SLAM trajectory's own error):

```bash
bash <lidar_slam_ros2>/scripts/run_rko_lio_graph_autoware_dogfood.sh \
  --bag $JEPLO_ROOT/bags_pc2_merged/EIL_Mix --lidar-topic /livox/lidar --imu-topic /livox/imu \
  --lidarslam-param <lidar_slam_ros2>/lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  --rko-param tools/benchmark/suites/rko_lio_jeplo.yaml \
  --output-dir maps/eil_mix --wait-for-offline-completion --skip-viewer --base-frame base_link
python3 tools/benchmark/align_map_to_ground_truth.py --map maps/eil_mix/map.pcd \
  --trajectory maps/eil_mix/traj_raw.tum --ground-truth $JEPLO_ROOT/gt/EIL_Mix.txt \
  --out maps/eil_mix_gt/map.pcd
export JEPLO_LIDARSLAM_MAP=$PWD/maps/eil_mix_gt/map.pcd
```

The dataset's own leave-one-out maps (`maps_loo_*`) are not used: their floor is
smeared over 0.7 m (-0.4 to +0.3 m), so the occupancy grid generated for global
search marks almost the whole room as occupied, and initialization fails or lands on
the wrong pose. A map from lidar_slam_ros2 of the same room has its floor within 0.1 m.

## Autoware Istanbul (vehicle, preset replay)

A second, narrower benchmark for a car at up to 21 m/s on grades: the localization bag
of the [Autoware Istanbul urban
evaluation](https://docs.autoware.org/main/simulation-evaluation/components_evaluation/localization_evaluation/urban-environment-evaluation/)
replayed through `param/istanbul_gtsam_smoother.yaml`, with the bag's twist and no
odometry. Unlike the runs above it does not go through the quickstart and does not
initialize globally: each run starts from the GNSS pose at the window start and is
scored against `/sensing/gnss/pose_with_covariance`, interpolated at each pose stamp.

```bash
gdown 1WPWmFCjV7eQee4kyBpmGNlX7awerCPxc -O "$ISTANBUL/map/"
gdown 1yEB5j74gPLLbkkf87cuCxUgHXTkgSZbn -O "$ISTANBUL/localization_bag/"
ros2 bag reindex "$ISTANBUL/localization_bag" -s sqlite3
python3 tools/benchmark/istanbul_prepare.py "$ISTANBUL/localization_bag" /tmp/istanbul_fixture
# one window through one workspace
python3 tools/benchmark/istanbul_replay.py --ws WS --fixture /tmp/istanbul_fixture \
  --bag "$ISTANBUL/localization_bag" --map "$ISTANBUL/map/pointcloud_map.pcd" \
  --start 2300 --duration 120 --out /tmp/istanbul_run
# a baseline and a candidate workspace, three windows, three runs each
python3 tools/benchmark/istanbul_compare.py --base-ws BASE_WS --cand-ws CAND_WS \
  --fixture /tmp/istanbul_fixture --bag "$ISTANBUL/localization_bag" \
  --map "$ISTANBUL/map/pointcloud_map.pcd" --out /tmp/istanbul_compare
```

`--set KEY=VALUE` overrides a preset parameter (`--cand-set` for the candidate only);
the preset is read from the checkout that holds the tools. Set `ROS2_CLI` and
`ROS_PYTHON` when `python3` is not the interpreter the ROS distro was built for.

| Window | Bag time | Why |
| --- | --- | --- |
| A | 0-60 s | parked for 15 s, then up to 10 m/s |
| B | 2820-2970 s | sustained 6-10 % grades |
| C | 2300-2420 s | the fastest climb, 4-5 % at up to 21 m/s |

Median horizontal RMSE of three runs on 2026-11-01 (4 vCPUs): 0.23, 0.75 and 0.46 m, at
8.9-10 poses/s (the scans arrive at 10 Hz; median alignment 11-12 ms); how they were
reached is in
[experiments/istanbul_ndt_tracking](../experiments/istanbul_ndt_tracking/README.md).
Results before then (0.25, 0.87 and 0.50 m at 1.6-3.6 poses/s on 2026-10-28) ran an
unoptimized NDT_OMP
([Unoptimized ndt_omp](../experiments/istanbul_ndt_tracking/README.md#unoptimized-ndt_omp)).
`score.json` and the `istanbul_compare.py` summary also report the alignment cost: the
share of the window's scans that were aligned, the alignment time and the registration
iterations.

`param/istanbul_small_gicp.yaml` is the same preset with `registration_method:
SMALL_GICP` (needs small_gicp, e.g. `ros-jazzy-small-gicp-vendor`; pass `--preset
istanbul_small_gicp.yaml`). On 2026-10-28 it gave 0.26, 0.76 and 0.45 m at 8.0-9.5
poses/s, about as the optimized NDT_OMP preset does now; the NDT_OMP preset it was
compared with then (0.26, 0.91 and 0.54 m at 1.3-3.1 poses/s) ran unoptimized. On the
online Koide `outdoor_hard_02b` check, SMALL_GICP failed all five runs (88-93%
coverage, 0.32-0.41 m RMSE) where NDT_OMP passed all four. It slows down off the map
and misses clouds there, so NDT_OMP stays the default. Most of its RMSE gap comes from
the off-map stretch the odometry carries, which says nothing about the backend; outside
it the gap is 0.192 against 0.177 m
([experiments/koide_backend_comparison](../experiments/koide_backend_comparison/README.md)).

What to keep in mind with this bag (`istanbul_prepare.py` and `istanbul_replay.py`
handle the first three):

- **Header stamps are 2,371,531.4 s (27 days) before the bag's receive time.** The bag
  carries its own `/clock` in the header time base: it is played instead of `--clock`,
  and window starts are mapped through the clouds' stamps.
- **The first GNSS message is stamped 6.3 s after the first cloud**, and the first
  messages repeat a stale pose 1.9 m and 146° off the next ones; they are dropped.
- **GNSS drops out during 750-1030 s and 3020-3215 s** (sigma up to 92 m); windows there
  have neither a usable start pose nor a usable reference.
- **The GNSS orientation is about 2° off the direction of travel** (median -2.26° over
  the bag), so yaw RMSE against it carries that bias, and dead reckoning from the GNSS
  initial yaw drifts sideways.
- The localizer reads `/localization/util/downsample/pointcloud` (1500 points,
  `base_link`) and `/localization/twist_estimator/twist_with_covariance`, which matches
  the GNSS-derived speed and yaw rate within 0.05 m/s and 0.002 rad/s. Grades reach
  9.6 % where GNSS is accurate (z from -65 to +177 m over the route).

## Boreas (vehicle, preset replay)

A third preset replay, with LiDAR, IMU and ground truth: the
[Boreas dataset](https://www.boreas.utias.utoronto.ca/) (Burnett et al., 2023), read
from the public bucket `s3://boreas`. A car drives the same Toronto route on many
days. One sequence is the localization run, and the map is built from another day's
scans at their ground-truth poses. Like Istanbul it starts from the ground-truth pose
and does not go through the quickstart. The published `/pcl_pose` is matched to the
ground-truth lidar pose with the same stamp, without alignment.

```bash
# the first 120 s of boreas-2021-09-02-11-42, a map from boreas-2020-11-26-13-58
# (~11 min, 1.7 GB; scans are downloaded and voxel-downsampled on the way)
python3 tools/benchmark/boreas_prepare.py /tmp/boreas_fixture --duration 120
python3 tools/benchmark/boreas_replay.py /tmp/boreas_fixture /tmp/boreas_run --ws WS
```

`boreas_prepare.py` writes the bag, `map.ply` and `reference.tum`:

- **The bag** holds `/velodyne_points`, voxel-downsampled to 0.25 m, and `/imu/data` at
  200 Hz. The IMU comes from `applanix/imu_raw.csv`, which keeps gravity, rotated
  into the lidar frame.
- **The map** holds the other run's scans within 60 m of the window's path, every 2 m,
  at their ground-truth poses, thinned to 0.3 m.
- **The fixture has no twist.** The post-processed ground-truth velocity is not a
  runtime input.

`boreas_replay.py` runs `param/boreas_ndt_velodyne.yaml` with the fixture's map, the
first ground-truth pose, and `base_frame_id: velodyne`. `--set KEY=VALUE` overrides a
preset parameter. It writes `summary.json` with the RMSE, its horizontal and z parts,
p95 and max error, the RMSE per 20 s, and the share of scans aligned and accepted.

The window covers parking-lot maneuvers, including a U-turn of 208° in 10 s, then city
streets at up to 9.7 m/s. On 2026-11-02, with 4 vCPUs:

| Preset | Runs | RMSE m | Horizontal / z RMSE m | p95 / max error m | Scans accepted |
| --- | --- | --- | --- | --- | --- |
| `boreas_ndt_velodyne.yaml` | 3 | **0.132** (all three) | 0.126 / 0.041 | 0.24 / 0.41 | **100%** |
| the same before this change | 1 | 53.2 | 53.2 / 0.65 | 92.9 / 95.9 | 1% |

The RMSE per 20 s stays between 0.07 and 0.17 m. The three runs are identical, because
every scan is aligned.

Over the first 600 s (`--duration 600`: 4.3 km, 5786 scans, a map of 20.9 M points from
3189 scans; 11 GB, about an hour to prepare), on 2026-11-02:

| Setting | Runs | RMSE m | Horizontal / z RMSE m | p95 / max error m | Scans processed |
| --- | --- | --- | --- | --- | --- |
| `boreas_ndt_velodyne.yaml` | 2 | **0.105** (both) | 0.097 / 0.042 | 0.20 / 0.41 | 99.9%, 100% |
| with `use_imu_preintegration: true` | 2 | **0.098**, 0.099 | 0.089-0.090 / 0.042 | 0.18 / 0.39 | 98.8%, 99.3% |

- **The preset holds over the longer window.** The RMSE per 20 s stays between 0.05 and
  0.17 m. The first 120 s, with the parking-lot maneuvers, hold the worst bins
  (0.07-0.17 m); after them no bin exceeds 0.15 m, so the whole scores lower than its
  start.
- **The IMU preintegration helps here too.** It lowers the RMSE in 27 of the 30 bins,
  only horizontally, and seeds all but 6 of the scans.
- **The unprocessed scans were dropped by the cloud queue, under load.** These runs
  overlapped a download.
  - **Where:** the dropped scans fall in the dense stretches, such as 84-96 s and
    104-116 s, where 4600-4900 points remain after filtering against 2700 elsewhere.
  - **Without the IMU** the node keeps up there on 4 vCPUs: over the first 120 s its
    pose lags `/clock` by 0.12 s at p95 and 0.23 s at most.
  - **With it** the lag was 0.46-0.70 s at p95 and 0.75-0.92 s at most, close to the
    10-scan `cloud_queue_depth` (about 1 s). Four such runs lost no scan. Since the
    smoother's anchor carries its residual, an update takes about half the time.
    Three runs since then lag 0.25-0.59 s at p95 and 0.56-0.83 s at most, so from run
    to run the lag varies about as much as the fix changes it
    ([experiments/imu_smoother_solve](../experiments/imu_smoother_solve/README.md#the-anchor-never-let-it-converge)).
  - **`ndt_num_threads: 8` on 4 vCPUs is not a loss.** With 4 threads the alignment
    takes 1-2 ms longer. With two cores kept busy, 36-43 more scans drop (1107
    against 1143 without the IMU, 1048 against 1091 with it), since more threads keep
    a larger share of the CPU.
  - **The deskew is off.** With the preintegration on, the default `relative_motion`
    deskew took the IMU smoother's poses and ran on about 45 000 points a scan. That
    accounted for the 3-4 ms longer scan preparation. It changed nothing: with it off
    the preintegration still scores 0.127 m, so the gain comes from the IMU seed. The
    preset now sets `use_continuous_time_deskew: false`. Without the IMU the deskew
    never ran (`continuous_time_deskew_imu_preintegration_disabled`), and the preset's
    trajectory is byte-identical.
  - **With two cores kept busy** the lag reaches 1.3 s, and the queue drops the oldest
    scans: 74 of 1158 with the IMU, 15 without.
  - **NDT_OMP's fix takes most of that lag away**
    ([ndt_omp_ros2 #15](https://github.com/rsasaki0109/ndt_omp_ros2/pull/15)). Two pairs
    of runs with two cores kept busy were alternated back to back, before and after it,
    on 2026-11-03:

    | Two cores kept busy | Lag p95 | Lag max | `alignment_time_sec` mean | Scans dropped |
    | --- | --- | --- | --- | --- |
    | with the IMU, before #15 | 0.58-0.77 s | 0.90-1.16 s | 35.8-36.2 ms | 0, 0 |
    | with the IMU, with #15 | 0.12-0.16 s | 0.34-0.41 s | 24.1-25.9 ms | 0, 0 |
    | without the IMU, before #15 | 0.17-0.18 s | 0.40 s | 36.9 ms | 0, 7 |
    | without the IMU, with #15 | 0.11-0.12 s | 0.23 s | 26.9-28.5 ms | 0, 0 |

    The RMSE did not change (0.127 m with the IMU, 0.132-0.133 m without). In this
    session even the runs before #15 dropped few scans, so it is the lag that shows the
    difference. The IMU runs no longer approach the 10-scan queue. Without load, the
    IMU run's lag is 0.06 s at p95 with #15 and 0.08 s before it.
  - **Not the transport:** a second subscriber with the same QoS received every cloud.
  - **The preset run that lost 6 scans** lost them the same way; its RMSE is the same.

What to keep in mind with this dataset (`boreas_prepare.py` handles the first three):

- **The ground-truth rotation is `(Rz(heading) Ry(pitch) Rx(roll))^T`.** With the active
  `Rz Ry Rx` of the removed converter, two scans 3 s apart overlap at 0.06 instead of
  0.54, and a map built from them is wrong.
- **The point `time` field is seconds from the scan stamp** (−0.052 to +0.052 s), which
  is mid-sweep, not microseconds.
- **The bag plays each cloud at its stamp, mid-sweep.** A driver publishes it after the
  sweep, about 52 ms later. So at a scan's callback the IMU covers only half its sweep:
  the gyro `imu_pose_history` deskew reports `pose_history_ends_too_early` on every
  scan and never runs.
- **Timings drift between sessions.** On the same 4-vCPU container the preset, with a
  byte-identical trajectory, aligned in 31.1 ms on average in one session and 20.1 ms in
  another. Compare timings only between runs made back to back.
- **NDT_OMP aligns faster since
  [ndt_omp_ros2 #15](https://github.com/rsasaki0109/ndt_omp_ros2/pull/15).** Run back
  to back, the preset aligned in 12.2-12.4 ms on average with it and 18.1-19.1 ms
  without, with a byte-identical trajectory. The runs above predate it
  ([experiments/ndt_alignment_cost](../experiments/ndt_alignment_cost/README.md#inside-align-false-sharing)).
- **`imu.csv` is post-processed and has gravity removed.** `imu_raw.csv` keeps it, in the
  Applanix frame turned by 180° about (1, −1, 0)/√2. In the lidar frame its yaw rate
  follows the ground-truth heading rate with correlation 1.0. Parked, it reads
  (0.094, 0.323, 9.794) m/s² against (0.092, 0.327, 9.801) from the ground-truth
  attitude.
- **The ~1 MB clouds need a reliable subscription.** With the best-effort sensor QoS,
  38 of 97 scans arrived (`qos_overrides` in the preset).
- **Gaps used to cost the most.** The seed advanced the last pose by one scan's
  motion. After a 0.6 s gap at 9.5 m/s it was 5 m behind, and NDT locked 4 m back
  along the road. It is now scaled to the time since the last pose
  (`previous_delta_max_extrapolation_sec`). With 0.6 s dropped every 10 s the window
  scores 0.132 m, as without gaps, against 0.8-6.3 m before
  ([experiments/previous_delta_time_scaling](../experiments/previous_delta_time_scaling/README.md)).
  The preset still reuses each 100 m target for 10 m and builds the next one in the
  background, with a 60 m scan range, so a scan does not wait for one.
- **The IMU preintegration helps a little; the legacy `use_imu` deskew broke the
  run.** With `--set use_imu_preintegration=true` the window scores 0.127 m (twice),
  with every scan processed; before the smoother's solve went sparse it skipped one
  scan in five. With
  `use_imu: true` it scored 1.7 m, with 6% of the scans accepted: that deskew
  integrated gravity, and was removed in 1.8.0. With both, the IMU callback used to wait behind every
  registration; it now has its own callback group
  ([imu_estimation.md](imu_estimation.md#validation-state-honest-limits)).

### Soak and fault injection

`soak_replay.py` runs one localizer for an hour on the 120 s fixture, played lap after
lap at 1x. Each lap's stamps and `/clock` are shifted past the laps before it, and an
`/initialpose` at the ground-truth start pose begins every lap after the first. Every
5 s it samples the node's resident memory, CPU use and threads from `/proc`.
`--fault KIND@LAP:START[+DURATION]` injects a fault into one lap: `lidar` or `imu`
drops those messages, `stall` stops the node (SIGSTOP, then SIGCONT), `reinit` sends
`/initialpose` at the ground-truth pose there, while moving. `--wrap` runs the node
under a profiler such as heaptrack.

```bash
python3 tools/benchmark/soak_replay.py /tmp/boreas_fixture /tmp/boreas_soak --ws WS \
  --minutes 60 --set use_imu_preintegration=true
python3 tools/benchmark/soak_replay.py /tmp/boreas_fixture /tmp/boreas_faults --ws WS \
  --minutes 20.4 --set use_imu_preintegration=true \
  --fault lidar@1:50+1 --fault lidar@2:50+5 --fault stall@5:50+2 --fault reinit@7:60
```

`summary.json` holds each lap's RMSE, max error, accepted scans, alignment time and pose
latency, the memory trend, and per fault the largest error from its start to 10 s after its end, the
time from its end until the error stays under 0.5 m for the rest of the lap, and the
RMSE after that.

An hour with `use_imu_preintegration: true` on v1.8.0, with 4 vCPUs, on 2026-11-07:

- **Tracking does not wear down.** All 29 laps score 0.127 m RMSE, 0.36 m at most, and
  every scan is accepted. The alignment takes 7.8-9.2 ms at the median and 23-31 ms
  at p95 in every lap. CPU use averages 74% of one core, with 43 threads.
- **Resident memory climbs from 1.5 to 1.9 GB, 2.2 GB at most, but nothing leaks.**
  Under heaptrack the heap in use bottoms out at 403-426 MB in every half lap for
  12 minutes, and peaks at 1.1-1.3 GB. glibc keeps what is freed across its malloc
  arenas: with `MALLOC_ARENA_MAX=2` the same hour stays at 0.9-1.1 GB.

Since then the node keeps no older registration targets alive, sizes the local-map crop
to its points, and limits glibc to two malloc arenas. Five laps each, with
`use_imu_preintegration: true`, on 2026-11-07:

| Build | Resident memory, max | Lowest in each lap | Heap in use (heaptrack) | Alignment median / p95 |
| --- | --- | --- | --- | --- |
| v1.8.0 | 2.0 GB | 1.3-1.6 GB | 0.40-1.3 GB | 8.0-9.2 / 24-31 ms |
| now | **0.82 GB** | **0.62-0.66 GB** | 0.28-0.69 GB | 8.1-9.0 / 24-28 ms |

- **The heap peaked with the local-map crops.** Eight older targets were kept alive,
  each a crop of the dense map of up to 1.4 million points in a buffer grown to 2.5
  million (79 MB). Freed, they halve the heap's peak.
- **The resident memory follows the arenas.** The smaller heap alone left it at
  1.5-2.2 GB; two arenas take it down, for 2-6% more of one core (83% against
  85-89% in the same session).
- **`MALLOC_ARENA_MAX` in the environment still wins.** The limit is set in
  `lidar_localization_node`; a component loaded into another container needs
  `MALLOC_ARENA_MAX=2` in that container's environment.

Faults 50 s into a lap, at 5-8 m/s, on v1.8.0 with `use_imu_preintegration: true`:

| Fault | Poses published during it | Largest error | Back under 0.5 m after |
| --- | --- | --- | --- |
| none (laps 0 and 8) | | 0.36 m | |
| LiDAR gap, 1 s | 0 | 0.24 m | 0.02 s |
| LiDAR gap, 2 s | 0 | **8.6 m** | 1.6 s |
| LiDAR gap, 5 s | 0 | no pose for the rest of the lap | never |
| LiDAR gap, 30 s | 0 | no pose for the rest of the lap | never |
| IMU gap, 10 s | 96 | 0.29 m | 0.05 s |
| node stopped, 2 s | 19 | 1.5 m | 0.06 s |
| node stopped, 10 s | 19 | **54 m** | never |
| `/initialpose` while moving | | 0.28 m | 0.05 s |

- **After more than 1 s without an accepted scan the IMU seed is dropped.** Its window
  runs from the last accepted scan, and above 1 s the seed falls back to the previous
  scan's motion, which advances at most 1 s. After the 5 s gap the seed had moved
  6.2 m of the 33.4 m driven, every scan was rejected, and the reinitialization
  request came 19 s later.
- **A wrong alignment can pass `score_threshold: 6`.** The poses more than 1 m off
  were accepted at fitness 0.5-5.9 with 50-74% of the points within 0.5 m of the map,
  against 96% at the median when tracking (85% at the 1st percentile).
  `min_inlier_ratio: 0.8` rejects them, but then the laps with the 2 s and 5 s gaps
  and both stops publish no pose after the fault either.

Since then the preset sets `min_inlier_ratio: 0.8`, and a rejected scan past the IMU
window is aligned once more from the IMU prediction, up to 10 s. On 2026-11-07:

| Fault | Poses published during it | Largest error | Back under 0.5 m after |
| --- | --- | --- | --- |
| LiDAR gap, 2 s | 0 | 0.25 m | 0.06 s |
| LiDAR gap, 5 s | 0 | 0.28 m | 0.07 s |
| LiDAR gap, 30 s | 0 | no pose for the rest of the lap | never |
| node stopped, 2 s | 0 | no pose for the rest of the lap | never |
| node stopped, 10 s | 0 | no pose for the rest of the lap | never |

- **The gaps of 2 and 5 s recover on the first scan after them.** That scan's
  previous-scan seed is rejected, and the retry from the IMU, over a 2.1 and a 5.1 s
  window, is accepted with 98-99% inliers. Both laps score 0.126-0.127 m RMSE, like
  the laps without a fault.
- **No wrong pose is accepted in any lap.** The longer faults end in a
  reinitialization request 14-18 s later.
- **The stops lose IMU samples too.** In the second after the node resumes, 5-15
  samples arrive per scan instead of 21, so the prediction no longer covers the window
  and there is nothing to retry from.

Since then a stop of 2 s recovers too. On 2026-11-08, twice each:

| Fault | Poses published during it | Largest error | Back under 0.5 m after |
| --- | --- | --- | --- |
| node stopped 2 s, 50 s into the lap, in a turn | 17-18 | 0.24 m | 0.06 s |
| node stopped 2 s, 30 s into the lap | 19 | 0.27 m | 0.04 s |
| node stopped 10 s | 0 | no pose for the rest of the lap | never |

- **The IMU gap stayed in every window.** A window starts at the last accepted scan,
  and a gap of more than 0.5 s between IMU samples broke its coverage, so after a stop
  no IMU prediction was made again: `imu_integrated_sample_count` stayed 0 while 18-31
  samples arrived per scan. For the retry, the sample before a gap is now held across
  it, up to the retry's 10 s.
- **Held across a turn, that prediction is short of it.** The 2 s stop at 50 s falls in a
  turn tightening from 5 to 12 deg/s. Its first retry was 8 deg short in yaw and 0.73 m
  off, and aligned with 67% inliers; turned by -4 to -12 deg it aligned with 98%, 3 cm
  from the reference, while turned the other way it never passed 63%. A rejected
  bridged retry is now aligned again turned by -6, 6, -12 and 12 deg, within 3 s of
  the last accepted scan, and the best that passes the gate is taken.
- **After 10 s the prediction is metres and 20 deg off,** beyond that search. The node
  rejects every scan and requests reinitialization, as before. With the global
  localization recovery nodes beside it (`--companion`), it does not recover either:
  their map-wide search misses there and answers in 18-21 s
  ([global localization](global_localization.md#on-the-boreas-fixture-after-a-10-s-stop)).

#### Under a memory limit

With the node alone in a memory cgroup, `--wrap` set to a script that writes its PID
into the cgroup's `cgroup.procs` and runs the node, on 2026-11-08, with
`use_imu_preintegration: true` and the preset's former 100 m crop:

| Memory limit | Result |
| --- | --- |
| 1 GB | an hour, 29 laps at 0.127 m RMSE, every scan accepted; 890 MiB at most, and the limit was never reached |
| 768 MB | killed 105 s into the first lap |
| 640 MB | killed at 95 s |
| 512 MB | killed at 80 s |

- **The peak comes with the dense stretch.** The kills fall where 1.4 million map
  points lie within the crop, as the heap's peak did. With `local_map_async_rebuild`
  the node holds a second target while it builds the next one.
- **The crop radius sets the peak** (troubleshooting:
  [The Node Is Killed for Memory](troubleshooting.md#symptom-the-node-is-killed-for-memory)).
  With the 80 m crop the preset now uses, an hour under 800 MB passed: 29 laps at
  0.127 m, 765 MiB at most, pose latency 57 ms at p95 and 222 ms at most. The cgroup
  reached its limit 4070 times and gave up page cache each time; nothing was killed.
  Without the IMU the 80 m crop scores 0.132 m like the 100 m one and peaks 75 MiB
  lower; the trajectories differ by 1.8 cm at most (3.0 cm with it).
- **The lowest memory of each lap rises over the hour**, by 50-100 MB, from 600-610
  to 660-710 MB with the 100 m crop and from 574 to 609-628 MB with the 80 m one.

Since then the node frees a replaced registration target before it builds the next
one, and returns the freed memory to the OS:

- **Three crops were alive during a rebuild.** heaptrack over one lap with the 80 m crop
  put 110 MB of crops in the 547 MiB peak: the one in use, the replaced one, still held
  by the spare registration objects with its search tree, and the new one. The next
  target is now built on fresh objects, and the replaced one is freed before the crop.
- **glibc kept what was freed.** It returns only the top of each heap, and a target is
  freed in the middle of it. With `malloc_trim` every 5 s (an `LD_PRELOAD` shim) the
  lowest memory of each lap stayed at 378-400 MB for an hour. The node now trims once
  a target change freed memory, at the end of that scan's callback: a trim takes 1.4 ms
  at the median and 15 ms at most.
- **An hour under 800 MB:** 29 laps at 0.127 m, 576 MiB at most (765 MiB before), the
  lowest memory of each lap 329-339 MB (580-632 MB before), and the limit never reached
  (4070 times before).
- **One lap each, alternated with the build before, three times:** 525-539 against
  613-652 MiB at the peak, the same alignment time (9.5-9.8 against 9.4-10.0 ms at the
  median) and RMSE, and pose latency 58-59 against 55-57 ms at p95: once per target
  change, the next scan waits for the trim.

`summary.json` also holds each lap's pose latency: from publishing a cloud to receiving
its pose, at the median, p95 and most, and how many poses came over 200 ms late.


## Low-resource budget (Go2, MID-360)

For a Jetson Orin Nano/NX on a Unitree Go2 or G1, shared with perception: how much
CPU and memory the localizer needs, and what can be cut
([the preset](mid360_legged_jetson.md#low-resource-preset)). Measured on 2026-11-18
on a 4-vCPU x86 host (Xeon, 2.3 GHz), the localizer pinned to two cores
(`taskset`) in its own cgroups, the replay harness and the odometry on the other
two. The Orin's Cortex-A78AE cores are slower for this code, so the CPU figures are a
lower bound. CPU is in cores: CPU seconds per second of data, the mean and the p95 of
1 s intervals. Memory is the resident set (VmRSS, with about 53 MB of shared-library
pages) and its anonymous part (RssAnon). Latency runs from the replay's control node
receiving a cloud to receiving its pose.

```bash
python3 tools/benchmark/boreas_replay.py FIXTURE OUT --ws WS \
  --preset param/mid360_legged_low_resource.yaml \
  --points-topic /livox/lidar --imu-topic /livox/imu --base-frame livox_frame \
  --wrap "budget.sh 2,3 256" \
  --companion "rko_lio online_node --ros-args --params-file rko_jeplo_online.yaml"
python3 scripts/run_koide_public_bag.py --data-dir DATA --output OUT \
  --preset param/mid360_legged_low_resource.yaml --set base_frame_id=livox_frame \
  --set imu_accel_scale=1.0 --wrap "budget.sh 2,3 0"
```

`budget.sh CPUS MEM_MB` writes its PID into a memory cgroup limited to MEM_MB (and,
for the quota runs, a cpu cgroup with `cpu.cfs_quota_us`), then
`exec taskset -c CPUS "$@"`. The JEPLO fixture is a bag converted with
`jeplo_to_pc2_bag.py`, a map of EIL_Mix built from RKO-LIO's trajectory and moved
into the motion-capture frame (580k points), and the ground truth as
`reference.tum`; the localizer starts from the first ground-truth pose. The Koide
runs also pass `use_odom_tf_prediction`, `publish_bridge_pose_when_lost`,
`use_sim_time`, `point_timestamp_unit: auto`, no pose timer and the reliable
cloud QoS; that bag is already in m/s^2.

### EIL_Stairs with odometry

RKO-LIO publishes `odom -> livox_frame`; `use_odom_tf_prediction` and
`publish_bridge_pose_when_lost` are on. Each row is `mid360_legged.yaml` with one
change, then the combinations. 137 s, 1370 scans:

| setting | RMSE / max (m) | aligned | cores mean / p95 | CPU ms/scan | peak RSS (MB) | latency median / p95 / max (ms) |
|---|---|---|---|---|---|---|
| `mid360_legged.yaml` | 0.077 / 0.82 | 1359/1370 | 0.27 / 0.36 | 27.2 | 132 | 24 / 34 / 80 |
| `mid360_legged.yaml`, odometry in m/s^2 | 0.066 / 0.27 | 1370/1370 | 0.24 / 0.33 | 24.5 | 123 | 22 / 30 / 125 |
| `ndt_num_threads: 2` | 0.066 / 0.25 | 1363/1370 | 0.26 / 0.38 | 26.7 | 123 | 24 / 36 / 86 |
| `ndt_num_threads: 1` | 0.066 / 0.27 | 1370/1370 | 0.24 / 0.33 | 24.1 | 133 | 26 / 35 / 90 |
| `ndt_max_iterations: 15` | 0.066 / 0.27 | 1370/1370 | 0.23 / 0.31 | 23.7 | 133 | 22 / 30 / 47 |
| `voxel_leaf_size: 0.4` | 0.067 / 0.29 | 1370/1370 | 0.18 / 0.25 | 18.7 | 122 | 20 / 26 / 42 |
| `voxel_leaf_size: 0.5` | 0.067 / 0.26 | 1370/1370 | 0.16 / 0.22 | 16.4 | 133 | 19 / 24 / 38 |
| `scan_max_range: 30` | 0.066 / 0.26 | 1370/1370 | 0.23 / 0.34 | 23.2 | 133 | 22 / 30 / 70 |
| `use_continuous_time_deskew: false` | 0.066 / 0.26 | 1370/1370 | 0.22 / 0.32 | 22.8 | 122 | 21 / 30 / 74 |
| `use_imu_preintegration: false` | 0.066 / 0.33 | 1361/1370 | 0.35 / 0.48 | 35.6 | 121 | 22 / 32 / 93 |
| `imu_smoother_window_size: 20` | 0.066 / 0.26 | 1370/1370 | 0.21 / 0.30 | 21.3 | 132 | 19 / 27 / 68 |
| `ndt_resolution: 2.0` | 0.293 / 1.22 | 1365/1370 | 0.24 / 0.33 | 24.0 | 124 | 22 / 31 / 71 |
| map voxelized at 0.1 m (133k points) | 0.081 / 0.32 | 1370/1370 | 0.22 / 0.30 | 22.0 | 88 | 22 / 29 / 52 |
| map voxelized at 0.2 m (28k points) | 0.103 / 0.38 | 1361/1370 | 0.21 / 0.29 | 21.4 | 79 | 22 / 28 / 41 |
| `min_scan_interval_sec: 0.19` (5 Hz) | 0.076 / 0.78 | 685/1370 | 0.15 / 0.22 | 15.1 | 133 | 24 / 34 / 64 |
| `enable_timer_publishing` (20 Hz) | 0.084 / 0.62 | 1370/1370 | 0.25 / 0.36 | 25.1 | 133 | 22 / 32 / 80 |
| threads 2, voxel 0.5, 15 iterations | 0.068 / 0.28 | 1370/1370 | 0.18 / 0.24 | 18.4 | 132 | 20 / 26 / 34 |
| the same, window 20 | 0.067 / 0.27 | 1370/1370 | 0.15 / 0.21 | 15.2 | 132 | 17 / 22 / 32 |
| the same, window 10 | 0.068 / 0.27 | 1370/1370 | 0.13 / 0.17 | 13.2 | 132 | 15 / 19 / 27 |
| the same (window 50), 5 Hz | 0.067 / 0.25 | 685/1370 | 0.12 / 0.16 | 12.4 | 121 | 21 / 27 / 37 |
| `mid360_legged_low_resource.yaml` | 0.067 / 0.26 | 1370/1370 | 0.21 / 0.28 | 21.7 | 132 | 21 / 28 / 38 |
| the same, repeat | 0.067 / 0.26 | 1370/1370 | 0.20 / 0.26 | 20.2 | 121 | 20 / 27 / 38 |
| the same, in 256 MB | 0.067 / 0.26 | 1370/1370 | 0.19 / 0.26 | 19.6 | 121 | 20 / 27 / 34 |
| the same, window 30 | 0.067 / 0.26 | 1363/1370 | 0.16 / 0.20 | 16.5 | 132 | 18 / 22 / 59 |
| the same, window 20, 5 Hz | 0.127 / 1.26 | 682/1370 | 0.10 / 0.13 | 9.7 | 132 | 17 / 22 / 33 |
| the same, 5 Hz | 0.072 / 0.52 | 685/1370 | 0.14 / 0.18 | 13.9 | 132 | 22 / 29 / 35 |

- **The scan voxel is the largest single cut**: 0.5 m leaves about 550 of 1550 points
  and a third of the CPU, at the same error. The NDT resolution is not: 2.0 m
  quadrupled the error.
- **The IMU smoother is the next largest cost.** perf with the 0.5 m voxel: the
  smoother's sparse solves took 25% of the node's CPU, the ROS executor and DDS 18%,
  NDT 16%, the kernel 13%, malloc and copies 11%, the fitness kd-tree 6%, scan
  conversion, deskew and voxel filter 4%. It re-optimizes all 50 scans of its window
  on every update. `imu_smoother_window_size` (new) at 20 or 10 cuts 18-29% here
  with the same error, but lost the Koide bag (below).
- **Without the IMU, more CPU, not less** (0.35 against 0.24 cores): NDT then needs
  more iterations from a worse seed.
- **Threads do not save CPU**, they spread it: one thread costs the same and adds
  4 ms of latency. Deskew (0.02 cores), the range (0.01) and the pose timer (0.01)
  cost little.
- **The map's point count sets the memory**: 580k points, 133k (0.1 m) and 28k
  (0.2 m) peaked at 123-133, 88 and 79 MB, but the error rose by 23% and 56%.
- **Five Hertz is marginal**: 0.12-0.15 cores and 0.067-0.076 m RMSE, but up to
  0.52-1.26 m off.
- The first two rows differ in RKO-LIO's acceleration unit: `g`, as the MID-360
  publishes it, in the first; m/s^2 in the second, where its initialization absorbed
  gravity into an 8.8 m/s^2 accelerometer bias. Run to run, the CPU varies by about
  0.02 cores and the resident memory between 121 and 133 MB.

RKO-LIO itself took 0.06-0.07 cores and 40 MB (13 MB anonymous).

### EIL_Stairs without odometry

| setting | RMSE / max (m) | aligned | cores mean / p95 | CPU ms/scan | peak RSS (MB) | latency median / p95 / max (ms) |
|---|---|---|---|---|---|---|
| `mid360_legged.yaml` | 0.130 / 1.76 | 1354/1370 | 0.29 / 0.43 | 29.4 | 132 | 15 / 29 / 68 |
| `voxel_leaf_size: 0.5` | 0.300 / 1.53 | 1207/1370 | 0.17 / 0.25 | 18.6 | 132 | 61 / 107 / 125 |
| threads 2, voxel 0.5, 15 iterations | 0.266 / 1.38 | 1201/1370 | 0.19 / 0.25 | 19.3 | 132 | 62 / 107 / 126 |
| threads 2, 15 iterations, window 20 | 1.018 / 13.18 | 1091/1370 | 0.31 / 0.74 | 31.2 | 132 | 64 / 109 / 126 |

Without odometry the localizer seeds from its own motion model. The 0.5 m scan then
locked onto the wrong stair level (1.5 m up) for seconds, with a good fitness, and
two threads, 15 iterations and a 20-scan window lost the robot (13 m off at 19 s).
Do not cut below `mid360_legged.yaml` without odometry.

### Under a CPU quota and a memory limit

The same two cores with a CFS quota (`cpu.cfs_quota_us` per 100 ms period); the rows
"window 20" are an earlier version of the preset with a 20-scan IMU window:

| run | quota (cores) | aligned | RMSE / max (m) | latency median / p95 / max (ms) |
|---|---|---|---|---|
| `mid360_legged.yaml` | 0.25 | 1253/1370 | 0.067 / 0.26 | 74 / 261 / 562 |
| `mid360_legged.yaml` | 0.25 | 1190/1370 | 0.067 / 0.25 | 153 / 252 / 765 |
| low resource | 0.25 | 1370/1370 | 0.067 / 0.26 | 20 / 97 / 113 |
| low resource | 0.15 | 1049/1370 | 0.069 / 0.26 | 112 / 212 / 408 |
| low resource, window 20 | 0.25 | 1370/1370 | 0.067 / 0.27 | 16 / 21 / 119 |
| low resource, window 20 | 0.15 | 1360/1370 | 0.068 / 0.27 | 26 / 129 / 233 |
| low resource, window 20 | 0.15 | 1349/1370 | 0.067 / 0.24 | 16 / 113 / 212 |
| low resource, window 20 | 0.10 | 1070/1370 | 0.068 / 0.24 | 136 / 238 / 437 |
| low resource, window 20, 5 Hz | 0.10 | 680/1370 | 0.066 / 0.23 | 17 / 116 / 310 |

- **Short of CPU, the node drops scans** (`cloud_queue_depth: 1`) instead of
  building up lag, and the odometry carries the pose: no quota run came more than
  0.27 m off. The preset needs about 0.2 cores here; at 0.15 it aligns 77% of the
  scans.
- **Memory**: `mid360_legged.yaml` and the preset passed in a 256 MB cgroup, the
  preset with window 20 also in 128 and 96 MB, at the unlimited error. The node's anonymous memory is
  67-78 MB with this map, and the cgroup reclaims page cache first.

### Koide outdoor_hard_02b

A handheld MID-360 outdoors, 298 s, about 100 s of it off the 2.0M-point map;
`run_koide_public_bag.py` with its own RKO-LIO:

| setting | RMSE / on map / p95 (m) | localizer cores mean / p95 | CPU ms/scan | peak / steady RSS (MB) | anonymous peak / steady (MB) | latency median / p95 / max (ms) | RKO-LIO cores, peak RSS |
|---|---|---|---|---|---|---|---|
| `koide_outdoor_hard_02b_online.yaml` (75 m crop, no IMU) | 0.240 / 0.171 / 0.54 | 0.44 / 0.76 | 60 | 361 / 297 | 289 / 244 | 28 / 95 / 456 | 0.31, 168 MB |
| `mid360_legged.yaml` | 0.245 / 0.193 / 0.58 | 0.70 / 1.57 | 95 | 360 / 357 | 306 / 302 | 43 / 183 / 551 | 0.31, 170 MB |
| `mid360_legged_low_resource.yaml` | 0.228 / 0.203 / 0.46 | 0.50 / 1.01 | 68 | 354 / 350 | 300 / 296 | 36 / 119 / 320 | 0.31, 173 MB |
| the same, repeat | 0.227 / 0.203 / 0.43 | 0.45 / 0.88 | 62 | 354 / 351 | 300 / 297 | 30 / 109 / 268 | 0.32, 171 MB |
| the same, IMU window 20 | 4.340 / 5.228 / 12.50 | 0.53 / 0.99 | 72 | 356 / 350 | 302 / 296 | 37 / 125 / 354 | 0.34, 172 MB |
| the same, IMU window 20, 30 iterations | 3.221 / 3.884 / 11.51 | 0.53 / 1.09 | 72 | 352 / 351 | 298 / 297 | 35 / 144 / 318 | 0.32, 170 MB |
| the same, IMU window 30 | 22.204 / 19.085 / 32.03 | 0.70 / 1.13 | 95 | 350 / 348 | 296 / 294 | 55 / 150 / 364 | 0.32, 170 MB |

- **The preset takes 0.45-0.50 cores instead of 0.70**, at the same error.
  `mid360_legged.yaml` dropped 3% of the scans (coverage 96.8%, a FAIL of the
  replay's 97% threshold).
- **A shorter IMU smoother window lost the pose**: from 67 s on, with 20 scans for
  about 40 s (up to 14-24 m off, both runs), with 30 scans for the rest of the bag
  (up to 38 m). With 50 the same settings held. The cause was one IMU retry seed, not
  the window; it is fixed ([below](#memory-and-cpu-in-code)).
- **Memory is the map's**: about 350 MB resident and 300 MB anonymous whatever the
  preset. In 256 MB and in 192 MB the node was killed while it built the NDT target
  (192 MB: at 194 MB anonymous). heaptrack put the heap's peak at 292 MB: the map
  points (XYZI, 32 bytes each) 63 MB, the NDT voxel grid 58 MB, the map file as read
  24 MB, and about 100 MB in FLANN kd-trees, mostly the one over the map that scores
  the fitness. Since then the node fits in 224 MB ([below](#memory-and-cpu-in-code)).
- **RKO-LIO costs as much as the localizer outdoors**: 0.31-0.34 cores and 170 MB
  (136 MB anonymous) here, against 0.07 cores and 40 MB indoors.

### Memory and CPU in code

The same setup on 2026-11-18, `main` before these changes against after: the fitness
kd-tree and map loading (memory), the IMU smoother's block solve (CPU) and no IMU
retry after an odometry seed. CPU is CPU seconds per second of data and the p95 of
1 s intervals; memory is peak / steady, anonymous then resident.

| run | RMSE / largest (m) | aligned | CPU s/s, p95 | ms/scan | anonymous MB | resident MB | latency median / p95 (ms) |
|---|---|---|---|---|---|---|---|
| EIL_Stairs, low resource, before | 0.067 / 0.24-0.26 | 1363-1370 | 0.20-0.22, 0.26-0.28 | 20-22 | 68-77 | 121-132 | 21 / 28-29 |
| EIL_Stairs, low resource, after | 0.067 / 0.26 | 1370 | 0.145, 0.18 | 14.5 | 44 | 98 | 15.5 / 20 |
| EIL_Stairs, `mid360_legged.yaml`, before | 0.066 / 0.25 | 1370 | 0.29, 0.42 | 29 | 78 | 132 | 24 / 36 |
| EIL_Stairs, `mid360_legged.yaml`, after | 0.066 / 0.25 | 1364 | 0.21, 0.28 | 21 | 46 | 100 | 18 / 25 |
| Koide, low resource, before | 0.228-0.236 (0.201-0.203 on the map) | | 0.51-0.52, 1.01 | 69-70 | 300-302 / 295 | 354-356 | 35 / 119-124 |
| Koide, low resource, after (3 runs) | 0.227-0.235 (0.201-0.203) | | 0.40-0.41, 0.70-0.71 | 54-55 | 218-221 / 215 | 272-275 | 26-27 / 90-93 |
| Koide, `mid360_legged.yaml`, before | 0.245 (0.193) | | 0.70, 1.57 | 95 | 306 / 303 | 360 | 43 / 183 |
| Koide, `mid360_legged.yaml`, after | 0.238 (0.193) | | 0.59, 0.96 | 80 | 225 / 219 | 280 | 37 / 137 |
| Boreas 120 s, before | 0.132 / 0.41 | 1158 | 0.79, 1.37 | 82 | 496 / 389 | 554 / 442 | 28 / 75 |
| Boreas 120 s, after | 0.132 / 0.41 | 1158 | 0.77, 1.41 | 80 | 397 / 272 | 456 / 326 | 26 / 77 |
| Boreas, `use_imu_preintegration`, before (2 runs) | 0.127 / 0.36 | 1158 | 0.73, 1.47-1.48 | 75-76 | 468-482 / 383-384 | 538-540 / 436-437 | 25 / 76-83 |
| Boreas, `use_imu_preintegration`, after (2 runs) | 0.127 / 0.36 | 1158 | 0.73-0.78, 1.55-1.59 | 76-81 | 375-377 / 305-313 | 434-438 / 359-366 | 23-25 / 90-110 |

- **The fitness kd-tree was the largest single block of memory.** PCL's FLANN tree
  copies the target's coordinates twice and adds an index mapping: 98 MB of heap for
  the 2.0M-point Koide map in a micro-benchmark. `CompactKdTree` keeps a 4-byte
  index per point and its nodes, 13 MB, and reads the points from the target; it
  returns the same squared distances (all 200 000 test queries bit-identical,
  near and up to 20 m off the map) and answered 23% faster. Freeing the map file as
  read (24 MB) and the xyz-to-XYZI temporary (32 MB) before the target is built took
  the load off the peak: the node's anonymous memory is now highest at the end of the
  run, not while it builds the target.
- **The 2.0M-point map now runs in a 224 MB cgroup** (low resource, 0.235 m); it was
  killed at 256 MB before. 192 MB, tried with the kd-tree change only, was killed
  during the run. The anonymous memory still grows by about 30 MB over the 298 s,
  likely glibc keeping freed memory: fixed `MALLOC_MMAP_THRESHOLD_` and
  `MALLOC_TRIM_THRESHOLD_` (4 and 8 MB) took 10-15 MB of it off; not adopted.
- **The IMU smoother is about four times cheaper.** Its normal equations are block
  tridiagonal over the poses plus the bias rows; they are now solved by blocks with a
  6 x 6 Schur complement, instead of triplets and a general sparse LDLT. On a
  synthetic 1500-scan walk an update took 0.46 instead of 2.0 ms (window 50), an IMU
  sample 0.76 instead of 0.99 us, with the same poses and biases to 1e-15. On the
  replays that is 0.055-0.06 cores, a quarter of the low-resource preset's CPU on
  EIL_Stairs (0.20 with the kd-tree change alone, 0.145 with this). On Koide the
  changes together took 0.51 to 0.40 cores.
- **Why a shorter IMU window lost the Koide pose**: it did not, by itself. At 435.7 s
  of the ground truth, 9.9 s into a streak of rejected scans whose odometry seeds were
  0.2-0.3 m off, the node aligned the scan again from the IMU prediction dead-reckoned
  over those 9.9 s. That alignment came 10-35 m off with a fitness of 2.9-4.3 (the
  threshold is 6) and was accepted, in 8 of 13 runs: three of the study's four with a
  20- or 30-scan window, and all five with the kd-tree change alone (their scans were
  processed about 7 ms sooner, which likely changed the IMU data the retry saw; not
  traced). A scan seeded from odometry no longer gets that retry; since then all six
  Koide runs held, two of them with a 20-scan window (0.227-0.228 m). With the
  smoother this cheap, 20 scans save only about 0.01 cores (0.134 against 0.145 on
  EIL_Stairs, the same on Koide), so the preset keeps 50.
- **Boreas** (`boreas_ndt_velodyne.yaml`, local map crops) gave a byte-identical
  trajectory without the IMU and one within 1.5 cm with it; its memory fell by the
  crops' kd-trees, about 100 MB. Its CPU did not change: the smoother is a small part
  of its 75-80 ms per scan, and there is no odometry seed.
