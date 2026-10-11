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
instead of the installed package.

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

These maps are not the ones the 2026-10-06 results below used.

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

## Results

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

On the large Long_Stairs and Outdoor maps, global search does not find the Go2.
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
