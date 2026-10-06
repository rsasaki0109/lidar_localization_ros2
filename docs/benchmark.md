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

- JEPLO (Unitree Go2): <https://huggingface.co/datasets/ASIG-X/JEPLO>. The suite uses
  `bags_pc2_merged` and `gt`. Ground truth covers the 10 m x 4 m capture area.
  Long_Stairs and Outdoor* only have maps built from the same sequence (`maps_loc`).
- Koide et al., hard localization: the outdoor MID-360 sequences with
  `map_outdoor_hard.ply` / `map_outdoor_kidnap.ply`. The kidnap sequences cover the
  sensor and carry it elsewhere.

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

RESULTS_PLACEHOLDER
