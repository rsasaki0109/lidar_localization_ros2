# Unitree Go2 with a Livox MID-360: localized in five minutes

Three commands take a Go2 (or any robot with a MID-360) from a recorded bag of
the area to a localized pose: build a map once, start the odometry, start
localization. Nothing needs a TF tree, a parameter file or an initial pose.

Requirements:
- ROS 2 Jazzy or Humble.
- [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2) and this
  package built ([local build](local_build.md)).
- `livox_ros_driver2` publishing `PointCloud2` on `/livox/lidar` and IMU on
  `/livox/imu`, which are the driver's defaults.

The MID-360 may be mounted upside down, as on the JEPLO Go2. The odometry
levels its frame with gravity at startup, and the IMU's acceleration in g is
detected and converted.

## 1. Build a map (once)

Record a bag of the area and build the map with lidar_slam_ros2:

```bash
lidarslam-map start my_area.bag     # writes output/my_area/map.pcd
```

## 2. Start the odometry

```bash
ros2 launch lidarslam rko_lio_odometry.launch.py
```

It publishes `odom -> livox_frame`. When replaying a bag, add `use_sim_time:=true`
and play the bag with `--clock`.

## 3. Start localization

```bash
ros2 run lidar_localization_ros2 quickstart.py --map output/my_area/map.pcd
```

quickstart finds the topics, frames and clock by itself and prints them on its
`Discovery:` line. It then searches the map for the robot, checks the answer
against the next scans, and runs a bringup check once localized. On a Go2 replay
the terminal reads:

```text
Waiting for the first LiDAR scan.
Searching the map for the robot (try 1 of 6).
Searching the map for the robot (try 2 of 6): found a candidate; confirming it from another view.
Checking the found pose against the next scans.
Localized from a map search.
[OK] pointcloud received on /livox/lidar
[OK] IMU received on /livox/imu
[OK] TF available: map <- odom
[OK] localization pose received on /pcl_pose
[OK] alignment status observed: ok
```

RViz opens with the map, the scan and the pose. `--no-rviz` skips it.

## If it does not localize by itself

The last line then says why, and what to do:

```text
Could not localize automatically: no unambiguous match was found. Set 2D Pose Estimate in RViz, or restart with --initial-pose.
```

- Click **2D Pose Estimate** in RViz near the robot, pointing the way it faces.
- If the robot starts where the mapping run started, restart with
  `--initial-pose 0 0 0 0 0 0 1` (x y z qx qy qz qw).
- On a map larger than a room, use the settings under [Large maps](#large-maps).

## Large maps

The global search matches the scan against a 2D grid of the map. A MID-360 upside
down on a Go2 sits about 0.45 m above the ground and sees at most 7 degrees above
its horizontal plane. Only about 4% of its points reach the default height band
(0.5 m and more above the sensor). With about 100 points, a building-sized or
outdoor map fits the scan perfectly at many places, and the search returns the
wrong ones. These options search from the sensor plane up on a thinner grid,
keep more candidates for the 3D check, and publish its refined pose:

```bash
ros2 run lidar_localization_ros2 quickstart.py --map output/my_area/map.pcd \
  --occupancy-inflate-radius-m 0.2 --global-z-min 0.0 --global-max-scan-points 1024 \
  --global-max-candidates 16 --refine-global-candidates
```

quickstart generates (and caches) the thinner grid itself. If you pass your own
`--occupancy-map`, generate it with `--inflate-radius-m 0.2
--max-obstacle-height-m 2.0`. Each search takes about 1-2 s instead of 0.2-1.5 s.
The settings fit a sensor mounted near obstacle height. A handheld or
car-mounted sensor keeps the defaults: on handheld Koide replays, a 256-point version of
these settings found the sensor later in all 12 runs and at a wrong place in 2, where
the defaults were right in all 12.

## How well it works

Measured on the JEPLO Go2 bags against motion-capture ground truth, on a
lidar_slam_ros2 map of a separate run ([benchmark](benchmark.md)):
- **Indoor capture room** (EIL_Box, Stairs, Mask1, Mask2, three runs each): all
  12 runs localized by themselves and none at a wrong place. The first pose came
  after 10-17 s, and tracking stayed within 3 cm (median).
- **Large maps** (Long_Stairs, Outdoor): the defaults search a sparse height band
  again from the sensor plane up (`--global-sparse-band-min-points 512`). That
  localized Long_Stairs (3 runs) and Outdoor2 (2 runs) correctly; Outdoor1 stayed
  ambiguous and asked for a manual pose. With the [Large maps](#large-maps) settings it
  localized all six runs (two per map) by itself and none at a wrong place, after
  11-15 s, and tracking stayed within 2 cm (median). In six later replays with the
  same search settings, five localized; in one Outdoor2 run all six tries stayed
  ambiguous and the startup asked for a manual pose. In the capture room the same
  settings localized all 12 runs correctly.
  These were measured on maps rebuilt from the raw bags, as the dataset no longer
  ships its own ([comparison](../experiments/go2_wide_map_global_init/README.md)).
- **Map from a different recording** (2026-11-18, the [Large maps](#large-maps)
  settings, two runs per pair, [cross-session table](benchmark.md#cross-session-go2-jeplo)).
  The large-map results above used maps from the same recording as the scans.
  - Long_Stairs map, Outdoor2 scans (1.5 h later, same route, room not rearranged), and
    the reverse: all 4 runs localized by themselves after 12-13 s, 0.02 m off, and
    tracked down the staircase within 0.02 m (median) of the ground truth in the room.
  - EIL_Mix map (capture room), EIL_Box, Stairs, Mask1 and Mask2 scans (props moved,
    part of the sensor covered in the Mask runs): all 8 runs started right after
    7.6-10.0 s. Tracking stayed at 0.02-0.03 m (median) but EIL_Stairs ran up to 1.3 m off
    for 18 s, which it did not on its own map.
  - Failures: on the EIL_Box map (a large box platform in the middle of the
    near-symmetric room) 4 of 6 starts on other runs were 1.05-5.50 m off, and even its
    own run once asked for a manual pose. On the Long_Stairs map, which lacks the props,
    the EIL runs mostly started right (5 of 6) but tracking slid 1-2.5 m along the room
    for up to 40 s, and EIL_Mask1 once asked for a manual pose and once started 1.7 m
    off. If the room has changed, check the first pose, or rebuild the map.

More: [quickstart and automatic initialization](quickstart.md),
[troubleshooting](troubleshooting.md), [frames](frame_contract.md).
