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

## How well it works

Measured on the JEPLO Go2 bags against motion-capture ground truth, on a
lidar_slam_ros2 map of a separate run ([benchmark](benchmark.md)):
- **Indoor capture room** (EIL_Box, Stairs, Mask1, Mask2, three runs each): all
  12 runs localized by themselves and none at a wrong place. The first pose came
  after 10-17 s, and tracking stayed within 3 cm (median).
- **Large maps** (Long_Stairs, Outdoor): the global search does not find the Go2.
  Its sensor is 0.4 m above the ground and leaves too few points for the 2D search.
  Start with `--initial-pose` or 2D Pose Estimate.

More: [quickstart and automatic initialization](quickstart.md),
[troubleshooting](troubleshooting.md), [frames](frame_contract.md).
