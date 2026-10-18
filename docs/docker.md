# Docker

The image builds this package and [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2)
on the official `ros:<distro>-ros-base` image. Use it to try the package without a
ROS install, to reproduce a bug report, or to check a change before pushing.

## Build

From the repository root:

```bash
docker build -f docker/Dockerfile -t lidar_localization_ros2 .
# Humble instead of Jazzy:
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble -t lidar_localization_ros2:humble .
```

The build takes about 15 minutes on 4 cores. Dependencies are installed from
`package.xml` before the sources are copied, so after a source edit only the colcon
step runs again.

## Check that it works

```bash
docker run --rm lidar_localization_ros2 \
  /ws/src/lidar_localization_ros2/docker/smoke_status.sh
```

It writes a small synthetic map, publishes a synthetic scan on `/cloud`, starts
`quickstart.py` with an explicit pose, and switches the scan so each status line
appears. It ends with:

```text
PASS  discovery: ...
PASS  tracking: Tracking.
PASS  weak: Tracking is weak: only 40 scan points left after filtering. ...
PASS  recovered: Tracking again after 3 s.
PASS  lost: Tracking lost: no scan has matched the map for 7 s. ...
PASS  no scans: No scan processed for 5 s. ...
All status checks passed (log: /tmp/lidarloc_smoke.XXXXXX/quickstart.log)
```

and exits non-zero with the log tail at the first line that does not appear. Add
`--rviz` to start RViz under Xvfb as well. The synthetic map and scans only exercise
the bringup and status path; they are not a benchmark (see [benchmark](benchmark.md)).

The test suite runs in the same image:

```bash
docker run --rm -w /ws/src/lidar_localization_ros2 lidar_localization_ros2 \
  python3 -m pytest -q test/
```

## Run on your data

Share the host network so the container sees your LiDAR driver or `ros2 bag play`,
and mount the map:

```bash
docker run --rm -it --network host --ipc host \
  -v /path/to/maps:/maps:ro \
  lidar_localization_ros2 \
  ros2 run lidar_localization_ros2 quickstart.py --map /maps/map.pcd --no-rviz
```

Use the same `ROS_DOMAIN_ID` (and RMW implementation) inside and outside the
container. For RViz from the container on a Linux desktop, add
`-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` and run `xhost +local:` on the host
first, then drop `--no-rviz`; or run RViz on the host with the same domain.

To try local source edits without rebuilding the image, mount the repository over
the copy in the image and rebuild in the container:

```bash
docker run --rm -it --network host \
  -v "$PWD":/ws/src/lidar_localization_ros2 \
  lidar_localization_ros2 \
  bash -c 'cd /ws && colcon build --symlink-install && bash'
```

## CI

`.github/workflows/docker.yml` builds this image and runs the smoke test, with and
without RViz, on pushes to `main` and on pull requests that touch the runtime,
launch, parameter, RViz or Docker files. Layers are cached in GitHub Actions, so a
source-only change rebuilds just the colcon step. It can also be started by hand
from the Actions tab (`workflow_dispatch`).
