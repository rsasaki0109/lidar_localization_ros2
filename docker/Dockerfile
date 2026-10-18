# ROS 2 image with lidar_localization_ros2 and ndt_omp_ros2 built.
#
#   docker build -f docker/Dockerfile -t lidar_localization_ros2 .
#   docker run --rm lidar_localization_ros2 /ws/src/lidar_localization_ros2/docker/smoke_status.sh
#
# ROS_DISTRO=humble also works (ros:humble). See docs/docker.md.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
# ndt_omp_ros2 has no jazzy branch; CI builds humble on both distros.
ARG NDT_OMP_BRANCH=humble
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# rviz2 and Xvfb let the smoke test start quickstart with RViz headless.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      libomp-dev \
      python3-colcon-common-extensions \
      python3-pytest \
      python3-rosdep \
      ros-${ROS_DISTRO}-rviz2 \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
RUN git clone --depth 1 -b ${NDT_OMP_BRANCH} \
      https://github.com/rsasaki0109/ndt_omp_ros2.git

# Dependencies first, from package.xml alone, so source edits keep this layer.
COPY package.xml /ws/src/lidar_localization_ros2/package.xml
RUN apt-get update \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install -r -y --rosdistro ${ROS_DISTRO} --from-paths /ws/src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/lidar_localization_ros2
WORKDIR /ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
