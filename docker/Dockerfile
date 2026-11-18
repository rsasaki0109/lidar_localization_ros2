# ROS 2 image with lidar_localization_ros2 and ndt_omp_ros2 built.
#
#   docker build -f docker/Dockerfile -t lidar_localization_ros2 .
#   docker build --target runtime -f docker/Dockerfile -t lidar_localization_ros2:runtime .
#
# ROS_DISTRO=humble also works (ros:humble). See docs/docker.md.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS base

FROM base AS build

ARG ROS_DISTRO
# ndt_omp_ros2 has no jazzy branch; CI builds humble on both distros.
ARG NDT_OMP_BRANCH=humble
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      libomp-dev \
      python3-colcon-common-extensions \
      python3-pytest \
      python3-rosdep \
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

# Copy the install tree into a clean ROS base for headless deployment.
FROM base AS runtime
ARG ROS_DISTRO
RUN case "${ROS_DISTRO}" in \
      jazzy) pcl_version=1.14 ;; \
      humble) pcl_version=1.12 ;; \
      *) echo "Unsupported runtime ROS distribution: ${ROS_DISTRO}" >&2; exit 1 ;; \
    esac \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      libgomp1 \
      "libpcl-common${pcl_version}" \
      "libpcl-filters${pcl_version}" \
      "libpcl-io${pcl_version}" \
      "libpcl-kdtree${pcl_version}" \
      "libpcl-registration${pcl_version}" \
      "libpcl-search${pcl_version}" \
      python3-numpy \
      python3-pil \
      python3-yaml \
      "ros-${ROS_DISTRO}-rosbag2-py" \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY --from=build /ws/install /ws/install
COPY docker/smoke_status.sh docker/synthetic_lidar.py /ws/src/lidar_localization_ros2/docker/
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Keep the development image as the default target, including RViz and Xvfb.
FROM build AS development
RUN apt-get update && apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-rviz2" \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
