FROM koide3/glim_ros2:jazzy@sha256:b487fb5c4f3fc2cd68407ce20c1952f31492124554d05b7e29f41c82a713dd38

LABEL org.opencontainers.image.description="GLIM v1.2.2 CPU benchmark with runtime evidence and sparse-frame guard" \
      org.opencontainers.image.revision.glim="faa264a1bce1bda406f73457e35511f56cdc2eaa" \
      org.opencontainers.image.revision.glim_ros2="fc72f4603ed2d3dffcdf94cda5617963afe57e82"

COPY patches/glim-runtime-instrumentation.patch /tmp/
COPY patches/glim-ros-runtime-and-sparse-guard.patch /tmp/

RUN cd /root/ros2_ws/src/glim \
    && patch -p1 < /tmp/glim-runtime-instrumentation.patch \
    && cd /root/ros2_ws/src/glim_ros2 \
    && patch -p1 < /tmp/glim-ros-runtime-and-sparse-guard.patch \
    && . /opt/ros/jazzy/setup.sh \
    && cd /root/ros2_ws \
    && colcon build --packages-select glim glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["bash"]
