ARG GLIM_BASE_IMAGE=lidarloc/glim-ros2:jazzy-v1.2.2-coreset
FROM ${GLIM_BASE_IMAGE}

ARG KISS_MATCHER_REVISION=669fe866b1c612daa3702292571c25dc1bc61c86
RUN git clone https://github.com/MIT-SPARK/KISS-Matcher.git /tmp/KISS-Matcher \
    && cd /tmp/KISS-Matcher \
    && git checkout "${KISS_MATCHER_REVISION}" \
    && make cppinstall PARALLEL=2 \
      CMAKE_EXTRA_ARGS=-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && rm -rf /tmp/KISS-Matcher

ARG GLIL_REVISION=e704bf85b6582b05e0611872463b04d5dfcb413b
ARG GLIM_ROS2_REVISION=cd4b2c9eb37a5c12c93d7339ce10167ebaa55288

LABEL org.opencontainers.image.revision.glil="${GLIL_REVISION}" \
      org.opencontainers.image.revision.glim_ros2="${GLIM_ROS2_REVISION}"

# Keep the benchmark image's runtime instrumentation, then overlay the pinned
# exact-coreset/tightly-coupled GLIM fork and external map-to-odom ROS consumer.
RUN git clone https://github.com/rsasaki0109/glil_unofficial.git /tmp/glil \
    && git -C /tmp/glil checkout "${GLIL_REVISION}" \
    && git clone https://github.com/rsasaki0109/glim_ros2.git /tmp/glim_ros2_fork \
    && git -C /tmp/glim_ros2_fork checkout "${GLIM_ROS2_REVISION}" \
    && cp /tmp/glil/include/glim/mapping/callbacks.hpp \
      /root/ros2_ws/src/glim/include/glim/mapping/callbacks.hpp \
    && cp /tmp/glil/src/glim/mapping/callbacks.cpp \
      /root/ros2_ws/src/glim/src/glim/mapping/callbacks.cpp \
    && cp /tmp/glil/include/glim/mapping/global_mapping.hpp \
      /root/ros2_ws/src/glim/include/glim/mapping/global_mapping.hpp \
    && cp /tmp/glil/src/glim/mapping/global_mapping.cpp \
      /root/ros2_ws/src/glim/src/glim/mapping/global_mapping.cpp \
    && cp /tmp/glil/include/glim/odometry/exact_quadratic_coreset.hpp \
      /root/ros2_ws/src/glim/include/glim/odometry/exact_quadratic_coreset.hpp \
    && cp /tmp/glil/include/glim/odometry/integrated_gicp_factor_coreset.hpp \
      /root/ros2_ws/src/glim/include/glim/odometry/integrated_gicp_factor_coreset.hpp \
    && cp /tmp/glil/include/glim/odometry/tightly_coupled_window.hpp \
      /root/ros2_ws/src/glim/include/glim/odometry/tightly_coupled_window.hpp \
    && cp /tmp/glil/include/glim/odometry/odometry_estimation_cpu.hpp \
      /root/ros2_ws/src/glim/include/glim/odometry/odometry_estimation_cpu.hpp \
    && cp /tmp/glil/include/glim/odometry/odometry_estimation_imu.hpp \
      /root/ros2_ws/src/glim/include/glim/odometry/odometry_estimation_imu.hpp \
    && cp /tmp/glil/include/glim/odometry/imu_prediction_guard.hpp \
      /root/ros2_ws/src/glim/include/glim/odometry/imu_prediction_guard.hpp \
    && cp /tmp/glil/src/glim/odometry/odometry_estimation_cpu.cpp \
      /root/ros2_ws/src/glim/src/glim/odometry/odometry_estimation_cpu.cpp \
    && cp /tmp/glil/src/glim/odometry/odometry_estimation_imu.cpp \
      /root/ros2_ws/src/glim/src/glim/odometry/odometry_estimation_imu.cpp \
    && cp /tmp/glim_ros2_fork/CMakeLists.txt \
      /root/ros2_ws/src/glim_ros2/CMakeLists.txt \
    && cp /tmp/glim_ros2_fork/include/glim_ros/rviz_viewer.hpp \
      /root/ros2_ws/src/glim_ros2/include/glim_ros/rviz_viewer.hpp \
    && cp /tmp/glim_ros2_fork/include/glim_ros/external_map_odom_smoother.hpp \
      /root/ros2_ws/src/glim_ros2/include/glim_ros/external_map_odom_smoother.hpp \
    && cp /tmp/glim_ros2_fork/src/glim_ros/rviz_viewer.cpp \
      /root/ros2_ws/src/glim_ros2/src/glim_ros/rviz_viewer.cpp \
    && mkdir -p /root/ros2_ws/src/glim_ros2/test \
    && cp /tmp/glim_ros2_fork/test/test_external_map_odom_smoother.cpp \
      /root/ros2_ws/src/glim_ros2/test/test_external_map_odom_smoother.cpp \
    && . /opt/ros/jazzy/setup.sh \
    && cd /root/ros2_ws \
    && colcon build --packages-select glim glim_ros \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && colcon test --packages-select glim_ros --event-handlers console_direct+ \
    && colcon test-result --verbose \
    && rm -rf /tmp/glil /tmp/glim_ros2_fork

COPY . /tmp/glim_prior_map_localizer
RUN . /opt/ros/jazzy/setup.sh \
    && . /root/ros2_ws/install/setup.sh \
    && cmake -S /tmp/glim_prior_map_localizer \
      -B /tmp/glim_prior_map_localizer/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/glim_prior_map_localizer/build -j2 \
    && ctest --test-dir /tmp/glim_prior_map_localizer/build --output-on-failure \
    && cmake --install /tmp/glim_prior_map_localizer/build \
    && rm -rf /tmp/glim_prior_map_localizer

ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
