FROM lidarloc/glim-ros2:jazzy-v1.2.2-coreset

ARG KISS_MATCHER_REVISION=669fe866b1c612daa3702292571c25dc1bc61c86

RUN git clone https://github.com/MIT-SPARK/KISS-Matcher.git /tmp/KISS-Matcher \
    && cd /tmp/KISS-Matcher \
    && git checkout "${KISS_MATCHER_REVISION}" \
    && make cppinstall PARALLEL=2 \
      CMAKE_EXTRA_ARGS=-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && rm -rf /tmp/KISS-Matcher

COPY . /tmp/glim_prior_map_localizer
RUN . /opt/ros/jazzy/setup.sh \
    && . /root/ros2_ws/install/setup.sh \
    && cmake -S /tmp/glim_prior_map_localizer \
      -B /tmp/glim_prior_map_localizer/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/glim_prior_map_localizer/build -j2 \
    && cmake --install /tmp/glim_prior_map_localizer/build \
    && rm -rf /tmp/glim_prior_map_localizer

ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
