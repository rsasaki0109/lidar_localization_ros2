# Using the registration without ROS 2 (`lidar_localization_core`)

The node's scan-to-map registration is a library of its own,
`lidar_localization_core`, that does not depend on ROS 2. The node links it,
and so can a simulator, an offline evaluation or any program that has a map
and scans but no ROS graph. It is the same code the node runs, not a copy.

## What is in it

- `lidar_localization::RegistrationEngine`
  ([registration_engine.hpp](../include/lidar_localization/registration_engine.hpp)):
  - The registration backend: NDT_OMP, GICP_OMP, PCL NDT or GICP, or small_gicp when it
    was found at build time.
  - The map, and the registration target built from it. The target is either the whole map
    or a crop around the pose (`enable_local_map_crop`). The crop can be rebuilt in the
    background (`local_map_async_rebuild`), and failed crops have a guard.
  - The NDT initializer for the GICP backends.
  - One alignment, with its fitness, inlier ratio and (NDT_OMP) localizability.
  - Its messages go to a `LogSink` you provide. The node forwards them to its ROS logger.
- A C API over it, for other languages and for FFIs
  ([core_c_api.h](../include/lidar_localization/core_c_api.h)):

  ```c
  lidarloc_config config = lidarloc_default_config();   /* NDT_OMP */
  lidarloc_core * core = lidarloc_create(&config, error, sizeof error);
  lidarloc_set_map(core, map_xyz, map_points);
  lidarloc_result result;
  lidarloc_align(core, scan_xyz, scan_points, guess /* 4x4 row-major */, &result);
  /* result.pose, has_converged, fitness_score, inlier_ratio, eigenvalues, ... */
  lidarloc_destroy(core);
  ```

  `lidarloc_align` does what the node does for one scan:
  1. Voxel-filters the scan, as `enable_scan_voxel_filter` does in the node.
  2. Refines the guess with the NDT initializer while that is still running.
  3. Sets the target around the guess.
  4. Aligns the scan.
  5. Reports the result.
- The policy headers (`include/lidar_localization/*_policy.hpp`) are ROS-free as well. Seed
  selection, the measurement gate, retries and recovery can be called directly; see the
  node's `component_*.cpp` for how they are combined.

The rest of the node is not in the core yet:
- seed selection;
- prediction;
- the IMU smoothers;
- deskew;
- the measurement gate and recovery orchestration;
- publishing.

These still live in `PCLLocalization`.

## Building it without ROS 2

It needs PCL, Eigen, OpenMP and the pclomp headers from an `ndt_omp_ros2` checkout. ROS is
not needed:

```bash
git clone -b humble https://github.com/rsasaki0109/ndt_omp_ros2.git
cmake -S lidar_localization_ros2 -B build-core \
  -DLIDAR_LOCALIZATION_CORE_ONLY=ON \
  -DNDT_OMP_ROS2_SOURCE_DIR=$PWD/ndt_omp_ros2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-core -j
ctest --test-dir build-core          # test/test_core_c_api.c
cmake --install build-core --prefix /opt/lidar_localization_core
```

This builds a static, position-independent `liblidar_localization_core.a`. Link it with PCL
(common, filters, kdtree, search, registration), OpenMP and the C++ standard library.

In the ROS 2 build (`colcon build`) the same library is built and linked into
`lidar_localization_component`.
