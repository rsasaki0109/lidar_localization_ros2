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
  - One alignment, with its fitness, inlier ratio and (NDT_OMP) localizability and score
    Hessian.
  - The map-distance prefilter (`setSourceNearTarget`,
    [map_distance_filter.hpp](../include/lidar_localization/map_distance_filter.hpp)): the
    scan points farther than a distance from the target at the guess, such as people and
    vehicles the map lacks, are left out of the alignment and its fitness. The policy
    that decides when the guess is trustworthy enough (`decideMapDistanceFilter`) is in
    the same header; the node applies it with `map_distance_filter_m`.
  - Its messages go to a `LogSink` you provide. The node forwards them to its ROS logger.
- A C API over it, for other languages and for FFIs
  ([core_c_api.h](../include/lidar_localization/core_c_api.h)):

  ```c
  lidarloc_config config = lidarloc_default_config();   /* NDT_OMP */
  lidarloc_core * core = lidarloc_create(&config, error, sizeof error);
  lidarloc_set_map(core, map_xyz, map_points);
  lidarloc_result result;
  lidarloc_align(core, scan_xyz, scan_points, guess /* 4x4 row-major */, &result);
  /* result.pose, has_converged, fitness_score, inlier_ratio, eigenvalues,
     score_hessian, ... */
  lidarloc_destroy(core);
  ```

  `lidarloc_align` does what the node does for one scan:
  1. Voxel-filters the scan, as `enable_scan_voxel_filter` does in the node.
  2. Refines the guess with the NDT initializer while that is still running.
  3. Sets the target around the guess.
  4. With `map_distance_filter_m` above 0, keeps only the scan points within that
     distance of the target at the guess (the whole scan when fewer than
     `map_distance_filter_min_kept_ratio` of them are). The caller decides when its guess
     is good enough for that; leave it 0 at a start or after a reset.
  5. Aligns the scan.
  6. Reports the result, with `map_distance_filter_kept_points` and
     `map_distance_filter_applied`.
- `lidar_localization::ScanTracker`
  ([scan_tracker.hpp](../include/lidar_localization/scan_tracker.hpp)): what the node
  remembers from scan to scan, and what it does with it every scan:
  - The prediction state: the last accepted pose, the predicted pose and the motion between
    them. It is reset at an initial pose, moved by an accepted scan, and advanced without a
    measurement.
  - The measurement gate's own state: settled accepts since a reset, the windowed seed
    correction guard, the release agreement, and the agreement release (the corrections
    the guards rejected and those accepted within its drift horizon).
  - The gate's decision on an attempt, and the attempt's seed metrics.

  `scoreAlignment` turns the backend's result into the attempt the gate reads: the fitness,
  or the inlier ratio with `min_inlier_ratio`, the localizability, and the correction from
  the seed. The node keeps one `ScanTracker` and reads its fields under its old member names.
- The C API's `lidarloc_localize` runs the node's per-scan flow on these pieces, without
  the IMU:
  1. The seed: the caller's (for example, predicted from odometry), else the previous
     delta, else the tracked pose.
  2. The NDT initializer.
  3. The alignment, of the points near the map with `map_distance_filter_m` once the
     tracking is trusted (the node's decision, with the `map_distance_filter_*` fields of
     `lidarloc_tracking`).
  4. A retry from the last accepted pose (`enable_recovery_retry_from_last_pose`).
  5. The measurement gate (score threshold, `min_inlier_ratio`, the seed correction guard,
     its window and its agreement release, the
     `seed_correction_guard_agreement_release_*` fields of `lidarloc_tracking`, 0 = off).
  6. The update of the tracked pose and the prediction, as the node's raw registration pose
     backend does.
  7. With `lidarloc_set_reacquisition`, local re-acquisition after rejected scans given a
     seed (see below).

  `lidarloc_set_initial_pose` starts it, and `lidarloc_set_tracking` sets the gate, retry
  and prediction parameters:

  ```c
  lidarloc_set_initial_pose(core, start /* 4x4 */, 0.0);
  lidarloc_update update;
  lidarloc_localize(core, scan_xyz, scan_points, stamp_sec, NULL /* or an odometry seed */,
                    &update);
  /* update.accepted, update.status ("ok", "seed_correction_guard_rejected", ...),
     update.pose (the tracked pose), update.alignment (the attempt the gate judged) */
  ```
- Local re-acquisition (`proposeLocalReacquisition`,
  [local_reacquisition.hpp](../include/lidar_localization/local_reacquisition.hpp)): when
  odometry carried the pose through an outage and drifted beyond the registration's reach,
  a 2D branch-and-bound search on an occupancy grid around the bridged pose, near its
  heading; each candidate is refined by the registration, and the best is proposed only
  when it passes the score threshold and no other place fits nearly as well. The node runs
  it with `enable_local_reacquisition` on the occupancy map file it is given. In the C API,
  `lidarloc_set_reacquisition` enables it with a grid generated from the map
  (`occupancy_map_generation.hpp`), and `lidarloc_localize` runs it after
  `min_rejections` rejected scans that came with a seed. The seed must then be the
  odometry bridge (the last tracked pose moved by odometry). The next scan with a seed is
  aligned from the proposal, and the gate on it decides:

  ```c
  lidarloc_reacquisition reacquisition = lidarloc_default_reacquisition();
  reacquisition.enable = 1;
  lidarloc_set_reacquisition(core, &reacquisition);   /* before or after the map */
  lidarloc_localize(core, scan_xyz, scan_points, stamp_sec, odometry_seed, &update);
  /* update.reacquisition_attempted, reacquisition_reason ("proposed", "ambiguous", ...),
     reacquisition_pose; on the next scan seeded_from_reacquisition and accepted */
  ```
- Solution remapping (`remapDegenerateCorrection`,
  [degeneracy_remapping_policy.hpp](../include/lidar_localization/degeneracy_remapping_policy.hpp)):
  where NDT_OMP's Hessian is weakest along a horizontal translation it barely constrains
  (a corridor, a tunnel), the alignment's correction along it is dropped. `scoreAlignment`
  applies it, so the node (`enable_degeneracy_remapping`) and `lidarloc_localize`
  (`lidarloc_tracking.enable_degeneracy_remapping`) share it. Off by default. With
  `degeneracy_remapping_min_horizontal_constraint` it also drops the correction along the
  horizontal directions the map's surfaces under the scan barely face
  (`RegistrationEngine::horizontalConstraint`): bare ground, which NDT's Hessian misses.
- Global localization (`GlobalLocalizer`,
  [global_localization.hpp](../include/lidar_localization/global_localization.hpp)): the G2
  engine "bbs3d" search over the whole map (3D-BBS's voxel maps, `bbs3d_topk_search.hpp`),
  each of its `top_k` candidates refined by the registration, and the best kept when it
  passes the score threshold and no other place fits nearly as well (the decision of local
  re-acquisition). The C API builds the voxel maps from the map on the first call:

  ```c
  lidarloc_global_config global = lidarloc_default_global_config();
  lidarloc_global_result found;
  lidarloc_global_localize(core, scan_xyz, scan_points, &global, &found);
  if (found.found) {
    lidarloc_set_initial_pose(core, found.pose, stamp_sec);
  }
  /* found.reason ("proposed", "ambiguous", ...), fitness, second_fitness, search_ms */
  ```
- The NDT score Hessian (`score_hessian`, NDT_OMP with `localizability`): the second
  derivatives of the NDT score at the pose, in (x, y, z, roll, pitch, yaw). The score is
  at a maximum there, so the Hessian is negative definite, and its negated inverse is the
  shape of the pose covariance, up to a scale that depends on the NDT parameters and the
  sensor noise.
- The policy headers (`include/lidar_localization/*_policy.hpp`) are ROS-free as well, so the
  rest of the node's decisions can be called directly. See the node's `component_*.cpp` for
  how they are combined.

The rest of the node is not in the core yet:
- seed selection with the IMU, twist and odometry TF;
- the IMU and GTSAM smoothers;
- deskew;
- the G2 startup checks around the search (confirmations, the trusted start) and the
  recovery supervisor;
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
