# Changelog

## Unreleased

### Added

- `min_inlier_ratio` (off at 0) gates on the share of scan points within
  `inlier_ratio_distance_m` (0.5 m) of a map point, instead of the fitness score.
  - **Why:** where the map misses structure the sensor sees, a few points metres from
    any map point dominate the fitness score's mean of squared distances. On Koide
    `outdoor_hard_01a`, alignments within 0.1-0.2 m of the ground truth score 10-24
    against a threshold of 6, and wrong poses score lower than right ones.
  - **The score:** `score_threshold * (1 - inlier_ratio) / (1 - min_inlier_ratio)`, so
    the measurement gate, the pose covariance, the map-to-odom anchor gate and the
    recovery keep their thresholds. A scan of fewer than `inlier_ratio_min_points`
    (200) points gets a ratio of 0.
  - **`/alignment_status`** adds `registration_fitness_score`, the backend's own
    fitness, and `inlier_ratio`, also with the gate off. `fitness_score` stays what
    the gate reads.
  - **The distance must suit the map.** On Istanbul at 0.5 m, good alignments have
    inlier ratios of 0.58-0.93, and 0.8 lost a window: the GTSAM smoother reads the
    same score against `gtsam_fitness_nominal` and all but ignored the scans. At 1.0 m
    the three Istanbul windows match or beat the gate off. Boreas does not change.
  - **The online Koide profile** (`tools/readme_media/koide_outdoor_hard_02b_online.yaml`)
    sets 0.8. With RKO-LIO odometry, `outdoor_hard_01a` goes from 3.64 m to 0.43 m
    RMSE, and `outdoor_hard_02b`, the release check's sequence, stays within 0.01 m
    ([experiments/inlier_ratio_gate](experiments/inlier_ratio_gate/README.md)).
  - **Without odometry** it is not enough: 01a still loses the track at about 107 s.

### Removed

- The legacy `use_imu` deskew, deprecated in 1.6.0. It integrated the accelerometer
  without removing gravity: on Boreas it scored 1.7 m RMSE with 6% of the scans
  accepted, against 0.127 m with the IMU preintegration.
  - **`use_imu` is still declared,** so that `use_imu: true` is not silently ignored:
    the node now fails to configure with a message. `use_imu: false`, or leaving it
    out, works as before.
  - **Migration:** use `use_imu_preintegration`, with `use_continuous_time_deskew` for
    the deskew ([docs/imu_estimation.md](docs/imu_estimation.md)).
  - **The config generator** refuses `--imu-mode legacy`, and no longer writes
    `use_imu`. `--imu-mode both` and `--enable-imu` still mean the preintegration.
  - **Gone with it:** `include/lidar_localization/lidar_undistortion.hpp`, its two
    tests, the IMU callback's legacy path and its lock, and `use_imu: false` in the
    presets under `param/`.
  - **Test:** `test_imu_callback_group` checks that `use_imu: true` fails to configure.

## 1.7.0 - 2026-11-03

### Added

- `/alignment_status` reports the IMU preintegration smoother's state after each
  scan's update. The new keys are `imu_gyro_bias_{x,y,z}_rad_s`,
  `imu_accel_bias_{x,y,z}_m_s2` and `imu_velocity_{x,y,z}_m_s`, and they read `nan`
  while the smoother is off or uninitialized.
  - **Koide `outdoor_hard_01a`:** they show that the IMU runs are lost by resets, not
    by the biases (0.7-1.0°/s gyro, 0.07 m/s²).
  - **The resets:** from 107 s on, NDT accepts poses on degraded scans that are up to
    16° off. The divergence guard takes them as right and resets the smoother to them,
    zeroing the biases and giving it 3.8 m/s where the ground truth moves at 1.1 m/s.
  - **Boreas** never trips that guard
    ([experiments/koide_imu_ab](experiments/koide_imu_ab/README.md#the-smoother-is-reset-to-wrong-measurements)).


### Changed

- NDT_OMP aligns about a third faster in the node, with the same result
  ([ndt_omp_ros2 #15](https://github.com/rsasaki0109/ndt_omp_ros2/pull/15), released in
  ndt_omp_ros2 0.2.0). CI and the Docker image build ndt_omp_ros2's `humble` branch, so
  they get it; a workspace with its own checkout needs to pull it, and a binary install
  needs 0.2.0. With an older ndt_omp_ros2 the node works as before, only slower.
  - **What was slow:** in every derivative pass, each thread added its sums into its own
    entry of shared vectors, for every point. Those entries share cache lines, so the
    threads kept invalidating each other's caches. The voxel lookups also went through a
    `std::map`.
  - **Boreas 120 s preset**, two pairs of runs back to back: `alignment_time_sec`
    averages 12.2-12.4 ms instead of 18.1-19.1 ms, with a p90 of 21-23 ms instead of
    32-34 ms. The trajectory is byte-identical.
  - **Under load**, with two cores kept busy and the IMU preintegration on, the pose lags
    `/clock` by 0.12-0.16 s at p95 instead of 0.58-0.77 s, and by 0.34-0.41 s at most
    instead of 0.90-1.16 s. That was two pairs of runs back to back; none of the four
    dropped a scan
    ([docs/benchmark.md](docs/benchmark.md#boreas-vehicle-preset-replay)).
  - **On its own**, on 154 dense Boreas scans, `align()` takes 14 ms instead of 27.5 ms
    on 8 threads, and 12 ms instead of 33.6 ms on 4
    ([experiments/ndt_alignment_cost](experiments/ndt_alignment_cost/README.md#inside-align-false-sharing)).
- GICP_OMP (`registration_method: GICP_OMP`) aligns faster, with the same result
  ([ndt_omp_ros2 #16](https://github.com/rsasaki0109/ndt_omp_ros2/pull/16), in 0.2.0).
  Its cost and gradient had the same false sharing, and its correspondence search
  took slots from a shared atomic counter. On 154 dense Boreas scans against a 100 m
  crop thinned to 0.5 m, an alignment takes 10.9-11.6 ms instead of 17.9-19.5 ms on
  4 threads, with bit-identical transforms. No preset uses it.
- The fitness score of every registration attempt is computed on the registration
  threads (`include/lidar_localization/registration_fitness.hpp`).
  - **Why:** PCL's `getFitnessScore()` runs one nearest-neighbour search per scan
    point, one after another, after `align()` and outside `alignment_time_sec`. On
    Boreas that took 6.5-7 ms a scan, a fifth of the alignment again.
  - **Now:** it takes 2 ms on 4-8 threads. The searches are the same, and they are
    summed in point order as in PCL, so the score is bit-identical. None of 154
    Boreas scans differ, and the Boreas preset's 120 s trajectory is byte-identical.
  - **Cheaper NDT settings:** DIRECT1 neighbours and a looser epsilon cut the
    alignment by half or more, but converge less. From a seed 0.5 m off they end up
    to 0.6 m off, where the preset ends up to 0.3 m off. They are not adopted
    ([experiments/ndt_alignment_cost](experiments/ndt_alignment_cost/README.md)).
  - **Test:** `test_registration_fitness`
- `param/boreas_ndt_velodyne.yaml` sets `use_continuous_time_deskew: false`.
  - **What it saved:** with `use_imu_preintegration` on, the default `relative_motion`
    deskew ran on about 45 000 points a scan. That was the 3-4 ms longer scan
    preparation measured with the IMU.
  - **What it bought:** nothing. With it off the preintegration scores the same
    0.127 m; the gain comes from the IMU seed.
  - **Without the IMU:** the deskew never ran, and the trajectory is byte-identical.
  - **Fixture timing:** the Boreas fixture plays each cloud at its mid-sweep stamp,
    about 52 ms before a driver would. The gyro `imu_pose_history` deskew therefore
    cannot run on it. Timings on the same container drift between sessions
    ([docs/benchmark.md](docs/benchmark.md#boreas-vehicle-preset-replay)).

### Fixed

- The IMU preintegration smoother converges.
  - **The bug:** the prior that holds the first pose of its window had its weight on
    the diagonal but no residual. The other factors moved that pose a little every
    Gauss-Newton iteration, so the step never fell below the stopping limit, and most
    updates on Boreas ran all ten iterations.
  - **The fix:** the anchor now pulls the first pose back to where the update found
    it. Updates stop after about three iterations, and take 5 ms on Boreas instead
    of 9 ms (12-14 ms in the dense stretches).
  - **Boreas:** the RMSE is unchanged at 0.127 m. The node's lag behind real time is
    0.25-0.59 s at p95 and 0.56-0.83 s at most in three runs, against 0.46-0.70 and
    0.75-0.92 s before: about as much as it varies from run to run.
  - **Test:** `test_smoother_converges_in_a_few_iterations_with_a_full_window`
    ([experiments/imu_smoother_solve](experiments/imu_smoother_solve/README.md#the-anchor-never-let-it-converge)).

### Validation

- The release check: two full Koide `outdoor_hard_02b` replays at 1x passed on `main`
  at `6b1e693`, built without a build type, with ndt_omp_ros2 at `24c5df3`.
  - **Accuracy:** 0.233 m RMSE in both (0.181 m on the map), against 0.232 m for
    v1.6.0.
  - **Coverage:** 100.0% diagnostic coverage in both.
  - **Alignment:** 17 ms at the median and 45-46 ms at p95, against 36 and 106-112 ms.
    This ran on a slower host CPU (2.10 GHz against 2.80 GHz) in the same 4-vCPU
    container type
    ([docs/koide-online-replay.md](docs/koide-online-replay.md#v170-release-verification-2026-11-03),
    `docs/validation/v1.7.0-koide-20261103`).
- Koide `outdoor_hard_01a` online with RKO-LIO odometry, as the 02b check runs
  (`experiments/koide_imu_ab/online.py`, two runs each).
  - **Odometry carries the sequence:** both arms hold 0.17 m for the first 105 s, where
    the localizer alone lost the track at 35-111 s.
  - **Recovery:** they get through the hard stretch, and return to 0.12-0.36 m after
    the path leaves the map at 185-305 s.
  - **The preintegration:** with odometry it lowers only the hard stretch at
    105-185 s, from 0.88-0.95 to 0.71 m. The on-map RMSE goes from 0.64-0.67 to
    0.57 m, with the same acceptance and coverage.
- `ndt_num_threads: 8` stays in the Boreas preset. With 4 threads on 4 vCPUs the
  alignment takes 1-2 ms longer. With two cores kept busy, 36-43 more of the 1158
  scans drop
  ([docs/benchmark.md](docs/benchmark.md#boreas-vehicle-preset-replay)).
- Two changes to the IMU smoother's divergence guard were tried on Koide
  `outdoor_hard_01a` and are not kept.
  - **Keeping the smoother's velocity and biases on a reset:** the track is lost at
    107.0 s and 110.8 s, as before.
  - **Rejecting a measurement 2 m or 10° from the IMU prediction:** the track is lost
    earlier, and the error reaches 14-52 m. The scans that follow fail the score
    threshold too, and after a second a wrong pose is accepted.
  - **Boreas** trips neither guard
    ([experiments/koide_imu_ab](experiments/koide_imu_ab/README.md#neither-guard-change-saves-the-track)).

## 1.6.0 - 2026-11-03

### Added

- Boreas is a usable `LiDAR + IMU + GT` benchmark
  ([docs/benchmark.md](docs/benchmark.md#boreas-vehicle-preset-replay)).
  `tools/benchmark/boreas_prepare.py` builds a fixture from `s3://boreas`: a window of
  one sequence as a bag of lidar and raw IMU in the lidar frame, a map of another
  day's scans at their ground-truth poses, and the reference.
  `tools/benchmark/boreas_replay.py` replays it through a preset and scores it. On the
  first 120 s of `boreas-2021-09-02-11-42`, `param/boreas_ndt_velodyne.yaml` scores
  0.132 m RMSE, max 0.41 m, with every scan accepted (three identical runs). As it was,
  the preset scored 53 m with 1% of the scans accepted.
  - The old runs failed for three reasons outside the localizer. The removed converter
    used the wrong ground-truth rotation convention. The ~1 MB clouds dropped on the
    best-effort sensor QoS. A 300 m local map was rebuilt on every scan.
  - The preset now subscribes to `/velodyne_points` as reliable. It reuses a 100 m
    crop for 10 m and builds the next one in the background, uses a 60 m scan range
    and a queue of 10, and takes no twist (the old one was post-processed ground
    truth).
- The online Koide check (`run_koide_public_bag.py`) reports the RMSE on and off the
  map in `summary.json` (`rmse_on_map_m`, `rmse_off_map_m`, `off_map_window_sec`) and
  prints the first next to the RMSE. From 163.3 to 259.0 s after the first expected
  cloud no configuration registers: the map has no point within 2.4-8.2 m of the
  trajectory from 179 to 232 s. There the error follows the tilt of the last accepted
  scan, so the overall RMSE does not compare configurations well. The gate is unchanged.
  The online profile on `main` scored 0.176-0.183 m on the map in 11 runs, against
  0.222-0.232 m overall.

### Changed

- NDT and NDT_OMP targets are ready sooner. `pcl::Registration` builds a search tree
  over the target points in the first `align()` after `setInputTarget()`. NDT aligns
  with its voxel grid and uses that tree only for `getFitnessScore()`, but the two
  builds ran one after the other. The localizer now builds the tree on another thread
  while `setInputTarget()` builds the voxel grid. This covers every NDT target: the
  local map crop, the background rebuild, the full map at load, the map topic. The
  alignment and the fitness score do not change. A new target plus its first alignment
  takes 300 instead of 518 ms for an 80 m crop of the dense Koide outdoor map, and
  65-152 instead of 91-190 ms for 150 m Istanbul crops. With
  `local_map_async_rebuild: false`, the alignment p95 fell from 34-50 to 21-23 ms on
  Istanbul (8.4-8.9 instead of 8.2-8.6 Hz). It fell from 144 to 114 ms on the online
  Koide check (99.7% against 99.2% coverage)
  ([experiments/istanbul_ndt_tracking](experiments/istanbul_ndt_tracking/README.md#search-tree-alongside-the-voxel-grid)).
- `continuous_time_deskew_mode: imu_pose_history` runs without `use_imu_preintegration`.
  It only rotates points with a gyro-integrated history, but it used to wait for IMU
  preintegration, which also moves the published pose to the IMU smoother backend. The
  IMU callback now keeps that history on its own (in its own callback group), and the
  seed and the published pose stay as they are. Other modes are unchanged.
- The online Koide profile uses it. 100-132 s into `outdoor_hard_02b` the sensor
  turns at 83 deg/s (median); undeskewed, a cloud taken at 150 deg/s or more needed 14
  NDT iterations and ended 0.30 m off, gyro-deskewed 4 and 0.08 m (offline). Six online
  runs scored 99.5-100% diagnostic coverage, 0.222-0.232 m RMSE and 112-127 ms
  alignment p95, against 99.0-99.3%, 0.233-0.240 m and 139-156 ms for three runs
  without it ([experiments/koide_gyro_deskew](experiments/koide_gyro_deskew/README.md)).
- The online Koide profile (`tools/readme_media/koide_outdoor_hard_02b_online.yaml`, used
  by `run_koide_public_bag.py`) ignores points beyond 60 m (`scan_max_range`, was 100 m;
  0.2% of the points) and builds the next local map in the background
  (`local_map_async_rebuild`). On a 4-vCPU Jazzy container, five runs passed at
  0.235-0.239 m RMSE and 97.2-99.2% diagnostic coverage. With the old profile, six runs
  gave 0.252-0.275 m and 95.2-99.1%, and two fell below the 97% gate
  ([experiments/koide_online_coverage](experiments/koide_online_coverage/README.md)).
  The v1.5.0 verification in `docs/koide-online-replay.md` now lists those six runs and
  the v1.3.0 tag on the same setup (97.0% and 97.9%), not only the first two.
- Three misspelled members are renamed: `corrent_pose_with_cov_stamped_ptr_`,
  `map_recieved_` and `initialpose_recieved_` are now `current_…` and `…_received_`.
  No topic, parameter or diagnostic key changes.

### Deprecated

- The legacy `use_imu` deskew is deprecated. It integrates the accelerometer without
  removing gravity, times each point from its azimuth relative to the first point of
  the cloud, and needs an IMU orientation; on the first 120 s of Boreas it scored 1.7 m
  with 6% of the scans accepted, where the preset scores 0.132 m. The node warns when
  `use_imu` is on. The config generator's `--enable-imu` and `--imu-mode both` now
  enable only the IMU preintegration and print a note; they used to add the legacy
  deskew. `--imu-mode legacy` still writes `use_imu: true`, with a warning. No preset
  turned it on.

### Removed

- Eight experiment-only parameter sets are removed and no longer installed:
  - the six `param/nav2_ndt_urban_*` derivatives (`borderline_only`,
    `recovery_retry_from_last_pose`, its `_r3` and `_r3_gap1_seed15` variants,
    `rejected_seed_update` and its `_r1` variant);
  - `param/odometry/`, the FAST-LIO, KISS-ICP and GLIM configurations for Koide
    outdoor.

  Nothing in the package read them; the scripts that did went in 6825f05. Each
  derivative is `nav2_ndt_urban.yaml` with three to six overrides, and the recovery
  retry that won their comparison is in `param/public_istanbul_60s_benchmark.yaml`.
  `git checkout 7af1287` has all eight. A launch that passes one of them by path
  needs `nav2_ndt_urban.yaml` and the overrides instead.
- `param/benchmark/` is removed and no longer installed. Its 123 run manifests and
  comparison specs were inputs to the offline runners removed in 6825f05, and nothing
  has read them since; `git checkout 6825f05^` has both. The route-crop recovery
  example from one of them is now in
  [docs/global_localization.md](docs/global_localization.md#route-crop-candidates-opt-in-wp3).

### Fixed

- An IMU gap no longer holds the last sample through every scan of it.
  - **The bug:** `planCausalImuIntervals` compared its 0.5 s hold limit with the scan
    window instead of the age of the held sample, so each 0.1 s window looked covered.
  - **Koide `outdoor_hard_01a`:** the bag's 0.65 s without IMU at 104.6 s seeded six
    scans from one sample. After the next cloud gap the prediction was 0.99 m and 4.9°
    off; with the fix it is 0.61 m and 1.3° off.
  - **The track:** it now holds up to the blocked sensor at 110.2 s, and is still lost
    there.
  - **Boreas:** its IMU has no gap, and its run is unchanged.
- With `use_imu` and `use_imu_preintegration` both on (what the config generator's
  `--imu-mode both` and `--enable-imu` used to write), the IMU callback now has its own callback
  group, as it has with the preintegration alone. It used to share the cloud
  callback's, so the samples waited behind each registration and arrived in bursts
  after the scan had used a stale window. On the first 120 s of Boreas, one run got a
  median of 0 new samples per scan and another reported too long an integration
  window on 374 of 466 scans; a scan now gets 22-23 samples (median). The legacy
  deskew state takes a new lock (`test/test_imu_callback_group.cpp`). The run still
  fails, because of the legacy deskew:
  `use_imu` alone scores 1.7 m with 6% of the scans accepted, where the preset scores
  0.132 m. It integrates the accelerometer without removing gravity. The
  preintegration alone scores 0.120-0.122 m with 81-82% of the scans processed
  ([docs/imu_estimation.md](docs/imu_estimation.md#validation-state-honest-limits)).
- The IMU preintegration smoother solves its normal equations sparsely. The matrix is
  block tridiagonal over the window plus the bias rows; it was factored densely, up to
  10 times per accepted scan. Once the 50-pose window was full an update took 87-96 ms
  (synthetic drive, one core), so with an NDT alignment on top a 10 Hz node skipped
  scans: 18-19% of the first 120 s of Boreas. Assembled sparse, with the ordering
  computed once per update, it takes 13 ms and gives the same final pose to 1e-9 m. Boreas
  with `use_imu_preintegration` now processes every scan and scores 0.127 m (two
  runs; 0.132 m without the IMU)
  ([experiments/imu_smoother_solve](experiments/imu_smoother_solve/README.md)).
- The previous-delta seed is scaled to the time since the last pose, at most
  `previous_delta_max_extrapolation_sec` (new, default 1.0 s; 0 restores one delta per
  scan). It used to advance the last pose by one inter-scan motion whatever time had
  passed. So it lagged after dropped scans: on Boreas a 0.6 s gap at 9.5 m/s left it
  5 m behind, and NDT locked 4 m back along the road. It also overshot on the scan
  after a gap. On the first 120 s of Boreas with 0.6 s dropped every 10 s, the preset
  scores 0.132 m RMSE (max 0.41 m), as without gaps, against 0.8-6.3 m (max 7-60 m).
  With an 80 m scan range, where scans wait for the local map, it scores 0.139 m
  against 0.93 m. Without gaps nothing changes. The first motion after a reset, from an
  initial pose to the first scan, is applied once and not rescaled
  ([experiments/previous_delta_time_scaling](experiments/previous_delta_time_scaling/README.md)).

### Validation

- Why Boreas with `use_imu_preintegration` skipped 0.7-1.2% of its scans over 600 s.
  The cloud queue dropped them: those runs shared the CPU with a download, and in
  the dense stretches the preintegration puts the node up to 0.9 s behind real time
  even without one, against 0.23 s without the IMU, on 4 vCPUs (`cloud_queue_depth` is
  10 scans).
- The IMU preintegration on a second dataset: Koide `outdoor_hard_01a`, the localizer
  alone without odometry (`experiments/koide_imu_ab/replay.py`, the A/B prepared
  before 6825f05). Without it the localizer loses the track at 35-49 s; with it, it
  tracks to 111 s in both runs (766-774 poses of 2860 clouds against 228-321), at the
  same 0.07 m RMSE where both track. Neither covers the 380 s sequence.
  `koide_public_bag.prepare_si_bag` now converts any Koide sequence, not only
  `outdoor_hard_02b`.
- Boreas over the first 600 s (4.3 km): the preset scores 0.105 m RMSE in two runs
  (max 0.41 m, every scan accepted), and with `use_imu_preintegration` 0.098-0.099 m
  (max 0.39 m, 98.8-99.3% of the scans processed). `boreas_prepare.py` now retries a
  download cut off mid-body, and merges the map 100 scans at a time: the 600 s
  preparation was killed at 14 GB before, the map is byte-identical
  ([docs/benchmark.md](docs/benchmark.md#boreas-vehicle-preset-replay)).
- Re-measured the target build times quoted in `docs/troubleshooting.md` and
  `param/istanbul_gtsam_smoother.yaml`, which came from the unoptimized NDT_OMP before
  v1.5.0. A new NDT_OMP target takes 0.08-0.19 s for a 150 m Istanbul crop after the
  crop (0.6-0.8 s was quoted) and 0.5 s for an 80 m Koide crop (4.6-4.9 s); GICP_OMP
  takes 0.25-0.44 s (1-2 s). Without `local_map_async_rebuild` the Istanbul preset
  keeps 8.3-8.7 Hz (0.9-2.9 Hz was quoted), with it 8.9-10 Hz, at the same accuracy
  ([experiments/istanbul_ndt_tracking](experiments/istanbul_ndt_tracking/README.md#re-measured-with-the-optimized-build),
  `target_build_bench.cpp`).
- Compared `NDT_OMP` and `SMALL_GICP` on the online Koide `outdoor_hard_02b` check, the
  second public dataset after Istanbul. SMALL_GICP failed all five runs (88-93%
  diagnostic coverage, 0.32-0.41 m RMSE), NDT_OMP passed all four (99.6-100%,
  0.227-0.232 m). On the map SMALL_GICP aligned faster (22 against 38 ms median). Off the
  map it slowed down (its slowest scans took 1-8 s) and missed clouds. `NDT_OMP` stays
  the default
  ([experiments/koide_backend_comparison](experiments/koide_backend_comparison/README.md)).
- The RMSE of the online Koide check rests largely on one scan. From 163 to 259 s the
  sensor is off the map and the odometry carries the pose from the last accepted scan,
  so the tilt of that scan sets the error there. `anchor_difference.py` finds that the
  SMALL_GICP runs carry the NDT_OMP trajectory under an anchor tilted 0.47 deg further
  (0.05 m residual), and SMALL_GICP's anchor is not the worse one. Outside the stretch
  the backends score 0.192 and 0.177 m. Re-scored that way, the gyro deskew scores
  0.177 against 0.193 m. The 10 ms end gap tolerance, dropped for its RMSE, lost only in
  the stretch and ties in the map (0.175 m). Deskewing the remaining 31% of the
  clouds is not pursued
  ([experiments/koide_gyro_deskew](experiments/koide_gyro_deskew/README.md#re-scored-outside-the-off-map-stretch)).

## 1.5.0 - 2026-11-01

### Added

- `/alignment_status` reports `registration_iterations`: the iteration count of the
  selected registration attempt for `NDT`, `NDT_OMP`, `SMALL_GICP` and `SMALL_VGICP`
  (`-1` for the other backends), next to `alignment_time_sec`. A scan at or above
  `ndt_max_iterations` ran to the cap instead of converging.
- `tools/benchmark/istanbul_replay.py` scores the alignment cost into `score.json`
  (`alignment`): the share of the window's scans that were aligned, the alignment time
  (median, p95, max) and the iterations (median, p95, max, share at the cap).
  `istanbul_compare.py` prints them in a second table, and `record_pose.py` keeps
  `alignment_time_sec` and `registration_iterations` in `alignment.jsonl`.
- Add `local_map_async_rebuild` (default `false`, every registration backend): with the
  local map crop, the next target is built on a twin registration object in the
  background while scans keep aligning against the cached one, which covers every scan
  point until the pose has moved `local_map_radius - scan_max_range`. On the Istanbul map
  a 150 m crop took 0.6-0.8 s to build in the scan callback; with this the NDT_OMP pose
  rate at 21 m/s went from 0.9 to 5.2 Hz. `param/istanbul_gtsam_smoother.yaml` enables it
  ([experiments/istanbul_ndt_tracking](experiments/istanbul_ndt_tracking/README.md)).
  For the GICP backends the background task also voxel-filters the crop and aligns about
  100 of its points once, so the target covariances (small_gicp VGICP: the voxel map) and
  the search tree, which the backends build in the first `align()`, are ready when the
  scan callback swaps the target in. On the same preset with SMALL_GICP the pose rate went
  from 7.1-7.9 to 8.0-9.5 Hz (10 Hz scans) at unchanged accuracy.
- Add `param/istanbul_small_gicp.yaml`: `istanbul_gtsam_smoother.yaml` with
  `registration_method: SMALL_GICP`. On the three Istanbul windows it gave 0.26, 0.76 and
  0.45 m horizontal RMSE at 8.0-9.5 Hz, against 0.26, 0.91 and 0.54 m at 1.3-3.1 Hz for the
  NDT_OMP preset in the same session (`docs/benchmark.md`). That NDT_OMP ran unoptimized
  (first entry); optimized, it gave 0.23, 0.75 and 0.46 m at 8.9-10 Hz.
- CMake also looks for `small_gicp_vendor`, so on Jazzy `ros-jazzy-small-gicp-vendor`
  from apt enables the `SMALL_GICP` and `SMALL_VGICP` backends without a local
  small_gicp install (Humble has no such package); the configure log says when
  small_gicp support is enabled.

### Changed

- `-O2 -g` is now added only when `CMAKE_BUILD_TYPE` is unset, so `Release` builds no
  longer carry debug info. This lowers compiler memory use and binary size on small
  boards. Builds without a build type are unchanged.
- The Autoware Istanbul replay tools moved from `experiments/twist_smoother_slope` to
  `tools/benchmark/istanbul_{prepare,replay,compare}.py`, and `docs/benchmark.md` now
  describes the Istanbul benchmark (windows, GNSS reference and its pitfalls, latest
  results). `docs/imu_estimation.md` marks the twist EKF experimental and recommends the
  twist GTSAM smoother. `test/test_istanbul_benchmark_tools.py` covers them without ROS:
  the GNSS reference cleanup, the window start in the header-stamp time base, the
  initial pose and overrides, the scores and the comparison runs and summary.
- The Jazzy CI job installs `ros-jazzy-small-gicp-vendor`, so it builds, tests and
  lints the `SMALL_GICP` and `SMALL_VGICP` code (`test_local_map_async_rebuild` covers
  them); the Humble job still builds without small_gicp.

### Fixed

- Fix `NDT_OMP` and `GICP_OMP` running unoptimized: `ndt_omp_ros2` sets no
  `CMAKE_BUILD_TYPE`, so a plain `colcon build` (as in `AGENTS.md` and CI) compiled its
  NDT and GICP code without optimization, and the localizer ran that code. The
  component now compiles the `pcl::PointXYZI` instantiations itself
  (`src/ndt_omp_instantiation.cpp`) with this package's flags, whatever build type
  `ndt_omp_ros2` has. On the Istanbul windows with `param/istanbul_gtsam_smoother.yaml`,
  NDT_OMP went from 3.1, 3.0 and 1.5 Hz to 8.9, 10.0 and 10.0 Hz (10 Hz scans; median
  alignment 223-262 ms to 11-12 ms), and horizontal RMSE from 0.37, 0.88 and 0.47 m to
  0.23, 0.75 and 0.46 m (median of three runs, 4 cores). Building `ndt_omp_ros2` with
  `-DCMAKE_BUILD_TYPE=Release` gave the same result
  ([experiments/istanbul_ndt_tracking](experiments/istanbul_ndt_tracking/README.md#unoptimized-ndt_omp)).
  The Istanbul NDT_OMP pose rates in the entries below match such a build. On Koide
  `outdoor_hard_02b`, `run_koide_public_bag.py` failed with that build (12-13 of 2,209
  clouds aligned) and now passes (0.254 and 0.262 m RMSE;
  [docs/koide-online-replay.md](docs/koide-online-replay.md#v150-release-verification-2026-11-01)).
- Log an error at map load when the full-map NDT voxel grid needs more cells than int32
  indexes: the target is then empty and NDT does not register (troubleshooting, *NDT
  Does Not Correct the Pose on a Large Map*). `param/istanbul_gtsam_smoother.yaml` now
  crops the map. On three windows of the Autoware Istanbul bag the preset went from
  5.9, 20.1 and 51.1 m to 0.26, 1.11 and 0.51 m horizontal RMSE (median of three runs)
  ([experiments/istanbul_ndt_tracking](experiments/istanbul_ndt_tracking/README.md)).
- Fix the twist GTSAM smoother (`use_gtsam_smoother`) dropping motion at every NDT
  update: it put the NDT pose, a measurement at the scan stamp, on a node built from all
  twist received until the scan was processed, so each update lost speed x processing
  delay and the next NDT seed started behind. It now splits the twist at the scan stamp
  and seeds NDT at the scan stamp. The twist subscription queues 1000 messages (was 5),
  so twist received while a scan holds the state lock (a local map crop) is no longer
  dropped.
- The twist GTSAM smoother's NDT seed follows the slope: `predict()` takes the last NDT
  roll and pitch, only `cos(pitch)` of the forward speed is planar odometry, the rest
  moves the seed's z, and the yaw rate turns the heading at `cos(roll) / cos(pitch)`. On
  the graded Istanbul windows z no longer falls behind while NDT is rejected (worst z RMSE
  of three runs 19.4 m to 2.0 m and 28.1 m to 0.47 m); the first measurement, made while
  NDT did not register, had rejected it
  ([experiments/twist_smoother_slope](experiments/twist_smoother_slope/README.md)). The
  experiment keeps a replay harness for the bag, and `docs/dev/benchmarking.md` again has
  working Istanbul download steps.
- Fix the twist EKF (`use_twist_ekf`) pulling the state back at every NDT update: it
  compared the NDT result, the pose at the scan stamp, with the state predicted up to
  the latest twist. It now carries the result along its predicted trajectory from the
  scan stamp to the state's time, and seeds NDT and publishes with the pose at the scan
  stamp. On the Istanbul windows A and C the EKF went from 19 and 270 m to 0.25 and
  0.54 m horizontal RMSE; window B still fails in z
  ([experiments/istanbul_ndt_tracking](experiments/istanbul_ndt_tracking/README.md)).

## 1.4.0 - 2026-10-27

### Added

- Maps far from the origin (#68): the localizer warns at load when map coordinates
  reach 131 km, where float32 points and poses are 1.6 cm or more apart. A synthetic
  check found Autoware MGRS maps (up to 100 km) usable as they are (the pose moved by
  at most 7.8 mm), while UTM coordinates moved it by up to 0.18 m and need shifting
  near the origin. See
  [troubleshooting](docs/troubleshooting.md#maps-far-from-the-origin-mgrs-utm-68)
  and [experiments/large_map_coordinates](experiments/large_map_coordinates/README.md).
- Add `/map_switch` (`std_msgs/String` holding a `.pcd` or `.ply` path): replaces the
  map at runtime and keeps the current pose. A map that fails to load or has no points
  leaves the current one. If the new map has a different origin, publish `/initialpose`
  right after the switch; the global localization node keeps the map it was started
  with.
- Add `publish_tf` (default `true`). `false` turns off every TF the localizer
  broadcasts, for setups where another node (for example `robot_localization` fusing
  `/pcl_pose`) owns `map -> odom`; `/pcl_pose` and the other topics are still
  published. See [Mode C in the frame contract](docs/frame_contract.md).
- Quickstart reports tracking health after startup. `localization_status_node.py`
  prints one line when tracking starts, weakens (few or no scan points, poor match,
  CPU overload), is lost, stops receiving scans, or comes back, each with what to do,
  and RViz shows the same status as a colored label above the robot (**Localization
  Status** display). A change is reported once it has held for 2 s. No accepted scan
  for 5 s reads as lost, or as "the pose follows odometry only" (orange) while an
  external `odom ->` TF still arrives: on the Koide outdoor_hard_02b replay RKO-LIO
  kept the pose within 0.31 m through 70 s of rejected scans, while on
  outdoor_kidnap_a, with the odometry stopped too, the pose was 55-72 m off. Odometry
  only cannot tell a good bridged pose from a wrong reset. Diagnostics only;
  `--no-status` (launch arg `show_status:=false`) turns it off. Topics:
  `/localization_status/status` (JSON: `phase`, `cause`, `label`) and
  `/localization_status/marker`.
- Add a Docker image (`docker/Dockerfile`, Jazzy by default, `ROS_DISTRO=humble`
  also builds) with ndt_omp_ros2, and `docker/smoke_status.sh`, which runs quickstart
  on a synthetic map and scan stream and checks the status lines; CI runs both. See
  [docs/docker.md](docs/docker.md).
- Add a sparse-band global search: `sparse_band_min_points` (G2 node, default 0 =
  off; launch arg `g2_sparse_band_min_points`; quickstart
  `--global-sparse-band-min-points`, **default 512**). A scan with fewer 2D points than
  this in the height band (counted before the scan-point cap) is searched again from
  the sensor plane up, with 1024 points, 32 candidates and NDT-refined poses, and the
  answer reports `sparse_band_search`. With quickstart defaults the JEPLO Go2 started
  correctly in all 12 capture-room runs (before: 3-5 wrong) and now also on the
  Long_Stairs and Outdoor2 self maps; Outdoor1 stays ambiguous and asks for a manual
  pose. On handheld Koide replays it fired on no query
  ([experiments/go2_wide_map_global_init](experiments/go2_wide_map_global_init/README.md#searching-again-only-when-the-band-is-sparse-round-4)).
- Add `seed_correction_guard_exact_start_translation_m` (node default `0`, off;
  quickstart configs `0.1`): when the first accepted update after an initial pose
  corrects it by at most this much, the seed correction guard's warmup ends at once.
  It stops the EIL_Box jump, where NDT accepted 1.7-2.0 m aliased corrections right
  after a correct initial pose and the track slid 4.5 m: on a controlled ground-truth
  start both runs held, where the previous quickstart settings lost both. On the EIL,
  wide-map and Koide regression replays it fired once, on a correct start
  ([experiments/post_init_jump](experiments/post_init_jump/README.md)).
- Add opt-in quickstart options for global search with a sensor low above the
  ground: `--global-z-min` / `--global-z-max` (the G2 scan height band, launch
  args `g2_z_min_m` / `g2_z_max_m`, defaults 0.5 / 5.0 m as before) and
  `--occupancy-inflate-radius-m` (inflation of the generated grid, default 0.6 m as
  before, cached separately). On the JEPLO Go2 self maps (Long_Stairs, Outdoor1,
  Outdoor2), which never initialized, the recipe in
  [go2_mid360_quickstart.md](docs/go2_mid360_quickstart.md#large-maps) initialized
  all six runs without a wrong start (later: 5 of 6, one Outdoor2 run stayed ambiguous
  and asked for a manual pose), and in the capture room all 12 EIL runs, where
  the defaults started wrongly in 5 of 16 on the same (rebuilt) map. The 2D search saturated: about 100 scan points
  above an upside-down MID-360 matched a 0.6 m inflated grid perfectly at many
  places, so the truth rarely reached NDT
  ([comparison](experiments/go2_wide_map_global_init/README.md)). Defaults are
  unchanged: only Go2 data was compared. The G2 node logs its height band at startup.
- Add `tools/benchmark/jeplo_to_pc2_bag.py` and `build_map_from_odometry.py`: the
  public JEPLO dataset now ships only raw Livox bags, so the Go2 suite's PointCloud2
  bags and self maps are rebuilt from them (docs/benchmark.md).
- Add an experimental opt-in G3 confirmation mode for odometry dropouts
  (`odometry_confirmation_mode:=segment_defer`, quickstart
  `--supervisor-odometry-confirmation-mode`). Answers confirm each other only across
  gapless odometry, and queries wait while odometry is out. The default
  (`window_waiver`) is unchanged; see
  [the synthetic comparison](experiments/odometry_dropout_confirmation/README.md).
  A Koide replay (three runs per mode) did not support promotion: the mode
  regresses `outdoor_kidnap_b` recovery, where RKO-LIO odometry drops out
  throughout.

### Changed

- Quickstart discovery and errors:
  - map errors list the `.pcd`/`.ply` next to a directory or missing file as `--map`
    options, and name wrong extensions and empty files;
  - only published topics count, the localizer's own `/map` and `/initial_map` are
    not scan candidates, and a running localizer (`/alignment_status` published)
    is reported;
  - when a `nav_msgs/Odometry` topic is published, discovery reads the topic list
    again and waits up to 10 s for its `odom ->` TF, and names the topic in a hint
    if the TF never arrives. RKO-LIO publishes the TF only after its first scans
    (7 s after a cold Koide bag started), and DDS can discover it after the bag
    player; Koide replays started with `odometry=none` and failed global
    initialization in both cases.
- `run_benchmark.py` lists runs whose RKO-LIO odometry stalled (registration loop
  over 10 s), and `score.json` keeps the longest loop. Such runs looked like
  localizer failures in the post_init_jump replays.
- Record why the Go2 global search settings stay opt-in
  ([experiments/go2_wide_map_global_init](experiments/go2_wide_map_global_init/README.md#should-these-settings-be-the-defaults-koide-handheld)).
  On handheld Koide replays, a 256-point version initialized later in all 12 runs and
  wrongly in 2; the defaults were right in all 12. On the Go2 data, consecutive
  answers of the defaults' saturated 2D search mostly agree 1-3 m off the robot,
  which the two-view startup consensus then confirms; starting each query's yaw
  sweep at another sample did not change that (not promoted).
- CI runs the C++ tests (`colcon test`) and the whole Python suite (`pytest test/`) on
  Humble and Jazzy (#238), with Node 24 actions and a ccache compiler cache (#240).

### Fixed

- `use_twist_ekf` predicts along slopes: the forward speed is split by the last NDT pitch
  into horizontal and vertical motion (z was held constant), and the body yaw rate turns
  the heading at `cos(roll) / cos(pitch)` of its value. On level ground the EKF is
  unchanged (bit-identical state and covariance). The twist GTSAM smoother stays planar.
- `use_odom` prediction applies the `/odom` angular velocity about the body axes
  (composed with the current orientation) instead of adding it to roll, pitch and yaw,
  which turned a tilted robot about the wrong axes. Before #236 a tilted pose was also
  rotated with zero angular velocity: roll and pitch were recomposed in the wrong
  order. On level ground the prediction is unchanged.

## 1.3.0 - 2026-10-09

- Add an opt-in 75 m local-map preset for online Koide replay with RELIABLE points
  and depth 10. Two paired repeats reduce diagnostic p95 lag by 44-56% and localizer
  peak RSS by about 44%, at comparable map-frame position RMSE (~0.26 m); see
  [online replay measurements](docs/koide-online-replay.md) (#212).

- Add `run_koide_public_bag.py`: one command verifies/downloads official Koide data,
  prepares SI IMU input, runs online RKO and localization at 1x, supplies one GT seed,
  and saves a map-frame accuracy/diagnostic coverage report with logs and provenance.
- Quickstart lists ambiguous sensor-topic candidates and the option to select one,
  explains missing inputs, and reports configuration-write errors with a suggested
  `--output` option instead of a Python traceback.
- Bound visualization path history to 2000 poses by default (`path_max_poses`),
  including lifecycle initialization, and report its size in diagnostics (#134).
- Infer FLOAT64 timestamp units once per cloud; support explicit seconds/ns
  overrides while preserving floating-point `t` seconds (#208).
- Add an opt-in soft odom correction gate that preserves independent measurement
  rejections, with regression tests for poor fitness and timestamp boundaries (#208).

### Added

- `min_registration_points` (default 0, off): a scan with fewer points than this after
  filtering is skipped like an empty one (`filtered_scan_too_sparse`) instead of being
  registered on almost no geometry. On Koide `outdoor_kidnap_b`, scans with 6-56 points,
  taken while the sensor was covered, had been accepted with 2 m / 21 deg corrections.

- Opt-in `level_map_to_odom`: `map -> odom` keeps only its yaw, so a tilted registration
  does not tilt the poses bridged on a gravity-aligned odometry. On Koide
  `outdoor_hard_01a` (165 s outside the map) RMSE 3.72 -> 0.67 m; on `outdoor_hard_02b`
  0.26 -> 0.35 m. See troubleshooting before enabling it.
- The point cloud subscription accepts the standard ROS reliability override
  (`qos_overrides.<topic>.subscription.reliability`).
- Opt-in local re-acquisition after an odometry-bridged outage
  (`enable_local_reacquisition`, `local_reacquisition_occupancy_yaml`, ...). While scans
  are rejected and the pose is carried by external odometry, a branch-and-bound search
  restricted to the bridged heading window proposes candidates around the bridged pose,
  refines them with the normal registration against the target already cropped there,
  and seeds the next scan from an unambiguous fit; the normal measurement gate on that
  scan decides whether it becomes the new `map -> odom`.
- `local_map_update_distance` reuses the cropped local-map registration target until
  the crop center has moved that far (default `0` re-crops whenever the center moves).
- Opt-in seed-agnostic jump guard: `enable_seed_correction_guard`,
  `seed_correction_guard_translation_m`, `seed_correction_guard_yaw_deg`,
  `seed_correction_guard_release_rejections`, `seed_correction_guard_warmup_accepts`.
  Rejections report `seed_correction_guard_rejected` and count as rejected measurements
  for recovery/reinitialization.

- Added `quickstart.py` and `quickstart.launch.py` for one-command configuration,
  sensor-topic discovery, localization, RViz, and actionable bringup output.
- Added map-content-bound pose persistence and guarded startup initialization. A saved
  pose must pass pre-publication NDT scoring, while a BBS_2D candidate requires active
  3D NDT scoring and distinct-scan consensus. Stale, weak, ambiguous, inconsistent, or
  exhausted candidates fall back to RViz without publishing an identity pose.
- Added `rviz/localization_mid360.rviz` for MID-360 profiles (`/livox/points`) and a
  `Global Candidates` PoseArray display (`/global_localization_node/candidates`) in both
  RViz configs; `quickstart.launch.py` now selects the RViz config by profile.
- Added `watch_alignment.py` for colored one-line `/alignment_status` monitoring with
  next-action hints and optional CSV recording (diagnostics only).
- Added `watch_startup.py` for one-line `/startup_initialization/status` progress
  (saved pose → global search → verify → RViz fallback).
- `quickstart.py` now prints an actionable `Next:` line (verify pose output, keep the
  robot stationary for guarded search, or set 2D Pose Estimate in RViz), including in
  `--dry-run`.
- Bringup doctor failures now point at the right docs (`troubleshooting.md`,
  `map_alignment.md`, `frame_contract.md`, `site_setup.md`, `global_localization.md`).
- G2 `global_localization_node.py` publishes JSON query progress on `~/status`
  (`started`/`search`/`scoring`/`done`) so 10–20 s queries are observable while they run.
- The public validation dashboard now includes an engineering-evidence table (WP1/WP2/
  Koide rows, labeled not-a-claim) with artifact links and the GIF gallery pointer.

### Changed

- `param/nav2_ndt_urban.yaml` sets `local_map_update_distance: 10.0` again (it was
  reverted in #153). With RKO-LIO keeping its motion across scan gaps (rko_lio#16), live
  1.0x replays of the Koide `outdoor_hard_02b` README setup publish ~6.5 instead of
  ~1.1 poses/s, at RMSE 0.33-0.36 m in 4 of 4 runs (0.31-0.98 m without it).
- `param/mid360_legged.yaml` registers against the full map (`enable_local_map_crop:
  false`). The 80 m crop was rebuilt on every scan, which on JEPLO (Go2, MID-360)
  processed 20% of the Long_Stairs scans and lost Outdoor1 (20.8 m ATE). Reusing it for
  20 m restores the accuracy, but in live replays with the online RKO-LIO front end each
  re-crop of the dense outdoor map still stalled the output for 1.8-2.0 s; without the
  crop the accuracy is the same and the longest gap is 0.1 s. The crop settings
  (`local_map_radius: 80.0`, `local_map_update_distance: 20.0`) stay for maps too large
  to register against whole.
- Continuous-time deskew now defaults on in the component, launch files, and shipped
  presets. Readiness guards preserve the original scan when point timing or motion data
  is unavailable; `use_continuous_time_deskew:=false` remains the rollback switch.
- Quickstart global initialization now defaults on when a matching occupancy map is
  supplied. It remains inactive without that required asset and can be disabled with
  `--no-auto-initialize`.
- Public rosbag regression launches now use simulated time and explicitly preserve the
  HDL no-IMU/IMU comparison conditions instead of inheriting launch defaults.

### Fixed

- Global re-initialization seeds its reset at the height G2 verified the candidate at
  (NDT's converged z, else the scoring z) instead of the last localizer pose's z. After a
  kidnap that pose had been bridged on drifting odometry and was 12.5 m above the ground
  on Koide `outdoor_kidnap_b`, so resets within ~1 m in xy could not register.
- The TF buffer keeps 120 s instead of 10 s. An initial pose is anchored to `map -> odom`
  through `odom -> base` at its own stamp, and global re-initialization stamps its reset
  with the scan it searched; with a 13.6 s search the odom TF was already gone, so every
  reset was dropped ("extrapolation into the past") on a real-time Koide
  `outdoor_kidnap_b` replay (7 of 7; 0 of 8 after the change).
- `generate_occupancy_map_from_pcd` is back, as a C++ executable. The Python tool was
  removed with the offline tooling, but `nav2_navigation.launch.py`
  (`generate_map_from_pcd:=true`), local re-acquisition and global localization still need
  an occupancy grid of the 3D map, and the launch failed when asked to generate one. The
  output matches the removed tool (Koide `outdoor_kidnap` map: 9 of 893k cells differ) and
  loads with the same reader as local re-acquisition.
- The standalone, Nav2 and MID-360 launch files activate the localizer through bounded
  `GetState`/`ChangeState` calls (`start_lifecycle_node.py`) instead of chained transition
  events, which could miss a transition and leave the node inactive. Activation failure,
  an unexpected state or a 60 s deadline now fails the helper visibly; a slow but valid
  reply is still accepted.
- Registration results with NaN or infinite fitness are rejected as
  `registration_fitness_non_finite`, even when score-threshold rejection is disabled; they
  cannot update the rejected seed or pass consistency recovery, and diagnostics classify
  them as `bad_match`. Non-finite registration poses are rejected before they reach the
  pose backend.
- Rejected updates are counted when motion prediction is disabled, so the recovery retry
  from the last pose still triggers.
- After a rejected-update streak, deskew keeps the held relative motion paired with its
  own duration instead of the whole interval since the last accepted scan, which
  understated the motion rate.
- Accepting an initial pose invalidates the previous `map -> odom` anchor and odometry
  bridge history, even when the odometry TF for the new stamp is not available yet.
- PointCloud2 point and time readers honour row padding (`row_step` larger than
  `width * point_step`) instead of reading padding bytes as points.
- Legacy IMU deskew initializes its history before the first update and looks up
  interpolation samples in the right direction.
- The measurement-gate warning reports the actual rejection reason, fitness, threshold and
  correction instead of always claiming the fitness exceeded the threshold.
- `scripts/setup_local_env.sh` resolves the local workspace root across the supported
  layouts.
- NDT no longer registers against a thinned map when `enable_local_map_crop` is on. The
  full-map path hands NDT the unfiltered map, but the crop path voxel-filtered the crop
  with the scan `voxel_leaf_size`, so enabling the crop changed the target density. On
  JEPLO Outdoor1 (crop radius 80 m, `local_map_update_distance: 5.0`) that locked onto a
  wrong place (ATE 22.9 m, 99.6% accepted); the unfiltered crop gives 0.03 m, as without
  a crop. GICP-family backends keep the filtered crop.
- `lidar_localization.launch.py` and `mid360_legged_localization.launch.py` no longer
  overwrite the parameter YAML with launch-argument defaults. Unset arguments keep the
  YAML value (or the previous default when the YAML omits it); explicit arguments still
  win, and the static TF publishers use the resolved `base_frame_id`. Previously, e.g. a
  generated config with `use_continuous_time_deskew: false` or a YAML `base_frame_id`
  was silently replaced by the launch default. `mid360_legged.yaml` now carries the
  former launch default `map_path: /map/map.pcd`.
- Direct twist reception no longer waits for the localization state lock, so twists
  keep arriving while a scan is being registered. Each admitted scan uses the latest
  finite twist at or before its source stamp (bounded 1024-sample history) for both
  its registration seed and rejected-scan advancement, instead of whichever message
  arrived last. Non-finite twists are ignored. EKF/GTSAM twist routes still update
  under the state lock, and callbacks from a cleaned-up configuration are discarded.
- Twist prediction no longer extrapolates an unstamped initial pose by the maximum
  prediction interval before the first simulated clock. Rejected-scan prediction
  uses the same time validation as registration seeds.
- Twist prediction no longer counts the inter-scan motion twice: after an accepted
  update the stored prediction was already extrapolated by the previous delta, and the
  twist seed then integrated the same motion again on top of it. The stored pose
  now consistently refers to its timestamp, including when twist becomes available
  after previous-delta fallback; previous-delta extrapolation happens at seed selection.
- The first scan after map load no longer stalls for ~1 s: `pcl::Registration` builds
  its target search tree lazily in the first `align()` (even for NDT, which does not use
  it). The full-map target is now warmed up at map load, so the scans that follow are
  not dropped while the robot keeps moving.
- Floating-point per-point time fields named `t` / `offset_time` / `timestamp` are now
  read as seconds; only integer ones are nanoseconds. An absolute float64 `t` (e.g.
  converted Livox clouds) was scaled by 1e-9, collapsing the scan duration to ~0.

## 1.2.0 - 2026-07-22

Tightly coupled GLIL, exact point-cloud downsampling, and guarded kidnapped-pose
recovery. Release boundary: PR #128 merged at `04bc8c7`; Humble and Jazzy CI passed,
the Python suite reported 314 tests plus 140 subtests passed, and the pinned GLIL
container passed its embedded CTest suite.

### Tightly Coupled GLIL

- Combined IMU preintegration, exact-coreset scan-to-scan GICP, and exact-coreset
  scan-to-prior-map GICP in one GLIM fixed-lag factor graph, following the estimator
  structure of Koide et al. ICRA 2024 and deferred exact downsampling of Koide et al.
  ICRA 2025.
- Added continuous `odom -> base_link` output through map loss and one bounded
  `map -> odom` authority; map-factor loss, rejected candidates, and recovery never
  reset raw odometry or introduce a second TF parent.
- Added verified map-state epochs, time-aligned 32-candidate BBS/GLIL re-ranking, and
  three-frame full-resolution consensus for kidnapped-pose recovery.
- Recovery loss now checks both overlap fraction and the absolute map-factor inlier
  floor, closing the misleading 12/12-style sparse-overlap case while ignoring empty
  transport frames.
- Published reproducible 01a/01b/02a/02b and outdoor-kidnap A/B runners, measurements,
  and six GLIL-only replay GIFs in `docs/koide_gif_gallery.md`.

### Validation Boundary

- Full 01a, 01b, and 02a runs passed accuracy, continuity, real-time playback, queue,
  and TF/reset gates. The final 02b run passed accuracy and TF gates; its CPU-contended
  playback result is explicitly labeled rather than claimed as clean throughput.
- Outdoor kidnap A and B both ended `recovered_true`, at 0.074 m and 0.041 m final
  error respectively, with zero TF jumps, zero unauthorized resets, and drained final
  queues.
- The tightly coupled GLIL path is the measured main configuration; NDT is not launched
  alongside it. Legacy split prior-map localization remains available for comparison.

### Added

- `failure_category` key on `/alignment_status` distinguishing
  `missing_map` / `missing_initial_pose` / `weak_overlap` / `bad_match` /
  `stale_prediction` / `overload` / `healthy`, with `*_active` companion flags
  and three `diagnostics_*` tuning parameters (docs: `troubleshooting.md`)
- `scripts/analyze_pose_covariance_calibration.py` to calibrate `/pcl_pose`
  covariance against ground-truth replays
- G2 runtime global localization service: `scripts/global_localization_node.py`
  answers `std_srvs/Trigger` queries on `~/query` with map-wide BBS_2D
  candidates on `~/candidates` (opt-in node; default launch unchanged), with
  ROS-free query logic in `scripts/global_localization_query.py`
- `scripts/render_global_localization_demo_gif.py` renders the
  kidnapped-start -> global localization -> tracking-resume demo GIF
- `scripts/diagnose_local_map_crop_coverage.py` decides offline whether a Boreas
  `local_map_crop_too_small` cliff is a map-coverage problem or prediction-driven
  divergence, by counting map points within `local_map_radius` of every
  ground-truth pose (no ROS / no localizer); core unit-tested in
  `test/test_local_map_crop_coverage.py`
- G3 guarded automatic reinitialization: `scripts/reinitialization_supervisor_node.py`
  connects `/reinitialization_requested` to the G2 query service and republishes
  `/initialpose` only when explicit safety guards pass (opt-in node; default
  launch unchanged). The decision logic is the ROS-free state machine in
  `scripts/reinitialization_supervisor_policy.py` -- candidate-score floor, minimum
  reset spacing, post-reset recovery evidence, and a non-self-resetting attempt
  ceiling -- regression-tested against unsafe-publication and false-acceptance
  sequences in `test/test_reinitialization_supervisor_policy.py`
- `docs/global_localization.md`: operations guide for running the G2 service and
  the G3 supervisor (commands, parameters, and the supervisor's safety guards)

### Changed

- `make_bbs_relocalization_attempts.py` branch-and-bound search is ~8x faster
  (precomputed per-yaw/level integer offset tables + adaptive FFT hit maps)
  with exactly identical candidate output
- `/pcl_pose` covariance default model is now `error_floor`, calibrated against
  ground truth on Koide outdoor_hard_01a and Autoware Istanbul (per-axis 2σ
  coverage ≥ 96 % on the calibration runs); the v1.1 heuristic remains
  available as `pose_covariance_mode: fitness_scaled` (docs:
  `pose_covariance.md`, issue #72)
- Documented the single-threaded-executor invariant that serializes the node's
  shared pose state without locks, so a future executor change cannot silently
  introduce a data race (issue #54)

## 1.1.0 - 2026-06-11

Reliability sprint, Jazzy support, and the artifact-first relocalization /
global-localization foundation. Release boundary:
`run_release_regression_suite.sh` `overall_pass=true` on a Jazzy build
(2026-06-11, commit `64da698`); see `docs/dev/public_validation_log.md`.

### Added

- `scripts/run_public_demo.sh` for the 30-minute Autoware Istanbul public demo path
- `scripts/run_public_validation_dashboard.sh` for Markdown/HTML public validation dashboards
- `param/benchmark/v1_1_boreas_dry_run_endpoint.example.yaml` for the v1.1 MVP dry-run endpoint chain
- `scripts/run_v1_1_relocalization_smoke.sh` to guard the v1.1 claim boundary
- `docs/dev/global_localization_roadmap.md` defining the G1/G2/G3 global localization phases
- `docs/dev/development_plan.md` dated execution plan linked from README and the roadmap
- `scripts/make_map_grid_relocalization_attempts.py` map-wide candidate baseline that needs no route prior
- `scripts/make_bbs_relocalization_attempts.py` BBS_2D branch-and-bound candidate generator over the occupancy-grid pyramid

### Reliability / Claim Boundary

- add `docs/frame_contract.md` for `map` / `odom` / `base_link` expectations and issue #58 / #27 guidance
- publish TF immediately on `/initialpose` and add explicit frame/map diagnostics for issue #47
- add `docs/troubleshooting.md` for bringup failures and `/alignment_status` interpretation (issues #70, #35, #43, #48)
- add `docs/map_alignment.md` for map frame / initial pose alignment (issues #75, #44)
- add `docs/pose_covariance.md` for `/pcl_pose` covariance semantics (issue #72)
- document Istanbul public demo RMSE run variance; identical seed/map can yield ~`1.2 m` or late-run drift outliers
- tune public Istanbul 60 s preset with `local_map_crop` and `recovery_retry_from_last_pose` (`r3`, `gap<=1 s`, `seed<=15 m`)
- reconfirm public regression suite after drift tuning; Istanbul gate `1.00 m` and HDL IMU checks pass (2026-06-10)
- enable `recovery_retry_from_last_pose` on Koide outdoor manifests; `180 s` acceptance rises from `71` to `473+` ok rows
- reconfirm Koide outdoor manifests after recovery tuning; acceptance holds on `180 s` reconfirm (`486/753 ok_rows`) with run-to-run RMSE variance
- fix `boreas_localization_120s.yaml` dataset paths; baseline smoke shows `37/961 ok_rows` with `local_map_crop_too_small` bottleneck
- add `param/boreas_ndt_velodyne.yaml` and Boreas cliff sweeps; first `12 s` tracks (~`0.04 m`) before fitness reject streaks, `local_map_radius=300` improves 60 s RMSE to `39.5 m`
- tune Boreas preset throughput (`voxel 1.5`, `ndt threads 8`, `cloud_queue_depth 1`); 120 s matched rises to `45` with align median `0.018 s`
- add Boreas seed-management sweep; `post_reject_strict` + `rejected_seed_update` + `max_twist_prediction_dt=0.1` improves 60 s matched to `49` / RMSE `45.7 m`; 120 s confirm matched `52` / RMSE `47.8 m`
- guard `use_odom` integration until initial pose is valid and keep pose finite for NDT init guess
- ignore non-finite `/initialpose` payloads and stop eager scan replay during pose reset
- v1.1 relocalization is documented as a validated dry-run `/initialpose` command artifact only
- guarded reset publication and post-reset observation remain experimental and outside the public MVP endpoint
- automatic runtime recovery and production-grade global relocalization are explicitly out of scope

### ROS 2 Distro Support

- README badges and support matrix now show **Jazzy-first / Humble-compatible**
- GitHub Actions workflow updated to `actions/checkout@v4` and `push` on `main`
- Jazzy CI still builds against `ndt_omp_ros2` `humble` branch until a dedicated Jazzy branch exists upstream
- `scripts/setup_local_env.sh` resolves the ROS distro (`LIDAR_LOCALIZATION_ROS_DISTRO`, `ROS_DISTRO`, humble, jazzy) instead of hardcoding Humble
- HDL sample fetch patches rosbag2 `offered_qos_profiles` metadata per distro so `ros2 bag play` works on Jazzy
- public regression suite passes on a Jazzy build (2026-06-11): Istanbul `0.673 m` translation RMSE, HDL medians `468.5 / 459.0` pose rows
- Nav2 plugin type names switched from the Humble lookup form (`nav2_navfn_planner/NavfnPlanner`, `nav2_behaviors/*`) to the `::` type form required by Iron+ and accepted by Humble
- `param/nav2_humble_pointcloud.yaml` works with Jazzy bringup: explicit `plugin_lib_names` removed (double registration crashes Jazzy `bt_navigator`), `collision_monitor` and minimal `docking_server` sections added for the Iron+ lifecycle set
- Nav2 reinit supervisor regression prints route-bounded occupancy-map bootstrap commands when the map artifact is missing on a fresh workspace

## 1.0.0 - 2026-03-30

Initial repo-ready Nav2-focused release.

- added Nav2-oriented launch flows with `nav2_lidar_localization.launch.py` and `nav2_navigation.launch.py`
- added smoke and replay helpers including `run_nav2_demo_smoke`, `run_nav2_replay_smoke`, and `send_nav2_goal.py`
- added occupancy map generation from PCD with `generate_occupancy_map_from_pcd.py`
- added benchmarking and comparison helpers for rosbag-based localization evaluation
- added local-map crop, twist-based prediction, borderline seed rejection gate, diagnostics, and replay-oriented utilities
- fixed `NDT_OMP` runtime keep-alive issues seen in replay and fixed the shutdown-only `SIGINT` crash path
- set the recommended Nav2 preset to `param/nav2_ndt_urban.yaml`

Known limitation at this release:

- long-duration urban replay robustness is improved enough for smoke and controlled validation, but not yet solved well enough to claim fully stable long-horizon operation in all cases
