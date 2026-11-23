# Changelog

## Unreleased

**Upgrading / behaviour change for anyone running `quickstart.py` with an odometry
TF:** when odometry prediction is on (`use_odom_tf_prediction: true`, whether quickstart
discovered the odom TF or you passed `--odom-tf-prediction`), three settings that were
opt-in are now on by default, under every profile (`standalone`, `nav2` and `mid360`
alike): `--seed-correction-window-sec 2.0` (which also sets the release agreement
`seed_correction_guard_release_agreement_scans: 10`), `--global-nms-radius-m 1.5` (3.0
before) and `--global-consensus-near-alias-samples 3` (0 before). Without odometry
prediction all three keep the old defaults (0, 3.0 m, 0). The config generator
(`create_lidar_localization_config.py`), `param/mid360_legged.yaml` and
`quickstart.launch.py` called directly keep the old values too. To get the old
behaviour back with odometry:

```bash
quickstart.py ... --seed-correction-window-sec 0 \
  --global-nms-radius-m 3 --global-consensus-near-alias-samples 0
```

Explicit flags always win. Quickstart prints the resolved values and their source
(`default with odometry prediction`, `default` or `command line`) on a `Settings:` line
and at the top of the generated parameter file. Costs: more starts ask for a manual pose
where every view is ambiguous (3 instead of 1 on the JEPLO Long_Stairs map), and the
release agreement can hold the odometry a few seconds longer before a right correction.
The evidence is from MID-360 sensors with RKO-LIO odometry only; other sensors and
odometry sources with odometry prediction get the same defaults but were not measured.

**Upgrading:** the new settings default to the old behaviour:
`seed_correction_guard_trusted_start_position_std_m` on the localizer (`0` = off), the
startup node's `global_pose_position_std_m` (`0` = `position_std_m`; launch arg of the
same name) and quickstart's `--trusted-global-start-std-m` (`0` = off).

### Changed

- **With odometry prediction, quickstart turns on the window guard, the 1.5 m candidate
  suppression and the near-alias rule** (see Upgrading above). Measured on v1.12.0 with
  the default (standalone) profile and RKO-LIO odometry prediction, the three settings
  against the old defaults (JEPLO Go2 and Koide replays, both a Livox MID-360): on the
  16 cross-session cases (32 runs) 29 localized, 1 of them wrong, and 3 asked for a
  manual pose, against 31, 3 and 1; the wrong one was the post-initialization pull the
  trusted global start addresses, none was a wrong G2 answer. The first pose came after
  a median 9.1 s instead of 10.8 s, and slides along the room fell from a p95 of
  0.6-1.8 m to 0.12-0.27 m. The Go2 EIL suite stayed 8 of 8 right (first pose 8.9 s
  instead of 11.4 s) and Koide outdoor_hard 4 of 4; on 01a the old defaults lost the
  pose at about 113 s, the new ones did not. The 1.5 m radius was evaluated only
  together with odometry, so without it the radius stays 3.0 m.
  `--global-search-engine bbs3d` and `--trusted-global-start-std-m` stay opt-in (bbs3d's
  first pose came after a median 16.6 s)
  ([benchmark.md](docs/benchmark.md#defaults-with-odometry-prediction)). The Go2
  quickstart page's commands run with odometry and now get these settings by default.
- The benchmark suites that run with odometry (all Go2 suites and `koide_outdoor.yaml`)
  now get these defaults, so their results are not directly comparable with earlier
  ones; add the three flags above to a suite's `quickstart_args` to compare with them.
  `go2_jeplo_cross_session.yaml` drops its now redundant `--global-nms-radius-m 1.5` and
  `--global-consensus-near-alias-samples 3` flags (it gains the window guard).
  `tools/benchmark/run_benchmark.py` records `/localization/pose_with_covariance` when a
  suite runs quickstart with `--profile mid360` or `nav2`, which publish the pose there
  instead of `/pcl_pose` (`pose_topic:` overrides).

### Added

- **Opt-in trusted global start** (`seed_correction_guard_trusted_start_position_std_m`
  on the localizer; quickstart `--trusted-global-start-std-m`, only with odometry
  prediction, which also sets the startup node's `global_pose_position_std_m`). On the
  JEPLO EIL_Stairs map with EIL_Box's scans, a G2 fix 0.05 m from the ground truth was
  pulled 0.6 m and then 1.9 m by the first two scans and the track locked onto the room
  turned around, 4.6 m off. The fix came as the Go2 stepped down (pitch 10 to 45
  degrees, 0.5 m lower, within 0.5 s): those scans registered 0.4-2.1 m off, with
  fitness down to 0.02, while the odometry stayed within 0.2 m, and the seed correction
  guards were still in their warmup, which the first 0.5 m correction restarted.
  Started from the ground truth the localizer was pulled 1.2 m as well, so neither the
  fix nor its seed was wrong. With the option the startup publishes an NDT-refined fix
  that odometry moved from its scan to now with that position standard deviation, and
  the localizer trusts an `/initialpose` claiming at most it: when the first scan after
  it is seeded from odometry, the guards hold from that scan. Replayed with the fix at
  the same moment: 2 of 6 runs ended 4.6 m off before, 0 of 4 after (max 0.26-0.35 m).
  RViz's 2D Pose Estimate (0.5 m), unrefined fixes, fixes odometry could not move and
  saved poses keep the warmup. A trusted fix that is off stays off until the guards'
  release, 30 scans later. Live, on the 14 cross-session cases with the window guard
  (two runs each): 27 starts, 1 wrong, 1 manual pose (25, 0, 3 before); the wrong start
  and the manual pose were G2's (the room turned around; no unambiguous view), and one
  fix G2 placed 0.7 m along the room was held for 3 s (p95 0.73 m in that run;
  elsewhere p95 at most 0.31 m, max 0.68 m). Off by default because of that cost.
  ([benchmark.md](docs/benchmark.md#eil_stairs--eil_box-a-right-start-pulled-onto-the-turned-room-and-the-fix),
  [troubleshooting.md](docs/troubleshooting.md#symptom-pose-jumps-along-a-corridor-like-axis)).

## 1.12.0 - 2026-11-23

**Upgrading:** every new setting defaults to the old behaviour:
`enable_seed_correction_window_guard` (`false`; `seed_correction_window_guard_sec` 1.0,
`seed_correction_window_guard_translation_m` 0.3; quickstart
`--seed-correction-window-sec` 0 = off), `seed_correction_guard_release_agreement_scans`
(`0`, release as before; quickstart sets 10 only together with
`--seed-correction-window-sec` and odometry prediction), `global_search_engine` on the
G2 node (`bbs2d`; launch `g2_global_search_engine`, quickstart
`--global-search-engine`) and the startup node's `global_consensus_near_alias_samples`
(`0` = off). `global_search_engine: bbs3d` uses the new `bbs3d_cpp` module, built from a
vendored subset of 3D-BBS (MIT, `third_party/bbs3d`, declared in `package.xml`) whenever
pybind11 and the Boost hash headers are found; no new dependency. Four behaviours
change by default:

- `param/mid360_legged.yaml` and the `mid360` quickstart profile set
  `imu_accel_scale: 9.80665`. If your MID-360 driver publishes acceleration in m/s^2
  instead of g (`livox_ros_driver2` publishes g), set it back to `1.0`: the localizer
  does not detect the unit.
- Global initialization with NDT-refined answers (`--refine-global-candidates` or the
  sparse-band search) accepts two of them only within 0.5 m and 10 degrees of each
  other, and rejects a refined top answer that registers less than twice as well as a
  refined candidate 1 m away or turned 20 degrees. A start may need one or two more
  queries. The second check turns off with a negative
  `registration_refined_alternative_min_separation_m` or `_min_yaw_deg` on the startup
  node; the 0.5 m and 10 degree bounds are startup-model defaults that the node does
  not expose as parameters.
- Quickstart keeps looking for an odometry node for up to 3 s after the cloud and IMU
  appear, so a start without odometry takes up to 3 s longer; `--base-frame` with
  `--odom-tf-prediction` or `--no-odom-tf-prediction` skips the wait.
- `tools/benchmark/run_benchmark.py` waits up to 30 s for the odom TF of a suite that
  launches odometry, and marks a run that still configured no odometry invalid (left
  out of the scores, benchmark exit code 1); `expect_odometry: false` turns this off.

### Changed

- **Global initialization rejects aliased NDT-refined answers.** With
  `--refine-global-candidates` or the sparse-band search (Unitree Go2), two answers must
  agree within 0.5 m and 10 degrees instead of 2 m and 20
  (`global_consensus_refined_translation_m`, `global_consensus_refined_yaw_deg`:
  defaults of the startup model, not declared as node parameters), and
  the top must register at most half as well as the best refined candidate 1 m away or
  turned 20 degrees, not only one 5 m away
  (`registration_refined_alternative_min_separation_m`, `_min_yaw_deg`). G2 answers mark
  such candidates `registration_refined`. On the Go2 cross-session replays this took
  wrong starts from 4 of 28 runs to 0, with 24 right starts instead of 20, the same 4
  manual poses, and the first pose after a median 11.0 s instead of 10.3 s; replaying
  G2 answers from 889 start points, wrong starts fell from 154 to 41 and manual poses
  rose from 29 to 98 ([benchmark.md](docs/benchmark.md#why-the-wrong-starts-passed-and-the-fix)).
  Unrefined answers are checked as before; negative values turn either check off
  (on the node, only the alternative check's two parameters can be set).

### Added

- **Opt-in near-alias startup rule and settings for a map from another session**
  (`global_consensus_near_alias_samples` on the startup node, default 0 = off;
  quickstart `--global-consensus-near-alias-samples`). On the JEPLO EIL_Stairs map with
  EIL_Box's scans, 4 of 8 live starts were wrong (two of them 1.8 m along the room).
  The G2 search keeps only the best pose within `nms_radius_m` on the 2D grid, before
  NDT refinement (quickstart: 3 m); in the capture room a place 1-2 m along the room
  often scores higher on the grid, so the right pose was never refined although it
  registered 4.5 times better (42 of 120 scans). `--global-nms-radius-m 1.5` keeps it:
  over the 15 cross-session pairs the top was right for 72% of the scans instead of
  64%, queries took 0.46 s instead of 1.16 s (median), and the replayed startup had 5
  wrong starts instead of 52 of 995, but 186 manual poses instead of 108, because the
  nearby-alternative check now sees the place along the room next to right answers.
  The new rule accepts a refined fix rejected only by an alternative at the same
  heading once three such or gated fixes agree through odometry (the room turned
  around, or a place 5 m away, still rejects): 5 wrong, 833 right (835), 157 manual.
  Live, EIL_Stairs → EIL_Box: 7 of 8 runs right, none wrong (4 of 8 wrong before); the
  other 14 cross-session cases: 25 of 28 right, none wrong, 3 manual poses, as before,
  first pose after a median 9.3 s instead of 11.6 s
  ([benchmark.md](docs/benchmark.md#eil_stairs--eil_box-why-starts-were-wrong-and-the-fix)).
  The Go2 docs recommend both for a map from another session
  ([go2_mid360_quickstart.md](docs/go2_mid360_quickstart.md#a-map-from-another-session));
  the cross-session suite uses them and has the EIL_Stairs → EIL_Box case. Defaults
  are unchanged.

- **Opt-in 3D global search for multi-level maps** (`global_search_engine: bbs3d` on
  the G2 node, default `bbs2d`; launch arg `g2_global_search_engine`; quickstart
  `--global-search-engine bbs3d`). The map-wide search runs as a 3D branch-and-bound
  over the voxel maps of [3D-BBS](https://github.com/KOKIAOKI/3d_bbs) (MIT, a small
  CPU subset vendored in `third_party/bbs3d` with its licence) instead of on the 2D
  occupancy grid, so a scan on the stairs or in the hall below a floor is not placed
  on another level. It returns the 8 best poses at least 3 m apart
  (`bbs3d_top_k`, `bbs3d_min_separation_m`), which the existing NDT refinement,
  ranking and startup alternative checks then use; each search stops after 10 s
  (`bbs3d_timeout_sec`) or 4 M open nodes (`bbs3d_max_queue_nodes`), and a query that
  arrives while one runs is dropped. The voxel maps are built once per map load
  (0.1 s and a few MB for the JEPLO maps; the node logs the build time and RSS). It
  costs CPU (median 2-10 s per query on 4 threads against about 1-3 s for bbs2d) and
  does not help with rooms aliased by 180 degrees. Built as the `bbs3d_cpp` module
  whenever pybind11 is found (no new dependency); when the module or `map_path` is
  missing the node logs an error and answers with bbs2d. Results and costs:
  [global_localization.md](docs/global_localization.md#using-3d-bbs-global_search_engine-bbs3d).
- **Opt-in windowed seed correction guard** (`enable_seed_correction_window_guard`,
  `seed_correction_window_guard_sec`, `seed_correction_window_guard_translation_m`;
  quickstart `--seed-correction-window-sec`, only with odometry prediction). It rejects
  a scan when the corrections accepted within the window plus its own would move the
  pose more than 0.3 m (horizontally) away from the odometry, reporting
  `seed_correction_window_guard_rejected`; it shares the seed correction guard's warmup
  and release. On the Go2 cross-session replays a run of degraded scans, each within
  the per-scan guard, slid the pose 1-2.7 m along the room's weak axis, after which the
  per-scan guard rejected the way back. From the true pose, with a 2 s window, the
  maximum error on the Long_Stairs map fell from 1.8-2.7 m to 0.40 m (EIL_Box) and from
  1.3-2.2 m to 0.55-0.90 m (EIL_Stairs); live, every slide over 1 m was gone except one
  EIL_Box → EIL_Stairs run of four, where an odometry jump and the per-scan guard's
  release still ended 3.1 m off. Off by default
  ([benchmark.md](docs/benchmark.md#why-tracking-slid-along-the-room-and-the-fix)).
- **Opt-in release agreement for the seed correction guards**
  (`seed_correction_guard_release_agreement_scans`, `_translation_m`, `_yaw_deg`;
  quickstart: 10 scans, 0.3 m, 5 degrees with `--seed-correction-window-sec`). After 30
  rejections in a row the guards accepted whatever the registration returned on that
  scan; on the EIL_Box map an EIL_Stairs scan sequence aliases 2.4 m away for about a
  second, and the release that fell inside it ended 3.1 m off. Now a released answer is
  accepted only when its correction agrees with each of the last N rejected ones;
  otherwise the guards keep the odometry and report
  `seed_correction_guard_release_disagreed`. With forced 0.7 m odometry jumps the
  release fell on an aliased answer in three runs; it held all three, two then released
  onto the right pose (0.01 m off) and the third lost the pose when RKO-LIO itself
  diverged. Holds of right answers cost 0.1-1.8 s. On the live cross-session suite the
  hold engaged in two runs only, so it is otherwise unchanged (EIL_Box → EIL_Stairs never
  reached a release in 4 new runs). 0 (the default) releases as before
  ([benchmark.md](docs/benchmark.md#why-a-release-locked-onto-an-aliased-answer-and-the-fix)).

### Fixed

- **Benchmark runs no longer start without the odometry they launch.** Quickstart's
  discovery decided on the ROS graph as soon as the bag's cloud and IMU appeared; DDS
  reported the already running RKO-LIO to it up to a second later, so 7 of 66 Go2
  cross-session runs of the #345/#346 replays (all 3 "manual" starts of #345) started
  with `Discovery: odometry=none` and scored the no-odometry mode (search on the raw,
  upside-down MID-360 scan). `run_benchmark.py` now waits for the odom TF on /tf before
  it starts quickstart (`tools/benchmark/wait_for_odometry.py`, 30 s,
  `odometry_tf_timeout_sec`; this also covers a bag that starts late from a cold disk),
  passes `--odom-tf-prediction --base-frame <child of odom>` unless the case sets them,
  and marks a run that still configured no odometry invalid (`invalid.txt`): it is
  stopped, left out of the scores, listed in `summary.md`, and the benchmark exits
  with 1. `expect_odometry: false` in a suite turns this off. 30 of 30 cross-session
  starts (the first 40 s of each case, twice) then ran with odometry: 28 right, 0
  wrong, 2 manual (ambiguous Long_Stairs pairs).
- **Quickstart keeps looking for an odometry node for up to 3 s** after the cloud and IMU
  appear, instead of a single second look, so odometry started together with or just
  before quickstart is found. Replaying the benchmark's start-up 24 times, 3 starts
  missed a running RKO-LIO before (a fourth started before a cold bag played, which only
  the benchmark's TF wait covers) and none after. Without odometry the start takes up
  to 3 s longer; passing `--base-frame` with `--odom-tf-prediction` or
  `--no-odom-tf-prediction` skips the wait ([quickstart.md](docs/quickstart.md)).
- **`mid360_legged.yaml` and the `mid360` quickstart profile read the MID-360 IMU as
  m/s^2**: `livox_ros_driver2` publishes g, and only `mid360_legged_low_resource.yaml`
  converted it. On EIL_Stairs (RKO-LIO running) the RMSE fell from 0.20-0.27 to
  0.12-0.14 m; a bag already in m/s^2 (Koide) needs `1.0`
  ([benchmark.md](docs/benchmark.md#imu-acceleration-units)).

### Documentation

- **Running on small computers** ([troubleshooting.md](docs/troubleshooting.md#running-on-small-computers)):
  the low-resource knobs as a platform-neutral list with their measured effect on a
  Go2, a handheld MID-360 and a Velodyne vehicle, and when they are unsafe. On Boreas
  (no odometry) only a coarser scan voxel saved CPU (2.0 m: 10-20% for 2% RMSE); fewer
  threads and iterations saved none, and the pose timer raised the RMSE to 0.55 m
  ([benchmark.md](docs/benchmark.md#boreas-with-the-small-computer-knobs)).

### Validation

- The release check: two full Koide `outdoor_hard_02b` replays at 1x passed on
  `release/v1.12.0` (`main` at `f500a59` plus the version bump), built without a build
  type, with ndt_omp_ros2 at `17ce1e1`, as for v1.11.0.
  - **Accuracy:** 0.242 m RMSE in both (0.170 m on the map), against 0.238-0.244 m
    (0.171 m) for v1.11.0.
  - **Coverage:** 100.0% diagnostic coverage in both.
  - **Alignment:** 18 ms at the median and 44-46 ms at p95, against 14-15 and 36-37 ms,
    **on a host CPU at 2.10 GHz where v1.11.0's was 2.30 GHz**. v1.11.0 rebuilt and
    replayed twice on this host measured 18 and 44-47 ms (0.242 m), so the difference is
    the host, not the release
    ([docs/koide-online-replay.md](docs/koide-online-replay.md#v1120-release-verification-2026-11-23),
    `docs/validation/v1.12.0-koide-20261123`).
  - **Resources:** the localizer used 0.56-0.57 cores on average (v1.11.0 on this host:
    the same) and held 207-210 MB anonymous memory in steady state.
- Boreas, `param/boreas_ndt_velodyne.yaml` on the release build: 0.132 m RMSE over the
  first 120 s, 0.127 m with `use_imu_preintegration: true`, every scan accepted, as on
  1.11.0.
- The opt-in features of this release (G2 `bbs3d`, the window guard, release agreement,
  the near-alias rule) are off in both checks; their results are the Go2/JEPLO replays
  cited above, not re-run for the release.

## 1.11.0 - 2026-11-18

**Upgrading:** no configuration needs changing on Humble or Jazzy, and every new
setting defaults to the old behaviour: `cloud_reliable_qos` (`false`; Foxy only,
ignored with a warning elsewhere), `enable_stale_reset_replay` (`false`) and
`imu_smoother_window_size` (`50`). `param/mid360_legged_low_resource.yaml` is a new
preset and needs odometry. ROS 2 Foxy is now a best-effort target that builds from
`main` (ndt_omp_ros2 needs its `humble` branch at rsasaki0109/ndt_omp_ros2#18 or later
for PCL 1.10). The fitness score's compact kd-tree gives the same scores with less
memory. One behaviour differs: a scan seeded from the odometry TF is no longer aligned
again from the IMU prediction after a rejection; scans without an odometry seed keep
that retry. The `foxy`, `humble` and `jazzy` branches are gone: build every
distribution from `main`, or check out the tags `archive/foxy`, `archive/humble` and
`archive/jazzy` for their last commits.

### Added

- **Cross-session results on the Go2 (JEPLO)**: the map from one recording, the scans
  from another, no initial pose ([benchmark.md](docs/benchmark.md#cross-session-go2-jeplo),
  `tools/benchmark/suites/go2_jeplo_cross_session.yaml`). Long_Stairs and Outdoor2
  (same route, 1.5 h apart) localized each other in all 4 runs, 0.02 m off. In the
  capture room the EIL_Mix map localized four other runs in all 8 tries; the EIL_Box
  map started 4 of 6 runs 1-5.5 m off, and the Long_Stairs map let tracking slide up to
  2.5 m where the props had moved. The README GIF now uses the Long_Stairs map with
  the Outdoor2 scans instead of a map from the same recording.
  `jeplo_to_pc2_bag.py --duration` keeps the first seconds of a bag.
- **`param/mid360_legged_low_resource.yaml`**, for a Jetson Orin Nano/NX on a
  Unitree Go2 or G1-class robot with a MID-360: `mid360_legged.yaml` with a 0.5 m
  scan voxel, two NDT threads, 15 iterations, odometry TF seeds, no pose timer and
  the MID-360's acceleration in g. It needs odometry. With the localizer pinned to two
  x86 cores and RKO-LIO beside it, on JEPLO EIL_Stairs (Go2, motion capture) it took
  about 0.2 cores instead of 0.27 at the same error (0.067 m against 0.077 m), and on
  Koide outdoor_hard_02b 0.45-0.50 cores instead of 0.70 (0.23 m against 0.25 m).
  Without odometry the coarse scan locked onto the wrong stair level, so keep
  `mid360_legged.yaml` there
  ([mid360_legged_jetson.md](docs/mid360_legged_jetson.md#low-resource-preset),
  [benchmark.md](docs/benchmark.md#low-resource-budget-go2-mid-360)).
- **`imu_smoother_window_size`** (default `50`, as before): the scans the IMU smoother
  re-optimizes on every update, a quarter of the node's CPU with a 0.5 m scan. 20
  saved 18% of the CPU on the Go2 bag, but 20 and 30 lost the pose for 40 s or more
  on the Koide outdoor bag, so no preset lowers it.
- **The replay harnesses report CPU and memory**: `boreas_replay.py`,
  `soak_replay.py` and `run_koide_public_bag.py` sample the localizer's CPU time
  from `/proc` next to its memory, and `summary.json` gives the cores in use (mean
  and p95), CPU per second of data and per scan, peak and steady resident and
  anonymous memory, pose latency, and the scans processed and accepted.
  `boreas_replay.py` replays any fixture of the same layout (`--points-topic`,
  `--imu-topic`, `--base-frame`), runs the localizer under `--wrap COMMAND` (e.g.
  `taskset` or a cgroup) and starts `--companion` nodes such as an odometry, whose
  resources it samples too; `run_koide_public_bag.py` takes `--preset`, `--set` and
  `--wrap`.

- **ROS 2 Foxy builds, passes its tests and runs**, as a best-effort target (Foxy is
  end-of-life); CI builds and tests it. Humble and Jazzy build and behave as before.
  On Foxy:
  - ndt_omp_ros2 builds with Foxy's PCL 1.10 from its `humble` branch since
    rsasaki0109/ndt_omp_ros2#18.
  - The cloud subscription keeps the best-effort sensor QoS by default: Foxy has no
    `qos_overrides.<topic>.subscription.reliability` parameter. The ~1 MB clouds of the
    Boreas replay then lose fragments in transport, and only about 970 of 1158 scans
    reach the node. Set the new `cloud_reliable_qos: true` to subscribe reliably there
    (all 1158 scans arrive, same RMSE). Humble and Jazzy ignore the parameter, with a
    warning, and keep using `qos_overrides`.
  - The node uses a small TF listener of its own in place of Foxy's, which can crash
    or hang when the node is destroyed.
  - `run_koide_public_bag.py` and the other helpers that read bags need `rosbag2_py`,
    which Foxy does not have. The `tools/benchmark` replays use
    `ros2 bag play --clock`, which Foxy's rosbag2 lacks, and Foxy cannot read the
    metadata of bags written by newer rosbag2 releases.

- **A reset older than the scan it is applied to is tracked up to that scan**, opt-in
  with the localizer parameter `enable_stale_reset_replay` (default `false`; also
  `stale_reset_replay_min_age_sec`, `0.5`, and `stale_reset_replay_max_age_sec`,
  `20.0`). A global localization answer reaches the localizer seconds after its scan,
  metres behind a moving vehicle, which restarted there with no velocity and rejected
  every later scan. The localizer now keeps its recent scans, tracks from the reset
  through them as in normal tracking, and moves the reset to the newest scan with the
  velocity tracked on the way; a replay that loses track leaves the reset as it was.
  On the Boreas closed loop with the fast settings, 5 of 10 stops of 10 s recovered,
  against none before, with no wrong pose accepted
  ([global_localization.md](docs/global_localization.md)).

### Changed

- **The fitness score's search tree no longer copies the map.** The fitness and the
  inlier ratio were scored on PCL's FLANN kd-tree over the registration target, which
  holds two copies of its coordinates and an index mapping: about 100 MB for the
  2.0M-point Koide outdoor map. `CompactKdTree` indexes the target in place (13 MB
  there) and returns the same squared distances, so the scores, the gate and the
  trajectories do not change. Map loading also frees the file as read before the target
  is built and no longer copies it for `/initial_map` or converts an xyz map through a
  temporary cloud. With the localizer pinned to two cores
  ([benchmark](docs/benchmark.md#memory-and-cpu-in-code)): on Koide outdoor_hard_02b the
  node holds 218 instead of 300 MB anonymous (272 instead of 356 MB resident) and runs
  in a 224 MB memory cgroup, where it was killed at 256 MB before; on JEPLO EIL_Stairs
  (a 580k-point map) 44 instead of 77 MB; on the Boreas fixture (local map crops) 272
  instead of 389 MB anonymous after the start.
- **The IMU smoother solves its window about four times faster**: by blocks along the
  chain of poses with a Schur complement for the biases, instead of a general sparse
  LDLT, with each IMU factor's information computed once. The poses and biases are the
  same to rounding. For `mid360_legged_low_resource.yaml` on EIL_Stairs the node
  needs 0.145 instead of 0.20-0.22 cores, on Koide outdoor_hard_02b 0.40 instead of
  0.51 (with the change above).
- The `foxy`, `humble` and `jazzy` branches are removed: every supported
  distribution builds from `main`. Their last commits are kept as the tags
  `archive/foxy`, `archive/humble` and `archive/jazzy`. The frozen `dashing`
  branch stays for ROS 2 Dashing.

### Fixed

- **A scan seeded from the odometry TF no longer retries from the IMU prediction.**
  After a rejected alignment the node aligned the scan again from the IMU state
  dead-reckoned since the last accepted scan, up to 10 s back. On Koide
  outdoor_hard_02b, 9.9 s into a rejection streak whose odometry seeds were 0.2-0.3 m
  off, that retry came 10-35 m off with a fitness under the threshold and was accepted
  in 8 of 13 runs of the MID-360 presets; the pose then stayed metres off for 20 s or
  more. This, not the window, was why a shorter `imu_smoother_window_size` "lost the
  pose" there: with the retry gone 20 scans held too. Scans without an odometry seed
  keep the retry.

### Validation

- The release check: two full Koide `outdoor_hard_02b` replays at 1x passed on
  `release/v1.11.0` (`main` at `190099e` plus the version bump), built without a build
  type, with ndt_omp_ros2 at `17ce1e1` (its 0.2.0 release plus PCL 1.10 support).
  - **Accuracy:** 0.244 and 0.238 m RMSE (0.171 m on the map in both), against 0.242 m
    (0.170-0.171 m) for v1.10.0.
  - **Coverage:** 100.0% diagnostic coverage in both.
  - **Alignment:** 14-15 ms at the median and 36-37 ms at p95, against 16 and 39-41 ms,
    **on a host CPU at 2.30 GHz where v1.10.0's was 2.10 GHz**, so the times are not
    like-for-like; v1.9.0 measured 13 and 33-34 ms on this CPU model
    ([docs/koide-online-replay.md](docs/koide-online-replay.md#v1110-release-verification-2026-11-18),
    `docs/validation/v1.11.0-koide-20261118`).
  - **Resources:** the localizer used 0.46-0.47 cores on average and held 209-211 MB
    anonymous memory in steady state.
- Boreas, `param/boreas_ndt_velodyne.yaml` on the release build: 0.132 m RMSE over the
  first 120 s, 0.127 m with `use_imu_preintegration: true`, every scan accepted, as on
  1.10.0.

## 1.10.0 - 2026-11-13

**Upgrading:** no configuration needs changing, and every new setting defaults to the
old behaviour. One behaviour differs for a deployment that runs the reinitialization
supervisor (`global_localization_recovery.launch.py`): it now ends its episode on an
`/initialpose` that it did not publish, so a reset by an operator or another node
stops a pending recovery instead of being followed by the supervisor's own reset.

### Added

- **Three settings answer a global localization query in 2 s instead of 19 s** on the
  Boreas fixture: `g2_ndt_local_map_radius:=0.0`, `g2_ndt_num_threads:=4` and
  `g2_ndt_scan_max_range:=80.0` on `global_localization_recovery.launch.py` (node
  parameters `ndt_local_map_radius`, `ndt_num_threads` and the new
  `ndt_scan_max_range`). The defaults are unchanged.
  - **Radius 0** prepares the whole map's scoring target once at startup instead of a
    crop per candidate on every query, which took 13-14 of the 19 s. The node holds
    about 240 MiB more on Boreas.
  - **The range cap** scores only scan points within it. With every point the true
    pose's fitness was 300-770, so no answer was ever published; within 80 m it is
    0.05-0.21.
  - **Over 16 scans with no prior**, with `g2_registration_refine_candidates:=true`: 8
    right answers, none wrong and 8 not published, where the defaults published none.
    The map-wide search still misses the right pose for those 8
    (`docs/global_localization.md`).
  - The fitness score of the default crop path is unchanged, and it is now threaded
    too.
  - **In the closed loop they do not recover a 10 s stop on Boreas,** 0 of 10: every
    published answer was right for its scan and 12.6-134 m behind the vehicle when
    applied. They stay opt-in.
- **`run_koide_public_bag.py --stall START_SEC:DURATION_SEC`** freezes only the
  localizer for DURATION_SEC, while RKO-LIO and the player run, and scores the stop
  into `summary.json` as `tools/benchmark/soak_replay.py` scores a stall.
  - **With odometry a 10 s stop recovers.** On `outdoor_hard_02b`, the first scan is
    accepted 0.2-1.0 s after a 10 s stop, in a turn and on a straight, seeded from the
    odometry transform 13-18 m from the last accepted pose. No run requested
    reinitialization, and no pose was more than 0.54 m off
    (`docs/global_localization.md`).
  - A replay that ends while a process is frozen now resumes it before stopping it.

### Fixed

- **The reinitialization supervisor no longer resets a localizer that is tracking**
  after someone else's `/initialpose`. It ends its episode on an `/initialpose` it did
  not publish (reason `external_reset` in the event log), as the localizer already
  treats that reset as authoritative.
  - **What happened:** on the looped Boreas fixture, a check query answered on a scan
    from before an operator's reset set `alias_confirmed`. The episode kept querying
    with no request raised, and 28 s later reset a localizer tracking at 96% inliers
    to a pose 12.6 m behind it; the track was lost for the rest of the lap.
  - A request still raised after the operator's reset starts a new, debounced episode
    with a fresh attempt count.
- **`tools/benchmark/soak_replay.py` scores a fault only on the poses of aligned
  scans.** The pose the localizer republishes on `/initialpose` counted as tracking,
  so five stops after which no scan was accepted scored as recovered in 21-50 s. The
  fault scores in `docs/benchmark.md` do not change: 24 saved runs re-score the same.

### Validation

- The release check: two full Koide `outdoor_hard_02b` replays at 1x passed on
  `release/v1.10.0` at `8d7183e` (`main` at `f4f5213` plus the version bump), built
  without a build type, with ndt_omp_ros2 at its 0.2.0 release. The localizer
  executable is byte-identical to 1.9.0's.
  - **Accuracy:** 0.242 m RMSE in both (0.170-0.171 m on the map), against 0.240-0.242 m
    (0.170 m) for v1.9.0.
  - **Coverage:** 100.0% diagnostic coverage in both.
  - **Alignment:** 16 ms at the median and 39-41 ms at p95, against 13 and 33-34 ms,
    **on a host CPU at 2.10 GHz where v1.9.0's was 2.30 GHz**, so the times are not
    like-for-like
    ([docs/koide-online-replay.md](docs/koide-online-replay.md#v1100-release-verification-2026-11-13),
    `docs/validation/v1.10.0-koide-20261113`).
- Boreas, `param/boreas_ndt_velodyne.yaml` on the release build: 0.132 m RMSE over the
  first 120 s, 0.127 m with `use_imu_preintegration: true`, every scan accepted, as on
  1.9.0.

## 1.9.0 - 2026-11-10

**Upgrading:** 1.9.0 removes ten features that no preset turned on, among them the
twist EKF (see Removed). Their switches stay declared, and configuring one of them
`true` now fails with a message that names it; their thresholds are no longer
declared, and ROS ignores them. A configuration that left them off is unaffected.
`lidar_localization_node` now limits glibc to two malloc arenas unless
`MALLOC_ARENA_MAX` is set; a component loaded into another container needs it set in
that container's environment.

### Changed

- **`global_localization_recovery.launch.py start_localization:=false`** starts only the
  global localization node and the supervisor, beside a localizer already running.
  `tools/benchmark/soak_replay.py --companion COMMAND` starts such a command once the
  soaked localizer is active, in its ROS domain. On the Boreas fixture, after a 10 s
  stop, the recovery nodes did not recover the track: their map-wide search missed and
  each answer took 18-21 s (`docs/global_localization.md`).
- **The node frees a replaced registration target before it builds the next one, and
  returns freed memory to the OS.** On the Boreas fixture with the preset (80 m crop,
  `use_imu_preintegration: true`):
  - **Peak:** 525-539 MiB instead of 613-652 MiB in one lap, alternated with the build
    before, at the same RMSE and alignment time. Pose latency at p95 is 58-59 ms
    instead of 55-57 ms.
  - **An hour under 800 MB:** 576 MiB at most instead of 765 MiB. The lowest memory of
    each lap stays at 329-339 MB, where it rose by 84 MB an hour.
  - **Why:**
    - During a background rebuild three crops were alive: the one in use, the replaced
      one with its search tree, still held by the spare registration objects, and the
      new one. The next target is now built on fresh objects, and the replaced one is
      freed first.
    - glibc returned only the top of its heaps. `malloc_trim` now runs once a target
      change freed memory, at the end of that scan's callback (1.4 ms at the median).
- **`param/boreas_ndt_velodyne.yaml` crops the map at 80 m instead of 100 m.** The node
  peaks 75-160 MiB lower (642-707 MiB against 782-803 MiB in one lap) at the same RMSE,
  0.132 m, or 0.127 m with `use_imu_preintegration: true`, and the same pose latency.
  The 20 m between the crop and the 60 m scans still lets the next crop be built in the
  background. With the 80 m crop the node ran an hour under an 800 MB memory limit;
  with the 100 m one it needed more than 768 MB.
- **`docs/troubleshooting.md` has a section for a node killed for memory:** the crop
  radius against the scan range, `viz_downsample`, a coarser map and `MALLOC_ARENA_MAX`,
  measured on Boreas.
- **`tools/benchmark/soak_replay.py` reports each lap's pose latency**, from publishing a
  cloud to receiving its pose, and the poses later than 200 ms.
- **The node holds 0.6-0.8 GB instead of 1.5-2.2 GB** on the looped Boreas fixture
  (`tools/benchmark/soak_replay.py`, with `use_imu_preintegration: true`), at the same
  alignment times. Nothing leaked before; the memory was kept.
  - **No older registration targets are kept alive**
    (`registration_target_cloud_keep_alive_count` defaults to 0, from 8). The
    registration objects hold their targets through shared pointers. Under
    AddressSanitizer, three laps with a re-initialization while moving and a LiDAR gap
    reported nothing, and the node shut down cleanly.
  - **The local-map crop is sized to its points.** Grown point by point, a crop of up
    to 1.4 million points kept 2.5 million points' worth of memory (79 MB).
  - With both, the heap in use peaks at 0.69 GB instead of 1.34 GB.
  - **`lidar_localization_node` limits glibc to two malloc arenas** unless
    `MALLOC_ARENA_MAX` is set. With one arena per thread, about 40 threads kept up to
    2.2 GB resident whatever the heap held. A component loaded into another container
    needs `MALLOC_ARENA_MAX=2` in that container's environment.
- `param/boreas_ndt_velodyne.yaml` sets `min_inlier_ratio: 0.8`.
  - **Why:** after LiDAR gaps of 2 s and stops of 10 s injected with
    `tools/benchmark/soak_replay.py`, the preset accepted poses 8.6-117 m off. They
    passed `score_threshold: 6` with only 50-74% of the points within 0.5 m of the map,
    against 96% at the median when tracking (85% at the 1st percentile). With the gate
    the node rejects them and requests reinitialization 15-21 s later.
  - **Tracking is unchanged:** over the 120 s fixture the trajectory is byte-identical.
    With `use_imu_preintegration: true` the smoother weighs scans by the gate's score,
    and the poses move by 3 cm at most (RMSE 0.127 m either way).

### Fixed

- **A node stopped for 2 s recovers** with `use_imu_preintegration`. On the looped
  Boreas fixture it published no pose for the rest of the lap; now it is back within
  0.06 s and 0.27 m, in a turn and on a straight.
  - **Why:** the IMU samples queued for a stopped node overflow, and the gap stayed in
    every window from the last accepted scan, so the IMU prediction was never made
    again. For the retry from the IMU, the sample before a gap is now held across it
    (up to the retry's 10 s).
  - Held across a tightening turn, that prediction was 8-14 deg short in yaw and
    rejected. A rejected bridged retry is aligned again turned by +-6 and +-12 deg,
    within 3 s of the last accepted scan, and the best that passes the gate is taken.
    Turned the wrong way, no fit passed 63% inliers against the 80% the gate needs.
  - After a 10 s stop the node still requests reinitialization.
- **A rejected scan past the IMU window is aligned again from the IMU prediction.**
  With `use_imu_preintegration`, a scan more than 1 s after the last accepted one is
  seeded from the previous scan's motion, which advances at most 1 s. When that
  alignment is rejected and the IMU covers the window, up to 10 s, the scan is aligned
  once more from the IMU prediction, before the retry from the last pose.
  - **On the looped Boreas fixture** (`tools/benchmark/soak_replay.py`, with
    `min_inlier_ratio: 0.8`), LiDAR gaps of 2 s and 5 s now recover on the first scan
    after them, within 0.36 m, where every later scan was rejected.
  - **The 1 s rule stays for the first alignment.** Seeding from the IMU for up to 10 s
    instead recovered the Boreas gaps too, but on Koide `outdoor_hard_01a` without
    odometry, with a handheld MID-360's IMU and the gate, it lost the track at 48.6 s
    instead of 140.7 s: after short rejection streaks its prediction had drifted where
    the previous scan's motion recovers. With the retry the track holds to 148.9 s.

### Removed

- **Three odometry-bridge options that no preset turned on:**
  `constrain_odom_tf_prediction_to_planar`, `constrain_odom_tf_prediction_height_only`
  and `enable_map_odom_anchor_fitness_gate` (with `map_odom_anchor_max_fitness` and
  `map_odom_anchor_max_correction_rotation_deg`). No preset, launch file, script or doc
  set them.
  - The three switches stay declared, so configuring with one of them `true` now fails
    with a message that says so, as `use_imu` does since 1.8.0
    (`include/lidar_localization/removed_parameters.hpp`). The two thresholds are no
    longer declared, and ROS ignores them.
  - `level_map_to_odom` stays: `docs/troubleshooting.md` measures it on Koide.
  - With the defaults nothing changes: the Boreas trajectory is byte-identical.
- **Four measurement-gate options that no preset turned on:**
  - `enable_odom_tf_prediction_correction_guard` (with
    `odom_tf_prediction_correction_guard_translation_m` and `_yaw_deg`): rejected a scan
    whose correction from the odometry seed was too large.
  - `enable_odom_tf_prediction_recovery_correction_guard` (with
    `odom_tf_prediction_recovery_min_rejections`, `_max_fitness`,
    `_guard_translation_m` and `_guard_yaw_deg`): widened that guard after a rejection
    streak.
  - `enable_soft_odom_correction_gate`: blended a scan that guard rejected with the
    odometry prediction. `docs/imu_estimation.md` called it experimental and untested
    on real data.
  - `enable_consistency_recovery_gate` (with `consistency_recovery_min_rejections`,
    `_score_margin`, `_max_translation_m` and `_max_yaw_deg`): accepted a scan over the
    score threshold after a rejection streak when its correction was small.
  - Their switches are refused when `true`, like the three above. `/alignment_status`
    no longer reports `soft_odom_correction_gate_enabled`.
  - `enable_seed_correction_guard` stays: the quickstart turns it on with odometry
    prediction.
- **The twist EKF** (`use_twist_ekf`, with the seven `ekf_sigma_*` and three
  `ekf_fitness_*` parameters): a constant-velocity EKF on the twist that seeded NDT and
  set the published covariance.
  - **Why:** the docs called it experimental. On the graded and fast Istanbul windows
    it lost track in three of six runs, where `use_gtsam_smoother` kept all of them, and
    they recommended the smoother instead. No preset turned it on; four set it to
    `false`.
  - It took its 369-line filter, a seed source, a pose backend and a covariance mode with
    it. `use_twist_ekf: true` is refused at configure.
- **Two more gates that no preset turned on:**
  - `enable_localizability_guard` (with `localizability_min_xy_eigen_ratio`): dropped the
    previous-delta seed when the scan's XY spread was narrow. The docs called it an
    experimental proxy. `horizontal_localizability_eigenvalue_ratio` stays in
    `/alignment_status`; `localizability_guard_enabled`, `localizability_guard_active`
    and the `localizability_guard_current_pose` seed source go.
  - `enable_open_loop_strict_score_threshold` (with
    `open_loop_strict_min_accepted_gap_sec`, `_min_seed_translation_m` and
    `_score_threshold`): lowered the score threshold once no scan had been accepted for a
    while and the seed had moved far. No doc mentioned it. `/alignment_status` no longer
    reports `open_loop_strict_score_threshold_active`, and
    `create_lidar_localization_config.py` drops its four `--open-loop-strict-*` flags.
  - Both switches are refused when `true`.
  - `enable_registration_localizability_diagnostics` stays: it only reports, and
    `tools/benchmark/record_pose.py` records its output. `enable_local_reacquisition`
    stays: `docs/troubleshooting.md` gives it as the fix for a lost odometry-bridged
    pose and measures it on Koide.

### Validation

- The release check: two full Koide `outdoor_hard_02b` replays at 1x passed on
  `release/v1.9.0` at `8dc235f` (`main` at `028a0e2` plus the version bump), built
  without a build type, with ndt_omp_ros2 at its 0.2.0 release.
  - **Accuracy:** 0.242 and 0.240 m RMSE (0.170 m on the map), against 0.242-0.245 m
    (0.171 m) for v1.8.0.
  - **Coverage:** 100.0% diagnostic coverage in both.
  - **Alignment:** 13 ms at the median and 33-34 ms at p95, against 12-13 and 32-33 ms,
    on the same CPU model
    ([docs/koide-online-replay.md](docs/koide-online-replay.md#v190-release-verification-2026-11-10),
    `docs/validation/v1.9.0-koide-20261110`).
- Boreas, `param/boreas_ndt_velodyne.yaml` on the release build: 0.132 m RMSE over the
  first 120 s, 0.127 m with `use_imu_preintegration: true`, every scan accepted, as on
  1.8.0.

## 1.8.0 - 2026-11-07

### Added

- `min_inlier_ratio` (off at 0) gates on the share of scan points within
  `inlier_ratio_distance_m` (0.5 m) of a map point, instead of the fitness score.
  - **Why:** where the map misses structure the sensor sees, a few points metres from
    any map point dominate the fitness score's mean of squared distances. On Koide
    `outdoor_hard_01a`, alignments within 0.1-0.2 m of the ground truth score 10-24
    against a threshold of 6, and wrong poses score lower than right ones.
  - **The score:** `score_threshold * (1 - inlier_ratio) / (1 - min_inlier_ratio)`: 0
    for a perfect scan and `score_threshold` at `min_inlier_ratio`, so the measurement
    gate keeps its threshold. The pose covariance, the recovery and the EKF and GTSAM
    noise models read the same score; the reinitialization trigger's fitness explosion
    test reads the backend's own fitness. A scan of fewer than
    `inlier_ratio_min_points` (200) points gets a ratio of 0.
  - **`/alignment_status`** adds `registration_fitness_score`, the backend's own
    fitness, and `inlier_ratio`, also with the gate off. `fitness_score` stays what
    the gate reads.
  - **The distance must suit the map.** On Istanbul at 0.5 m, good alignments have
    inlier ratios of 0.58-0.93, so a preset that rejects above the threshold would drop
    a third of them at 0.8. The Istanbul preset does not reject: its GTSAM smoother
    weights the scans by the same score, and its three windows scored 0.35-0.38,
    0.89-0.90 and 0.55 m 3D RMSE, against 0.45-0.47, 0.93-0.97 and 0.56-0.57 m without
    the gate. Boreas does not change.
  - **The online Koide profile** (`tools/readme_media/koide_outdoor_hard_02b_online.yaml`)
    sets 0.8. With RKO-LIO odometry, `outdoor_hard_01a` goes from 3.64 m to 0.43 m
    RMSE, and `outdoor_hard_02b`, the release check's sequence, stays within 0.01 m
    ([experiments/inlier_ratio_gate](experiments/inlier_ratio_gate/README.md)).
  - **Without odometry**, 01a holds within 1 m to 140.2 s at 0.8 (106.4 s without the
    gate) and to 147-152 s at 0.7. There even alignments from the ground truth have
    inlier ratios of 0.48-0.76 and land up to 1 m off.

- `tools/benchmark/istanbul_replay.py` adds `last_pose_sec` and `longest_pose_gap_sec`
  to `score.json`, and `istanbul_compare.py` prints the longest gap. The errors cover
  only the published poses: `nav2_ndt_urban.yaml` scored 0.42 m 3D RMSE on the 2820 s
  window while publishing nothing after 34.7 s, a 115 s gap.

### Changed

- `param/nav2_ndt_urban.yaml` rotates the twist seed with the twist's angular velocity
  (`twist_prediction_use_angular_velocity: true`, the node default). Without it the
  seed keeps the old heading.
  - **Istanbul 2820 s:** on a 145° turn at up to 37°/s, the seed lagged 3.7° per scan.
    The preset lost the track 34.8 s in and published nothing for 115 s. It now tracks
    the whole window, at 0.974 m 3D RMSE.
  - **Istanbul 2300 s:** 0.595 m against 0.591 m.
  - **Istanbul 0 s:** 0.623 m against 0.504 m, the same in three runs. All of it is at
    42-48 s, on a straight where the seed changes by 0.1° per scan, and NDT settles
    elsewhere at a hard stretch.
  - The other presets that use twist prediction keep `false`; they were tuned on their
    own robots and data.
- `param/nav2_ndt_urban.yaml` rebuilds the local map in the background
  (`local_map_async_rebuild: true`), as the Boreas and Istanbul presets and the online
  Koide profile already do.
  - **Koide `outdoor_hard_01a`, without odometry:** rebuilding the 150 m crop in the
    callback left 10% of the clouds without a diagnostic. Coverage is now 99.8-99.9%.
    With `min_inlier_ratio` 0.8, the track holds within 1 m to 140.2 s, against 105.9 s.
    Without the gate it ends at 106.4 s either way.
  - **Koide `outdoor_hard_02b`, without odometry:** within 1 m until 28.7 s instead of
    13.1 s. The preset loses this sequence without odometry either way.
  - **Istanbul with this preset:** the 0 s and 2300 s windows score 0.504 and 0.591 m
    3D RMSE, against 0.507 and 0.586 m, with every aligned scan accepted. The preset
    loses the 2820 s window at 34.8 s either way: 345-348 of its 1500 scans are
    accepted, where `istanbul_gtsam_smoother.yaml` accepts 1499.
- `experiments/koide_imu_ab/replay.py` rebuilds the local map in the background
  (`local_map_async_rebuild`), as the online profile does. Rebuilt in the callback, the
  150 m crop of the Koide map dropped 10% of the clouds; a gap over 1 s drops the IMU
  prediction and can end the track. `--set local_map_async_rebuild=false` reproduces the
  earlier runs.

### Fixed

- With `min_inlier_ratio`, the reinitialization trigger's fitness explosion test reads
  `registration_fitness_score`. The gate's score tops out at
  `score_threshold / (1 - min_inlier_ratio)`, 30 at the defaults, so the test's 1000
  could never fire. Without the gate the two scores are the same.
- `tools/benchmark/istanbul_replay.py` delays playback by 2 s and records
  `alignment.first_scan_sec`. `ros2 bag play` published before discovery matched the
  localizer, so a run could start 0.65-1.15 s into the window from the window start's
  pose: up to 18 m behind on the 2820 s window. Three Istanbul runs diverged that way,
  one of them the "0.5 m loses the 2300 s window" of the inlier gate's first record.

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

### Validation

- The release check: two full Koide `outdoor_hard_02b` replays at 1x passed on
  `release/v1.8.0` at `d853fbd` (`main` at `64a14db` plus the version bump), built
  without a build type, with ndt_omp_ros2 at its 0.2.0 release.
  - **Accuracy:** 0.242 and 0.245 m RMSE (0.171 m on the map), against 0.233 m
    (0.181 m) for v1.7.0. The online profile now sets `min_inlier_ratio: 0.8`; off the
    map the drift is 0.353-0.357 m against 0.319-0.320 m.
  - **Coverage:** 100.0% diagnostic coverage in both.
  - **Alignment:** 12-13 ms at the median and 32-33 ms at p95, against 17 and 45-46 ms,
    on a faster host CPU (2.30 GHz against 2.10 GHz)
    ([docs/koide-online-replay.md](docs/koide-online-replay.md#v180-release-verification-2026-11-07),
    `docs/validation/v1.8.0-koide-20261107`).
- Istanbul, `param/istanbul_gtsam_smoother.yaml`: 0.24-0.27, 0.74-0.77 and 0.45-0.47 m
  horizontal RMSE on the three windows, three runs each with clean starts, as for
  v1.5.0 (0.23, 0.75 and 0.46 m).

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
