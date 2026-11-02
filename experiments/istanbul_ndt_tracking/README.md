# Why NDT lost the Autoware Istanbul route

Question: why did `param/istanbul_gtsam_smoother.yaml` drift to 6-24 m horizontal RMSE
on the Istanbul localization bag
([twist_smoother_slope](../twist_smoother_slope/README.md)), although the twist matches
GNSS? Three causes stacked, and each one hid the next.

## 1. The full-map NDT target was empty

Without `enable_local_map_crop` the NDT voxel grid spans the map's bounding box,
15.2 x 12.7 km and 284 m of height: 5.5e10 cells at `ndt_resolution: 1.0` (6.9e9 at 2 m;
it first fits at 3 m, 2.0e9). pclomp indexes the grid with int32, so it printed
`Leaf size is too small for the input dataset. Integer indices would overflow.`, left the
target empty, and NDT returned its initial guess on every scan (corrections of 0.008 m).
The pose was the twist dead reckoning from the GNSS initial pose.

The cross-track drift that looked like a yaw-rate error came from that initial pose: the
GNSS orientation in this bag is 1.9-2.3° off the direction of travel (median -2.26° over
the bag, computed from GNSS positions above 5 m/s), and 300 m at 1.9° is 10 m. Wherever
NDT registers, its yaw matches the direction of travel within 0.06°, so **yaw RMSE
against GNSS carries a ~2° reference bias** in the tables below.

The localizer now logs an error with the map's extent when it builds such a grid (the
full-map target, or the NDT initializer of the GICP backends), and the preset crops.

## 2. The smoother placed each NDT pose at the processing time

With the crop, NDT registered (fitness 0.2 while parked, against 0.74 for the GNSS
pose), but the replay diverged once the car reached about 3 m/s: 53 m horizontal RMSE in
window A. The pose fell behind along the track while the cross-track error stayed small.

`TwistGtsamSmoother::update()` accepted the scan stamp and did not use it. The NDT pose is
the pose at the scan stamp, but it was put on a node built from all twist received until
the scan was processed, and the accumulated twist was then cleared. Every update dropped
the motion made while the scan waited and was aligned, and the next seed started behind
by speed x delay: 1-1.5 m at 3 m/s, 10-20 m at 10 m/s behind a 1.7 s target rebuild, out
of NDT's reach. Without registration (cause 1) the NDT pose equals the seed, so nothing
showed.

The smoother now counts each twist message toward the scan its stamp precedes, carries
the later ones over to the new node, and seeds NDT with the pose predicted at the scan
stamp (`test/test_twist_gtsam_smoother.cpp`).

## 3. Twist was dropped while a scan held the state lock

The local map crop and target build run under the state lock, 1.7 s per rebuild here,
and with a pose backend enabled the twist callback waits for that lock. The twist
subscription kept the default depth of 5 (50 ms at 100 Hz), and the smoother skips a gap
over 1 s, so each rebuild cost about 1.2 s of motion: with rebuilds every 30 m the pose
fell back 10-14 m at each one. The subscription now queues 1000 messages.

`TwistEkf::update()` took no stamp either and had cause 2 as well; see *Twist EKF* below.

## Method

`tools/benchmark/istanbul_compare.py` replays three windows three times per variant with
`param/istanbul_gtsam_smoother.yaml`, starts from the GNSS pose at the window start and
scores `/pcl_pose` against the GNSS reference.

| Window | Bag time | Why |
| --- | --- | --- |
| A | 0-60 s | the old 60 s regression start (parked for 15 s, then up to 10 m/s) |
| B | 2820-2970 s | sustained 6-10 % grades |
| C | 2300-2420 s | the fastest climb: 4-5 % at up to 21 m/s |

GNSS is accurate in all three (sigma at most 0.52 m). It is not during 750-1030 s and
3020-3215 s (sigma up to 92 m): the earlier window C at 3160 s started inside that outage,
so the seed and the reference were tens of meters off, and its results (also in
[twist_smoother_slope](../twist_smoother_slope/README.md)) say nothing.

## Result

Median of three runs (range). *Preset* is `971e29a` with the preset as it was (no crop);
*+ crop* is the same build with `enable_local_map_crop: true`, `local_map_radius: 150.0`,
`local_map_update_distance: 10.0`; *fix* is this change with the same crop.

| Window | Variant | Horizontal RMSE m | z RMSE m | Yaw RMSE deg | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A | preset | 5.900 (4.122-6.160) | 0.731 (0.731-0.733) | 0.712 (0.463-0.799) | 12.322 (8.267-12.903) |
| A | + crop | 53.044 (49.685-55.307) | 0.647 (0.467-2.512) | 6.343 (3.328-8.711) | 120.036 (119.412-130.277) |
| A | **fix** | **0.261** (0.224-0.267) | **0.236** (0.232-0.256) | 1.850 (1.841-1.866) | **0.514** (0.476-0.589) |
| B | preset | 20.053 (19.695-20.228) | 38.420 (38.420-38.432) | 0.803 (0.789-0.933) | 58.056 (57.523-58.109) |
| B | + crop | 249.014 (235.273-334.694) | 33.232 (28.967-37.411) | 11.788 (10.565-25.801) | 332.022 (331.656-429.600) |
| B | **fix** | **1.106** (0.905-9.065) | **4.231** (0.518-37.894) | 1.766 (1.686-2.891) | **12.451** (2.003-48.903) |
| C | preset | 51.050 (51.030-79.897) | 48.099 (47.772-48.468) | 0.281 (0.265-1.624) | 99.248 (98.788-131.563) |
| C | + crop | 251.893 (220.978-380.528) | 38.186 (34.772-39.882) | 10.325 (8.055-13.328) | 308.469 (262.940-466.047) |
| C | **fix** | **0.511** (0.472-0.585) | **0.618** (0.427-0.797) | 2.117 (2.092-2.151) | **1.494** (1.108-2.356) |

- The crop alone makes every window worse: NDT now registers, and cause 2 throws it off
  once the car moves.
- With the fix, eight of nine runs stay within 1.2 m horizontal RMSE (the six of A and C
  within 0.6 m). The ninth is the window B z failure under *Known limits*.
- Yaw RMSE rises to about 2° wherever NDT registers: that is the GNSS orientation bias
  (cause 1), not the localizer. The preset's low yaw RMSE comes from keeping the GNSS
  initial yaw.
- While parked in window A the fix stays within 0.1-0.3 m of the GNSS position, at NDT
  fitness 0.2.
- The *fix* runs queued 200 twist messages. One run per window with the committed build
  (1000 messages, preset with the crop): 0.223, 0.918 and 0.504 m horizontal RMSE.

## Twist EKF

The EKF compared the NDT result, the pose at the scan stamp, with the state predicted up
to the latest twist, so every update pulled the state back by speed x delay. It now
keeps its predicted trajectory, carries the result along it from the scan stamp to the
state's time (turned by the measured heading), shifts the trajectory by every
correction, and seeds NDT and publishes with the pose at the scan stamp
(`test/test_twist_ekf.cpp`).

The same three windows with `use_twist_ekf: true`, `use_gtsam_smoother: false` and the
crop, against `main` at `2d91060`, median of three runs (range):

| Window | Variant | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- |
| A | main | 19.047 (14.213-33.546) | 8.721 (6.180-9.675) | 51.949 (38.384-81.512) |
| A | **fix** | **0.253** (0.205-0.340) | **0.501** (0.254-0.804) | **0.836** (0.560-2.068) |
| B | main | 456.330 (412.452-471.294) | 63.798 (22.753-131.827) | 644.058 (559.150-685.918) |
| B | fix | 36.978 (28.515-46.145) | 43.951 (19.919-50.019) | 83.134 (48.802-89.706) |
| C | main | 270.216 (248.568-323.545) | 34.836 (11.362-42.738) | 342.849 (303.544-392.624) |
| C | **fix** | **0.537** (0.516-1.220) | **0.750** (0.728-2.661) | **1.445** (1.369-6.223) |

Window B still fails with the EKF, in z first: on the 6 % descent its z fell 2.5 m below
GNSS within 23 s while NDT matched (fitness 0.4), NDT then rejected the scans, and the
pose diverged. The smoother tracks the same window. Why the EKF's z does not follow the
accepted NDT z is not established here; no preset enables the EKF. Measured 2026-10-28
on ROS 2 Jazzy, 4 cores.

A larger position process noise is not the fix. One window B run each tracked with
`ekf_sigma_pos: 0.1` (0.87 m) and `0.5` (0.98 m) where the default 0.01 failed, but three
runs per window of 0.01 against 0.1 failed equally often: three of the six window B and C
runs failed with either value (horizontal RMSE 10-226 m), and the others tracked within
1.3 m. On this container the EKF on windows B and C fails intermittently, and the
default stays 0.01.

## Known limits

- **z on grades.** The smoother is planar and kept the last NDT z. In one of the three
  window B runs NDT was rejected for a few scans on the 6 % descent, z fell behind by
  about 1 m/s, and NDT did not recover (9.1 m horizontal, 37.9 m z RMSE). Fixed since by
  the slope-aware z seed ([twist_smoother_slope](../twist_smoother_slope/README.md),
  measured again on top of this fix).
- **Throughput.** Every 10 m the scan callback cropped the map (0.06 s) and built the
  NDT target of the 150 m crop (0.6-0.8 s, 141k points) before aligning, so the pose rate
  fell to 0.9-3 Hz while driving (0.9 Hz at 21 m/s in window C). Fixed since by
  `local_map_async_rebuild` (*Background target rebuild* below). These times came from
  the unoptimized build; optimized, the target takes 0.05-0.14 s, and without the
  background rebuild the preset keeps 8.3-8.7 Hz (*Re-measured with the optimized
  build*).

Measured 2026-10-27 on ROS 2 Jazzy, 4 cores; baseline = `971e29a`. This is a
public-dataset replay, not a physical-robot test.

## Background target rebuild

With `local_map_async_rebuild: true` (NDT_OMP) the next target is built on a twin NDT
object while scans keep aligning against the cached one, which still covers every scan
point until the pose has moved `local_map_radius - scan_max_range` (50 m here); then the
two objects are swapped. Same windows, preset with the crop, against `main` at
`2d91060`, median of three runs (range):

| Window | Variant | Pose rate | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A | main | 2.9 Hz | 0.230 (0.228-0.236) | 0.235 (0.220-0.244) | 0.474 (0.459-0.516) |
| A | async | **5.6 Hz** | 0.241 (0.237-0.265) | 0.301 (0.293-0.327) | 0.689 (0.647-0.768) |
| B | main | 1.3 Hz | 2.469 (1.019-25.806) | 3.426 (0.604-37.073) | 8.310 (2.327-55.426) |
| B | async | **5.8 Hz** | **0.843** (0.819-0.844) | **0.552** (0.535-0.582) | **2.058** (1.916-2.227) |
| C | main | 0.9 Hz | 0.501 (0.500-0.523) | 0.588 (0.579-0.603) | 1.568 (1.513-1.699) |
| C | async | **5.2 Hz** | **0.459** (0.452-0.470) | **0.351** (0.328-0.367) | **0.969** (0.954-0.979) |

The pose rate is the median count of published poses over the window length. Window B's
z failure on `main` (one run of three) is the planar-z limit above; at the higher rate
NDT kept z in all three runs. Window A, parked for 15 s and then at up to 10 m/s, comes
out slightly worse: 0.01 m horizontal and 0.07 m z RMSE (0.30 against 0.24 m).

That window A difference is a sampling effect, not a worse pose (three more runs per
variant on 2026-10-28). While parked every variant had the same z RMSE, 0.24-0.25 m (a
constant map-to-GNSS z offset). While moving, the background rebuild published about four
times as many poses (155-173 against 40-48), so its RMSE covers more and harder moments.
Scored only at the scan stamps both published (13-26 per run), it was better
horizontally in seven of nine pairs, and z went either way. A 5 m update distance or
three NDT threads (a stale cached target, or CPU contention with the rebuild) changed
nothing.

Measured 2026-10-28 on ROS 2 Jazzy, 4 cores. This is a public-dataset replay, not a
physical-robot test.

## Background rebuild for the GICP backends

`local_map_async_rebuild` first covered NDT_OMP only. The GICP backends build most of
their target lazily, in the first `align()` after `setInputTarget()`: the PCL search
tree (every backend), the target covariances, and the small_gicp VGICP voxel map. So the
background task now crops, voxel-filters the crop for the GICP backends as the scan
callback does, sets the target on the twin and aligns about 100 of its points once;
the scan callback then swaps in a target that is ready.

Target build cost on the 150 m crop around the window A start (141k points,
`voxel_leaf_size: 0.2`, 4 threads; two runs each):

| Backend | Crop | Voxel filter | `setInputTarget` | First `align()` on 100 points |
| --- | --- | --- | --- | --- |
| NDT_OMP | 0.10-0.11 s | - | 1.37-1.59 s | 0.07-0.10 s |
| SMALL_GICP | 0.08-0.09 s | 0.02 s | 0.02 s | 0.17 s |
| SMALL_VGICP | 0.09-0.10 s | 0.02 s | 0.02-0.03 s | 0.27 s |
| GICP_OMP | 0.09 s | 0.02 s | 0.00 s | 1.96-2.17 s |

The preset with `registration_method: SMALL_GICP` (small_gicp 2.1.0 from
`ros-jazzy-small-gicp-vendor`), without and with the background rebuild, one build,
median of three runs (range):

| Window | Variant | Pose rate | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A | sync | 7.1 Hz | 0.256 (0.255-0.256) | 0.175 (0.175-0.175) | 0.485 (0.485-0.485) |
| A | async | **8.0 Hz** | 0.256 (0.256-0.257) | 0.176 (0.176-0.176) | 0.486 (0.486-0.486) |
| B | sync | 7.9 Hz | 0.774 (0.768-0.775) | 0.388 (0.388-0.390) | 1.802 (1.718-1.813) |
| B | async | **9.5 Hz** | 0.761 (0.760-0.762) | 0.396 (0.395-0.396) | 1.678 (1.677-1.680) |
| C | sync | 7.2 Hz | 0.450 (0.450-0.452) | 0.115 (0.115-0.115) | 0.873 (0.871-0.876) |
| C | async | **9.4 Hz** | 0.450 (0.450-0.450) | 0.114 (0.114-0.115) | 0.871 (0.871-0.871) |

The scans arrive at 10 Hz; accuracy is unchanged. With SMALL_GICP the preset also
tracked as well as with NDT_OMP in this session (0.26, 0.77 and 0.45 m against 0.26,
0.91 and 0.54 m horizontal RMSE below) at 7-9 Hz against 1.3-3.1 Hz; small_gicp is an
optional dependency, so the preset stays on NDT_OMP;
`param/istanbul_small_gicp.yaml` keeps this configuration.

NDT_OMP, the preset as it is, `main` against this change (which adds the warm-up align
in the background), median of three runs (range):

| Window | Variant | Pose rate | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A | main | 3.1 Hz | 0.259 (0.255-0.340) | 0.338 (0.259-0.369) | 0.779 (0.605-0.829) |
| A | change | 2.6 Hz | 0.260 (0.239-0.266) | 0.291 (0.285-0.390) | 0.694 (0.658-0.737) |
| B | main | 2.3 Hz | 0.912 (0.870-0.918) | 0.631 (0.587-0.661) | 2.275 (2.142-2.288) |
| B | change | 2.6 Hz | 0.916 (0.776-0.995) | 0.760 (0.579-0.812) | 2.350 (1.962-2.703) |
| C | main | 1.3 Hz | 0.542 (0.483-0.562) | 0.328 (0.327-0.367) | 1.115 (0.964-1.119) |
| C | change | 1.3 Hz | 0.530 (0.485-0.531) | 0.370 (0.329-0.386) | 1.072 (0.975-1.155) |

Every range overlaps: no measurable change for NDT_OMP. These NDT runs vary more from
run to run than the SMALL_GICP ones; the container ran as slowly as in *All fixes
together*.

GICP_OMP could not show the effect: on window C it lost track with and without the
background rebuild (0.3-0.7 Hz, 49-51 m horizontal RMSE, three runs each). On window A
`main` and this change ran it alike, 0.65 and 0.83 Hz (one run each), and published only
8-13 poses in the first 14 s while parked, when no target is rebuilt.

Two of the 45 replays failed in the harness (one stopped the localizer before it
logged anything; the other's log was not kept) and were run again.

Measured 2026-10-28 on ROS 2 Jazzy, 4 cores. This is a public-dataset replay, not a
physical-robot test.

## All fixes together

`main` after #248 (slope-aware z seed), #249 (Twist EKF) and #250 (background rebuild),
preset as on `main` (crop, `local_map_async_rebuild`), against `main` after #247
(`2d91060`). Median of three runs (range):

| Window | Variant | Pose rate | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A | #247 | 1.6 Hz | 0.234 (0.227-0.240) | 0.223 (0.217-0.239) | 0.526 (0.504-0.534) |
| A | now | 3.6 Hz | 0.251 (0.247-0.255) | 0.279 (0.274-0.326) | 0.681 (0.667-0.687) |
| B | #247 | 0.7 Hz | 5.460 (3.516-11.765) | 33.411 (22.196-40.622) | 43.880 (28.477-50.455) |
| B | **now** | 3.5 Hz | **0.874** (0.842-0.880) | **0.681** (0.557-0.791) | **2.246** (2.061-2.334) |
| C | #247 | 0.6 Hz | 2.723 (1.483-7.676) | 42.967 (30.167-46.231) | 59.165 (45.804-63.119) |
| C | **now** | 1.6 Hz | **0.498** (0.487-0.567) | **0.377** (0.331-0.405) | **1.037** (0.991-1.094) |

- The container ran slower than in the earlier comparisons: the current build published
  1.6-3.6 Hz here, where the background rebuild alone reached 5.2-5.8 Hz, and the #247
  build 0.6-1.6 Hz, against 0.9-2.9 Hz before. At 0.6-0.7 Hz the #247
  build lost z on the grades in all six runs of B and C (z RMSE 22-46 m), against one or
  two of three before; its result swings with the machine's speed. The current build kept
  every run within 0.88 m horizontal RMSE, at 1.6 Hz on window C.
- Window A is slightly worse, 0.02 m horizontal and 0.06 m z, as with the background
  rebuild alone.

`param/public_istanbul_60s_benchmark.yaml` (the v1.1 regression template, kept as it
was) on window A with the current build, three runs each: as it is, it published 6 poses
in 60 s (it rebuilds the crop on every scan). With `local_map_update_distance: 10.0` and
`local_map_async_rebuild: true` it published 62-181 poses, 0.39, 0.42 and 3.03 m
horizontal RMSE; 101 of its 164 aligned scans were rejected at fitness above 6. That
preset seeds NDT from the twist prediction, without the smoother.

Measured 2026-10-28 on ROS 2 Jazzy, 4 cores.

## Unoptimized ndt_omp

The NDT_OMP pose rates above (1.3-3.6 Hz with 10 Hz scans) came from an unoptimized
`ndt_omp_ros2`. Its CMakeLists sets no `CMAKE_BUILD_TYPE`, so `colcon build` without
`--cmake-args -DCMAKE_BUILD_TYPE=Release`, the command in `AGENTS.md`, compiles
`libndt_omp` without `-O`. The localizer includes only `pclomp/ndt_omp.h`, so it ran
that library's NDT and GICP code; this package's own `-O2` did not reach it (the G2
candidate scoring includes `ndt_omp_impl.hpp` and was not affected).

`ndt_align_bench.cpp` calls the same `NormalDistributionsTransform<PointXYZI>` with the
preset's NDT settings on one scan and the 150 m crop around it, from three seeds around
the converged pose (median of 15 aligns, ms):

| Scan (crop) | ndt_omp build | Threads | Converged seed | 0.3 m, 0.5 deg off | 1.0 m, 1 deg off | `setInputTarget` |
| --- | --- | --- | --- | --- | --- | --- |
| window A start, 1500 points (203k) | no build type | 1 | 204 | 932 | 2127 | 1907 |
| | Release | 1 | 15 | 75 | 153 | 182 |
| | no build type | 4 | 56 | 251 | 546 | 1847 |
| | Release | 4 | 4 | 19 | 41 | 217 |
| window C climb, 1271 points (118k) | no build type | 4 | 29 | 166 | 274 | 1047 |
| | Release | 4 | 2 | 12 | 22 | 63 |

The two builds took the same iterations (2, 13-15 and 25-31) to the same poses.

The preset on the three windows, `main` as built by `AGENTS.md`, with `ndt_omp_ros2` built
as Release, and with `src/ndt_omp_instantiation.cpp` (the component compiles the
instantiations itself; `ndt_omp_ros2` again without a build type). Median of three runs
(range):

| Window | Variant | Pose rate | Alignment median | Alignment p95 | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | no build type | 3.1 Hz | 231 ms | 460 ms | 0.373 (0.270-0.510) | 0.315 (0.291-0.436) | 1.049 (0.785-1.877) |
| A | Release | 8.9 Hz | 10.6 ms | 21 ms | 0.238 (0.224-0.280) | 0.346 (0.288-0.400) | 0.776 (0.650-0.968) |
| A | **in-package** | 8.9 Hz | 10.8 ms | 23 ms | **0.230** (0.225-0.323) | 0.305 (0.290-0.333) | 0.690 (0.649-0.995) |
| B | no build type | 3.0 Hz | 223 ms | 511 ms | 0.883 (0.870-0.944) | 0.606 (0.546-0.849) | 2.220 (2.195-2.444) |
| B | Release | 10.0 Hz | 10.6 ms | 25 ms | 0.758 (0.753-0.764) | 0.545 (0.531-0.546) | 1.800 (1.725-1.833) |
| B | **in-package** | 10.0 Hz | 11.0 ms | 25 ms | **0.752** (0.746-0.755) | 0.533 (0.523-0.563) | 1.727 (1.718-1.846) |
| C | no build type | 1.5 Hz | 262 ms | 528 ms | 0.466 (0.442-0.754) | 0.362 (0.333-0.898) | 0.932 (0.929-3.104) |
| C | Release | 10.0 Hz | 11.8 ms | 25 ms | 0.459 (0.457-0.459) | 0.331 (0.327-0.332) | 0.981 (0.979-0.997) |
| C | **in-package** | 10.0 Hz | 12.0 ms | 27 ms | **0.459** (0.458-0.459) | 0.333 (0.327-0.334) | 0.982 (0.977-0.985) |

- Both optimized builds align every scan of windows B and C. On window A every variant
  lacks the first 0.3 s and the last 6 s of the window (a replay boundary, not dropped
  scans), hence 8.9 Hz. The median iterations were 7-10 in every variant, and at most
  2 % of the scans ran to `ndt_max_iterations`.
- The unoptimized build gives the rates measured above for this preset (2.3-3.6 Hz on
  windows A and B, 1.3-1.6 Hz on C). Optimized, NDT_OMP tracks as well as SMALL_GICP did
  in *Background rebuild for the GICP backends* (0.26, 0.76 and 0.45 m at 8.0-9.5 Hz)
  and at the same rate: SMALL_GICP's rate advantage there came from this build.
- The timings in the sections above were measured with the unoptimized build. The
  direction of their comparisons holds (both variants had it); the absolute NDT_OMP and
  GICP_OMP times and the size of the gains do not (*Re-measured with the optimized
  build* below).

Measured 2026-11-01 on ROS 2 Jazzy, 4 cores (load 0.6-1.1 when each comparison started).

```bash
# ndt_omp_ros2 built with and without -DCMAKE_BUILD_TYPE=Release, in WS_A and WS_B
python3 experiments/istanbul_ndt_tracking/extract_scan.py "$ISTANBUL/localization_bag" \
  /tmp/istanbul_fixture 20 /tmp/scan_20
g++ -O2 -std=c++17 -fopenmp experiments/istanbul_ndt_tracking/ndt_align_bench.cpp \
  -o /tmp/ndt_align_bench -I"$WS_A/install/ndt_omp_ros2/include" -I/usr/include/pcl-1.14 \
  -I/usr/include/eigen3 "$WS_A/install/ndt_omp_ros2/lib/libndt_omp.a" -lpcl_io \
  -lpcl_common -lpcl_kdtree -lpcl_search -lpcl_filters -lpcl_registration -lpcl_octree
/tmp/ndt_align_bench "$ISTANBUL/map/pointcloud_map.pcd" /tmp/scan_20/scan.pcd \
  /tmp/scan_20/pose.txt 4 15
```

## Re-measured with the optimized build

The target build times and the background rebuild comparison above ran the unoptimized
build. Re-measured on `main` at `16e315c`, where the component compiles NDT_OMP and
GICP_OMP itself.

**Target build.** `target_build_bench.cpp` does what the localizer does for a new
target, with the preset's NDT and GICP settings and 4 threads:

- the 2D crop;
- `setInputTarget()`;
- one `align()` of a 100-point probe, which builds what `pcl::Registration` sets up in
  the first `align()`: the search tree over the target points, and for GICP_OMP the
  target covariances.

GICP_OMP gets the crop voxel-filtered: 0.2 m on Istanbul, which keeps every point
(0.02-0.03 s), 0.5 m on Koide (253k and 233k points, 0.09-0.10 s). Its
`setInputTarget()` takes no time. "No build type" links `libndt_omp` as `AGENTS.md`
builds it; "in-package" adds `src/ndt_omp_instantiation.cpp`, as the component does.
Median of five:

| Crop | Points | Build | Crop | NDT_OMP `setInputTarget` | NDT_OMP first `align()` | GICP_OMP first `align()` |
| --- | --- | --- | --- | --- | --- | --- |
| Istanbul 150 m, window A start | 203k | no build type | 0.10 s | 1.95 s | 0.12 s | 3.55 s |
| | | in-package | 0.09 s | **0.14 s** | 0.05 s | **0.44 s** |
| Istanbul 150 m, window C climb | 118k | no build type | 0.10 s | 1.04 s | 0.07 s | 1.96 s |
| | | in-package | 0.10 s | **0.05 s** | 0.03 s | **0.25 s** |
| Koide outdoor 80 m | 753k | no build type | 0.02 s | 6.32 s | 0.29 s | 4.23 s |
| | | in-package | 0.02 s | **0.27 s** | 0.23 s | **0.51 s** |
| Koide outdoor 75 m | 695k | no build type | 0.02 s | 5.88 s | 0.28 s | 4.02 s |
| | | in-package | 0.02 s | **0.24 s** | 0.21 s | **0.51 s** |

The Koide crops are centred on the `outdoor_hard_02` GT pose with the most map points
within 80 m (x -19.34, y -78.57).

- **NDT_OMP builds its target 14-25 times faster, GICP_OMP 8 times.** A new 150 m
  Istanbul target now takes 0.18-0.28 s with the crop, an 80 m Koide one 0.52 s.
- **Half of the Koide NDT_OMP time is the search tree.** Skipping that tree cut the
  first `align()` from 233 to 1.5 ms (80 m crop, three runs). NDT_OMP does not use it
  to align, but `getFitnessScore()`, which the localizer reports, searches it. The
  localizer now builds it alongside the voxel grid (*Search tree alongside the voxel
  grid* below).

**Background rebuild.** The preset with `local_map_async_rebuild` on, as it ships, and
off (`sync`), one build, alternated, median of three runs (range):

| Window | Variant | Pose rate | Alignment p95 | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- | --- | --- |
| A | async | 8.9 Hz | 22 ms | 0.233 (0.224-0.320) | 0.351 (0.284-0.449) | 0.937 (0.646-1.036) |
| A | sync | 8.3 Hz | 32 ms | 0.239 (0.232-0.289) | 0.334 (0.308-0.363) | 0.714 (0.695-0.984) |
| B | async | 10.0 Hz | 24 ms | 0.766 (0.760-0.774) | 0.534 (0.533-0.541) | 1.756 (1.738-1.764) |
| B | sync | 8.7 Hz | 50 ms | 0.773 (0.766-0.779) | 0.545 (0.538-0.556) | 1.795 (1.766-1.856) |
| C | async | 10.0 Hz | 26 ms | 0.455 (0.454-0.457) | 0.330 (0.318-0.333) | 0.973 (0.943-0.984) |
| C | sync | 8.3 Hz | 46 ms | 0.457 (0.452-0.458) | 0.333 (0.325-0.347) | 0.971 (0.963-0.981) |

- **The background rebuild matters much less now.** Without it the preset keeps
  8.3-8.7 Hz (0.9-2.9 Hz unoptimized in *Background target rebuild*). With it every
  scan is aligned: window A lacks only its first 0.3 s and last 6 s, a replay boundary.
- **Accuracy is the same within the run-to-run ranges.** Unoptimized, the preset without
  it lost 0.04 m horizontal and 0.24 m z RMSE on window C.
- The preset keeps it on, for 10 Hz at 21 m/s.

Measured 2026-11-01 on ROS 2 Jazzy, 4 cores (load 0.8-1.6 when the benchmark and the
replays started).

```bash
# WS built as in AGENTS.md (ndt_omp_ros2 without a build type)
FLAGS="-O2 -std=c++17 -fopenmp -Iinclude -I$WS/install/ndt_omp_ros2/include \
  -I/usr/include/pcl-1.14 -I/usr/include/eigen3"
LIBS="$WS/install/ndt_omp_ros2/lib/libndt_omp.a -lpcl_io -lpcl_common -lpcl_kdtree \
  -lpcl_search -lpcl_filters -lpcl_registration -lpcl_octree"
g++ $FLAGS experiments/istanbul_ndt_tracking/target_build_bench.cpp \
  src/ndt_omp_instantiation.cpp -o /tmp/target_build_bench $LIBS   # in-package
g++ $FLAGS experiments/istanbul_ndt_tracking/target_build_bench.cpp \
  -o /tmp/target_build_bench_no_build_type $LIBS
/tmp/target_build_bench "$ISTANBUL/map/pointcloud_map.pcd" 66459.05 43619.96 150 0.2 5
/tmp/target_build_bench "$ISTANBUL/map/pointcloud_map.pcd" 68988.40 49585.30 150 0.2 5
/tmp/target_build_bench koide-data/map_outdoor_hard.ply -19.34 -78.57 80 0.5 5
python3 tools/benchmark/istanbul_compare.py --base-ws "$WS" --cand-ws "$WS" \
  --fixture /tmp/istanbul_fixture --bag "$ISTANBUL/localization_bag" \
  --map "$ISTANBUL/map/pointcloud_map.pcd" --out /tmp/istanbul_async \
  --cand-set local_map_async_rebuild=false
```

## Search tree alongside the voxel grid

`pcl::Registration` builds its search tree over the target points in the first
`align()` after `setInputTarget()`. NDT aligns with its voxel grid, which
`setInputTarget()` builds, and uses the tree only for `getFitnessScore()`. Yet the
two builds ran one after the other. `setNdtTargetWithSearchTree()`
(`include/lidar_localization/registration_target_search_tree.hpp`) builds the tree on
another thread while `setInputTarget()` runs and hands it over. The localizer sets
every NDT and NDT_OMP target through it: the local map crop, the background rebuild,
the full map at load, the map topic. The fitness score and the alignment do not
change (`test_registration_target_search_tree`).

**Target build.** `target_build_bench.cpp`, median of five, in-package build, the
target set plus the first `align()`:

| Crop | Points | `setInputTarget` + first `align()` | Tree alongside + first `align()` |
| --- | --- | --- | --- |
| Istanbul 150 m, window A start | 203k | 141 + 49 = 190 ms | 147 + 5 = **152 ms** |
| Istanbul 150 m, window C climb | 118k | 64 + 27 = 91 ms | 62 + 3 = **65 ms** |
| Koide outdoor 80 m | 753k | 275 + 243 = 518 ms | 298 + 2 = **300 ms** |
| Koide outdoor 75 m | 695k | 246 + 207 = 453 ms | 270 + 2 = **271 ms** |

Built alongside, the tree adds at most 24 ms to the voxel grid, where it used to add
27-243 ms after it: the target is ready 20-42% sooner.

**Replay.** The preset with `local_map_async_rebuild: false`, so that the scan
callback waits for each new target. `main` at `9c9edd7` against this change,
alternated, median of three runs (range):

| Window | Variant | Pose rate | Alignment p95 | Horizontal RMSE m | z RMSE m | 3D p95 m |
| --- | --- | --- | --- | --- | --- | --- |
| A | main | 8.2 Hz (8.1-8.2) | 34 ms | 0.235 (0.232-0.302) | 0.300 (0.287-0.333) | 0.654 (0.631-0.954) |
| A | **change** | **8.4 Hz** (8.3-8.4) | **21 ms** | 0.234 (0.231-0.260) | 0.282 (0.276-0.402) | 0.660 (0.626-0.932) |
| B | main | 8.6 Hz (8.5-8.7) | 50 ms | 0.758 (0.752-0.765) | 0.561 (0.543-0.606) | 1.861 (1.807-1.929) |
| B | **change** | **8.9 Hz** (8.9-9.0) | **23 ms** | 0.759 (0.757-0.763) | 0.547 (0.543-0.551) | 1.766 (1.763-1.776) |
| C | main | 8.2 Hz (8.0-8.2) | 47 ms | 0.453 (0.449-0.455) | 0.330 (0.326-0.349) | 0.957 (0.952-0.994) |
| C | **change** | **8.8 Hz** (8.5-8.8) | **22 ms** | 0.455 (0.449-0.459) | 0.341 (0.326-0.345) | 0.991 (0.960-1.010) |

- **More scans aligned without the background rebuild.** The pose rate rose by 0.2-0.6
  Hz, with no overlap between the ranges.
- **The alignment p95 halved.** In `main` the first scan after a new target built the
  tree in its `align()`; now none does.
- **Accuracy is the same within the ranges.**

Measured 2026-11-02 on ROS 2 Jazzy, 4 cores.

## Rerun

```bash
python3 tools/benchmark/istanbul_prepare.py "$ISTANBUL/localization_bag" /tmp/istanbul_fixture
# BASE_WS built from 971e29a, CAND_WS from this change
python3 tools/benchmark/istanbul_compare.py \
  --base-ws BASE_WS --cand-ws CAND_WS --fixture /tmp/istanbul_fixture \
  --bag "$ISTANBUL/localization_bag" --map "$ISTANBUL/map/pointcloud_map.pcd" \
  --out /tmp/istanbul_compare \
  --set enable_local_map_crop=false --cand-set enable_local_map_crop=true
```

`istanbul_replay.py` reads the preset from this checkout, which now crops: the overrides turn the
crop off for the baseline, as the preset was, and back on for the candidate. Pass only
`--set enable_local_map_crop=true` to compare both builds with the crop.
