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
  `local_map_async_rebuild` (*Background target rebuild* below).

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
