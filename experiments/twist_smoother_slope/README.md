# Slope-aware twist GTSAM smoother (Autoware Istanbul)

**Result: promoted on the second measurement.** First measured while NDT did not
register on this preset, the candidate changed only z and was rejected. That measurement
was void: the full-map NDT target overflowed and was empty, the smoother dropped motion at
every update, and window C started inside a GNSS outage
([istanbul_ndt_tracking](../istanbul_ndt_tracking/README.md) fixed those). Measured again
on top of that fix, it removed the z failures on the graded windows (below).

## Change

The twist smoother (`use_gtsam_smoother`) keeps one planar pose (x, y, yaw) per scan and
takes z, roll and pitch from NDT. Between scans it accumulates the twist's forward speed as
planar motion and seeds NDT with the last NDT z. The change passes the last NDT roll and
pitch to `predict()`, the same change as #241 for the twist EKF:

- only `cos(pitch)` of the forward speed is planar odometry;
- the rest, `-vx sin(pitch) dt`, moves the z of the NDT seed until the next update;
- the body yaw rate turns the heading at `cos(roll) / cos(pitch)` of its value.

With roll = pitch = 0 the arithmetic is the old one (the first version gave
bit-identical seeds, poses and z against `main` in 100 random level-ground predict/update
runs). On top of the scan-stamp smoother, the climb received after a scan carries over to
the new node like the planar motion (`test/test_twist_gtsam_smoother.cpp`).

## Fixture

The map (256 MB, 18.7 M points) and the 57-minute localization bag of the
[Autoware Istanbul urban evaluation](https://docs.autoware.org/main/simulation-evaluation/components_evaluation/localization_evaluation/urban-environment-evaluation/):

```bash
gdown 1WPWmFCjV7eQee4kyBpmGNlX7awerCPxc -O "$ISTANBUL/map/"
gdown 1yEB5j74gPLLbkkf87cuCxUgHXTkgSZbn -O "$ISTANBUL/localization_bag/"
ros2 bag reindex "$ISTANBUL/localization_bag" -s sqlite3
python3 experiments/twist_smoother_slope/prepare.py "$ISTANBUL/localization_bag" /tmp/istanbul_fixture
```

The localizer reads `/localization/util/downsample/pointcloud` (1500 points, `base_link`)
and `/localization/twist_estimator/twist_with_covariance`; `/sensing/gnss/pose_with_covariance`
is the reference. The route climbs up to 10.5 % (bridges, tunnels; z from -65 to +177 m).
The twist matches the GNSS-derived speed and yaw rate within 0.05 m/s and 0.002 rad/s
(sampled every 5 s over the first minute).

Pitfalls of this bag, all handled by `replay.py`:

- **Header stamps are 2,371,531.4 s (27 days) before the bag's receive time.** The bag
  carries its own `/clock` in the header time base: play it instead of `--clock`, and map
  window starts through the clouds' stamps (`time_map.txt`).
- **The first GNSS message is stamped 6.3 s after the first cloud**, so it does not mark the
  bag start.
- **The first GNSS messages repeat a stale pose** 1.9 m and 146° off the next ones; they are
  dropped. After them the reference matches the preset's initial pose, so
  `/sensing/gnss/pose` is already usable as the `base_link` reference.
- **`public_istanbul_60s_benchmark.yaml` cannot keep up on 4 cores.** It rebuilds the local
  NDT target from the full map on every scan (`local_map_update_distance: 0`) and processed
  about 21 of 600 scans in 60 s, so its documented 1.2 m could not be checked here
  (later measurements in [istanbul_ndt_tracking](../istanbul_ndt_tracking/README.md)).
  `istanbul_gtsam_smoother.yaml` built the full-map target once and kept up (9-10
  poses/s), but that target was empty (see the intro).

## Second measurement (promoted)

`compare.py` with the preset as it is (local map crop on), against `main` at `2d91060`.
Median of three runs (range):

| Window | Variant | Horizontal RMSE m | z RMSE m | Yaw RMSE deg | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A 0-60 s | main | 0.250 (0.238-0.259) | 0.231 (0.226-0.251) | 1.873 (1.855-1.875) | 0.564 (0.476-0.619) |
| A | slope z | 0.249 (0.230-0.257) | 0.255 (0.221-0.256) | 1.857 (1.854-1.861) | 0.508 (0.493-0.568) |
| B 2820-2970 s | main | 1.135 (0.973-1.318) | 2.483 (0.569-**19.373**) | 1.751 (1.644-1.806) | 5.629 (2.355-27.300) |
| B | slope z | **0.934** (0.931-1.177) | **0.797** (0.482-1.979) | 1.696 (1.643-1.761) | **2.470** (2.242-5.858) |
| C 2300-2420 s | main | 0.502 (0.497-1.056) | 0.609 (0.460-**28.098**) | 2.136 (2.116-2.213) | 1.330 (1.126-40.126) |
| C | slope z | **0.478** (0.475-0.568) | **0.448** (0.368-0.468) | 2.127 (2.072-2.130) | **1.086** (0.977-1.331) |

- On `main`, z fell behind on a grade in two of the six graded runs (19.4 m and 28.1 m z
  RMSE): while NDT was rejected, the seed kept the last NDT z and NDT did not catch up. With
  the slope-aware seed no run lost z.
- On the flat window A nothing changes beyond run-to-run spread.
- The yaw RMSE carries the ~2° GNSS orientation bias described in
  [istanbul_ndt_tracking](../istanbul_ndt_tracking/README.md).
- The container restarted during repeat 2 of window C. The two runs right after it
  processed only 75 poses (109-125 before) and both lost track (22 m and 35 m), so they
  were run again; the reruns are in the table. Throughput at 21 m/s is a known limit of
  the crop.

The change is now runtime code (`include/lidar_localization/twist_gtsam_smoother.hpp`,
`test/test_twist_gtsam_smoother.cpp`); the earlier patch is in Git history.

## First measurement (void)

Against v1.4.0, before the NDT target and smoother fixes, with the preset as it was
then (no crop).

### Method

`compare.py` runs `replay.py` for the baseline and the candidate alternately, three times
over three windows, with `param/istanbul_gtsam_smoother.yaml`. Each run starts from the GNSS
pose at the window start and scores `/pcl_pose` against the GNSS reference interpolated
at each pose stamp.

| Window | Bag time | Why |
| --- | --- | --- |
| A | 0-60 s | the old 60 s regression start (parked for 15 s, then up to 10 m/s) |
| B | 2820-2970 s | sustained 7-9 % grades |
| C | 3160-3280 s | up to 10.5 % (inside a GNSS outage; replaced by 2300-2420 s) |

### Result

Median of three runs (range):

| Window | Variant | Horizontal RMSE m | z RMSE m | Yaw RMSE deg | 3D p95 m |
| --- | --- | --- | --- | --- | --- |
| A | baseline | 5.900 (4.122-6.160) | 0.731 (0.731-0.733) | 0.712 (0.463-0.799) | 12.322 (8.267-12.903) |
| A | candidate | 5.687 (5.586-5.833) | **1.497** (1.484-1.505) | 0.632 (0.600-0.686) | 12.153 (11.981-12.536) |
| B | baseline | 20.053 (19.695-20.228) | 38.420 (38.420-38.432) | 0.803 (0.789-0.933) | 58.056 (57.523-58.109) |
| B | candidate | 20.504 (20.503-21.935) | **27.168** (26.740-27.679) | 1.025 (0.916-1.042) | 58.135 (56.756-58.752) |
| C | baseline | 23.537 (23.533-23.540) | 12.187 (12.187-12.192) | 0.368 (0.367-0.370) | 33.008 (32.976-33.032) |
| C | candidate | 23.451 (23.450-23.455) | **8.828** (8.763-8.837) | 0.361 (0.361-0.361) | 33.618 (33.513-33.660) |

- **The baseline does not track.** In window A it holds 0.02 m while parked, then drifts
  cross-track at about 2.4° of the distance driven (13 m after 300 m) while its heading stays
  within 0.7° of GNSS. NDT fitness climbs above the preset's `score_threshold: 6.0`
  (158 rejections in 60 s), and the pose bridges on the twist between rejections.
- **The candidate only moves z.** While NDT is rejected, the published pose is the seed.
  The seed's z now follows the last accepted NDT pitch, which helps on long grades (B, C)
  but walks z away when that pitch is off (A).
- A 20 s parked replay checks the harness: 0.019 m horizontal, 0.012 m z, 0.04° yaw RMSE.

Measured 2026-10-27 on ROS 2 Jazzy, 4 cores; baseline = v1.4.0 (`9dba2aa`).
This is a public-dataset replay, not a physical-robot test.

## Rerun

```bash
# BASE_WS built from 2d91060 (main before this change), CAND_WS from this change
python3 experiments/twist_smoother_slope/compare.py \
  --base-ws BASE_WS --cand-ws CAND_WS --fixture /tmp/istanbul_fixture \
  --bag "$ISTANBUL/localization_bag" --map "$ISTANBUL/map/pointcloud_map.pcd" \
  --out /tmp/istanbul_compare
```

Set `ROS2_CLI` and `ROS_PYTHON` when `python3` is not the interpreter the ROS distro was
built for. About 45 minutes for the 18 runs.
