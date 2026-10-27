# Rejected: slope-aware twist GTSAM smoother (Autoware Istanbul)

**Result: not promoted.** The candidate changed only z: better on two graded windows,
twice as large on the flat start window, horizontal unchanged. The baseline preset did
not track any window (6-24 m horizontal RMSE), so neither variant was in a usable regime.
Runtime code is unchanged; the candidate is kept as [candidate.patch](candidate.patch).

> **Later finding:** neither variant could track, because NDT did not register: the
> full-map NDT target of this preset overflowed and was empty, and with a crop the
> smoother dropped motion at every update. Window C also started inside a GNSS outage.
> See [istanbul_ndt_tracking](../istanbul_ndt_tracking/README.md), which fixes both and
> measures window C at 2300 s instead. The candidate was measured without registration;
> it needs measuring again on top of that fix (the patch predates it and adds the same
> test file, so it has to be rebased).

## Candidate

The twist smoother (`use_gtsam_smoother`) keeps one planar pose (x, y, yaw) per scan and
takes z, roll and pitch from NDT. Between scans it accumulates the twist's forward speed as
planar motion and seeds NDT with the last NDT z. The candidate passes the last NDT roll and
pitch to `predict()`, the same change as #241 for the twist EKF:

- only `cos(pitch)` of the forward speed is planar odometry;
- the rest, `-vx sin(pitch) dt`, moves the z of the NDT seed until the next update;
- the body yaw rate turns the heading at `cos(roll) / cos(pitch)` of its value.

With roll = pitch = 0 the arithmetic is the old one: 100 random level-ground
predict/update runs gave bit-identical seeds, poses and z against `main`. The patch adds
the smoother's first unit tests (`test/test_twist_gtsam_smoother.cpp`).

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
  about 21 of 600 scans in 60 s, so its documented 1.2 m could not be checked here.
  `istanbul_gtsam_smoother.yaml` built the full-map target once and kept up (9-10
  poses/s), but that target was empty (see the note above).

## Method

`compare.py` runs `replay.py` for the baseline and the candidate alternately, three times
over three windows, with `param/istanbul_gtsam_smoother.yaml`. Each run starts from the GNSS
pose at the window start and scores `/pcl_pose` against the GNSS reference interpolated
at each pose stamp.

| Window | Bag time | Why |
| --- | --- | --- |
| A | 0-60 s | the old 60 s regression start (parked for 15 s, then up to 10 m/s) |
| B | 2820-2970 s | sustained 7-9 % grades |
| C | 3160-3280 s | up to 10.5 % |

## Result

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
# baseline and candidate in two workspaces
git worktree add --detach /tmp/cand main
git -C /tmp/cand apply "$PWD/experiments/twist_smoother_slope/candidate.patch"
# build each workspace with colcon, then:
python3 experiments/twist_smoother_slope/compare.py \
  --base-ws BASE_WS --cand-ws CAND_WS --fixture /tmp/istanbul_fixture \
  --bag "$ISTANBUL/localization_bag" --map "$ISTANBUL/map/pointcloud_map.pcd" \
  --out /tmp/istanbul_compare
```

Set `ROS2_CLI` and `ROS_PYTHON` when `python3` is not the interpreter the ROS distro was
built for. About 45 minutes for the 18 runs.
