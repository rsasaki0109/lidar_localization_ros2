# Gate on the inlier ratio, not the fitness score

Question: Koide `outdoor_hard_01a` loses the track at 107-111 s without odometry, and
with odometry it accepts 19% of the scans in the hard stretch at 105-185 s. Is the
data that bad, or does the gate reject good alignments?

## The fitness score rejects good alignments

`koide_probe.cpp oracle` aligns every scan from its ground-truth pose, with the
localizer's settings:
- NDT_OMP, resolution 1.0, step 0.1, epsilon 0.01, 35 iterations, DIRECT7;
- a 1-100 m scan voxelized at 0.5 m;
- a 150 m crop of the map.

`export_scans.py` writes the scans. For each scan the probe reports two numbers:
- the fitness score the gate reads: the mean squared distance from each scan point to
  its nearest map point;
- the **inlier ratio**: the share of scan points within 0.5 m of a map point.

| Seconds | Scans | Fitness under 6 | Inlier ratio at least 0.8 | Inlier ratio, median | Final error, median |
| --- | --- | --- | --- | --- | --- |
| 100-110 | 50 | 47 | 47 | 0.96 | 0.05 m |
| 110-125 | 104 | 87 | 98 | 0.92 | 0.06 m |
| 125-135 | 92 | 10 | 89 | 0.87 | 0.10 m |
| 135-155 | 194 | 5 | 89 | 0.78 | 0.15 m |
| 155-175 | 170 | 2 | 145 | 0.86 | 0.10 m |
| 175-185 | 83 | 0 | 0 | 0.59 | 0.16 m |

- **From 125 s on, alignments within 0.1-0.2 m of the ground truth score 10-24.** The
  map misses structure the sensor sees, and points a few metres from any map point
  dominate a mean of squared distances. The gate's threshold is 6.
- **The inlier ratio counts each such point once.** It stays at 0.78-0.92 until the
  path leaves the map around 175-185 s.
- **The data is not the limit until 175 s.** The score is.

The fitness also prefers wrong poses. `koide_probe.cpp search` aligns one scan from
791-2219 seeds around the localizer's last accepted pose. The radius is 5-10 m, the yaw
±45°.

| Scan | Lowest fitness | Its error | Its inlier ratio | The ground-truth solution |
| --- | --- | --- | --- | --- |
| 111.3 s | 2.29 | 8.3 m, 34° | 0.40 | fitness 3.70, ranked 73rd; inlier ratio 0.92, ranked 1st |
| 112.7 s | 2.19 | 9.5 m, 28° | 0.42 | ranked 269th by fitness, 1st by inlier ratio (0.92) |
| 115.2 s | 1.87 | 11.7 m, 29° | 0.46 | ranked 611th by fitness, 1st by inlier ratio (0.92) |

The track itself breaks the same way. At 106.5 s the localizer accepted a pose 1.9 m
and 16° off, with fitness 4.75 (`koide_probe.cpp eval` on its trajectory). Its inlier
ratio was 0.47. The ground-truth pose of that scan has 0.96.

## The gate

`min_inlier_ratio` (off at 0) makes the score the inlier ratio on the fitness scale:

```text
score = score_threshold * (1 - inlier_ratio) / (1 - min_inlier_ratio)
```

So the score is 0 when every point is an inlier and `score_threshold` at
`min_inlier_ratio`. The measurement gate, the pose covariance, the map-to-odom anchor
gate and the recovery all read it, so their thresholds keep their meaning.
`registrationFitness` (`registration_fitness.hpp`) gets the ratio from the same
nearest-neighbour searches as the fitness, so the fitness stays bit-identical.
`/alignment_status` reports the backend's own fitness as `registration_fitness_score`,
and the ratio as `inlier_ratio`, whether or not the gate is on. A scan of fewer than
`inlier_ratio_min_points` (200) points gets a ratio of 0: at 116.9 s the blocked sensor
left 7 points, and all 7 lay near the map.

## Online, with RKO-LIO odometry

`experiments/koide_imu_ab/online.py` runs the online `outdoor_hard_02b` profile. The
runs were on one 4-vCPU container, 2026-11-04.

`outdoor_hard_01a`, one run each:

| `min_inlier_ratio` | RMSE, all | 105-185 s, hard (accepted) | 185-305 s, off the map | From 305 s | On the map |
| --- | --- | --- | --- | --- | --- |
| off | 3.636 m | 0.945 m (19%) | 6.52 m | 0.733 m | 0.664 m |
| 0.7 | 0.492 m | 0.157 m (81%) | 0.79 m | 0.498 m | 0.274 m |
| **0.8** | **0.428-0.434 m** | **0.136 m (66%)** | **0.66 m** | **0.503 m** | **0.272 m** |

The 0.8 row is the sweep's run; the profile's run scored 0.434 m over the whole
sequence.

The hard stretch now registers. The path then leaves the map from a pose that is right,
so odometry drifts 0.7 m instead of 9.9 m before the scans register again.

`outdoor_hard_02b` (the release check's sequence; off map is 163.3-259.0 s):

| `min_inlier_ratio` | Runs | RMSE, all | On the map | Off the map |
| --- | --- | --- | --- | --- |
| off | 3 | 0.233 m | 0.181-0.182 m | 0.319-0.320 m |
| 0.7 | 2 | 0.280-0.283 m | 0.172-0.179 m | 0.431 m |
| **0.8** | 3 | **0.236-0.242 m** | **0.170-0.171 m** | 0.338-0.353 m |

- **On the map, both thresholds help.**
- **Off the map, 0.7 costs 0.11 m.** It accepts twice as many scans at the map's edge,
  150-180 s, and the drift from there is larger.
- **0.8 keeps the RMSE over the whole sequence within 0.01 m of off.** So 0.8 is the
  value for the online Koide profile.

## Boreas and Istanbul

- **Boreas:** at 0.7 and at 0.8 the preset's trajectory is byte-identical to `main`'s
  (0.132 m), because every scan passes either way. With the gate off it is too.
- **Istanbul (`istanbul_replay.py`, the GTSAM smoother preset):** 0.8 at 0.5 m lost
  the 2300 s window, but 0.8 at 1.0 m left all three windows within run-to-run spread
  or better:

| Window | Off | 0.8 at 0.5 m | 0.8 at 1.0 m |
| --- | --- | --- | --- |
| 0 s, 60 s long | 0.450 m | 0.347 m | 0.369 m |
| 2820 s, 150 s long | 0.937 m | 0.896 m | 0.927 m |
| 2300 s, 120 s long | 0.559-0.573 m | 139.5 m | 0.562 m |

The values are 3D RMSE over each window.

- **The map sets the inlier distance.** Without the gate, the Istanbul alignments have
  inlier ratios of 0.58-0.93 at 0.5 m (5th to 95th percentile, median 0.84) and
  0.93-1.00 at 1.0 m. At 0.5 m, 0.8 reads a third of the good alignments as bad.
- **The smoother reads the score too.** It scales a measurement's variance by
  `(score / gtsam_fitness_nominal)^2`, with a nominal score of 1.0. At 0.5 m every
  recorded scan scored above 18 within the first second, so the smoother all but
  ignored them, and the track was lost, with up to 268 m of z error.
- **`inlier_ratio` is in `/alignment_status` with the gate off,** so it can be read on a
  stretch that tracks well before choosing the distance and the threshold.

## Without odometry, to 140-151 s

`experiments/koide_imu_ab/replay.py` runs the localizer alone, with
`use_imu_preintegration`. It now rebuilds the local map in the background, as the
online profile does. "Tracked until" is the last pose; every pose up to it lies within
1 m of the ground truth (2026-11-04):

| `min_inlier_ratio` | Runs | Tracked until | 0-105 s | 105-185 s, hard (accepted) |
| --- | --- | --- | --- | --- |
| off | 1 | 106.4 s, then up to 15.6 m off | 0.078 m | 2.56 m (20%) |
| **0.8** | 3 | **140.2 s** (all three) | 0.066 m | **0.123-0.124 m** (36%) |
| 0.7 | 3 | 147.2 s (two), 151.5 s | 0.073-0.075 m | 0.137-0.139 m (44-48%) |

The 151.5 s run of 0.7 also had the IMU seed consistency gate off and a 15° correction
guard.

- **The gate carries the localizer through 106-140 s.** Without it, the fitness score
  accepts the 106.5 s pose that is 1.9 m and 16° off, as before.
- **Where 0.8 ends, at 140.3 s, the alignments are still close.** NDT corrects the IMU
  seed by 0.1-0.45 m, but the inlier ratio is 0.75-0.79. At 135-155 s the alignments
  from the ground truth have a median of 0.78 (the first table). After a second of
  rejections the IMU prediction runs out, the previous delta seeds the scans, and they
  land at 0.07-0.31.
- **Where 0.7 ends, the scans do not register even from the ground truth.** At
  151.6-153.5 s the alignments from the ground truth have inlier ratios of 0.48-0.76,
  and NDT moves them up to 1.0 m off (0.91 m at 151.6 s, 1.00 m at 153.0 s). At 147.9 s
  the ground truth has 0.42.
- **Off the map, at 185-305 s, there is nothing to register,** so without odometry
  the track could not continue past it anyway.

These did not get further:

| Change | `min_inlier_ratio` 0.8 | 0.7 |
| --- | --- | --- |
| none | 140.2 s | 147.2 s |
| IMU seed consistency gate off, correction guard 15° instead of 4° | 140.2 s | 151.5 s (one run) |
| the same, and the IMU prediction kept for 3 s instead of 1 s | 137.7 s | 144.3 s |

- **The IMU seed consistency gate turns off after the 0.9 s cloud gap at 104.8 s.**
  The prediction across it is 0.61 m off, over the 0.5 m limit. A retry from the last
  pose at 106.7 s is accepted, and the gate turns on again after 5 scans.
- **A longer IMU prediction does not help.** At 0.8 the IMU-seeded scans of
  137.9-140.7 s are still rejected, at 0.62-0.79. At 0.7 the seed has drifted by 144.7 s:
  NDT moves it by 0.1-0.3 m to an inlier ratio of 0.38-0.52, where the ground truth has
  0.83-0.85.

**The earlier runs rebuilt the map in the callback.** Each rebuild dropped clouds, 10% in
all (diagnostic coverage 90.3%, now 99.8-99.9%). At 0.8 the track then ended at 105.9 s;
with the IMU seed gate off at 128.2 s, and with the guard at 15° too at 140.2 s. That made
both look like the cause. With the background rebuild, neither changes 0.8.

## Rerun

```bash
source /opt/ros/jazzy/setup.bash
python3 experiments/inlier_ratio_gate/export_scans.py KOIDE_DATA /tmp/scans 100 190
g++ -O3 -DNDEBUG -std=c++17 -fopenmp -I"$WS/src/ndt_omp_ros2/include" \
  -I/usr/include/pcl-1.14 -I/usr/include/eigen3 \
  experiments/inlier_ratio_gate/koide_probe.cpp -o /tmp/koide_probe \
  -lpcl_common -lpcl_filters -lpcl_kdtree -lpcl_search -lpcl_registration
/tmp/koide_probe KOIDE_DATA/map_outdoor_hard.ply /tmp/scans oracle
# seeds around (X, Y, Z, YAW), radius 6 m in 1 m steps, yaw +-45 deg in 15 deg steps
/tmp/koide_probe KOIDE_DATA/map_outdoor_hard.ply /tmp/scans search \
  -47.65 73.34 -12.38 73.1 6 1.0 45 15 FIRST LAST x
python3 experiments/koide_imu_ab/online.py KOIDE_DATA outdoor_hard_01a /tmp/run \
  --ws WS --rko RKO_ONLINE_NODE --set min_inlier_ratio=0.8
# Without odometry
python3 experiments/koide_imu_ab/replay.py KOIDE_DATA outdoor_hard_01a /tmp/alone \
  --ws WS --set use_imu_preintegration=true --set min_inlier_ratio=0.8
# Istanbul windows: --start 0 --duration 60, 2820 150, 2300 120
python3 tools/benchmark/istanbul_replay.py --ws WS --fixture FIXTURE --bag BAG --map MAP \
  --start 2300 --duration 120 --out /tmp/ist \
  --set min_inlier_ratio=0.8 --set inlier_ratio_distance_m=1.0
```
