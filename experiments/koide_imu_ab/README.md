# IMU preintegration on Koide `outdoor_hard_01a`

Question: does the IMU preintegration help on a second dataset? Boreas (a car, a
Velodyne and an Applanix IMU) gained 0.005-0.007 m RMSE with it
([docs/imu_estimation.md](../../docs/imu_estimation.md#validation-state-honest-limits)).
Koide `outdoor_hard_01a` is a handheld Livox MID-360 with its built-in IMU, 380 s on the
outdoor map.

## Setup

`replay.py` reruns the A/B prepared before 6825f05
(`param/benchmark/koide_hard_localization_outdoor_hard_01a_full380_ndt_omp_imu_{off,on}.yaml`,
`git show 6825f05^:...`): `param/nav2_ndt_urban.yaml` with its Koide overrides
(NDT_OMP, 0.5 m scan voxels, `livox_frame`, 1-100 m, score threshold 6, recovery
retry after 3 rejections), and only `use_imu_preintegration` differing.

- **The localizer runs alone.** There is no odometry, unlike the online Koide check
  of `outdoor_hard_02b`, which bridges with RKO-LIO.
- **It starts at the ground-truth pose 4 s into the bag** and plays from there
  with `/clock`: 2860 clouds.
- **The IMU is converted from g to m/s²** once (`koide_public_bag.prepare_si_bag`,
  now for any Koide sequence).
- **`/pcl_pose` is matched to `traj_lidar_outdoor_hard_01.txt` by stamp** (within
  0.05 s), without alignment.

## Result

Two runs each, 2026-11-03, ROS 2 Jazzy, 4 vCPUs:

| `use_imu_preintegration` | Poses / 2860 clouds | Last accepted scan | RMSE, first 30 s | RMSE / max, all poses |
| --- | --- | --- | --- | --- |
| false | 228, 321 (8%, 11%) | 35 s, 49 s | 0.071 m (both) | 0.372 / 5.1 m, 0.154 / 2.4 m |
| **true** | **766, 774 (27%)** | **111 s** (both) | 0.114 m, 0.072 m | 0.770 / 16.4 m, 0.157 / 2.1 m |

- **Without the IMU the localizer loses the track at 35-49 s.** The fitness climbs
  from about 31 s, then every scan is rejected, and the error never comes back:
  from about 60 s every scan requests a reinitialization.
- **With it, the localizer tracks through that stretch to 111 s in both runs.**
  That is 2.3-3.2 times as long, at the same accuracy where both track (0.071-0.072 m
  over the first 30 s). The exception is a spike above 1 m at 22 s in one run.
- **Neither covers the sequence.** After 111 s the IMU runs are lost too, and their
  last poses are wrong by up to 16 m. That run's RMSE over all poses is the worst of
  the four for this reason. Compare the runs by coverage first, as the doc says.
- **Both IMU runs stop at 111 s because the bag has holes there:** gaps in the clouds,
  a gap in the IMU and a blocked sensor (next section). A whole Koide sequence needs
  odometry, as in the online check.

So on Koide the preintegration does not show an accuracy gain where both track. It
shows robustness: it carries the localizer through a stretch where it otherwise
loses the track.

## Why the IMU runs stop at 111 s

The localizer processes every cloud of the bag; the bag itself has holes. All four
come within 8 s, in seconds from the start pose:

| When | What the bag has |
| --- | --- |
| 103.2-104.6 s | no cloud (1.4 s) |
| 104.6-105.25 s | no IMU (0.65 s, its only IMU gap) |
| 105.3-106.2 s | no cloud (0.9 s) |
| 110.2-110.7 s | the sensor is blocked: 75-100% of the points within 1 m, 0-2568 left after filtering |

**The IMU dropout exposed a bug.** `planCausalImuIntervals` compared its 0.5 s hold
limit with the scan window, not with the age of the held sample. So the last sample
before the dropout was held through every scan of it, each 0.1 s window looking
covered. After the 0.9 s cloud gap, the prediction at 106.2 s was 0.99 m and 4.9° off,
and the track degraded until the blind stretch. The limit now counts from the held
sample's stamp. Scans more than 0.5 s into the dropout get no IMU seed, and the pose
is updated alone. The prediction at 106.2 s is then 0.61 m and 1.3° off.

**Both runs with the fix then track through 106-110 s, but they still stop at the
blind stretch:**

| Build | Poses / 2860 clouds | Error above 2 m from | RMSE, first 30 s |
| --- | --- | --- | --- |
| before the fix | 766, 774 | 110.2 s, 111.7 s | 0.114 m, 0.072 m |
| with the fix | 780, 781 | 111.6 s (both) | 0.072 m (both) |

**What is left is the IMU-only prediction across the blind stretch.**
- The last good scan is at 109.8 s. The ground truth then turns 16.5° and moves 1.2 m
  before the next usable scan, at 110.7 s.
- The prediction there is about 1 m and 10° off. NDT moves it only 0.26 m and 3° on
  the three-quarters-blocked scan, and the pose ends 0.93 m and 11.5° from the
  ground truth.
- The next scans are rejected, and the retry lands 3 m off.
- The bag's gyro, integrated over the same 0.9 s, comes within 0.7° of the ground
  truth, so the IMU data is not the problem.

## The smoother is reset to wrong measurements

`/alignment_status` now reports the smoother's biases and velocity
(`imu_gyro_bias_*`, `imu_accel_bias_*`, `imu_velocity_*`). One more run with them
shows that the biases are not the cause:

- **The estimated biases are plausible.** The gyro bias is 0.7-1.0°/s through
  105 s, which would turn a 0.9 s prediction by under 1°. The accelerometer bias is
  0.07 m/s². On Boreas, with its Applanix IMU, they are 0.02-0.07°/s and
  0.01-0.03 m/s².
- **What breaks the state is a reset onto a wrong measurement.**
  - At 107.0 s NDT accepted, with a warning, a pose 1.9 m and 16° from the ground
    truth. The smoother disagreed with it by 1.5 m and 15°.
  - Above its fixed 2 m / 10° limit, the divergence guard takes the LiDAR as right. So
    it reset the smoother to that pose: biases back to zero, and a velocity of 3.8 m/s
    from the two poses across the 0.6 s before it. The ground truth moved at 1.1 m/s.
  - All later scans were rejected, so nothing corrected it.
- **Every IMU run here goes through such resets.** The two runs before the fix and the
  two after it reset 3-5 times each from about 107 s on, at 11-47° of disagreement;
  the run above reset once. The 10° prediction at 110.7 s comes after such resets at
  107 s.
- **Boreas never trips the guard**, over 120 s or 600 s.

When a measurement only just passes and the smoother disagrees by more than 10°, the
guard cannot tell which of the two is wrong. On this sequence it is the measurement.
Rejecting such a measurement, or keeping the smoother's velocity on a reset, would be
the next thing to try.

## Rerun

```bash
# DATA_DIR holds map_outdoor_hard.ply, gt/ and outdoor_hard_01a.zip from
# https://zenodo.org/records/10122133 (md5 d126024c8f310c2c48239f136d7e0ed0).
for imu in false true; do
  python3 experiments/koide_imu_ab/replay.py DATA_DIR outdoor_hard_01a \
    /tmp/koide_imu_$imu --ws "$WS" --set use_imu_preintegration=$imu
done
```
