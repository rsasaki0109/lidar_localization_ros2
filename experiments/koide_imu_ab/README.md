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
- **Why both IMU runs stop at 111 s is not known.** The map has points under the
  path there; whether it has enough structure to register against was not checked.
  A whole Koide sequence needs odometry, as in the online check.

So on Koide the preintegration does not show an accuracy gain where both track. It
shows robustness: it carries the localizer through a stretch where it otherwise
loses the track.

## Rerun

```bash
# DATA_DIR holds map_outdoor_hard.ply, gt/ and outdoor_hard_01a.zip from
# https://zenodo.org/records/10122133 (md5 d126024c8f310c2c48239f136d7e0ed0).
for imu in false true; do
  python3 experiments/koide_imu_ab/replay.py DATA_DIR outdoor_hard_01a \
    /tmp/koide_imu_$imu --ws "$WS" --set use_imu_preintegration=$imu
done
```
