# Solution remapping on Koide `indoor_hard_01`

Question: does solution remapping (`enable_degeneracy_remapping`, added in 1.14 from a
simulated road tunnel) help on recorded data, and should quickstart turn it on?

Data: the [Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133)
(Koide et al., CC BY 4.0), sequence `indoor_hard_01` (an Azure Kinect carried through
corridors and rooms, "quick sensor motion and point cloud degeneration"), its map
`map_indoor_hard.ply` and its ground truth `traj_lidar_indoor_hard_01.txt`. 2378 clouds
at 15 Hz, each voxel-downsampled to 5 cm (about 9,000 points).

## Tools

- `extract.py BAG_DIR MAP.ply GT.txt OUT_DIR` (needs `rosbags` and `numpy`) writes the map,
  the clouds and the ground-truth pose at each cloud's stamp.
- `replay.c` runs every cloud through `lidarloc_localize` (the C API: the node's per-scan
  tracking, gate and seed correction guards, without ROS) with NDT_OMP at 0.5 m on the
  whole map, the seed correction guard at 0.5 m and the window guard at 0.3 m over 2 s (as
  quickstart turns them on with an odometry seed). Two modes:
  - tracking: each seed is the tracked pose moved by the ground-truth motion since the
    previous cloud, its translation scaled by `SCALE` (an odometry reading long);
  - per scan (`PER_SCAN_OFFSET_M`): each cloud on its own, tracking restarted at the
    previous cloud's ground truth and seeded with this cloud's ground truth moved that far
    horizontally (in a direction that turns by the golden angle from cloud to cloud).

```bash
python extract.py indoor_hard_01 map_indoor_hard.ply traj_lidar_indoor_hard_01.txt out
gcc -O2 -o replay replay.c -I$PREFIX/include $PREFIX/lib/liblidar_localization_core.a \
  -lpcl_registration -lpcl_features -lpcl_filters -lpcl_sample_consensus -lpcl_search \
  -lpcl_kdtree -lpcl_octree -lpcl_common -lflann_cpp -llz4 -lgomp -lstdc++ -lm
# REMAP MIN_HC SCALE: off; by the Hessian; by the Hessian and the map's surfaces
PER_SCAN_OFFSET_M=0.02 ./replay out/map.bin out/scans.bin out/gt.bin 1 0.02 1.0 trace.csv
```

## Per scan: where NDT leaves a good seed

Seeded 2 cm from the truth (an odometry-grade seed), NDT should stay put. Position error
after each cloud (3D, m), over 2378 clouds:

| remapping | median | p90 | p99 | clouds > 0.1 m | clouds > 0.3 m |
|---|---|---|---|---|---|
| off | 0.032 | 0.211 | 1.24 | 398 | 164 |
| by the Hessian (`weak_ratio` 0.05) | 0.027 | 0.166 | 1.02 | 314 | 125 |
| and the map's surfaces (`min_horizontal_constraint` 0.02) | 0.026 | **0.096** | 0.94 | **227** | **94** |

Without remapping NDT moved one cloud in six more than 10 cm away from a seed that was 2
cm off. Remapping by the Hessian kept a fifth of them, and by the map's surfaces too, 43
%. The surfaces under the cloud pinned one horizontal direction by less than 0.02 in 679
clouds (29 %) and both in 13: corridors, not bare floor. What remains is mostly error the
remapping cannot drop: height, tilt and the yaw the 3D error includes, and corrections
across a corridor.

Seeded 0.3 m off instead, remapping keeps the seed's error along the weak direction, by
design: median 0.039 m off, 0.078 by the Hessian, 0.119 with the surfaces; p99 1.24, 1.13,
0.95. It assumes the seed is right along the direction the scan cannot see, so it is for
an odometry seed, not for a guess.

## Tracking: the sequence defeats every configuration

Tracked from the first cloud, the localizer lost the sequence in every configuration
tried, with or without remapping, with an exact odometry (`SCALE` 1.0) or one 5 % long:
RMSE 4-37 m, and which configuration did least badly changed with the guards' settings
(without the window guard, remapping by the surfaces with the long odometry ended at 0.51
m RMSE; with it, at 21 m). Once a cloud is accepted metres off nothing brings the pose
back, so these runs say nothing about remapping. All of them first went more than 1 m off
36 s in, where the sensor turns at 49 deg/s (median; 96 deg/s at p90) against 34 deg/s
before; the replay does not deskew.

## Conclusion

On recorded data, remapping does what it did in simulation: it keeps NDT from moving a
good seed along directions the scan does not constrain, and the map's surfaces find more
of those directions than the Hessian alone. It stays opt-in: it trusts the seed along those
directions, which is right for an odometry seed and wrong for a coarse guess, and this
sequence gives no tracking result to justify turning it on by default.
