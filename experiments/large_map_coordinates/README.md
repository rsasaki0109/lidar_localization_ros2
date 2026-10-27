# Maps far from the origin (#68)

Question: does NDT lose accuracy when the map's coordinates are large, as in an
Autoware MGRS map (up to 100 km) or a UTM map (millions of meters)? Map points,
the initial guess and the result are float32 in the localizer, and float32 values
are 7.8 mm apart at 100 km and 0.25 m apart at 3.9e6 m.

## Method

[ndt_offset_check.cpp](ndt_offset_check.cpp) builds one synthetic street scene
(ground, facades, blocks, poles, parked cars; 478k points) and 12 scans from
known poses (40 m range, 2 cm noise, 0.5 m voxel). Each scan starts from a guess
up to 0.4 m off in x and y, 0.1 m in z and 1.7 deg in yaw. The scene is then shifted by each offset,
stored as float32, and every scan is registered with NDT_OMP at the
`param/localization.yaml` settings (resolution 1.0, step 0.1, epsilon 0.01, 35
iterations, DIRECT7).

The useful column is **vs0**: how far each trial's result moves against the same
trial at offset 0. Only the offset differs, so vs0 is the effect of the offset
alone. The error against ground truth includes a ~68 mm bias of this synthetic
scene, which is the same at every offset.

## Result

12 trials, all converged:

| Offset (m) | Error median (mm) | Error max (mm) | vs0 median (mm) | vs0 max (mm) |
| --- | --- | --- | --- | --- |
| (0, 0, 0) | 67.8 | 72.2 | 0 | 0 |
| (10000, 10000, 0) | 67.3 | 71.5 | 0.67 | 3.16 |
| (99950, 99950, 50), MGRS worst case | 64.3 | 72.8 | 3.46 | 7.84 |
| (350000, 3900000, 0), UTM | 97.7 | 183.4 | 64.07 | 183.21 |

- MGRS-sized coordinates move the result by at most one float32 step (7.8 mm). They
  are usable as they are.
- UTM-sized coordinates move it by up to 0.18 m, and the map file itself (float32)
  already rounds points to 0.25-0.5 m. Shift such maps near the origin first.

The localizer warns at map load from 131072 m (2^17, 1.6 cm float32 spacing) on.

This is a synthetic check of numerical precision, not a dataset benchmark.

## Rerun

From the repository root, with the workspace built (`../build_ws`), about 35 minutes
on 4 cores:

```bash
g++ -O2 -std=c++17 -fopenmp experiments/large_map_coordinates/ndt_offset_check.cpp \
  -o /tmp/ndt_offset_check \
  -I../build_ws/install/ndt_omp_ros2/include -I/usr/include/pcl-1.14 -I/usr/include/eigen3 \
  -L../build_ws/install/ndt_omp_ros2/lib -lndt_omp \
  -lpcl_registration -lpcl_filters -lpcl_kdtree -lpcl_search -lpcl_common -lflann_cpp -llz4
/tmp/ndt_offset_check
```

Measured 2026-10-27 on ROS 2 Jazzy (PCL 1.14, ndt_omp_ros2 `humble` branch).
