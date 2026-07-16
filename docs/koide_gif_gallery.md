# Koide Dataset GIF Gallery

This page covers every reference trajectory in the
[Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133).
The route-overview GIFs animate published ground truth; they are dataset previews,
not localization accuracy results. Measured localization replays are listed separately.

## Dataset coverage

The publication provides eight reference trajectories in eleven downloadable ROS 2 bag
parts. Outdoor recordings are split into `a` and `b` archives while sharing one continuous
reference trajectory.

| Reference trajectory | ROS 2 bag archive(s) | Environment |
|---|---|---|
| `indoor_easy_01` | `indoor_easy_01` | Indoor |
| `indoor_easy_02` | `indoor_easy_02` | Indoor |
| `indoor_hard_01` | `indoor_hard_01` | Indoor |
| `indoor_kidnap_01` | `indoor_kidnap_01` | Indoor, kidnapped |
| `indoor_kidnap_02` | `indoor_kidnap_02` | Indoor, kidnapped |
| `outdoor_hard_01` | `outdoor_hard_01a`, `outdoor_hard_01b` | Outdoor |
| `outdoor_hard_02` | `outdoor_hard_02a`, `outdoor_hard_02b` | Outdoor |
| `outdoor_kidnap` | `outdoor_kidnap_a`, `outdoor_kidnap_b` | Outdoor, kidnapped |

## Indoor routes

### Indoor easy 01

![Koide indoor_easy_01 ground-truth route](../images/koide/indoor_easy_01.gif)

### Indoor easy 02

![Koide indoor_easy_02 ground-truth route](../images/koide/indoor_easy_02.gif)

### Indoor hard 01

![Koide indoor_hard_01 ground-truth route](../images/koide/indoor_hard_01.gif)

### Indoor kidnap 01

![Koide indoor_kidnap_01 ground-truth route](../images/koide/indoor_kidnap_01.gif)

### Indoor kidnap 02

![Koide indoor_kidnap_02 ground-truth route](../images/koide/indoor_kidnap_02.gif)

## Outdoor routes

### Outdoor hard 01

![Koide outdoor_hard_01 ground-truth route](../images/koide/outdoor_hard_01.gif)

### Outdoor hard 02

![Koide outdoor_hard_02 ground-truth route](../images/koide/outdoor_hard_02.gif)

### Outdoor kidnap

![Koide outdoor_kidnap ground-truth route](../images/koide/outdoor_kidnap.gif)

## Measured localization replays

These GIFs are generated from recorded `/pcl_pose`, not copied from ground truth. Each
run covers the first 30 seconds of its bag, except `outdoor_kidnap_a`, which starts at
`+0.6 s` so the published ground truth overlaps the replay. Indoor runs are LiDAR-only;
outdoor runs use scan-bounded dual-queue IMU preintegration with the correction guard
and `imu_accel_scale: 9.80665` (the Koide Livox bags publish acceleration in g).

The table below is the 2026-07-17 measurement on a quiet machine. Two earlier defects
were fixed before it: the IMU acceleration unit/variance bugs (dual-queue
preintegration work) and a missing `imu_accel_scale` in the generated benchmark
manifests. The 2026-07-13 artifacts are archived next to the current runs
(`runs_20260713/`). Coverage is the accepted-pose time span over the 30 s window; a
low coverage means the estimator stopped publishing rather than tracked badly.

This is a short-window engineering check, not a full-sequence benchmark. Local
diagnostics cannot prove accuracy: `indoor_kidnap_01` locks onto a wrong,
locally-self-consistent match (map aliasing) that only ground truth reveals.

| Bag | Mode | Matched poses | Coverage | Translation RMSE | Rotation RMSE | Result |
|---|---|---:|---:|---:|---:|---|
| `indoor_easy_01` | LiDAR | 315 | 94.3% | 0.061 m | 1.46 deg | Bounded |
| `indoor_easy_02` | LiDAR | 342 | 92.9% | 0.127 m | 1.25 deg | Bounded |
| `indoor_hard_01` | LiDAR | 289 | 93.3% | 5.444 m | 131.08 deg | Failed |
| `indoor_kidnap_01` | LiDAR | 52 | 15.6% | 7.237 m | 115.92 deg | Aliased |
| `indoor_kidnap_02` | LiDAR | 223 | 66.2% | 8.624 m | 119.07 deg | Failed |
| `outdoor_hard_01a` | LiDAR + IMU | 93 | 90.0% | 0.198 m | 0.99 deg | Bounded |
| `outdoor_hard_01b` | LiDAR + IMU | 84 | 91.7% | 0.668 m | 23.31 deg | Degraded |
| `outdoor_hard_02a` | LiDAR + IMU | 91 | 91.0% | 0.181 m | 0.97 deg | Bounded |
| `outdoor_hard_02b` | LiDAR + IMU | 101 | 92.3% | 0.243 m | 1.06 deg | Bounded |
| `outdoor_kidnap_a` | LiDAR + IMU | 97 | 93.7% | 0.209 m | 0.58 deg | Bounded |
| `outdoor_kidnap_b` | LiDAR + IMU | 106 | 91.7% | 0.271 m | 0.73 deg | Bounded |

Versus the 2026-07-13 measurement, the IMU fixes repaired every outdoor failure mode
but one: `outdoor_hard_01a` 0.400 m/48 poses -> 0.198 m/93 poses, `outdoor_hard_02a`
14 poses (83% of the window unscored) -> 91 poses, `outdoor_hard_02b` 1.593 m/12.0 deg
-> 0.243 m/1.06 deg, `outdoor_kidnap_b` 0.569 m/5.1 deg -> 0.271 m/0.73 deg.
`outdoor_hard_01b` improved (1.347 m/28.6 deg -> 0.668 m/23.3 deg) but still has one
late rotation-divergence event with the error growing at the window end. The indoor
hard/kidnap failures are unchanged and tracked separately: sparse deskew-incapable
depth-camera scans (~100 filtered points) destabilize NDT on `indoor_hard_01` /
`indoor_kidnap_02`, and `indoor_kidnap_01` needs global verification, not local
tuning. `indoor_easy_02` showed one transient bad run during the sweep (archived as
`runs/indoor_easy_02_outlier_run1`); the tabulated rerun matches its historical
behavior.

### Indoor easy 01 measured

![Koide indoor_easy_01 measured localization](../images/koide/measured/indoor_easy_01.gif)

### Indoor easy 02 measured

![Koide indoor_easy_02 measured localization](../images/koide/measured/indoor_easy_02.gif)

### Indoor hard 01 measured

![Koide indoor_hard_01 measured localization](../images/koide/measured/indoor_hard_01.gif)

### Indoor kidnap 01 measured

![Koide indoor_kidnap_01 measured localization](../images/koide/measured/indoor_kidnap_01.gif)

### Indoor kidnap 02 measured

![Koide indoor_kidnap_02 measured localization](../images/koide/measured/indoor_kidnap_02.gif)

### Outdoor hard 01a measured

![Koide outdoor_hard_01a measured localization](../images/koide/measured/outdoor_hard_01a.gif)

### Outdoor hard 01b measured

![Koide outdoor_hard_01b measured localization](../images/koide/measured/outdoor_hard_01b.gif)

### Outdoor hard 02a measured

![Koide outdoor_hard_02a measured localization](../images/koide/measured/outdoor_hard_02a.gif)

### Outdoor hard 02b measured

![Koide outdoor_hard_02b measured localization](../images/koide/measured/outdoor_hard_02b.gif)

### Outdoor kidnap a measured

![Koide outdoor_kidnap_a measured localization](../images/koide/measured/outdoor_kidnap_a.gif)

### Outdoor kidnap b measured

![Koide outdoor_kidnap_b measured localization](../images/koide/measured/outdoor_kidnap_b.gif)

## Reproduce the route gallery

Keep the large maps, bags, generated occupancy maps, and reference CSV files outside the
repository. For example, with the dataset on an external SSD:

```bash
scripts/render_koide_dataset_gallery.sh \
  --data-dir /media/sasaki/aiueo/datasets/koide_hard_localization
```

Prepare the measured 30-second benchmark manifests after sourcing ROS 2:

```bash
python3 scripts/prepare_koide_localization_gif_benchmarks.py \
  --data-dir /media/sasaki/aiueo/datasets/koide_hard_localization \
  --output-root /media/sasaki/aiueo/datasets/koide_hard_localization/generated/localization_gif_benchmarks
```

Each GIF contains 64 frames at 648 x 450. The red arrow is derived from trajectory motion,
including a minimum spatial baseline at stops and trajectory endpoints, so it represents
the direction of travel rather than the sensor quaternion.

Dataset citation: Kenji Koide, *Hard Point Cloud Localization Dataset*, Zenodo, 2023,
<https://doi.org/10.5281/zenodo.10122133>. Dataset files are distributed under CC BY 4.0.
