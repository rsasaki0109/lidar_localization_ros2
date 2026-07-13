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
outdoor runs use guarded IMU preintegration.

Indoor IMU was evaluated on `indoor_easy_01` with the published
`depth_camera_link` to `imu_link` extrinsic and the correction guard enabled. Across
three 30-second repeats, LiDAR-only achieved median translation/rotation RMSE of
0.070 m / 1.58 deg and 100.0% diagnostic OK, while LiDAR + IMU achieved
2.749 m / 4.69 deg and 94.2% diagnostic OK. The IMU transform failure count was zero
in all three IMU runs, but every IMU run had higher translation RMSE, so indoor IMU was
not adopted for these replays.

This is a short-window engineering check, not a full-sequence benchmark. `Partial` means
the matched poses were locally accurate but tracking coverage was low. A high diagnostic
OK rate does not prove accuracy: `indoor_kidnap_01` is the clearest counterexample, with
100% diagnostic OK rows but 11.10 m translation RMSE against ground truth.

| Bag | Mode | Matched poses | Translation RMSE | Rotation RMSE | Diagnostic OK | IMU seed | Result |
|---|---|---:|---:|---:|---:|---:|---|
| `indoor_easy_01` | LiDAR | 302 | 0.066 m | 1.37 deg | 100.0% | n/a | Bounded |
| `indoor_easy_02` | LiDAR | 332 | 0.053 m | 1.35 deg | 100.0% | n/a | Bounded |
| `indoor_hard_01` | LiDAR | 83 | 6.012 m | 46.63 deg | 25.6% | n/a | Failed |
| `indoor_kidnap_01` | LiDAR | 298 | 11.095 m | 131.45 deg | 100.0% | n/a | Aliased |
| `indoor_kidnap_02` | LiDAR | 183 | 9.808 m | 107.51 deg | 53.7% | n/a | Failed |
| `outdoor_hard_01a` | LiDAR + IMU | 48 | 0.400 m | 3.88 deg | 58.8% | 63.8% | Partial |
| `outdoor_hard_01b` | LiDAR + IMU | 46 | 1.347 m | 28.64 deg | 50.7% | 62.7% | Failed |
| `outdoor_hard_02a` | LiDAR + IMU | 14 | 0.186 m | 0.51 deg | 16.7% | 17.9% | Partial |
| `outdoor_hard_02b` | LiDAR + IMU | 63 | 1.593 m | 12.03 deg | 59.8% | 68.6% | Failed |
| `outdoor_kidnap_a` | LiDAR + IMU | 91 | 0.191 m | 0.89 deg | 89.8% | 96.9% | Bounded |
| `outdoor_kidnap_b` | LiDAR + IMU | 97 | 0.569 m | 5.12 deg | 84.2% | 93.9% | Drifted |

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
