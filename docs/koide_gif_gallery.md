# Koide GLIL-Style Prior-Map Localization Gallery

This page tracks GLIL-only localization on the
[Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133). The main
configuration jointly optimizes IMU preintegration, exact-coreset scan-to-scan GICP,
and exact-coreset scan-to-prior-map GICP in one GLIM fixed-lag graph. It follows the
[sliding-window estimator structure of Koide et al. ICRA 2024](https://arxiv.org/abs/2402.05540)
and the [deferred exact point-cloud downsampling method of Koide et al. ICRA 2025](https://arxiv.org/abs/2505.01017).
NDT is not launched.

Ground truth appears only as a dashed evaluation overlay. Standalone animated
ground-truth routes are intentionally omitted because they do not demonstrate
localization accuracy or recovery behavior.

## Measured localization replays

### Tightly coupled GLIL — NDT-free

The tightly coupled implementation is under final four-sequence and kidnap measurement.
Unlike the earlier split, scan-to-map results are not solved independently and converted
to smoothed corrections. Each accepted map observation constrains the same active pose
states as scan-to-scan and IMU factors. Loss of map overlap omits only map factors;
continuous range-inertial odometry and the last bounded `map -> odom` remain available.
Verified recovery creates a new graph map-state epoch after three consistent frames,
without resetting raw odometry or adding a second TF authority.

The final 01a/01b/02a/02b table and GLIL-only GIFs will replace this measurement notice
after the pinned image completes the acceptance matrix.

### Legacy split prior-map baseline (2026-07-19)

These retained results are the first full live replay of the GLIL-style split: GLIM supplies continuous
LiDAR/IMU `odom -> base_link`, while sparse prior-map VGICP registrations update only a
smoothed `map -> odom`. It does **not** run NDT alongside GLIM. Direct VGICP is the
normal submap-rate tracker; KISS-Matcher runs only as an acquisition or recovery
fallback. Rejected localization therefore freezes the last map anchor instead of
interrupting or deforming GLIM odometry.

| Bag | Front end | Coverage | Translation ATE | Final error | Median 10 m RPE | Rotation ATE | Processing p95 | Gate |
|---|---|---:|---:|---:|---:|---:|---:|---|
| `outdoor_hard_01a` | GLIM exact coreset 32 + sparse VGICP | 99.21% | 1.142 m | 3.112 m | 0.157 m | 1.479 deg | 28.3 ms | Pass |
| `outdoor_hard_01b` | GLIM exact coreset 32 + sparse VGICP | 99.04% | 1.539 m | 2.055 m | 0.191 m | 1.598 deg | 53.8 ms | Pass |
| `outdoor_hard_02a` baseline repeat 1 | GLIM exact coreset 32 + sparse VGICP | 99.15% | 2.059 m | 3.078 m | 0.180 m | 1.676 deg | 35.8 ms | ATE fail |
| `outdoor_hard_02a` baseline repeat 2 | GLIM exact coreset 32 + sparse VGICP | 99.15% | 2.023 m | 3.049 m | 0.183 m | 1.715 deg | 37.2 ms | ATE fail |
| `outdoor_hard_02a` Z-bridge repeat 1 | GLIM exact coreset 32 + sparse VGICP | 99.15% | 1.926 m | 2.854 m | 0.181 m | 1.683 deg | 38.5 ms | Pass |
| `outdoor_hard_02a` Z-bridge repeat 2 | GLIM exact coreset 32 + sparse VGICP | 99.15% | 1.795 m | 2.827 m | 0.181 m | 1.703 deg | 30.4 ms | Pass |
| `outdoor_hard_02b` | GLIM exact coreset 32 + sparse VGICP | 99.03% | 0.908 m | 1.412 m | 0.177 m | 1.765 deg | 47.9 ms | Pass |

The startup yaw is fixed, accepted translation displacement uses gain 0.2, and GLIM
ROS applies a 2 s first-order transition. All four sequences retained at least 99%
output coverage, and every run had zero TF jumps and zero unauthorized resets. Direct
and KISS fallback candidates were rejected once they stopped satisfying the gates; the
frozen map correction plus live odometry carried the remaining output.

The original `outdoor_hard_02a` policy failed only the 2.0 m ATE gate in two repeats.
Its submap-30 prior-map candidate was good enough to enter two-submap recovery consensus,
but freezing Z while waiting moved the next submap away from the prior-map surface. The
Z-bridge variant keeps XY and yaw frozen while applying the candidate's vertical
map-to-odom correction. It passed two new full repeats at 1.926 m and 1.795 m ATE with
zero TF jumps and zero unauthorized resets. This is opt-in with vertical gain 1.0; the
default remains backward compatible. The published ground-vehicle gate planarizes Z.
A separate non-planar check of repeat 2 improved from the earlier 10.4--10.7 m range to
9.21 m ATE but still failed, so full 3D drift and kidnapped-pose recovery remain open.
Ground truth is only the dashed evaluation overlay.

#### Four-bag Z-bridge regression

Vertical gain 1.0 was subsequently replayed across all four outdoor-hard bags with the
same image and unchanged gates. It passed at least once on every bag, but remains opt-in
because one of two 01b repeats exceeded the ATE limit.

| Bag | Repeat | Translation ATE | Final error | Median 10 m RPE | Processing p95 | Gate |
|---|---:|---:|---:|---:|---:|---|
| `outdoor_hard_01a` | 1 | 1.106 m | 2.738 m | 0.160 m | 60.3 ms | Pass |
| `outdoor_hard_01b` | 1 | 2.219 m | 2.412 m | 0.183 m | 78.3 ms | ATE fail |
| `outdoor_hard_01b` | 2 | 1.867 m | 2.022 m | 0.192 m | 94.1 ms | Pass |
| `outdoor_hard_02a` | 1 | 1.926 m | 2.854 m | 0.181 m | 38.5 ms | Pass |
| `outdoor_hard_02a` | 2 | 1.795 m | 2.827 m | 0.181 m | 30.4 ms | Pass |
| `outdoor_hard_02b` | 1 | 1.097 m | 1.538 m | 0.175 m | 81.4 ms | Pass |

Cross-composing the 01b artifacts isolated the failure: repeat-1 raw GLIM odometry with
the baseline map anchor still produced 2.210 m ATE, while baseline raw odometry with the
repeat-1 Z-bridge anchor produced 1.562 m. The anchor changed ATE by only about 0.02 m;
the failed repeat was dominated by raw-odometry variability. This clears the Z bridge of
a direct 01b regression, but one-pass-per-bag evidence is not sufficient to enable it by
default.

The raw-odometry variance was then isolated to parallel ordering in random-grid
preprocessing and the odometry optimizer. With both coreset-profile thread counts fixed
to one, two new full 01b replays produced byte-identical 2,218-row `odom_lidar.txt`
files (SHA-256 `45d49d87932febc06b4ab18b29fc1ba58fc6f6ed043a7157523fe4e75f383642`).
Both passed at 1.407079 m ATE and 1.957 m final error; RPE was 0.18690/0.18694 m and
processing p95 was 44.3/47.5 ms. Sparse prior-map VGICP still varied its Z anchor by up
to 4.3 cm, but the resulting planar ATE difference was below 0.000001 m. The vertical
bridge remains opt-in until this deterministic profile is replayed on the other bags and
the non-planar drift is resolved.

#### Outdoor hard 01a

![Koide outdoor_hard_01a GLIL live map-to-odom replay](../images/koide/measured/glil/outdoor_hard_01a_live_map_odom.gif)

#### Outdoor hard 01b

![Koide outdoor_hard_01b GLIL live map-to-odom replay](../images/koide/measured/glil/outdoor_hard_01b_live_map_odom.gif)

#### Outdoor hard 02a — Z-bridge repeat 2, all planar gates passed

![Koide outdoor_hard_02a GLIL live map-to-odom replay](../images/koide/measured/glil/outdoor_hard_02a_live_map_odom.gif)

#### Outdoor hard 02b

![Koide outdoor_hard_02b GLIL live map-to-odom replay](../images/koide/measured/glil/outdoor_hard_02b_live_map_odom.gif)

## Dataset reference

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

## Reproduce measured replays

Keep the large maps, bags, generated occupancy maps, and reference CSV files outside the
repository.

Prepare the evaluation and rendering assets after sourcing ROS 2:

```bash
python3 scripts/prepare_koide_localization_gif_benchmarks.py \
  --data-dir /media/sasaki/aiueo/datasets/koide_hard_localization \
  --output-root /media/sasaki/aiueo/datasets/koide_hard_localization/generated/localization_gif_benchmarks
```

The GLIL full runs used these startup crop seeds. They are approximate runtime
initialization inputs, not evaluator ground truth:

| Bag | X | Y | Z | Yaw |
|---|---:|---:|---:|---:|
| `outdoor_hard_01a` | -86.040205 | -8.857126 | -11.043077 | -82.0063 deg |
| `outdoor_hard_01b` | 103.703859 | -3.360581 | -11.276523 | -3.0042 deg |
| `outdoor_hard_02a` | -104.343538 | -10.324673 | -11.620099 | 163.3613 deg |
| `outdoor_hard_02b` | 103.616685 | -1.693832 | -11.022091 | 4.4813 deg |

This example runs tightly coupled 01a. Substitute the sequence, duration, output,
reference, seed, and metric label for the other rows:

```bash
DATA=/media/sasaki/aiueo/datasets/koide_hard_localization
RUN=/tmp/glil_outdoor_hard_01a

python3 scripts/run_koide_glim_odometry_benchmark.py \
  --bag "$DATA/sequences/outdoor_hard_01a" \
  --reference "$DATA/benchmark/outdoor_hard_01a/reference.csv" \
  --prior-map "$DATA/map_outdoor_hard.ply" \
  --prior-map-bootstrap-center -86.040205 -8.857126 -11.043077 \
  --prior-map-bootstrap-yaw-deg -82.0063 \
  --prior-map-tightly-coupled \
  --tightly-coupled-num-threads 8 \
  --image lidarloc/glim-ros2:jazzy-v1.2.2-tightly-coupled \
  --output "$RUN"

python3 scripts/render_koide_localization_gif.py \
  --occupancy-yaml "$DATA/generated/gif_gallery/occupancy/outdoor_hard/map.yaml" \
  --estimated-csv "$RUN/pose_trace.csv" \
  --reference-csv \
    "$DATA/generated/localization_gif_benchmarks/assets/outdoor_hard_01a_reference.csv" \
  --output-gif images/koide/measured/glil/outdoor_hard_01a_tightly_coupled.gif \
  --frames 72 --fps 9 \
  --sequence-label "Koide outdoor_hard_01a (full 380 s)" \
  --estimate-label "GLIL map-frame pose" \
  --title "GLIL: tightly coupled exact-coreset range-inertial localization" \
  --metrics-label \
    "Pending final pinned-image measurement"
```

`--prior-map-vertical-gain` belongs to the legacy split mode and must not be combined
with `--prior-map-tightly-coupled`.

All full-sequence GLIL GIFs use 72 frames and are 648 x 450. The red arrow is derived
from trajectory motion, including a minimum spatial baseline at stops and trajectory
endpoints, so it represents direction of travel rather than the sensor quaternion.

Dataset citation: Kenji Koide, *Hard Point Cloud Localization Dataset*, Zenodo, 2023,
<https://doi.org/10.5281/zenodo.10122133>. Dataset files are distributed under CC BY 4.0.
