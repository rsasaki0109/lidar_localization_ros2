# Koide outdoor_hard_01a GLIM completion result

The fixed CPU GLIM GICP configuration completed the 380 s `outdoor_hard_01a`
sequence in three independent runs. All coverage, arrival, continuity, safety,
accuracy, realtime, and repeatability gates passed without estimator access to
ground truth.

| Run | Coverage | ATE RMSE | End error | 10 m RPE trans / rot | Processing p95 / scan period |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 99.210% | 1.736 m | 4.425 m | 0.160 m / 0.674 deg | 0.0884 / 0.1000 s |
| 2 | 99.210% | 1.866 m | 4.603 m | 0.156 m / 0.679 deg | 0.0737 / 0.1000 s |
| 3 | 99.210% | 1.529 m | 3.871 m | 0.161 m / 0.676 deg | 0.0664 / 0.1000 s |

Each run had zero backwards timestamps, non-finite poses, unbounded queue
growth, detected jumps, resets, or crashes. Nine invalid sparse scans with fewer
than 100 post-filter points were explicitly skipped in every run. The maximum
pose gap was 0.1002 s and final lag was zero.

## Reproduce

Build the instrumented image from the pinned official CPU image:

```bash
docker build \
  -t lidarloc/glim-ros2:jazzy-v1.2.2-instrumented-guarded \
  experiments/koide_odometry_glim_gicp6500
```

The archived validation image ID is recorded in `results.json`. A source rebuild
may have a different local image ID; use `--allow-image-mismatch` only after
recording that ID with the new artifacts.

Run one repeat:

```bash
scripts/run_koide_glim_odometry_benchmark.py \
  --bag /path/to/sequences/outdoor_hard_01a \
  --reference /path/to/benchmark/outdoor_hard_01a/reference.csv \
  --output /tmp/koide_glim_run_01 \
  --allow-image-mismatch
```

Omit `--allow-image-mismatch` when using the archived validated image rather than
a source rebuild. Every run records the effective image ID in `summary.json`.

Known fix (2026-07-16): the first archived copy of the runner omitted
`-p auto_quit:=true`, so `glim_rosbag` spun forever in `rclcpp::spin` after
playback finished instead of dumping. The runner now passes `auto_quit` and
accepts `--requested-duration-sec` so the same script drives the other
outdoor_hard sequences (rule: floor of the GT reference span in seconds).

Run that command three times with distinct output directories, then aggregate:

```bash
python3 scripts/summarize_koide_odometry_completion.py \
  --completion-json /tmp/koide_glim_run_01/odometry_completion.json \
  --completion-json /tmp/koide_glim_run_02/odometry_completion.json \
  --completion-json /tmp/koide_glim_run_03/odometry_completion.json \
  --output-json /tmp/koide_glim_3repeat.json
```

## Scope and interpretation

This result uses LiDAR and IMU with CPU GICP, a 5 s smoother, 6,500-point random
downsampling, and two preprocessing/odometry threads. It reports the high-rate
IMU trajectory after GLIM's online self-generated submap/global graph correction.
That is SLAM-style drift correction, not localization against a prior map and not
global reinitialization. It also applies an explicit constant-z ground-vehicle
constraint during export. Neither transform uses reference/GT data.

Therefore this establishes the project-level full-sequence LiDAR/IMU trajectory
gate. A stricter front-end-only `odom_imu.txt` gate should be tracked separately
if a correction-free odometry claim is required.

Exact metrics and provenance are in [results.json](results.json), fixed parameters
are in `param/odometry/glim_koide_outdoor_gicp6500`, and the two minimal upstream
patches are in `patches/`.

## Dataset completion progress (2026-07-16, interim)

The same fixed configuration was run unchanged on the remaining outdoor_hard
sequences. References were produced by clipping the per-route GT to each bag's
time window; the requested duration is the floor of the GT reference span
(302 / 363 / 298 s), matching the 380 s rule used for `outdoor_hard_01a`.

Three gate runs per sequence (plus one preliminary run each) all passed the
plan-level completion gates: coverage >= 99%, arrival within 1 s, zero
crash/NaN/reset/pose-jump, and distance-normalized end drift <= 1%.

| Seq | Run | Coverage | ATE RMSE | End error (drift) | 10 m RPE trans / rot |
| --- | --- | ---: | ---: | ---: | ---: |
| 01b | 1 | 99.040% | 1.968 m | 2.625 m (0.62%) | 0.262 m / 0.794 deg |
| 01b | 2 | 99.040% | 1.870 m | 2.133 m (0.50%) | 0.266 m / 0.791 deg |
| 01b | 3 | 99.040% | 1.821 m | 2.490 m (0.58%) | 0.246 m / 0.804 deg |
| 02a | 1 | 99.146% | 2.398 m | 4.058 m (0.73%) | 0.143 m / 0.600 deg |
| 02a | 2 | 99.146% | 2.187 m | 3.983 m (0.72%) | 0.158 m / 0.599 deg |
| 02a | 3 | 99.146% | 2.233 m | 3.835 m (0.69%) | 0.156 m / 0.591 deg |
| 02b | 1 | 99.027% | 1.304 m | 2.233 m (0.53%) | 0.244 m / 0.893 deg |
| 02b | 2 | 99.027% | 1.539 m | 2.603 m (0.62%) | 0.248 m / 0.898 deg |
| 02b | 3 | 99.027% | 1.584 m | 2.225 m (0.53%) | 0.225 m / 0.894 deg |

Against the stricter `outdoor_hard_01a` primary-goal thresholds (kept fixed, not
relaxed): 01b and 02b consistently exceed the 10 m RPE translation median gate
(0.225--0.266 m vs 0.20 m) and 02a consistently exceeds the translation ATE gate
(2.187--2.398 m vs 2.0 m). These are recorded as-is per sequence.

The realtime gate was NOT established in the runs above: they overlapped
external CPU load (load average ~9 from unrelated processes), which inflated
processing p95 (0.129--0.145 s under load vs 0.048 s measured quiet). Queue
depth still drained to zero and playback held 1.0x in every run. The
quiet-machine 3-repeat below supersedes those p95 values.

## Dataset completion result (2026-07-17, quiet-machine 3-repeat)

The 9 runs were repeated on a quiet machine (gate: 1-min load < 2.5, at most
one persistent external ~1-core process; per-run `load.log` recorded median
load 1.8--3.3 during playback). All 9 runs pass every plan-level completion
gate, now including the realtime gate:

| Seq | Run | Coverage | ATE RMSE | End error (drift) | 10 m RPE trans / rot | p95 / 0.100 s |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 01b | 1 | 99.040% | 1.966 m | 2.325 m (0.55%) | 0.276 m / 0.803 deg | 0.0439 s |
| 01b | 2 | 99.040% | 1.893 m | 2.365 m (0.55%) | 0.257 m / 0.816 deg | 0.0440 s |
| 01b | 3 | 99.040% | 1.808 m | 2.011 m (0.47%) | 0.249 m / 0.800 deg | 0.0444 s |
| 02a | 1 | 99.146% | 1.884 m | 3.006 m (0.54%) | 0.141 m / 0.600 deg | 0.0335 s |
| 02a | 2 | 99.146% | 2.125 m | 3.591 m (0.65%) | 0.160 m / 0.595 deg | 0.0340 s |
| 02a | 3 | 99.146% | 2.341 m | 3.759 m (0.68%) | 0.150 m / 0.590 deg | 0.0335 s |
| 02b | 1 | 99.027% | 1.277 m | 1.943 m (0.46%) | 0.211 m / 0.889 deg | 0.0419 s |
| 02b | 2 | 99.027% | 1.518 m | 2.399 m (0.57%) | 0.225 m / 0.899 deg | 0.0426 s |
| 02b | 3 | 99.027% | 1.434 m | 2.373 m (0.53%) | 0.231 m / 0.910 deg | 0.0421 s |

Zero crash/NaN/reset/pose-jump/queue-divergence events in all runs; final lag
0; max pose gap ~0.008 s. Against the stricter fixed `outdoor_hard_01a`
thresholds (unchanged, not relaxed): 01b and 02b still exceed the 10 m RPE
translation gate (0.211--0.276 m vs 0.20 m), and 02a straddles the 2.0 m ATE
gate (1.884 / 2.125 / 2.341 m; run 1 passes ALL strict gates).

### Open-loop (correction-free) export comparison

Re-converting the same dumps without `--apply-global-correction` (keeping the
constant-z export) isolates the pure front-end odometry. On the return legs
the per-submap graph correction is what inflates 10 m RPE: open-loop RPE_t is
0.165--0.174 m (01b) and 0.165--0.169 m (02b) -- both PASS the 0.20 m gate the
corrected export fails -- and 02b end error also improves (1.65--2.03 m vs
2.23--2.60 m). 01a open-loop passes all strict gates (ATE 1.77/1.81 m, RPE_t
~0.155 m, runs 2-3). 02a stays above the ATE gate in both variants (open-loop
2.41--2.70 m); its error is uniform along-track drift that first exceeds 2 m
at t~170--190 s and peaks (4.1--4.8 m) on the final ~150 m straight, not at a
turn; the mid-run z-error hump in raw 3D numbers is a start-pose-alignment
artifact (real GT elevation varies only 1.15 m). This supports keeping
odometry continuous in the odom frame and applying graph-style corrections
downstream (map -> odom), per the development plan's architecture rule.
Open-loop artifacts: `pose_trace_openloop_planar.csv` /
`odometry_completion_openloop_planar.json` in each gate-run directory.
