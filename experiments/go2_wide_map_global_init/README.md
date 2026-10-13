# Go2 global initialization on wide maps (JEPLO)

[docs/benchmark.md](../../docs/benchmark.md) records no global initialization on the
Unitree Go2 self maps of JEPLO `Long_Stairs`, `Outdoor1` and `Outdoor2`, while the four
`EIL_*` sequences in the 10 m x 4 m capture room initialize. This experiment finds the cause
and compares global search (G2) settings. It does **not** change any default. The setting that won is
exposed as opt-in quickstart options (`--global-z-min`, `--global-z-max`,
`--occupancy-inflate-radius-m`) plus existing ones, and documented as a recipe for low
sensors in [go2_mid360_quickstart.md](../../docs/go2_mid360_quickstart.md#large-maps).

## Data, and what differs from the earlier benchmark

`ASIG-X/JEPLO` on Hugging Face returned 404 on 2026-10-11 (also with a token). The public copy,
<https://huggingface.co/datasets/ASIG-Lab/JEPLO-Dataset>, has only the raw bags
(`/livox/lidar` as `livox_ros_driver2/msg/CustomMsg` at 20 Hz, `/livox/imu`) and `gt/`. It
has neither `bags_pc2_merged` nor `maps_loc`. All files used were checked against the
dataset's LFS sha256 (bags) and git blob ids (gt, metadata). Everything below therefore runs on
rebuilt inputs:

- **Bags**: [`tools/benchmark/jeplo_to_pc2_bag.py`](../../tools/benchmark/jeplo_to_pc2_bag.py)
  merges two sweeps into a 10 Hz `PointCloud2` stamped mid-sweep, with an absolute
  per-point `timestamp`.
- **Odometry**: RKO-LIO `offline_node` with
  [`rko_lio_jeplo.yaml`](../../tools/benchmark/suites/rko_lio_jeplo.yaml). Fitted to the
  ground truth on the first 40 s, its error after returning to the capture room is 2-4 cm
  (Long_Stairs 249 s, Outdoor1 693 s, Outdoor2 639 s).
- **Maps**: [`tools/benchmark/build_map_from_odometry.py`](../../tools/benchmark/build_map_from_odometry.py)
  (every second cloud, deskewed, 0.8-40 m, 0.05 m voxels), moved into the ground-truth frame
  with `align_map_to_ground_truth.py` (trajectory residual 0.035-0.040 m for the self maps).
  The `EIL_*` cases use an **EIL_Mix map built the same way** (residual 0.067 m), not the
  lidar_slam_ros2 map of docs/benchmark.md.

Because of the EIL map, the EIL numbers here are not comparable with docs/benchmark.md. On
this map the baseline itself initializes wrongly in 5 of 16 EIL runs (docs: 0 of 12 on the
lidar_slam_ros2 map).

The Long_Stairs and Outdoor starts do not match the EIL_Mix map in the ground-truth frame:
NDT fitness at the true pose is 3.7-13.5 there, against 0.01 for EIL_Mask2. They were
recorded on other days, so the room or the motion-capture frame differs. They are
therefore only searched on their own maps.

## What fails

`g2_offline_probe.py` runs the runtime `GlobalLocalizationEngine` on a scan every second of
the ground-truth-covered start (the capture room). Scans are levelled with the RKO-LIO
attitude, as `global_localization_node.py` does, and the probe records each candidate's
error, the 2D (BBS) score and NDT fitness of the true pose, and the points left in the
height band.

| | Long_Stairs | Outdoor1 | Outdoor2 | EIL_Box | EIL_Stairs | EIL_Mask1 | EIL_Mask2 |
|---|---|---|---|---|---|---|---|
| scan points in the band (z 0.5-5 m above the sensor) | 102 | 117 | 115 | 97 | 140 | 146 | 132 |
| top candidate within 1 m / 15 deg (share of queries) | 0.04 | 0.00 | 0.00 | 0.07 | 0.16 | 0.40 | 0.23 |
| BBS score of the true pose (median) | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| BBS score of the top candidate (median) | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| NDT fitness of the true pose (median) | 0.010 | 0.011 | 0.010 | 0.041 | 0.017 | 0.011 | 0.010 |
| NDT fitness of the top candidate (median) | 0.346 | 0.115 | 0.081 | 0.113 | 0.082 | 0.018 | 0.057 |

The 2D search saturates. With about 100 points and a grid whose occupied cells are
inflated by 0.6 m (generator default) plus one cell of dilation in G2, every scan point
lands on an occupied cell at many places. All 8 returned candidates score 1.00, and so does the
truth. Branch-and-bound stops as soon as it holds 8 leaves at score 1.0, so which places it
returns depends on the search order (yaw index first), not on the scan. The truth was among
them in 4% (Long_Stairs) and 0% (Outdoor) of the queries. NDT tells the truth apart
clearly (fitness 0.01 against 0.08-0.35), but it never sees it.

The replay shows the same (`run_benchmark.py`, quickstart defaults). On Long_Stairs, six
queries returned 110-117 band points, top candidates 17-25 m off at yaw 0-15 deg with BBS
score 1.0. Four retries were `ambiguous_registration_retry`, then `manual_pose_required`.

Causes, one by one:

- **Height band and the upside-down sensor.** The MID-360 sees up to 52 deg below and 7 deg
  above its horizontal plane. Mounted upside down on the Go2, almost all returns lie below
  the sensor: in the levelled scans the 99th percentile of elevation is 7-8 deg. Only 4.1-4.5% of
  the points are 0.5 m or more above the sensor, which is the band's default lower edge. Another
  30% lie between the sensor plane and 0.5 m. The generated grid marks
  obstacles 0.4-2.0 m above the ground, while the band starts 0.9 m above the ground for a
  sensor 0.42-0.48 m high (ground truth). Lowering the band edge to the sensor plane
  (`z_min_m: 0.0`) gives 276-434 points.
- **Grid inflation.** The 0.6 m inflation is what makes many places score 1.0. A 0.2 m
  inflation (or a 0.1 m grid) breaks the ties.
- **Too few candidates.** Even a good grid returns tied places. 32 (or 16) candidates let
  NDT see the truth.
- **The 1.7-2.3 m offsets noted in docs/benchmark.md** are the plateau around the truth:
  inside one non-maximum-suppression neighbourhood the first tied cell is kept, a corner of
  the 0.8 m halo. With `registration_refine_candidates` the published pose is the NDT
  result instead (simulated start error 0.01-0.05 m against 1.1-1.4 m).
- **Not causes:** `g2_registration_sensor_height_m` only sets the NDT seed height (0.45
  instead of 1.0 changed nothing, variant `k`). Map size matters only through saturation:
  the larger the map, the more places score 1.0 (Outdoor 0.00 against EIL 0.07-0.40 top-1).

## Variants (offline)

`matrix.yaml` lists the variants (overrides of quickstart's G2 settings) and cases.
Each cell is the share of queries whose top candidate is within 1 m and 15 deg of the truth.
"Gated wrong" counts queries whose top candidate passes the single-query startup gates
(score, fitness ratio) while more than 1 m off. A two-query odometry consensus and
verification still follow at runtime.

| variant | Long_Stairs | Outdoor1 | Outdoor2 | EIL_Box | EIL_Stairs | EIL_Mask1 | EIL_Mask2 | gated wrong (wide / EIL) | query s (wide, median) |
|---|---|---|---|---|---|---|---|---|---|
| `a_baseline` | 0.04 | 0.00 | 0.00 | 0.07 | 0.16 | 0.40 | 0.23 | 14 / 70 | 1.4 |
| `b_cand32` | 0.24 | 0.02 | 0.04 | 0.09 | 0.16 | 0.40 | 0.27 | 18 / 78 | 1.6 |
| `c_refine` | 0.09 | 0.00 | 0.00 | 0.27 | 0.40 | 0.72 | 0.64 | 14 / 40 | 1.4 |
| `d_cand32_refine` | 0.40 | 0.02 | 0.09 | 0.47 | 0.44 | 0.72 | 0.68 | 11 / 41 | 1.6 |
| `e_zmin0` | 0.51 | 0.00 | 0.00 | 0.33 | 0.49 | 0.52 | 0.64 | 18 / 53 | 1.6 |
| `f_zmin0_cand32_refine` | 0.91 | 0.09 | 0.13 | 0.69 | 0.67 | 0.88 | 0.91 | 2 / 21 | 1.7 |
| `g_nodilate` | 0.47 | 0.00 | 0.00 | 0.18 | 0.36 | 0.56 | 0.50 | 28 / 63 | 1.3 |
| `h_infl02` | 0.60 | 0.11 | 0.00 | 0.36 | 0.47 | 0.44 | 0.50 | 20 / 46 | 1.2 |
| `i_res01` | 0.82 | 0.58 | 0.16 | 0.53 | 0.51 | 0.48 | 0.64 | 16 / 39 | 1.2 |
| `j_acc1s` | 0.78 | 0.00 | 0.02 | 0.31 | 0.51 | 0.60 | 0.59 | 11 / 50 | 1.2 |
| `k_sensor045` | 0.04 | 0.00 | 0.00 | 0.07 | 0.16 | 0.40 | 0.23 | 19 / 73 | 1.4 |
| `l_res01_zmin0_cand32_refine` | 1.00 | 0.96 | 0.96 | 0.87 | 0.82 | 0.84 | 0.86 | 0 / 10 | 6.5 |
| `q_infl02_zmin0_pts1024_cand32_refine` | 1.00 | 0.96 | 0.93 | 0.78 | 0.84 | 0.88 | 0.86 | 0 / 12 | 2.7 |
| `s_res01_zmin0_cand32_refine_depth6` | 1.00 | 0.96 | 0.96 | 0.87 | 0.82 | 0.84 | 0.86 | 0 / 10 | 7.2 |
| `t_infl02_zmin0_cand32_refine` | 1.00 | 0.98 | 0.87 | 0.76 | 0.84 | 0.84 | 0.86 | 0 / 12 | 3.4 |
| `u_infl02_zmin0_pts1024_cand32` | 1.00 | 0.96 | 0.93 | 0.73 | 0.87 | 0.76 | 0.82 | 0 / 15 | 3.1 |
| `v_infl02_zmin0_pts1024_cand8_refine` | 1.00 | 0.47 | 0.18 | 0.78 | 0.84 | 0.84 | 0.86 | 4 / 13 | 1.3 |
| `w_zmin0_pts1024_cand32_refine` | 0.96 | 0.13 | 0.24 | 0.76 | 0.76 | 0.80 | 0.91 | 0 / 18 | 2.0 |
| `x_infl02_pts1024_cand32_refine` | 0.76 | 0.58 | 0.24 | 0.51 | 0.64 | 0.72 | 0.68 | 6 / 26 | 1.4 |
| `y_infl02_zmin0_pts1024_cand16_refine` | 1.00 | 0.87 | 0.40 | 0.78 | 0.84 | 0.88 | 0.86 | 0 / 12 | 1.5 |
| `z_infl02_zmin0_pts512_cand16_refine` | 1.00 | 0.87 | 0.40 | 0.76 | 0.84 | 0.88 | 0.86 | 0 / 12 | 1.5 |
| `yc_infl02_cand16_refine` | 0.71 | 0.20 | 0.11 | 0.49 | 0.67 | 0.68 | 0.73 | 12 / 27 | 0.9 |
| `yd_infl02_zmin0_cand16_refine` | 1.00 | 0.73 | 0.29 | 0.76 | 0.84 | 0.84 | 0.86 | 0 / 12 | 1.0 |

`infl02` is a grid generated with `--inflate-radius-m 0.2`, `res01` one with
`--resolution 0.1 --inflate-radius-m 0.3`. With 1024 points on the 0.1 m grid
(`m`, `n`, `o`, `p`, `r`) a query took 14-35 s on Long_Stairs and Outdoor1, too slow for a
startup query, so those were dropped. Their top-1 shares were 0.98-1.00.

Reading:

- No single change is enough. Around `q`, removing the lower band (`x`), the thin grid
  (`w`) or the extra candidates (`v`) each costs most of the Outdoor gain. Removing NDT
  refinement (`u`) keeps the top-1 share but publishes the coarse cell (0.2-0.4 m off).
- Accumulating 1 s of scans (`j`) helps Long_Stairs only.
- `q` and `l` find the truth in 93-100% of the wide-map queries with no gated wrong answer.
  `q` is 2-3 times faster because it keeps the 0.2 m grid.
- Every combined variant also improves the EIL room, from 70 gated wrong answers to 10-12.
  The rest are 1-2.6 m near misses and three 180 deg turns in the symmetric room.
  The two-query consensus has to stop those.

## Replays

`run_benchmark.py` on the rebuilt bags and maps, one 4-core machine, Jazzy, replay at 1x.
The localizer does not keep up with every scan there. Wrong init: the first 2 s of poses
are more than 1 m off. Lost: the run's median error is above 1 m.

| case | baseline: init / wrong / lost | `q`: init / wrong / lost | `y`: init / wrong / lost |
|---|---|---|---|
| Long_Stairs | 0/1 | 1/1, 0, 0 (first pose 22.8 s, median 0.02 m) | 2/2, 0, 0 (12.3-12.6 s, 0.02 m) |
| Outdoor1 | 0/1 | 1/1, 0, 0 (13.4 s, 0.02 m) | 2/2, 0, 0 (11.1-12.6 s, 0.02 m) |
| Outdoor2 | 0/1 | 1/1, 0, 0 (14.1 s, 0.02 m) | 2/2, 0, 0 (11.1-14.6 s, 0.02 m) |
| EIL_Box | 4/4, 1, 1 | 4/4, 1, **4** | 3/3, 0, 0 |
| EIL_Stairs | 4/4, 0, 0 | 4/4, 0, 0 | 3/3, 0, 0 |
| EIL_Mask1 | 4/4, 3, 2 | 4/4, 0, 0 | 3/3, 0, 0 |
| EIL_Mask2 | 4/4, 1, 1 | 4/4, 0, 0 | 3/3, 0, 0 |

Six later replays of `y` on the wide maps, run for [post_init_jump](../post_init_jump/README.md)
(only tracking parameters varied), localized five; in one Outdoor2 run every try stayed
ambiguous and the startup asked for a manual pose. None started at a wrong place.

`q` lost every EIL_Box run although its answer was right (0.01-0.05 m in three of four).
Its queries take about 1.9 s on the EIL map instead of 0.2 s, so it started tracking 11-12 s
into the bag, just as the Go2 walks off the box (0.5-0.7 m/s, down 0.6 m). The first
scans after an initial pose arrive 0.8-0.9 s apart on this machine (`failure_category:
overload`), and the localizer accepted a 2.68 m correction (fitness 0.006) to an aliased
place 4.5 m along the corridor. The baseline started at 8.3 s, while the Go2 was still on the box. Started 3 s later
(`start_offset_sec: 3.0`, two runs), the baseline lost EIL_Box the same way once (median
4.99 m) and did not initialize the other time. So it is the localizer's tracking at that moment, not the
answer. `y` keeps `q`'s settings with 16 candidates. That halves the BBS time
(0.46 s against 1.23 s on the EIL map), and EIL_Box started at 9.1 s and held in all three runs.

## Should these settings be the defaults? (Koide, handheld)

The defaults also drive handheld and car-mounted sensors. On the Go2 data they also mislead
the startup check, which confirms an answer when the next one agrees with it after the
odometry motion. `consensus_report.py` counts such agreeing pairs of consecutive probe answers
by their distance from the truth, right (within 1 m), near (within 3 m) and far:

| data, variant | all pairs: right / near / far | gated pairs: right / near / far |
|---|---|---|
| JEPLO (7 cases, 272 queries), defaults | 6 / 27 / 5 | 5 / 25 / 1 |
| JEPLO, `y` | 181 / 8 / 2 | 163 / 8 / 0 |
| JEPLO, `yd` | 163 / 11 / 4 | 150 / 10 / 0 |
| Koide (6 cases, 251 queries), defaults | 138 / 3 / 17 | 106 / 0 / 0 |

With the defaults on the Go2, agreeing answers are mostly 1-3 m off: the corner of the tied
area around the robot, which the localizer then has to pull in (or does not: EIL_Mask1). 151
of the 272 top answers had yaw exactly 0 deg, the first yaw sample. On Koide the startup gates
stop every far agreement.

The Koide sequences (Zenodo 10122133, md5-checked) tested whether part of `y` can become the
default. A handheld sensor about 1.3 m up fills the scan cap (256 points by default), and
with 1024 points a query took 19-23 s. So the candidates keep 256 points (`koide_matrix.yaml`,
startup windows of 22-60 s):

| variant | 01a | 01b | 02a | 02b | kidnap_a | kidnap_b | gated wrong | query s |
|---|---|---|---|---|---|---|---|---|
| `a_baseline` | 0.80 | 0.62 | 0.67 | 0.28 | 0.96 | 0.60 | 6 | 2.7-4.5 |
| `yc_infl02_cand16_refine` | 0.62 | 0.67 | 0.63 | 0.38 | 0.76 | 0.72 | 2 | 6.5-8.9 |
| `yd_infl02_zmin0_cand16_refine` | 0.70 | 0.67 | 0.60 | 0.37 | 0.72 | 0.68 | 0 | 6.8-8.8 |

On JEPLO, `yd` keeps most of `y`'s gain (top-1 1.00 / 0.73 / 0.29 on the wide maps, no gated
wrong answer there), so it was replayed on Koide against the defaults, alternating, two rounds
(`koide_replay.yaml`; `yd` through `--occupancy-inflate-radius-m 0.2 --global-z-min 0.0
--global-max-candidates 16 --refine-global-candidates`):

| case | defaults: first pose (s), median xy (m) | `yd` |
|---|---|---|
| outdoor_hard_01a | 13.0, 0.12 / 17.8, 0.12 | 39.3, 0.95 / 34.7, 0.12 |
| outdoor_hard_01b | 22.7, 0.10 / 22.5, 0.09 | 53.0, 0.09 / 53.5, 0.09 |
| outdoor_hard_02a | 17.6, 0.12 / 16.1, 0.09 | 27.8, 0.11 / 40.8, 0.10 |
| outdoor_hard_02b | 14.0, 0.09 / 13.3, 0.09 | 22.3, 0.09 / 24.0, 0.08 |
| outdoor_kidnap_a | 17.0, 0.05 / 16.0, 0.04 | 23.7, 0.07 / not initialized |
| outdoor_kidnap_b | 21.1, 0.06 / 20.4, 0.04 | **wrong init** 36.1, 20.75 / **wrong init** 35.4, 14.12 |

The defaults initialized all 12 runs correctly. `yd` initialized later in every case and
wrongly twice. In kidnap_b its two answers were right (0.0-0.1 m) but took 5.6-8.6 s each, so
tracking began about 35 s in, as the sensor is covered and carried (at about 31 s), and was
lost there. The 0.95 m median of its first 01a run is a mid-run height jump (z 8-10 m off for
about 150 s) with no reset involved. It did not recur in the second round. **The defaults stay**,
and the low-sensor settings remain a Go2 option.

### Starting the yaw sweep elsewhere per query (not promoted)

Branch-and-bound keeps tied candidates in yaw-sample order. `--yaw-sweep-offset` in the probe
starts each query's sweep at a sample fixed by the scan stamp (the scan is rotated by a whole
number of samples and the answers rotated back, so the C++/Python kernels are untouched).
Variant `as_baseline_yaw_sweep`, defaults otherwise:

| data | all pairs: right / near / far | gated: right / near / far | simulated startups ok / near / far / none |
|---|---|---|---|
| JEPLO, defaults | 6 / 27 / 5 | 5 / 25 / 1 | 1 / 4 / 0 / 2 |
| JEPLO, yaw sweep | 8 / 26 / 5 | 8 / 21 / 0 | 0 / 5 / 1 / 1 |
| Koide, defaults | 138 / 3 / 17 | 106 / 0 / 0 | 6 / 0 / 0 / 0 |
| Koide, yaw sweep | 141 / 3 / 21 | 106 / 2 / 0 | 6 / 0 / 0 / 0 |

Far agreements do not drop, and one simulated JEPLO startup lands 46 m off (Outdoor1), which the
defaults never did. The repeats are not the yaw order: NDT re-ranks the tied places and settles
on similar ones anyway. Removing the saturation (the low-sensor settings) is what works.

## Searching again only when the band is sparse (round 4)

The low-sensor settings cannot be the default because they slow a handheld sensor down
(Koide above). But the two sensors differ plainly in one number: the 2D points in the
default height band (0.5-5 m above the sensor, cells at the grid resolution, before the
scan-point cap). The Go2 has a median of 115 (90th percentile 326); every Koide query
fills even a 1024-point cap. `sparse_band_min_points` (G2 node; quickstart
`--global-sparse-band-min-points`, default 0 = off) searches a scan again with `w`'s
settings (band from the sensor plane, 1024 points, 32 candidates, NDT-refined poses) only
when its band holds fewer points than this. The occupancy grid stays the default one
(0.6 m inflation).

Offline (`sb` counted after the 256-point cap, `sc` before it):

| variant | top1 near: Long_Stairs, Outdoor1, Outdoor2, Box, Stairs, Mask1, Mask2 | gated wrong (wide / EIL) | simulated startups right | fired (JEPLO / Koide queries) |
|---|---|---|---|---|
| `a_baseline` | 0.04 0.00 0.00 0.07 0.16 0.40 0.23 | 14 / 70 | 1 of 7 | - |
| `w_zmin0_pts1024_cand32_refine` (always) | 0.96 0.13 0.24 0.76 0.76 0.80 0.91 | 0 / 18 | 7 of 7 | all |
| `sb_sparse_band256` | 0.96 0.13 0.24 0.67 0.64 0.76 0.68 | 0 / 26 | 6 of 7 | 87% / 0 of 251 |
| `sc_sparse_band512` | 0.96 0.13 0.24 0.69 0.76 0.80 0.91 | 0 / 18 | 7 of 7 | 93% / 0 of 251 |

On Koide `sc` returned the defaults' answers for every query. Counting the band costs about
18 ms per query.

Replays with quickstart defaults plus `--global-sparse-band-min-points 512` (`go2_jeplo.yaml`,
default grid):

| case | runs | initialized | wrong init | first pose (s) | median xy (m) |
|---|---|---|---|---|---|
| EIL_Box, Stairs, Mask1, Mask2 | 3 each | 12 | **0** | 10.5-11.6 | 0.02-0.03 |
| Long_Stairs | 3 | 3 | 0 | 10.3 | 0.02 |
| Outdoor1 | 2 | 0 | 0 | - | - |
| Outdoor2 | 2 | 2 | 0 | 11.8 | 0.02 |
| Koide (`koide_replay.yaml`) | 1 each, 6 sequences | 6 | 0 | 14.8-25.0 | 0.05-0.13 |

- The EIL defaults started wrongly in 3-5 of 12 runs in the earlier replays (baseline above,
  post_init_jump regression); with the sparse-band search none did.
- Outdoor1 stayed ambiguous in all six tries of both runs and asked for a manual pose. It
  needs the finer grid of `y` (0.2 m inflation).
- The search fired on 1-6 queries per Go2 run and on none of 184 Koide queries, so Koide ran
  the defaults' code path (results within the run-to-run spread of earlier default replays).

## Decision

- **Quickstart default:** `--global-sparse-band-min-points 512` (round 4). It fires only
  when the default band is sparse, which no Koide query was; the G2 node default stays 0.
- **Recommended for a sensor low above the ground (Go2):** `y`, through quickstart:
  `--occupancy-inflate-radius-m 0.2 --global-z-min 0.0 --global-max-scan-points 1024
  --global-max-candidates 16 --refine-global-candidates`
  ([`go2_jeplo_low_sensor.yaml`](../../tools/benchmark/suites/go2_jeplo_low_sensor.yaml)).
- **Defaults unchanged.** On Koide (handheld), the 256-point version `yd` initialized
  later in all 12 runs and wrongly in 2, where the defaults were right in all 12.
- The tracking jump after an initial pose during fast motion (EIL_Box) is a localizer
  issue: the seed correction guard's warmup lets the aliased corrections through. An
  opt-in rule stops it on a controlled start
  ([post_init_jump](../post_init_jump/README.md)).

## Not verified

- The original `bags_pc2_merged` and `maps_loc` could not be obtained. Results on them,
  and on the lidar_slam_ros2 EIL_Mix map of docs/benchmark.md, were not measured.
- `y` itself (1024 points) was not replayed on Koide (19-23 s per query offline). No car data.
- Repeats are few: on the wide maps one baseline run, one `q` run and two `y` runs per
  map; in the room four baseline and `q` runs and three `y` runs per sequence.
- One machine (4 cores), ROS 2 Jazzy only. Humble was not built.

## Reproduce

```bash
# data: raw bags and gt/ from ASIG-Lab/JEPLO-Dataset, then for each sequence
python3 tools/benchmark/jeplo_to_pc2_bag.py $RAW/bags/Outdoor1 $JEPLO_ROOT/bags_pc2_merged/Outdoor1
ros2 run rko_lio offline_node --ros-args --params-file outdoor1_rko.yaml  # rko_lio_jeplo.yaml + bag_path, dump_results
python3 tools/benchmark/build_map_from_odometry.py --bag $JEPLO_ROOT/bags_pc2_merged/Outdoor1 \
  --trajectory Outdoor1.tum --out map_odom.pcd
python3 tools/benchmark/align_map_to_ground_truth.py --map map_odom.pcd --trajectory Outdoor1.tum \
  --ground-truth $JEPLO_ROOT/gt/Outdoor1.txt --out $JEPLO_ROOT/maps_loc/Outdoor1/map.pcd

# offline comparison (needs the built workspace for g2_ndt_score/bbs_cpp)
export JEPLO_PC2=$JEPLO_ROOT/bags_pc2_merged JEPLO_ODOM=<dir of *.tum> JEPLO_GT=$JEPLO_ROOT/gt JEPLO_MAPS=<maps>
python3 experiments/go2_wide_map_global_init/run_matrix.py experiments/go2_wide_map_global_init/matrix.yaml --out /tmp/g2matrix
python3 experiments/go2_wide_map_global_init/pivot.py experiments/go2_wide_map_global_init/results.json

# replays
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/go2_jeplo.yaml --out /tmp/go2
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/go2_jeplo_low_sensor.yaml --out /tmp/go2_low
```

Koide (Zenodo 10122133: the six outdoor sequences, `map_outdoor_*.ply`, `gt.zip`):

```bash
# RKO-LIO offline with lidarslam's param/rko_lio_mid360.yaml, lidar_topic /livox/points
export KOIDE_ROOT=<dir> KOIDE_ODOM=<dir of *.tum>   # KOIDE_ROOT/maps/<map>/{map.ply,occ,occ_infl02}
python3 experiments/go2_wide_map_global_init/run_matrix.py experiments/go2_wide_map_global_init/koide_matrix.yaml --out /tmp/kmatrix
export RKO_ODOMETRY_LAUNCH="lidarslam rko_lio_odometry.launch.py" RKO_MID360_PARAM=<lidarslam>/param/rko_lio_mid360.yaml
python3 tools/benchmark/run_benchmark.py experiments/go2_wide_map_global_init/koide_replay.yaml --out /tmp/koide_defaults
python3 tools/benchmark/run_benchmark.py experiments/go2_wide_map_global_init/koide_replay.yaml --out /tmp/koide_yd \
  --quickstart "ros2 run lidar_localization_ros2 quickstart.py --occupancy-inflate-radius-m 0.2 --global-z-min 0.0 --global-max-candidates 16 --refine-global-candidates"
```

`JEPLO_MAPS` holds `<sequence>/map.pcd` and the grids `occ/` (quickstart's generator
settings), `occ_infl02/` (`--inflate-radius-m 0.2`) and `occ_res01/`
(`--resolution 0.1 --inflate-radius-m 0.3`), all with `--max-obstacle-height-m 2.0`.
`results.json` holds the probe summaries of every case and variant.
