# Phase 1: Koide backend comparison (NDT_OMP vs SMALL_GICP vs SMALL_VGICP)

First result on the locally-available Koide data, `outdoor_hard_01a`, 60 s smoke
window. Run with the GT-aligned diagnostic-map manifests' data absent; this uses
the runnable `01a` sequence + `map_outdoor_hard.ply`. Numbers are indicative
(single 60 s smoke, machine just-idle ~4.5), not a final ranking.

## Headline: RMSE alone is a trap; look at throughput + ok-rate first

A naive translation-RMSE table makes the GICP backends look best, but that is a
mirage: with full-density Livox scans the GICP backends are throughput-bound and
produce only a handful of poses, so their RMSE is computed over ~10-15 early
poses near the seed, not over the window.

Full-density scans (`enable_scan_voxel_filter: false`):

| backend | max align time | poses | ok-rate | trans RMSE | note |
| --- | --- | --- | --- | --- | --- |
| NDT_OMP | 0.154 s | 195 | 89.4% | 0.266 m | healthy, tracks the window |
| SMALL_GICP | 3.425 s | 16 | 51.7% | 0.058 m | throughput-bound (11 non-converged); RMSE is a mirage |
| SMALL_VGICP | 1.531 s | 11 | 29.4% | 0.060 m | throughput-bound (13 non-converged); RMSE is a mirage |

The GICP backends take 1.5-3.4 s per alignment on full-density scans (10-22x
NDT), so the node drops most scans and never tracks the window.

## Fair comparison: identical downsampling (voxel 0.5, as NDT)

Re-run with `enable_scan_voxel_filter: true`, `voxel_leaf_size: 0.5` -- identical
preprocessing to the NDT baseline, isolating the registration backend:

| backend | max align time | poses | ok-rate | trans RMSE | rot RMSE |
| --- | --- | --- | --- | --- | --- |
| NDT_OMP | 0.154 s | 195 | 89.4% | 0.266 m | 2.46 deg |
| **SMALL_GICP** | 0.396 s | 126 | **100%** | **0.095 m** | **2.18 deg** |
| SMALL_VGICP | 0.630 s | 29 | 38.9% | 0.209 m | 2.56 deg |

### Findings (60 s smoke only -- see the full-window section below, which reverses this)

- **SMALL_GICP is the accuracy/stability winner on this easy window.** With downsampling it is fast
  enough (0.40 s max align, ~2.6x NDT but real-time-viable at the processed
  ~2 Hz), 100% ok-rate, and 2.8x better translation than NDT (0.095 vs
  0.266 m). Notably it had **zero** rejects, while NDT had a 6.2 s
  reject streak (23 rows, fitness up to 33.7) on the hard part of the window.
- **NDT_OMP is the fast baseline.** Lowest per-scan cost (0.15 s), processes the
  most scans (195), but less accurate and rejects ~10% on the hard segment.
- **SMALL_VGICP is not viable as configured.** Even downsampled, only 38.9%
  ok-rate (31 threshold rejects + 10 non-converged) at
  `vgicp_voxel_resolution: 0.5`. The voxelized model needs a finer resolution
  (e.g. 0.2-0.3) to be competitive on this Livox data -- a tuning follow-up.

### Caveats / next steps

- 60 s smoke only, single run (no repeat for variance), first window. The hard
  part of `outdoor_hard_01a` is later; a full 380 s run will stress all three.
- VGICP needs a `vgicp_voxel_resolution` sweep before any verdict.
- Full-density GICP being throughput-bound is data/sensor-specific (dense Livox
  returns); on a sparser lidar the trade-off differs.

## Full 380 s window (definitive): the ranking REVERSES

Re-run on the full `outdoor_hard_01a` window (380 s), same identical-downsampling
config. The 60 s smoke ranking is overturned:

| backend | poses | ok-rate | trans RMSE | rot RMSE | longest lost window |
| --- | --- | --- | --- | --- | --- |
| **NDT_OMP** | 476 | 57.3% | **0.668 m** | 11.2 deg | 75.4 s |
| SMALL_GICP | 87 | 11.2% | 11.70 m | 47.4 deg | 274.5 s |
| SMALL_VGICP | 35 | 6.7% | 22.68 m | 32.6 deg | 298.4 s |

- **NDT_OMP is the robust winner on the full hard window** (0.668 m). It survives the
  hard section -- 57% ok, `local_map_crop_too_small: 101`, `fitness_rejected: 247`, and
  one `recovery_retry_from_last_pose_recovered` -- and ends well-tracked.
- **SMALL_GICP collapses** (11.70 m): it is lost for 274.5 s (~72% of the run). Once it
  loses lock on the hard section it never recovers -- fitness explodes (max 1.3M),
  rejections pile up (523), and the crop starves (`local_map_crop_too_small: 106`).
- **SMALL_VGICP** is worse still (22.68 m, lost 298 s).

**The lesson:** SMALL_GICP looked like the winner on the easy first 60 s (0.095 m,
100% ok) but is *fragile* -- more accurate while locked, yet unable to hold lock
through the hard section, where it diverges catastrophically. NDT_OMP is less
accurate when locked but far more robust, which is what matters end-to-end. This is
a textbook case of why a single easy window is necessary-but-not-sufficient (the
Phase 3 lesson): the smoke ranking was exactly reversed by the full replay.

### Verdict

- **NDT_OMP: recommended default** -- robust across the full hard window.
- **SMALL_GICP: higher locked-in accuracy, but not a safe default** without
  robustness work (lock retention / recovery on the hard section).
- **SMALL_VGICP: not competitive** as configured.

Single run per backend; the divergence magnitude (11.7 m vs 0.67 m) is far beyond
run-to-run variance, but a repeat would firm up the numbers. The
`vgicp_voxel_resolution` sweep that was open here is resolved in the next section.

## SMALL_VGICP `vgicp_voxel_resolution` sweep (full 380 s): finer voxels do not help

The full-window SMALL_VGICP above used `vgicp_voxel_resolution: 0.5`. The open
question was whether a finer target voxelization would let VGICP hold lock through
the hard section. Swept 0.5 -> 0.3 -> 0.2 (full 380 s, identical config otherwise,
each run gated at load < 5 so alignment timing is valid):

| vgicp_voxel_resolution | poses | ok-rate | trans RMSE | rot RMSE | longest lost window |
| --- | --- | --- | --- | --- | --- |
| 0.5 (baseline) | 35 | 6.7% | 22.68 m | 32.6 deg | 298.4 s |
| 0.3 | 27 | 6.0% | 2.49 m | 7.9 deg | 348.7 s |
| 0.2 | 12 | 2.7% | 1.77 m | 5.4 deg | 368.8 s |

**The RMSE drop is a mirage -- it is the same early-window throughput trap.** As the
voxel gets finer the *number* goes down (22.68 -> 1.77 m), which looks like
improvement, but every robustness signal moves the wrong way:

- **ok-rate falls** (6.7% -> 6.0% -> 2.7%) and **poses collapse** (35 -> 27 -> 12).
- **the longest lost window grows** (298 -> 349 -> 369 s). At 0.2, VGICP loses lock at
  index 10 (~9 s in) and *never recovers* -- lost for 368.8 s of the 380 s run. The
  1.77 m RMSE is computed over the 10 matched poses of the first ~9 s only.
- this is **not** throughput-bound: `max_alignment_time_sec` stays ~1.25 s (same as
  0.5), so finer voxels are not simply too slow. Instead the finer per-voxel
  covariances make the fit more peaked and brittle -- `fitness_score_over_threshold`
  rejections dominate (371 rows at 0.3) and it cannot re-lock on the hard section.

**Verdict: the sweep refutes the hypothesis.** Finer `vgicp_voxel_resolution` does not
rescue SMALL_VGICP; it worsens lock retention while making the RMSE number
deceptively small. SMALL_VGICP stays non-competitive, and **NDT_OMP remains the
recommended default**. (Methodological echo of the smoke-vs-full reversal: read
ok-rate + lost-window + pose-count *before* RMSE.)

## Reproduce

```bash
source scripts/setup_local_env.sh
python3 scripts/benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60.yaml            # NDT_OMP
python3 scripts/benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60_small_gicp_ds.yaml
python3 scripts/benchmark_from_manifest \
  --manifest param/benchmark/koide_hard_localization_outdoor_hard_01a_smoke60_small_vgicp_ds.yaml
```

Run only on an idle machine (`load < ~5`) for valid alignment-time numbers. Read
`trajectory_eval.json` (RMSE) and `health_summary.json` (`max_alignment_time_sec`,
`ok_rate_percent`, `message_counts`) from each output dir.
