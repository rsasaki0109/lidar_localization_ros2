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

### Findings

- **SMALL_GICP is the accuracy/stability winner.** With downsampling it is fast
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
