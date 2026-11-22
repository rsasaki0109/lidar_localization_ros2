# 3D-BBS as a global search for G2 (offline evaluation)

Does [3D-BBS](https://github.com/KOKIAOKI/3d_bbs) (Aoki, Koide et al., ICRA 2024), a
branch-and-bound search over a hierarchical **3D** voxel map, place scans better than
the current G2 search, which runs branch-and-bound on a **2D** occupancy grid made from
the 3D map and then refines and ranks the top candidates with NDT? The question comes
from multi-level buildings (stairs, a room above a hall) where the 2D grid loses height.

Nothing here is part of the package build or the runtime nodes. The engine that came out
of it is the opt-in `global_search_engine: bbs3d` of the G2 node
([docs](../../docs/global_localization.md#using-3d-bbs-global_search_engine-bbs3d));
`eval_global_search.py --method g2 --config '{"global_search_engine": "bbs3d"}'` runs
that path. The results and the
recommendation are in
[docs/global_localization.md](../../docs/global_localization.md#3d-bbs-as-a-g2-engine-evaluated-2026-11-21).

## 3D-BBS

- Licence: **MIT** (Copyright (c) 2023 KokiAoki). Compatible with this BSD-2 package; a
  copy or a vendored build must keep the MIT notice. It was still built as an external
  library here and not copied into the package.
- Evaluated revision: `41529a34a2fb9618b5ff560fb3c2363f1615666d` (2025-10-30).
- CPU library (`-DBUILD_CUDA=OFF`): Eigen 3.4, OpenMP, Boost headers (hash) and
  **Boost.Filesystem**, which `libcpu_bbs3d.so` uses (`voxelmaps_io.cpp`) but does not
  link, so a consumer must link it. No PCL in the core (its test programs need PCL).
  Three source files, 3.6 s to build with 4 jobs, a 135 kB shared library.
- API: `set_tar_points(map, min_level_res, max_level)` builds the voxel hierarchy
  (sparse hash buckets per level), `set_src_points(scan)`, `localize()` returns the
  **single best** 4x4 pose and its score (scan points in occupied voxels). Roll and
  pitch are searched within +-0.02 rad by default, so the scan must be levelled, as G2's
  already is. No candidate list, no confidence other than the score.

## Files

| file | what |
|---|---|
| `bbs3d_pybind.cpp`, `CMakeLists.txt` | pybind11 module `bbs3d_py`: the library's `localize()`, and `localize_topk()`, the same branch-and-bound continued until k leaves that are not within `nms_radius_m` and `nms_yaw` of a better one (pruned below `relative_floor` x the best score; MIT code adapted, see the header) |
| `eval_global_search.py` | asks `g2` (GlobalLocalizationEngine, cross-session suite settings) or `3dbbs` (+ optional NDT refinement and ranking by the same G2 scorer) about the scans of a bag, levelled with the ground-truth attitude like `g2_offline_probe.py`; writes per-query candidates, errors (incl. height), time, peak RSS |
| `make_reference.py` | reference poses beyond the motion-capture area: the run's RKO-LIO odometry moved into the ground-truth frame with its own map's alignment, NDT-registered to the target map once per second |
| `compare.py` | table per result file, plus a startup replay through `quickstart_model.decide_startup` from every query (as `g2_offline_probe.py`'s consumers did); `--raw` scores 3D-BBS's own top before NDT |

## Reproduce

```bash
# 3D-BBS CPU library, outside the repository
git clone https://github.com/KOKIAOKI/3d_bbs.git $W/3d_bbs   # 41529a3
cmake -S $W/3d_bbs -B $W/bbs3d-build -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF
cmake --build $W/bbs3d-build -j4
# the binding
cmake -S experiments/3d_bbs_eval -B $W/pybuild -DCMAKE_BUILD_TYPE=Release \
  -DBBS3D_ROOT=$W/3d_bbs -DBBS3D_LIB=$W/bbs3d-build/libcpu_bbs3d.so \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build $W/pybuild
# with this package (bbs_cpp, g2_ndt_score) built and sourced:
python3 experiments/3d_bbs_eval/eval_global_search.py --method 3dbbs \
  --bbs3d-module-dir $W/pybuild --library-localize --refine \
  --bag $JEPLO_ROOT/bags_pc2_merged/EIL_Stairs --ground-truth $JEPLO_ROOT/gt/EIL_Stairs.txt \
  --map $JEPLO_ROOT/maps_loc/EIL_Box/map.pcd --occupancy <EIL_Box grid>.yaml --out box_stairs.json
python3 experiments/3d_bbs_eval/eval_global_search.py --method g2 ... --out box_stairs_g2.json
python3 experiments/3d_bbs_eval/compare.py box_stairs.json box_stairs_g2.json
```

The JEPLO maps, bags and grids are the ones of
[go2_wide_map_global_init](../go2_wide_map_global_init/README.md) (inflate 0.2 grids from
the quickstart cache). For `Long_Stairs` map -> `Outdoor2` scans the ground truth was made with

```bash
python3 experiments/3d_bbs_eval/make_reference.py --bag $JEPLO_ROOT/bags_pc2_merged/Outdoor2_0-165s \
  --odometry rko_results/Outdoor2_300_0/Outdoor2_300_tum_0.txt \
  --align maps/Outdoor2_165/map.align.json --map $JEPLO_ROOT/maps_loc/Long_Stairs/map.pcd \
  --out Outdoor2_on_Long_Stairs.tum --end-sec 166
```

(165 registrations, none rejected; corrections median 0.03 m, at most 0.12 m and
0.11 m in height.)

3D-BBS settings: JEPLO 0.5 m finest voxels, 6 levels, map thinned to 0.1 m, scan
1-30 m thinned to 0.5 m; Boreas 1.0 m, 6 levels, scan 1-80 m thinned to 2.0 m (the
upstream outdoor example). x, y, z searched over the whole map's bounding box, yaw over
360 degrees, 4 OpenMP threads. G2: the cross-session suite settings
(`go2_jeplo_cross_session.yaml`) and, for Boreas, the settings of the 1 km benchmark in
docs/benchmark.md. Every timed run held the shared CPU lock.

Variants compared (all on the same queries, one scan per second):

| name | flags |
|---|---|
| G2 | `--method g2` |
| 3D-BBS single answer (+ NDT) | `--method 3dbbs --library-localize --refine` (`compare.py --raw` scores it before NDT) |
| 3D-BBS top 8 + NDT | `--method 3dbbs --k 8 --refine --relative-floor 0.9 --timeout-ms 10000 --nms-radius-m 3.0 --nms-yaw-deg 360` (Boreas: 60 s limit) |

Short version of the outcome (details in the docs section): on the JEPLO pairs 66% (G2),
75% (3D-BBS + NDT) and 79% (top 8 + NDT) of the queries were right; on the stairs and the
hall 5 m below G2 put 17 of 38 answers on another level and 3D-BBS 3; the aliased
EIL_Box rooms stay aliased for both; on the Boreas fixture 12/17 (G2) against 17/17. The
cost is time and memory on scenes that are not distinctive (up to 201 s and 1.5 GB for a
single answer without a limit). Not verified: GPU timing, the 1 km Boreas map (not on
disk), `Outdoor2` map → `Long_Stairs` scans (scans not on disk), a live run in the node.
