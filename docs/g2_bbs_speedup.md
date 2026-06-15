# G2 BBS query speedup: profiling and the C++ port plan

The G3 live closed-loop work ([g3_live_closed_loop.md](g3_live_closed_loop.md))
isolated **query latency** as the dominant blocker to live recovery: a BBS query
takes ~5–23 s, so the candidate it returns is stale by the time the supervisor
publishes it. [Seed motion compensation](g3_live_closed_loop.md) (lever 2)
mitigates this from the consumer side; the root fix is to make the query itself
fast. This page records what actually costs the time and the resulting port plan,
so the implementation does not start from guesswork.

## Profiling the real map (Koide outdoor_hard)

`branch_and_bound_candidates` was profiled on the real occupancy map
(`outdoor_hard_occupancy`, 1439×1063 cells at 0.2 m, 581 306 occupied), a 512-point
scan, `angular_resolution 5°`, `pyramid_depth 4`, `max_candidates 16`. The machine
was loaded, so absolute times are inflated (~2× the idle ~8 s baseline noted in the
development plan); the **relative breakdown is the load-independent finding**:

| Component | cumulative share | calls | what it is |
| --- | --- | --- | --- |
| `gather_hits` (cold path) | **~61 %** | **250 554** | per-node integer gather over the rotated-scan offset cells, in numpy |
| `full_hit_map` (FFT) | ~18 % | 223 | one-shot FFT cross-correlation for hot (yaw, level) pairs |
| heap `heappop`/`heappush` | ~11 % | 564 k / 2 247 k | the best-first branch-and-bound queue |

The decisive correction to the development plan's earlier note ("the remaining
floor is the Python heap loop"): the heap loop is only ~11 %. The real floor is the
**250 k `gather_hits` calls**, each paying Python+numpy per-call overhead (array
alloc, `np.any`, fancy-index, `sum`) on a *tiny* array of offset cells. It is
interpreter/call overhead, not arithmetic.

## What does NOT work: tuning the FFT-switch threshold

`gather_hits` already switches a hot (yaw, level) pair from per-node gather to a
single FFT hit map once it has been queried `max(64, H·W/256)` times. The obvious
cheap hypothesis was: lower that threshold so more pairs use the fast FFT map and
fewer pay the 250 k cold gathers. **Tested and refuted** — every threshold gives
the same wall time within noise, and raising the FFT usage too far makes it
*worse* (the large-grid FFTs cost more than the gathers they replace):

| FFT-switch threshold | wall (loaded) | candidates identical? |
| --- | --- | --- |
| default `max(64, H·W/256)` | 18.2 s | — |
| 2048 | 25.6 s | yes |
| 1024 | 21.8 s | yes |
| 512 | 20.2 s | yes |
| 256 | 17.9 s | yes |
| 128 | 18.7 s | yes |
| 64 | 18.2 s | yes |

Two things are confirmed: the adaptive threshold is already near-optimal (no free
Python win), and the FFT path is **bit-exact** to the direct gather (every row's
candidates are identical — the integer-rounding argument holds). There is no cheap
algorithmic shortcut left in Python; sub-second requires leaving the interpreter.

## The port plan, and why it is simpler than it looks

The key insight from the profile: **the FFT path exists only to work around
Python's slow per-cell gather.** In C++ the direct gather is a tight loop over a
few hundred integer offsets with no per-call overhead — fast enough that the FFT
machinery is not needed at all. So the C++ port is *smaller* than the Python
module, not larger:

Port to C++ (pure, no external FFT dependency):

1. `build_occupancy_pyramid` + `build_upper_bound_pyramid` (max-pool + 1-cell dilate)
2. the per-(yaw, level) offset cache (rotate scan, floor to cells, dedup with counts)
3. the best-first heap search with k-th-best pruning and optional NMS
4. the direct integer `gather_hits` (no FFT branch)

Keep out of the port: the FFT `full_hit_map` (dropped — unneeded in C++), and all
map loading / CSV / ROS plumbing (stays in Python; the C++ core takes plain arrays).

Bit-exactness strategy: the C++ core computes the *same exact integer hit counts*
as Python, so the candidate list must be identical. This is verifiable without an
idle machine — run the existing `test_bbs_relocalization_attempts` fixtures and a
few medium synthetic maps through both and assert identical `(tx, ty, yaw, hit)`
tuples. Only the final wall-clock *speed* claim needs an idle machine.

Integration: expose the core via a small pybind11 module built by the existing
ament/CMake setup (PCL/Eigen already linked), imported by
`global_localization_node.py` with a Python fallback to the current implementation
when the extension is absent — same opt-in, no behavior change by default.

Expected payoff: removing the 61 % gather overhead and the 11 % Python heap loop
(72 % of the time) by moving them to compiled code should bring the idle ~8 s
query toward the sub-second target the roadmap's G1 optimization note calls for,
which — combined with lever 2 motion compensation — closes the live-recovery
latency gap.

## Status

- Profiling and the threshold refutation: **done** (this page).
- C++ core implementation + bit-exact fixture verification: **done**.
  `include/lidar_localization/bbs_branch_and_bound.hpp` is the pure-C++ port
  (pyramid + offset cache + heap + direct gather, no FFT). It is verified
  bit-exact against the Python reference by `test/test_bbs_branch_and_bound.cpp`,
  which replays four golden fixtures (single-yaw, multi-yaw, deep-pyramid, and
  NMS) frozen from the Python search by `scripts/generate_bbs_golden_fixtures.py`
  — every `(tx, ty, yaw_index, hit_count)` tuple matches.
- Directional speed (same machine, same load, same input — the **real**
  `outdoor_hard` map, 512-pt scan, 5° yaw, depth 4, NMS 8): the C++ core ran the
  query in **2.6 s vs the Python search's 14.6 s — ~5.5×** — and returned the
  identical top candidate (hit 499 at cell (397, 175), yaw 8), confirming
  bit-exactness on the full map too. The machine was loaded (the development
  plan's idle Python baseline is ~8 s), so this ratio — not the absolute time —
  is the load-independent result. Scaling the idle baseline by ~5.5× puts the
  ported query near ~1.5 s, and with a coarser yaw step (which the plan notes
  roughly halves the Python query) into the sub-second range that — combined with
  lever-2 seed motion compensation — closes the live-recovery latency gap.
- Idle-machine wall-clock benchmark and the pybind11 runtime wiring into
  `global_localization_node.py` (opt-in, Python fallback): **next**, the bench
  gated on a quiet shared machine (load < ~5).

## Reproduce

Bit-exactness (committed, machine-independent):

```bash
python3 scripts/generate_bbs_golden_fixtures.py   # refresh the golden from Python
g++ -std=c++17 -I include -I test \
    test/test_bbs_branch_and_bound.cpp -o /tmp/t && /tmp/t
```

The profiling/threshold/timing harness is ad-hoc (`/tmp`); the inputs are the
committed `outdoor_hard_occupancy` map and a seeded 512-point scan sampled from
occupied cells within 30 m of the map-centre pose. Re-derive the profile with
`cProfile` over `branch_and_bound_candidates`; re-derive the speed ratio by
dumping that map+scan and running it through both the Python search and the C++
core on the same idle machine.
