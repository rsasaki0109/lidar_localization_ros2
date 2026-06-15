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
- C++ core implementation + bit-exact fixture verification: **next**, machine-independent.
- Idle-machine wall-clock benchmark of the ported query: **after**, gated on a
  quiet shared machine (the development plan's load < ~5 rule).

## Reproduce

The profiling/threshold harness is ad-hoc (`/tmp`); the inputs are the committed
`outdoor_hard_occupancy` map and a seeded 512-point scan sampled from occupied
cells within 30 m of the map-centre pose. Re-derive with `cProfile` over
`branch_and_bound_candidates` and compare candidate tuples across threshold
overrides.
