# Tracking jump right after a correct initial pose (JEPLO EIL_Box)

In [go2_wide_map_global_init](../go2_wide_map_global_init/README.md), the Go2 lost EIL_Box in
all four runs with 32 global candidates. The global answer itself was right (0.01-0.05 m), but
within 1-3 s the track slid about 4.5 m along the aisle and stayed there. This experiment tested
whether the seed correction guard's warmup (`seed_correction_guard_warmup_accepts`, 5 in
quickstart configs) lets that slide through. **It does not explain the failures, and no runtime
code or default changes.**

## Mechanism

- The EIL_Box scene differs from the EIL_Mix map around the box: NDT fitness at the true pose
  is 0.041, against 0.010-0.017 on the other EIL sequences (offline probe).
- Failing runs started tracking 11-13 s into the bag, as the Go2 walks off the box (0.5-0.7
  m/s, down 0.6 m). Within 1-3 s NDT accepted several 1-2.5 m corrections (fitness
  0.004-0.04) along the aisle, from odometry seeds that had moved only 0.1-0.5 m.
- The guard was still warming up. On this 4-core machine the first scan after an initial
  pose often comes about 1 s later, which can reset the count of settled accepts. Every
  large accept resets it again. Once the track had settled at the wrong place, the guard
  armed there and rejected NDT's 0.4-1.6 m corrections back toward the truth.
- Runs with a correct initial pose that started tracking by 10.6 s never saw a correction
  above 0.25 m.

## Variants

`quickstart_overrides.py` runs quickstart with localizer parameters from
`LL_PARAM_OVERRIDES` (JSON), so `run_benchmark.py --quickstart` can vary
`seed_correction_guard_warmup_accepts` without changing a default.

Fixtures ([`fixtures.yaml`](fixtures.yaml)):

- `EIL_Box_cand32`: low-sensor global search with 32 candidates (slow queries, late start).
- `EIL_Box_late`: quickstart defaults, replay started 3 s into the bag.

| warmup | `EIL_Box_cand32`: start (bag s) and result per run | `EIL_Box_late` |
|---|---|---|
| 5 (current; runs from go2_wide_map_global_init) | 11.3, 11.8, 11.1, 12.3: all lost (4.5 m) | 1 lost (5.0 m), 1 not initialized |
| 3 | not initialized; 13.1: lost (first scan +2.04 m); 10.6: ok | 2 wrong inits |
| 2 | 9.3, 9.3, 10.3: all ok | 2 wrong inits |
| 1 | 10.3, 10.3, 9.8: all ok | 2 wrong inits |

## Reading

- No evidence that a smaller warmup helps. Every successful run started before the step-off,
  and none of them had a correction the guard would have judged. The start time depends on
  how many global queries the startup needs, so these fixtures do not control it.
- The one late start with a small warmup (warmup 3, 13.1 s) failed on the first scan after
  the initial pose, a 2.04 m correction no warmup of 1 or more can reject. Arming the guard
  from the first scan would also reject the pull-in of a default (unrefined) global pose,
  which is 1-1.5 m off.
- `EIL_Box_late` is a wrong *global* answer, not a tracking jump. With the default G2
  settings during the step-off, queries returned places 4-5 m off at yaw 0 deg. Two
  consecutive wrong answers agreed after the odometry motion, and the startup accepted them.
  With a saturated 2D score, branch-and-bound breaks ties in a fixed order, so the same wrong
  place comes back query after query. The two-view consensus assumes independent errors and
  does not catch it. The low-sensor settings avoid the saturation on the Go2.

## Not verified

- A controlled start (tracking from a known-good pose at a chosen moment of the step-off)
  was not built, so a guard change cannot be judged with these fixtures.
- 2-3 runs per variant, one 4-core machine, Jazzy. No Koide replay.

## Reproduce

```bash
export JEPLO_ROOT=... JEPLO_LIDARSLAM_MAP=... RKO_ODOMETRY_LAUNCH="lidarslam rko_lio_odometry.launch.py"
LL_PARAM_OVERRIDES='{"seed_correction_guard_warmup_accepts": 2}' \
  python3 tools/benchmark/run_benchmark.py experiments/post_init_jump/fixtures.yaml --out /tmp/pij_w2 \
  --quickstart "python3 experiments/post_init_jump/quickstart_overrides.py"
```

[`regression.yaml`](regression.yaml) (four EIL sequences with defaults, three wide maps with the
low-sensor settings) was prepared for a winning variant and not run.
