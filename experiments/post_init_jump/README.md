# Tracking jump right after a correct initial pose (JEPLO EIL_Box)

In [go2_wide_map_global_init](../go2_wide_map_global_init/README.md), the Go2 lost EIL_Box in
all four runs with 32 global candidates. The global answer itself was right (0.01-0.05 m), but
within 1-3 s the track slid about 4.5 m along the aisle and stayed there.

**Result:** a controlled ground-truth start reproduces the jump. An opt-in rule,
`seed_correction_guard_exact_start_translation_m`, stops it: 0.1 m held both runs that jumped
4.5 m with the quickstart settings. On every other replay it never fired, or it fired with no
effect on the result. It stays off (0) in quickstart configs until you decide otherwise.

## Mechanism

- The EIL_Box scene differs from the EIL_Mix map around the box: NDT fitness at the true pose
  is 0.041, against 0.010-0.017 on the other EIL sequences (offline probe).
- Failing runs started tracking 11-13 s into the bag, as the Go2 walks off the box (0.5-0.7
  m/s, down 0.6 m). Within 1-3 s NDT accepted several 1-2.5 m corrections (fitness
  0.004-0.04) along the aisle, from odometry seeds that had moved only 0.1-0.5 m.
- The seed correction guard (quickstart: 0.3 m, 15 deg, 30 rejections to release) only
  applies after 5 settled accepts, so it was still warming up. Every large accept resets
  that count. Once the track had settled at the wrong place, the guard armed there and
  rejected NDT's corrections back toward the truth.

## Controlled start

`gt_initial_pose.py` publishes the ground-truth pose on `/initialpose` once the replay reaches a
chosen bag time, and `quickstart_gt_start.py` runs it next to quickstart (with
`--no-auto-initialize`, so nothing else starts tracking). [`controlled.yaml`](controlled.yaml)
starts EIL_Box on the box (8.5 s) and during the step-off (11.5-13.0 s), and 8.5 s shifted
+1.0 / +1.5 m along the aisle. `quickstart_overrides.py` applies localizer parameters from
`LL_PARAM_OVERRIDES` (JSON) without changing a default. Two runs per row, max xy error:

| start | warmup 5 (quickstart) | warmup 1 | warmup 2, 3 | exact start 0.1 m |
|---|---|---|---|---|
| 8.5 s, 11.5 s | ok (0.31-0.33 m) | | | |
| 12.0 s | ok (0.38 m) | ok (0.42 m) | ok | ok (0.52 m) |
| 12.5 s | ok (0.52 m) | | | ok (0.37 m) |
| **13.0 s** | **both lost (4.5 m)** | ok (0.52 m) | both lost | **ok (0.42 m)** |
| 8.5 s +1.0 m | both lost | both lost | | both lost |
| 8.5 s +1.5 m | both lost | both lost | | |

- From 13.0 s, the first accepted correction was 0.05-0.07 m, and the next scan brought a
  1.7-2.0 m alias (fitness 0.004). With exact start the guard rejected it, and tracking went on
  with 0.02-0.24 m corrections.
- The shifted starts fail with every variant: the aisle constrains the along-track direction
  weakly, and NDT stops 3-4 m from the truth. This is a different problem.

## Why not a warmup of 1

A warmup of 1 also holds the 13.0 s start, but blocks the corrections that pull in a coarse
pose. `warmup_window_audit.py` judged every accepted correction above 0.3 m that only the
warmup let through (after the first settled accept, before the fifth) by the ground truth:

- JEPLO EIL replays (defaults, low-sensor settings, controlled starts): all 15 moved the pose
  away from the truth (aliases).
- Koide handheld replays (two repeats, kidnap sequences included): 75 toward, 77 away. Of the
  corrections from a pose more than 0.3 m off, 45 brought it closer to the truth; a warmup of
  1 would have rejected those pull-ins too.

On the EIL sequences with quickstart defaults (3 runs each), warmup 1 started wrongly in 6 of
12 runs, against 3 of 12 with warmup 5. Koide with warmup 1 matched warmup 5 in the one valid
repeat (the other repeat's odometry stalled, see Validity).

## Exact start

`seed_correction_guard_exact_start_translation_m` (default 0, off): if the first accepted
update after an initial pose corrects it by at most this much, the pose needed no pulling in,
so the warmup counts as done and the guard holds from the next scan. Otherwise the normal
warmup applies. 0.1 m separates the starts seen here: correct initial poses had a first
correction of 0.02-0.07 m, the default global search and Koide 0.11-2.6 m.

With 0.1 m, regression replays (`regression.yaml`, `koide_replay.yaml`):

| replay | runs | rule fired | result |
|---|---|---|---|
| EIL Box/Stairs/Mask1/Mask2, quickstart defaults | 12 | 0 | 5 of 12 wrong starts (warmup 5: 3 of 12) |
| Long_Stairs, Outdoor1, Outdoor2, low-sensor settings | 3 | 1 (Outdoor1) | 2 ok (max 0.09-0.10 m); Outdoor2 not initialized |
| Koide, repeat 1 | 6 sequences, 15 initial poses | 0 | as warmup 5 within run-to-run spread |
| Koide, repeat 2 | 6 sequences | (running) | |

Where the rule did not fire, the code path is the same as before, so those differences are
run-to-run spread of the global search. Wrong EIL starts had first corrections of 0.46-2.1 m;
the Outdoor2 global search returned ambiguous answers in all six tries before any scan was
tracked.

## Validity

- RKO-LIO stalled for 10-100 s in some runs after the container restarted, and the startup
  then failed for lack of odometry. Runs are kept only if RKO-LIO's worst registration loop
  is about 0.17-0.18 Hz (the first scan); stalled runs (0.00-0.08 Hz: one Koide warmup-1
  repeat, the first exact-start wide-map and Koide attempt) were re-run or excluded.
- 2-3 runs per variant on one 4-core machine, Jazzy.

## Not verified

- No sequence other than the controlled EIL_Box start exercised the rule where it changed
  the result. The case it would harm, a wrong initial pose that needs almost no correction,
  was not observed; the guard would then hold that pose until 30 rejections release it, as
  it does after the normal warmup.
- Whether a jump right after a correct initial pose happens outside this capture room.

## Reproduce

```bash
export JEPLO_ROOT=... JEPLO_LIDARSLAM_MAP=... RKO_ODOMETRY_LAUNCH="lidarslam rko_lio_odometry.launch.py"
LL_PARAM_OVERRIDES='{"seed_correction_guard_exact_start_translation_m": 0.1}' \
  python3 tools/benchmark/run_benchmark.py experiments/post_init_jump/controlled.yaml \
  --case EIL_Box_gt_13_0 --out /tmp/pij_exact \
  --quickstart "python3 experiments/post_init_jump/quickstart_gt_start.py"
python3 experiments/post_init_jump/warmup_window_audit.py /tmp/pij_w5/*/run* \
  --gt-for EIL_Box=$JEPLO_ROOT/gt/EIL_Box.txt
```

[`fixtures.yaml`](fixtures.yaml) holds the first, uncontrolled fixtures (start time set by
the global search). They could not separate warmup variants and are kept for reference.
