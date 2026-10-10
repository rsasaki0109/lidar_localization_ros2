# Odometry-dropout answer confirmation

Steps 1 and 2 of the first development target in [v1_status.md](../../docs/v1_status.md#next-work):
fixtures with odometry dropouts, and a comparison of confirmation policies against the
runtime supervisor. **No runtime code or default uses these variants.** The results come from
synthetic timelines, not a public replay.

## Problem

Normally, `reinitialization_supervisor_node._withhold_unconfirmed_answer` releases a G2 answer
only when a later answer agrees with it after the bridged odometry motion. Today the supervisor
releases the answer immediately in two cases:

- the odom TF had a gap of more than 2 s anywhere in the last 120 s (the dropout waiver from
  `42ded13`, needed for `outdoor_kidnap_b` recovery), or
- no `odom_bridge_pose` lies within 0.5 s of the answer's scan, for example because the scan falls
  inside the dropout.

On one quickstart benchmark run of Koide `outdoor_hard_02b`, RKO-LIO dropped frames that had too
few keypoints, and one answer reset the pose 128 m away.

## Variants

All variants share the supervisor's consensus test, defaults and attempt budget
(`max_attempts=3`). A withheld answer costs one attempt, as `weak_candidate_rejected` does.

| Variant | Rule |
|---|---|
| `window_waiver` | runtime today (mirrors `_withhold_unconfirmed_answer`) |
| `no_waiver` | the waiver deleted: answers are paired across gaps |
| `segment` | a pair confirms only if odometry has no gap between the two answers |
| `segment_bridge_guard` | `segment`, plus withhold an answer with no bridged pose while one was seen within the window |
| `segment_defer` | `segment`, but skip the query (no attempt spent) while the odometry is out |
| `segment_defer_speed_bound` | `segment_defer`; a pair across a gap agrees if it is reachable at 1.5 m/s |

## Results

Regenerate with `python3 dropout_confirmation.py --json results.json`. The test
`test/test_odometry_dropout_confirmation_experiment.py` fails when `results.json` is stale.

| Variant | Design | Passed | Wrong resets | Score |
|---|---|---:|---:|---:|
| `window_waiver` | runtime baseline | 6/10 | 4 | 60.0 |
| `no_waiver` | waiver removed | 5/10 | 3 | 50.0 |
| `segment` | gapless pair | 7/10 | 1 | 70.0 |
| `segment_bridge_guard` | gapless pair + withhold inside dropout | 7/10 | 0 | 70.0 |
| `segment_defer` | gapless pair + defer query inside dropout | 9/10 | 0 | 90.0 |
| `segment_defer_speed_bound` | segment_defer + speed-bound pair across gaps | 8/10 | 2 | 80.0 |

Each cell gives the outcome and, for a correct reset, the time to recover after tracking was lost:

| Fixture | `window_waiver` | `no_waiver` | `segment` | `segment_bridge_guard` | `segment_defer` | `segment_defer_speed_bound` |
|---|---|---|---|---|---|---|
| `hard02b_wrong_answer_after_short_dropout` | **FAIL**: wrong reset | pass: 18 s | pass: 18 s | pass: 18 s | pass: 18 s | pass: 18 s |
| `hard02b_wrong_answer_inside_dropout` | **FAIL**: wrong reset | **FAIL**: wrong reset | **FAIL**: wrong reset | pass: 14 s | pass: 14 s | pass: 14 s |
| `kidnap_covered_carry` | pass: 5 s | pass: 11 s | pass: 11 s | pass: 11 s | pass: 11 s | pass: 11 s |
| `kidnap_stationary_alias_across_gap` | **FAIL**: wrong reset | **FAIL**: wrong reset | pass: no reset | pass: no reset | pass: no reset | **FAIL**: wrong reset |
| `kidnap_answers_during_cover` | pass: 5 s | **FAIL**: no reset | **FAIL**: no reset | **FAIL**: no reset | pass: 11 s | pass: 11 s |
| `kidnap_intermittent_odometry` | pass: 6 s | **FAIL**: no reset | **FAIL**: no reset | **FAIL**: no reset | **FAIL**: no reset | pass: 12 s |
| `continuous_alias_then_correct` | pass: 22 s | pass: 22 s | pass: 22 s | pass: 22 s | pass: 22 s | pass: 22 s |
| `no_external_odometry` | pass: 5 s | pass: 5 s | pass: 5 s | pass: 5 s | pass: 5 s | pass: 5 s |
| `repeated_alias_across_short_dropout` | **FAIL**: wrong reset | **FAIL**: wrong reset | pass: no reset | pass: no reset | pass: no reset | **FAIL**: wrong reset |
| `odometry_lost_permanently` | pass: 6 s | pass: 6 s | pass: 6 s | **FAIL**: no reset | pass: 126 s | pass: 126 s |

A fixture passes when there is no wrong reset, and, where recovery is required, a correct one.

## Reading

- The runtime waiver resets wherever the first answer after a gap points. In these fixtures that
  is 4 of the 10 timelines, including both readings of the 02b failure.
- Deleting the waiver (`no_waiver`) is not a fix, as `v1_status.md` warns. Bridged motion across a
  gap still pairs answers, so a stationary-looking alias confirms. A short dropout also leaves the
  answer from inside the gap unguarded.
- `segment_defer` is the leading candidate: 0 wrong resets, and it passes every fixture except
  intermittent odometry. Its costs:
  - after a covered carry, one more query cycle (11 s instead of 5 s);
  - when the LIO never returns, no reset until the bridge has been absent for the whole 120 s
    window (126 s instead of 6 s);
  - no recovery at all when the odometry drops out between every two answers.
- Bounding the platform speed buys back the intermittent case. It also confirms an alias that a
  carried or slow-moving sensor would reach, which brings back wrong resets.

## What the replay must decide

The synthetic fixtures do not show which of these timelines the real sequences follow. Before any
runtime change, follow steps 3 and 4 of `v1_status.md`:

1. In the 02b run logs, find whether the 128 m answer came from a scan after the gap
   (log line: `odometry dropped out within ... not waiting`) or from a scan inside it (no such
   line, no bridged pose).
2. In `outdoor_kidnap_b`, check whether the odometry resumes continuously after each cover
   (`kidnap_covered_carry`) or keeps dropping out (`kidnap_intermittent_odometry`), and whether G2
   answers on covered scans (`kidnap_answers_during_cover`).
3. Replay `outdoor_hard_02b` and `outdoor_kidnap_b` with `window_waiver` and `segment_defer`.
   Use the same map, odometry, parameters and repeat count. Report wrong resets, time to
   recover, time within 3 m, reset count and upstream frame drops.

`segment_defer` is available as an opt-in on the supervisor
(`odometry_confirmation_mode:=segment_defer`). The runtime decides "odometry is out" from the
odom TF stamps (no stamp for more than `odometry_max_gap_sec`), where the simulator uses the
bridged pose. Both runs below use the same suite:

```bash
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/koide_outdoor.yaml \
  --case outdoor_hard_02b --case outdoor_kidnap_b --out /tmp/dropout_window_waiver
python3 tools/benchmark/run_benchmark.py tools/benchmark/suites/koide_outdoor.yaml \
  --case outdoor_hard_02b --case outdoor_kidnap_b --out /tmp/dropout_segment_defer \
  --quickstart "ros2 run lidar_localization_ros2 quickstart.py \
    --supervisor-odometry-confirmation-mode segment_defer"
```
