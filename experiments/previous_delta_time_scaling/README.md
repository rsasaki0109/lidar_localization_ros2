# Previous-delta seed after dropped scans

Question: the previous-delta seed advances the last pose by one inter-scan motion,
whatever time has passed. On Boreas a 0.6 s gap at 9.5 m/s left it 5 m behind, and
NDT locked 4 m back along the road at a passing fitness
([benchmark.md](../../docs/benchmark.md#boreas-vehicle-preset-replay)). Does rescaling
the motion to the elapsed time hold the track?

## Change

`previousDeltaMotionAt` (`include/lidar_localization/prediction_state_policy.hpp`)
rescales the last accepted motion. It took `last_relative_motion_duration_sec_`; it is
applied over the time since the predicted pose, at most
`previous_delta_max_extrapolation_sec` (default 1.0 s): the translation linearly, the
rotation by the same factor about its axis. The seed and the prediction advanced over a
rejected scan both use it. Without a usable duration or time anchor, or with the limit
at 0, it is the stored motion: one delta per scan, the old behavior.

It also stops an overshoot. After a gap, the stored motion spans the gap, and the old
seed applied all of it to the next scan 0.1 s later.

## Boreas

`tools/benchmark/boreas_replay.py` on the first 120 s of `boreas-2021-09-02-11-42`, one
build. The old behavior is the same build with
`previous_delta_max_extrapolation_sec: 0`.

- **Dropped scans.** `drop_scans.py` copies the fixture without 6 consecutive scans
  (0.6 s) every 10 s from 10 s on: 11 gaps, 66 of 1158 scans.
- **80 m range.** The preset with `scan_max_range: 80`. Scans there wait for the
  next local map now and then: the gaps that made the preset move to 60 m.

| Fixture | Seed | Runs | RMSE m | Max error m | Scans accepted |
| --- | --- | --- | --- | --- | --- |
| scans dropped | one delta per scan | 2 | 5.83, 6.34 | 39.3, 60.5 | 75%, 77% |
| scans dropped | **scaled to the time** | 2 | **0.132**, 0.132 | **0.41**, 0.41 | 94%, 94% |
| 80 m range | one delta per scan | 1 | 0.929 | 23.5 | 97% |
| 80 m range | **scaled to the time** | 1 | **0.139** | **0.39** | 98% |
| as prepared | one delta per scan | 1 | 0.132 | 0.41 | 100% |
| as prepared | scaled to the time | 1 | 0.132 | 0.41 | 100% |

- **The track holds across the gaps.** With 0.6 s missing every 10 s it scores what
  it scores with no scan missing (0.132 m), and every scan that arrives is accepted
  (1092 of the 1158 arrive).
- **Nothing changes without gaps.** The scale is the ratio of two scan intervals,
  about 1.
- **The preset keeps its 60 m range.** At 80 m, scans still wait for the local map,
  0.139 against 0.132 m.

Istanbul and the online Koide check seed from the twist smoother and the odometry,
not the previous delta, so this change does not reach them.

Measured 2026-11-02 on ROS 2 Jazzy, 4 vCPUs. This is a public-dataset replay, not a
physical-robot test.

## Rerun

```bash
python3 tools/benchmark/boreas_prepare.py /tmp/boreas_fixture --duration 120
python3 experiments/previous_delta_time_scaling/drop_scans.py /tmp/boreas_fixture \
  /tmp/boreas_fixture_drop6
for seed in 0.0 1.0; do
  python3 tools/benchmark/boreas_replay.py /tmp/boreas_fixture_drop6 /tmp/drop_$seed \
    --ws "$WS" --set previous_delta_max_extrapolation_sec=$seed
done
```
