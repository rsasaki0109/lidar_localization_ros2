# /alignment_status as a health signal

Development item 2: can `/alignment_status` tell a lost pose from a good one? The
[benchmark](../../docs/benchmark.md#results) found two problems:

- it reports OK on many poses that are far off (`outdoor_kidnap_b`);
- it raises alarms on 20-41% of good poses.

This experiment only analyses the data; no runtime code or default changes.

`analyze_health.py` joins each status row with the map-frame GT error of the pose
published for the same scan. A pose is **off** when it is more than 1 m from GT, and
**good** when it is within 0.3 m. A row counts as flagged when its level is not OK,
or when a candidate rule fires on an OK row.

```bash
python3 analyze_health.py --gt outdoor_kidnap_b=$KOIDE_ROOT/gt/traj_lidar_outdoor_kidnap.txt \
  /tmp/runs/outdoor_kidnap_b/run*
```

## Data

All data is official Koide data, replayed by `tools/benchmark/run_benchmark.py` at 1x on
ROS 2 Jazzy. The odometry was RKO-LIO with `rko_lio_mid360.yaml`, in the default
`window_waiver` mode.

- **Selection set**: `outdoor_hard_02b` and `outdoor_kidnap_b`, three runs each. These are
  the `window_waiver` runs of the
  [dropout replay](../odometry_dropout_confirmation/README.md#replay-result-2026-10-11-not-promoted).
- **Held-out set**: `outdoor_hard_01a`, `outdoor_hard_01b`, `outdoor_hard_02a` and
  `outdoor_kidnap_a`, one run each. The rules below were fixed before these runs.

The runs used a 4-core cloud container. Compare rules with each other, not with
`docs/benchmark.md`.

## Results (2026-10-11)

Each rule cell gives the share of off rows flagged and the share of good rows flagged.

| set | case | status rows | off | off reported OK | good | current | poor_accept_a | poor_accept_b |
|---|---|---:|---:|---:|---:|---|---|---|
| selection | `outdoor_hard_02b` | 4248 | 34 | 28 | 3959 | 0.18 / 0.26 | 0.21 / 0.26 | 0.18 / 0.26 |
| selection | `outdoor_kidnap_b` | 466 | 148 | 120 | 312 | 0.19 / 0.03 | 0.76 / 0.10 | 0.74 / 0.07 |
| held-out | `outdoor_hard_01a` | 2002 | 181 | 0 | 1395 | 1.00 / 0.27 | 1.00 / 0.27 | 1.00 / 0.27 |
| held-out | `outdoor_hard_01b` | 1403 | 0 | 0 | 1398 | - / 0.34 | - / 0.35 | - / 0.34 |
| held-out | `outdoor_hard_02a` | 2210 | 57 | 1 | 1824 | 0.98 / 0.38 | 1.00 / 0.39 | 1.00 / 0.39 |
| held-out | `outdoor_kidnap_a` | 38 | 0 | 0 | 36 | - / 0.03 | - / 0.03 | - / 0.03 |

The rules flag a row whose level is OK when both of these hold:

- `poor_accept_a`: `fitness_score > 0.5` and `correction_translation_m > 0.3`.
- `poor_accept_b`: `fitness_score > 1.0` and `correction_translation_m > 0.3`.

## Findings

**Off but reported OK.** This happened only on `outdoor_kidnap_b` (120 of 148 off rows).
- Those scans were accepted with a poor fit (median fitness 2.2 against 0.04 on good
  poses) and a large correction (median 0.9 m against 0.06 m).
- `poor_accept_a` raises the flagged share of off rows there from 0.19 to 0.76. It also
  flags 10% of good rows, where today's status flags 3%.
- The held-out runs contain only one off row reported OK, so they cannot confirm the
  catch rate. They only show that the rules add at most one point of false alarms.
- The fitness scale depends on the map. On `outdoor_hard_02b`, more than half of the good
  accepted scans have a fitness above 0.5. Only the correction term keeps the rule quiet
  there.

**Alarms on good poses.** These are consistent: 26-38% on every `outdoor_hard_*` sequence.
- Almost all are level 1 with `fitness_score_over_threshold_rejected`. NDT rejects the scan,
  and the odometry bridge carries the pose.
- RKO-LIO held those poses within 0.3 m for a median of 32 m (up to 108 m) since the last
  accepted scan.
- The status values cannot tell these from truly lost poses. The rejected rows that were
  more than 1 m off cover the same ranges of time since accept, distance since accept, and
  rejection streak.
- The status reports, correctly, that the pose is unverified. The benchmark counts every
  not-OK level as an alarm.

**Benchmark caveat found on the way.** In the `outdoor_kidnap_a` held-out run, every scan
after 36 s was rejected or empty, and the supervisor gave up after 6 attempts. The
localizer stopped publishing `/pcl_pose` for the remaining ~146 s. The summary table still
shows a 0.05 m median, because only the 40 published poses are scored; `tracked_fraction`
is the column that shows the loss.

## Benchmark changes (2026-10-11)

Items 2 and 3 of the original next steps are now in `tools/benchmark`.

- **tracked**: the summary table reports `tracked`. Its coverage now runs to the end of the
  replay, so a localizer that stops publishing loses it. Before, coverage ended at the last
  published pose.
- **lost**: the health table adds "lost" shares, which count only scans where the localizer
  requests reinitialization (or stopped reporting).

Re-scored on the same runs:

| case | tracked | flagged when off / good | lost when off / good |
|---|---:|---|---|
| `outdoor_hard_01a` | 0.75 | 1.00 / 0.27 | 1.00 / 0.03 |
| `outdoor_hard_01b` | 0.65 | - / 0.34 | - / 0.20 |
| `outdoor_hard_02a` | 0.80 | 0.98 / 0.38 | 0.98 / 0.25 |
| `outdoor_hard_02b` | 0.66 | 0.10 / 0.26 | 0.00 / 0.12 |
| `outdoor_kidnap_a` | 0.03 | 1.00 / 0.03 | 0.00 / 0.00 |
| `outdoor_kidnap_b` | 0.06 | 0.17 / 0.02 | 0.02 / 0.00 |

Reading the new columns:

- **Lost cuts alarms on good poses by a third to nine tenths** on the `outdoor_hard_*`
  sequences, and keeps the off poses of `01a` and `02a`.
- **But lost misses the kidnap failures**, so neither column alone is a health signal.
  `outdoor_kidnap_a` lost tracking after 36 s and its status went stale. Its few off poses
  carried WARN rather than a reinitialization request.
- **tracked now shows the `outdoor_kidnap_a` loss** (0.03). On this 4-core machine it stays
  at 0.65-0.80 even for good runs, because the localizer does not keep up with every
  scan. Compare it within one machine.

## Next steps

Nothing here is ready for runtime. Candidates:

1. **Validate `poor_accept_a` on more kidnap runs.** Only one sequence had off-but-OK
   scans. Use more repeats of `outdoor_kidnap_a`/`_b` and the indoor kidnap sequences.
2. **Done above**: lost shares in the health table, instead of WARN and ERROR. ERROR is
   only `filtered_scan_empty` here, so it does not mean lost.
3. **Done above**: `tracked` in the summary table, with coverage to the end of the replay.
4. **Publish an explicit pose source in `/alignment_status`**: registration, odometry bridge,
   or none. A consumer then does not have to infer it. This would need a runtime change and
   its own validation.
