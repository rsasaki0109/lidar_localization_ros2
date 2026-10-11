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

## Next steps

Nothing here is ready for runtime. Candidates:

1. **Validate `poor_accept_a` on more kidnap runs.** Only one sequence had off-but-OK
   scans. Use more repeats of `outdoor_kidnap_a`/`_b` and the indoor kidnap sequences.
2. **Score WARN and ERROR separately in the benchmark health table.** "Measurement
   rejected, pose carried by odometry" is not the same alarm as "lost". This changes the
   metric only.
3. **Show loss of tracking in the benchmark summary.** Report `tracked_fraction` next to
   the error percentiles, so a run that stops publishing does not look accurate.
