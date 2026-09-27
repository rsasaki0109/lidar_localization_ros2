# Go2 twist reception and TF outage handling

Integrated from validated candidate `aaf940f` on 2026-09-27. Runtime behavior and
layout changes retain their separate commits. Dataset-specific diagnostic traces
were removed before integration; the experiment READMEs preserve their history.

## Behavior and public contracts

- Direct twist reception takes a short history mutex, independently of scan
  registration and TF waits. EKF/GTSAM routes retain localization-state locking.
  Lock order is localization state, then history; never the reverse.
- Each admitted cloud snapshots the most recent finite twist at or before its
  source stamp from a bounded 1024-sample history. Prediction and rejected-scan
  advancement use that same value. Samples arriving later affect later scans.
  Duplicate stamps replace their value; out-of-order samples remain ordered.
  This is endpoint-velocity prediction, not interval integration. It does not
  impose a stale-sample timeout or detect confident-but-biased velocity.
- Cleanup increments a subscription generation while clearing history. Old
  callback closures cannot insert into a newly configured history or update its
  optional pose backend. Deterministic tests invoke real subscription closures.
- Odom TF paths first attempt an immediate exact lookup. After a timed wait
  fails, they skip another wait until the latest available source stamp changes.
  Successful lookup, initial pose, map update, frame-pair change and lifecycle
  cleanup reset suppression. The wait bound is 200 ms (previously 100 ms).
  No stale transform substitutes for an exact requested-time transform.
- ROS topics/frames, default cloud SensorDataQoS, queue depth, registration guards
  and YAML presets remain unchanged. Cloud reliability now accepts the standard
  ROS override `qos_overrides.<resolved-cloud-topic>.subscription.reliability`.
  For example, a remapped `/livox/lidar` input can explicitly use `reliable`.
  Reliability must be compatible with the publisher; there is no automatic global switch.

## Evidence and limits

The Release candidate passed 69 CTests, 234 Python tests and both bringup help
checks. Native-buffer fixtures exercise delayed insertion, missing/restored TF,
frame changes and resets. Lifecycle tests cover direct, EKF and GTSAM callback
routes. These are not comprehensive DDS scheduling or simulation-clock tests.

The final candidate passed the indoor leave-one-out five-sequence precision
screen and two combined-fault cases. A counterbalanced eight-case CPU campaign
compared it with `9b65a6b`, two runs per version for Box and Mask2. Candidate four
runs passed ATE <=0.15 m and maximum <=0.5 m at GT-covered published outputs.
The baseline failed the maximum-error gate once on Box (0.541 m).

**Box still has approximately 3.1-second output gaps under CPU load.** Mask2
had no substantial output gap in either version in this campaign. Shared-host
load varies, missing-output positions are unknown, and this is not proof of
robustness to all slip, sensor-stop or stale-input cases. No general runtime
speedup, full SLAM map-quality improvement or completion of the Go2 program is
claimed. Registration acceptance thresholds were not relaxed.

Local evidence under `/media/sasaki/aiueo2/jeplo_data/experiments/`:

- `go2_twist_source_layout`: final candidate build/tests and runtime hashes.
- `go2_twist_source_layout_loo`: all seven precision/coverage cases.
- `go2_twist_source_layout_cpu`: load, effective parameters, input loss and recovery.
- `go2_twist_main_integration`: integration review and fresh main-checkout build.

These are local field recordings, not open public benchmark results. Earlier
instrumented-parent repetitions are separate evidence, not final-binary repeats.
