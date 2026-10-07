# Soft (Huber-weighted) odom correction gate

This experiment checks whether the existing `odom_tf_prediction_correction_guard`
(a hard translation/yaw threshold in `measurement_gate_policy.hpp` that fully
accepts or fully rejects the NDT measurement) can be replaced by a continuous
weight, so the exact threshold value stops being a sharp behavior cliff.

## Origin

Found while reading a separate, unrelated research workspace's `r3dl`/`r3dl_core`
(a GTSAM factor-graph localizer forked from GLIM). Its wheel-odometry
`BetweenFactor` uses `gtsam::noiseModel::mEstimator::Huber` instead of a hard
gate — comment in that code: "keep it loose by default so scan-to-map can
still correct global alignment." `wheel_odom_use_robust` defaults to `true`
there, but `enable_wheel_odom` itself defaults to `false` in every shipped
config in that workspace, so that mechanism was never actually exercised with
real data either — it is a design idea borrowed from unused code, not a
validated result.

## What this candidate does

`soft_odom_correction_gate.hpp` gives each axis (translation, yaw) a Huber
IRLS weight (`1.0` inside its threshold, `threshold / value` beyond it),
combines them as `min(w_translation, w_yaw)` (same OR-style "either axis can
object" semantics as the hard gate), and floors the result to `0` below
`floor_weight` (default `0.05`) so a genuinely implausible jump is still
discarded outright, not blended in at a token 2% weight forever.
`blendOdomAndNdtPose()` is the step that would consume that weight: lerp on
translation, SLERP on rotation, between the odom-predicted pose and the
NDT-measured pose.

`test_soft_odom_correction_gate.cpp` runs both this candidate and the real
`evaluateMeasurementGate()` side by side on the same inputs:

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iinclude -Iexperiments/soft_odom_correction_gate \
  experiments/soft_odom_correction_gate/test_soft_odom_correction_gate.cpp \
  -o /tmp/test_soft_odom_correction_gate
/tmp/test_soft_odom_correction_gate
```

Confirmed behavior difference at the production default thresholds
(`0.3 m` / `5°`, from `config/loc_t16_odomseed.yaml`):

| correction_translation_m | hard gate (today) | soft gate (this candidate) |
|---|---|---|
| 0.05 | accept | weight 1.00 |
| 0.31 (3% past threshold) | **reject outright** | weight 0.97 (barely different) |
| 0.6 (2x threshold) | reject outright | weight 0.50 |
| 15.0 (50x threshold, implausible jump) | reject outright | weight 0.00 (floored — same outcome) |

## Decision

**2026-10-06: wired into runtime, opt-in, behind `enable_soft_odom_correction_gate`
(default `false`, zero behavior change when off).** Motivating evidence: on the
Koide outdoor Livox MID360 sequence (`outdoor_kidnap_a`, real GT, synthetic
wheel-odom drift injected via `synth_odom.py`, scale 1.01 / yaw bias
0.005 rad/s), the hard `odom_tf_prediction_correction_guard` showed a genuine
failure mode — once sustained odom drift pushed the correction past the
guard's threshold, it locked out ~20 s of objectively-good NDT corrections
(fitness stayed 0.04–0.07 throughout), then a much looser 30-reject-streak
recovery guard let through a bad/aliased match, causing a temporary breakdown.

Runtime copy (`include/lidar_localization/soft_odom_correction_gate.hpp`) is
unchanged from this experiment, just renamed into the `lidar_localization`
namespace. `applySoftOdomCorrectionGate()` in `component_alignment.cpp` only
intervenes on the exact `"odom_tf_prediction_correction_guard_rejected"` status
message, leaving every other gate path (score threshold, borderline-seed gate,
fitness explosion, etc.) untouched.

**A/B result, same bag/seed/thresholds, only the flag differs:**

| | A: hard guard (today's default) | B: soft gate enabled |
|---|---|---|
| horizontal median / p95 / max / RMSE | 9.0 cm / 164.1 cm / 1.99 m / 72.9 cm | 7.3 cm / 18.4 cm / 0.61 m / 10.0 cm |
| 3D median / p95 / max / RMSE | 9.3 cm / 322.4 cm / 3.55 m / 138.9 cm | 8.7 cm / 24.3 cm / 0.91 m / 14.2 cm |
| angle median / p95 | 0.37° / 7.3° | 0.34° / 1.0° |
| `/pcl_pose` count (same 206 s bag) | 252 | 1060 |

The soft gate removed the breakdown entirely (no deviation past 1 m anywhere
in run B, vs. 1.99 m/3.55 m in A) while leaving the typical-case median
essentially unchanged. It also accepted ~4x more measurements — instead of a
long lockout followed by one risky full-trust correction, it let many
partial-trust corrections through continuously.

**Caveats, read before trusting this further:** single run, single sequence,
*synthetic* odom drift (deterministic seed, but not real wheel odometry). Real
recalibrated wheel odometry on the actual robot (validated 2026-10-05, static
+ live driving/rotation) tracks well enough that it may rarely drift far
enough to even engage this guard — so the real-robot benefit could be much
smaller than this synthetic-drift scenario suggests. This result validates the
*mechanism* (blending beats hard-rejecting on a sustained-drift failure mode);
it is not yet evidence about typical real-robot gain. The production hard gate
(2.8 cm median / 5.1 cm RMSE / 0 lock-loss on the Koide `indoor_easy_01`
benchmark, confirmed on real wheel odometry 2026-10-05) is unaffected — the
new flag defaults off, so nothing currently deployed changes.

Not yet decided: whether to flip the default to `true`. That is a behavior
change to how accepted poses are computed and needs real-robot validation
(or at least an explicit, informed call) before it goes anywhere near
production, per this project's own rule on gating new runtime behavior.
