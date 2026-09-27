# Conditional accepted-scan recovery candidate

Experimental only; not wired into the ROS node, installed, or enabled by a preset.
The always-on scan-motion primary failed paced latest-pending tests (Box maximum
error 3.70 m, Mask2 1.06 m); see the external `go2_motion_paced_latest` receipt.
This candidate must pass those availability/false-acceptance constraints before
promotion. Synthetic helper tests do not prove recovery accuracy.

`AcceptedScanSeed` retains one immutable prepared accepted scan, its accepted map
pose/stamp/generation, and optionally its covariance. Healthy acceptance only
updates that reference; it performs no registration. Estimation always registers
the current scan against this accepted reference. Neither rejected motion nor an
unaccepted current cloud moves the reference. Target covariance may be reused
while the accepted reference is unchanged. A local registration object releases
its current source on return, so rejected clouds cannot accumulate.

The proposed caller first runs normal prediction and the existing NDT/last-pose
retry pipeline. Only a remaining rejected measurement may request this optional
seed. Existing retry eligibility (including accepted-gap cap, currently 1 s in the
fixed Go2 cases) must apply. A valid seed is not an accepted pose: run normal map
registration and all existing measurement gates again. No guard relaxation and
no standalone scan-odometry output. The provider does not implement dispatch,
retry-count limits, a ROS parameter, or output publication.

The caller must serialize access, supply immutable prepared clouds in the same
frame, reset on initialization/lifecycle changes, and recheck initialization
generation after expensive work. A stale generation request must not erase a
newer accepted reference. The helper rejects unavailable/invalid/mismatched inputs;
registration exceptions still propagate like the current native alignment call.
Concurrent reset behavior and full lifecycle integration remain unimplemented.

Fixed diagnostic GICP settings match the previous native screen: 20 covariance
neighbors, maximum correspondence .5 m, transform epsilon 5e-4, 30 outer
iterations, existing library rotation/covariance/BFGS defaults. The caller's
prepared cloud is used directly, with no extra downsample path. No new dependency.
A 1 s horizon is an existing retry limit, not a newly validated safety bound.

`test_seed.cpp` checks the source-to-target direction with known transformed
clouds, preservation of the accepted anchor across unaccepted estimates, time
horizon, reset/generation handling, frame mismatch, and nonfinite points. Run with
assertions enabled (`-UNDEBUG` after `-DNDEBUG`). Standalone test compilation is the
current validation scope; no ROS package build or live replay has been performed
because the experiment has not been connected to package targets.

Next: fixed real Box accepted-reference/current pairs, followed by isolated node
dispatch and reset tests, package build, and normal/fault paced replays. Preserve
the existing cheap healthy path and compare latency, drops, and false acceptance.

Optional rotation-only initialization is now available. A supplied world rotation
is converted into the accepted-reference frame; translation remains zero.
Nonfinite, non-orthogonal, or reflected rotations are rejected. This reuses
existing primary orientation and is not independent angular evidence.

Fixed real Box A/B (`go2_conditional_rotation_hint`): identity reproduces the prior
seed/final matrices exactly. Primary rotation supplies gated correct candidates
in both selected intervals (max accepted error about6cm); synthetic local-yaw
+90deg hints yield no gate-passing eligible candidates. This is a small fixed
sweep, not stateful/live validation or a guarantee for arbitrary bad hints.
GT is used only for labels. No new guard threshold, translation prior, sensor
subscription or history was added. Next step remains conditional node dispatch
and lifecycle integration followed by package build/paced regressions.
