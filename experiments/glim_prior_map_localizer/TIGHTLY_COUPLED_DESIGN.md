# Tightly Coupled GLIL Design and Acceptance Contract

This document is the implementation contract for combining the core methods from:

- Koide et al., *Tightly Coupled Range Inertial Localization on a 3D Prior Map
  Based on Sliding Window Factor Graph Optimization*, ICRA 2024; and
- Koide et al., *Tightly Coupled Range Inertial Odometry and Mapping with Exact
  Point Cloud Downsampling*, ICRA 2025.

The published ICRA 2024 localization implementation is closed source. This project
therefore implements the paper formulation from the published equations and validates
observable behavior rather than claiming source equivalence.

## Required estimator structure

For every active frame `i`, the fixed-lag graph owns pose `X(i)`, velocity `V(i)`, and
IMU bias `B(i)`. A frame may leave the graph only through the fixed-lag smoother's
marginalization. With `W` denoting the active window, the estimator minimizes one
objective containing all of the following:

1. preintegrated IMU factors between consecutive states;
2. binary scan-to-scan GICP factors from the newest frame to the preceding three
   active frames, plus selected overlapping keyframes;
3. unary scan-to-prior-map GICP factors for active frames with sufficient map overlap;
4. the marginal factors produced when old states leave the five-second window.

The scan-to-map result must not first be solved as an independent pose and converted to
a `map -> odom` observation. It must constrain the same `X(i)` optimized by the IMU and
scan-to-scan factors in the same smoother update. This is the boundary between the
target tightly coupled estimator and the previous GLIL-style split.

The prior map is the fixed target at the identity pose in the estimator's map frame.
The initial map pose seeds the graph's first `X(0)`. When the sensor has insufficient
map overlap, no map factor is inserted; scan-to-scan and IMU factors continue range
inertial odometry. Map factors resume when overlap returns, so past states still inside
the window participate in the correction.

## Exact point-cloud downsampling

Both binary scan-to-scan and unary scan-to-map GICP factors use the same exact-coreset
linearization implementation. Each point contributes the independent entries of
`H`, `b`, and `c`; a weighted Caratheodory subset must reproduce their full-set sum at
the sampling point to floating-point tolerance.

The factor follows the ICRA 2025 deferred-sampling state machine:

1. a full linearization caches per-point residual/Jacobian contributions;
2. if the next estimate is close to the previous linearization point, extract a coreset
   from that cache and evaluate it at the new estimate;
3. reuse the coreset while the estimate remains close to its sampling point;
4. otherwise perform another full linearization and refresh the cache.

Correspondences are exact nearest-neighbor queries. The coreset accelerates
linearization only; nonlinear error evaluation used for optimizer acceptance remains a
full-set calculation. Both unary and binary factors use the 28 independent entries of
the six-DoF relative-pose quadratic. For a binary factor, the target Jacobian is the
source Jacobian multiplied by one common adjoint at the sampling point, so preserving
the relative `H/b/c` also preserves every 12-DoF Hessian block exactly. Unary and binary
paths still require separate algebra tests.

## Initialization and recovery

The first implementation accepts a gravity-aligned 4-DoF seed and initializes the
graph directly in the map frame. It must also preserve the paper's initialization path:

1. estimate gravity, velocity, and IMU bias from a short point-cloud/IMU batch;
2. reduce global registration to 3- or 4-DoF using gravity (and optional sensor height);
3. initialize `X(0)`, `V(0)`, and `B(0)` from the accepted result.

Kidnap recovery is an additional project requirement. It uses an independently
verified global candidate and multi-frame consensus. Recovery must re-anchor the
active map-frame state without resetting raw continuous odometry, creating a second TF
parent, or publishing an unbounded pose discontinuity. The last valid public transform
remains available while a candidate is pending or rejected.

## ROS and TF invariants

- Exactly one authority publishes each dynamic TF edge.
- Raw continuous odometry remains available as `odom -> base_link`.
- Map corrections are exposed through one `map -> odom` edge or a directly equivalent
  single-parent chain; never both.
- Timestamps are monotonic and transforms are re-stamped while localization evidence
  is absent.
- NaN, optimizer fallback, rejected global candidates, and map loss cannot silently
  reset the public pose.

## Evidence required for acceptance

### Algebra and unit tests

- Exact equality of full-set and coreset `H/b/c` at the sampling point for unary and
  binary factors, including invalid correspondences and minimum coreset size.
- Deferred-sampling state transitions: full, deferred extraction, reuse, invalidation,
  and refresh.
- Prior-map factor insertion uses the current `X(i)` and the same pending nonlinear
  factor graph as IMU and scan-to-scan constraints.
- Map-disabled behavior is equivalent to tightly coupled range-inertial odometry.
- Overlap loss removes only map factors; it does not stop output or remove motion
  factors.
- Re-entry and recovery gates reject inconsistent candidates and never create a TF
  double parent.

### Runtime measurements

Run complete `outdoor_hard_01a`, `01b`, `02a`, and `02b` bags with a fixed image,
configuration, initialization policy, and deterministic thread profile. Record:

- planar and non-planar translation ATE, rotation ATE, final error, and 10 m RPE;
- output and map-factor coverage, maximum output gap, TF jumps, resets, and optimizer
  fallbacks;
- end-to-end processing p50/p95/max, queue depth, CPU utilization, and peak RSS;
- repeat-to-repeat raw and public trajectory hashes or numeric tolerances.

Then run the available indoor and outdoor kidnap sequences, including an explicit
injected-pose displacement test. A recovery passes only when the correct map basin is
confirmed, output remains continuous under the configured transition bound, and no
false recovery is accepted.

### Publication

The GLIL name may be used without a qualifier only after the factor-insertion evidence
and full-sequence gates above pass. Until then, user-facing material must say
`GLIL-style` or `GLIL-inspired`. The final publication includes pinned fork revisions,
container provenance, exact commands, recorded metrics, and regenerated GIFs.
