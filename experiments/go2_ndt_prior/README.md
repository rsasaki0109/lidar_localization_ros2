# Go2 NDT translation prior experiment

`ndt_twist_prior_weight` defaults to zero (existing behavior). Positive values
enable an isotropic translation penalty around the twist-predicted seed, scaled
by the number of source points. Valid parameter range is finite `[0, 100]`.
The penalty uses the existing ndt_omp implementation; it adds no optimizer.
It starts after the first accepted map match and only for a twist-selected seed.

In this experimental mode, twist prediction requires a matching base frame,
finite velocity/covariance, a positive timestamp no older than 250 ms or more
than 100 ms ahead of the scan, and positive covariance diagonals at most 0.25
(m/s)^2 or (rad/s)^2. Ineligible twist falls back to the existing prediction
priority chain. The registration seed diagnostic shows that fallback; the
configured prior weight is logged, included in alignment diagnostics and
available through ROS parameter dump.
These are admission heuristics, not proof of a correct or nonslipping velocity.
Coherent foot slip with falsely confident covariance can still bias the prior.

The fixed-scan pilot (17 EIL_Box scans, previous accepted estimate plus leg
twist as seed; GT for scoring only) reduced errors above 0.5 m from 10 to 0 at
weight 1.0. This is not closed-loop evidence. Keep the feature experimental
until all indoor LOO sequences, repetitions and fault-injected twist streams
are evaluated. Do not enable geometry-only localizability guards during this
pilot: the NDT Hessian includes the added prior information.

Merge `params.yaml` over `jeplo_localization_leg.yaml` to create a full replay
configuration. Use the frozen replay tools with a fresh output directory and
the experimental workspace as `LIDARLOC_WS`. Preserve guard/recovery settings
for the first comparison, then measure whether rejection gaps decrease.

The experimental node keeps the stored prediction at its timestamp in both
twist and previous-delta modes, extrapolating the latter at seed selection.
This prevents counting an interval twice when twist becomes available after a
fallback. A transition regression covers acceptance and rejection around a
dropout; default-off trajectory parity must also be checked.

`inject_twist_fault.py` copies a single-file SQLite bag to a fresh directory
and changes only `/leg_twist` over an explicit storage-time interval. Modes
are dropout, two-second stale stamps, NaN velocity, +1 m/s x velocity with
high covariance, and the same bias with the original confident covariance.
The original input hash and fault window are recorded in `fault_manifest.json`.
The last mode measures the limit of the admission heuristic, not a fault it
is expected to detect. These are synthetic faults in recorded data.
