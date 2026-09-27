# Conditional accepted-scan recovery experiment

This isolated branch wires recovery behind the default-off
`enable_conditional_scan_recovery` parameter. No production preset enables it.
It is not validated for promotion: bad motion seeds can still pass map gates,
and previous always-on motion/GICP variants failed full-rate replay.

## Current node path

Apply `override.yaml` after the fixed Go2 parameters to test the candidate.
Normal prediction, NDT and the existing last-pose retry run first. After a
remaining failure, existing retry bounds (enable/count/gap/seed distance) decide
whether to generate a candidate. The one-second Go2 gap cap is an inherited
limit, not a demonstrated safety bound. Primary orientation initializes relative
GICP; primary translation never enters that initialization.

`AcceptedScanSeed` retains only the last backend-accepted prepared cloud and its
map pose/stamp/generation. Healthy acceptance retains references without running
GICP. Rejected estimates never move the anchor or extend its horizon. Reference
GICP uses .5 m correspondence, 20 covariance neighbors, 5e-4 transform epsilon
and 30 iterations; target covariance can be reused until the anchor changes.

The candidate is refined against the map with `MapRefiner` (GICP_OMP, 2 m
correspondence, 20 neighbors, .01 transform epsilon, 30 iterations). Its map
voxel size follows the node voxel parameter (.2 m in the tested Go2 preset).
The target comes from the existing crop/target selection. A changed target
pointer creates a new cache. Cache construction and registration occur only on
the failed path; first-use cost is included in its alignment timer. Candidate
seed time is logged separately. No scan-motion pose is published directly.

The ordinary measurement gates and pose backend still decide acceptance. Only
when the gate passes and the backend advances its accepted count at this scan's
stamp does the prepared cloud become the new reference. PCL range filtering
strips frame metadata, so the opt-in path restores the known base-frame name.

## State and diagnostics

Cloud callbacks serialize provider/refiner access. Private local ownership keeps
in-flight objects alive while the state lock is released. Initialpose resets
both live caches; shutdown/generation checks prevent stale results from being
stored or applied. Map refinement also checks target pointer identity after
alignment. Accepted map messages, deactivation and cleanup clear both caches.
Actual concurrent reset/map-change replay remains required before promotion.

`CONDITIONAL_RECOVERY` logs distinguish dispatch, eligibility skips, provider
validity/time, map alignment and `map_gicp` gate outcomes. Successful candidates
use `conditional_scan_recovery_recovered`; prediction-source labels and the
configured primary method continue to describe the normal pipeline. Trace
replay has logging overhead and is not timing-equivalent to a trace-free run.

## Evidence and limits

Synthetic helper tests cover transform direction, immutable accepted anchors,
gap/generation/frame checks, rotation-hint validity, invalid map/source/seed,
cache reuse and replacement-map independence. Release assertions stay enabled.
`MapRefiner` reproduces all32 prior fixed GICP2m final matrices, fitness and
convergence exactly, including poor seeds. This proves extraction parity only.

The earlier NDT-refinement node completed eight normal/fault Box/Mask2 runs but
adopted zero conditional candidates. Its diagnostic run generated11 seeds; NDT
moved10 to wrong basins and rejected the remaining correct final for excess
correction. The fixed map-GICP2m comparison passed5/6 primary-rotation candidates
and rejected all corrupt-yaw90 candidates, but all6 artificial +2m translation
seeds passed the numeric limits at wrong positions. Those are fixed
counterexamples, not observed live outputs or a reason to relax gates.

The current map-GICP node needs package validation, real normal/fault and input
drop replay, and in-flight reset testing. No dependency or guard threshold was
added. The helper and its test remain intentionally discardable experiments.
