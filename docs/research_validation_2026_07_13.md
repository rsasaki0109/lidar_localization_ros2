# Localization research validation — 2026-07-13

This note records the adoption decisions behind the July 2026 correctness and
research pass. Raw replay output remains under `/tmp`; compact reproducible
results live beside each experiment as `validation_2026_07_13.json`.

## Adopted correctness changes

- Link the component and Eigen-using tests through `Eigen3::Eigen`; this directly
  addresses the missing transitive include/link contract seen in Humble CI.
- Serialize state shared between the default callback group and `/initialpose`,
  let only the expensive registration backend run outside that lock, and discard
  its result if the initial-pose generation changed meanwhile.
- Save the bias used to integrate every IMU factor and apply the standard
  first-order a-posteriori bias correction before evaluating old factors. The
  rotation-only deskew history now subtracts the current gyro bias as well.

The IMU correction follows the bias-Jacobian approach described by
[Forster et al.](https://dellaert.github.io/files/Forster16tro.pdf). It fixes an
internal inconsistency, but it does not justify enabling IMU or deskew by default:
all candidates failed the three-repeat Koide safety gate.

## BBS top-K verifier decision

[KISS-Matcher](https://arxiv.org/abs/2409.15615) combines Faster-PFH features,
graph-based outlier pruning, and a robust pose solver. We compared its global
estimate against the same BBS top-5 set used by the existing bounded NDT scorer.
Across three Koide queries, top-5 oracle coverage was 2/3. BBS-first produced two
false accepts; bounded NDT recovered both covered cases with no false accepts;
KISS safely abstained on all three but took a median 9.06 s at 1 m and 2.58 s at
2 m. The 0.5 m run exceeded three minutes and about 2.5 GiB RSS. Bounded NDT
therefore remains the verifier; KISS stays an optional offline experiment.

## Correspondence-direction decision

[X-ICP](https://arxiv.org/abs/2211.16335) motivates measuring how individual
point-to-plane correspondences contribute along Hessian eigendirections.
[LP-ICP](https://arxiv.org/abs/2501.02580) extends this idea to point-to-line and
point-to-plane features. Our controlled approximation correctly exposed three
null directions for one plane and recovered full rank from three-axis line
features. On the three Koide scans, however, adding line contributions did not
change numerical nullity. More importantly, the existing NDT-Hessian signal
preceded real replay failure in 0/3 repeats. These diagnostics remain offline and
must not gate runtime pose updates yet.

## Validation boundary

Jazzy builds and tests are available locally; Humble is not installed, so the
Humble CI fix is verified by its explicit imported-target dependency and must be
confirmed by GitHub CI. The real-bag study used the available public Koide
sequence. Synthetic geometry and KISS's deterministic-transform smoke fixture
provide cross-domain regression, but a second independent public rosbag was not
available locally.
