#pragma once

#include <cstddef>

namespace glim_prior_map_localizer
{

struct PriorMapOverlapDecision
{
  std::size_t inliers{0};
  std::size_t samples{0};
  double fraction{0.0};
  bool sufficient{false};
};

inline PriorMapOverlapDecision decidePriorMapOverlap(
  std::size_t inliers,
  std::size_t samples,
  std::size_t minimum_inliers,
  double minimum_fraction)
{
  const double fraction = samples == 0 ? 0.0 :
    static_cast<double>(inliers) / static_cast<double>(samples);
  return PriorMapOverlapDecision{
    inliers,
    samples,
    fraction,
    samples != 0 && inliers >= minimum_inliers && fraction >= minimum_fraction};
}

inline bool isRecoveryLossEvidence(
  const PriorMapOverlapDecision & overlap,
  std::size_t minimum_inliers,
  double loss_fraction)
{
  // A populated scan can lose the map either through a low overlap fraction
  // or because the absolute correspondence count collapses.  The latter is
  // important immediately after a sensor discontinuity: 12/12 is a perfect
  // fraction but is still far below the map-factor observability floor. Empty
  // preprocessing frames carry no geometric evidence and are ignored.
  return overlap.samples != 0 &&
    (overlap.inliers < minimum_inliers || overlap.fraction < loss_fraction);
}

}  // namespace glim_prior_map_localizer
