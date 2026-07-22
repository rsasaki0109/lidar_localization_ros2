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
  std::size_t minimum_samples,
  double loss_fraction)
{
  return overlap.samples >= minimum_samples && overlap.fraction < loss_fraction;
}

}  // namespace glim_prior_map_localizer
