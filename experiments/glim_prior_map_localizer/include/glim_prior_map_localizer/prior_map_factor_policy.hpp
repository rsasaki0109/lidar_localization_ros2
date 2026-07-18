#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

namespace glim_prior_map_localizer
{

struct PriorMapFactorGateParams
{
  std::size_t min_final_inliers{20};
  double max_local_correction_translation_m{5.0};
  double max_local_correction_rotation_deg{10.0};
  bool allow_global_relocalization{true};
  double max_global_anchor_translation_disagreement_m{2.0};
  double max_global_anchor_rotation_disagreement_deg{10.0};
};

struct PriorMapFactorGateInput
{
  bool solution_valid{false};
  bool transform_finite{false};
  std::size_t final_inliers{0};
  double local_correction_translation_m{0.0};
  double local_correction_rotation_deg{0.0};
  bool previous_global_anchor_available{false};
  double global_anchor_translation_disagreement_m{0.0};
  double global_anchor_rotation_disagreement_deg{0.0};
};

struct PriorMapFactorGateDecision
{
  bool accepted{false};
  bool accepted_as_global_relocalization{false};
  std::string reason{"solution_invalid"};
};

inline PriorMapFactorGateDecision decidePriorMapFactor(
  const PriorMapFactorGateParams & params,
  const PriorMapFactorGateInput & input)
{
  if (!input.solution_valid) {
    return {false, false, "solution_invalid"};
  }
  if (!input.transform_finite) {
    return {false, false, "transform_non_finite"};
  }
  if (input.final_inliers < params.min_final_inliers) {
    return {false, false, "insufficient_final_inliers"};
  }

  const bool local_consistent =
    std::isfinite(input.local_correction_translation_m) &&
    std::isfinite(input.local_correction_rotation_deg) &&
    input.local_correction_translation_m <=
    params.max_local_correction_translation_m &&
    std::abs(input.local_correction_rotation_deg) <=
    params.max_local_correction_rotation_deg;
  if (local_consistent) {
    return {true, false, "local_correction_consistent"};
  }

  if (!params.allow_global_relocalization) {
    return {false, false, "local_correction_exceeded"};
  }
  if (!input.previous_global_anchor_available) {
    return {false, false, "global_consensus_pending"};
  }

  const bool global_consistent =
    std::isfinite(input.global_anchor_translation_disagreement_m) &&
    std::isfinite(input.global_anchor_rotation_disagreement_deg) &&
    input.global_anchor_translation_disagreement_m <=
    params.max_global_anchor_translation_disagreement_m &&
    std::abs(input.global_anchor_rotation_disagreement_deg) <=
    params.max_global_anchor_rotation_disagreement_deg;
  if (!global_consistent) {
    return {false, false, "global_consensus_rejected"};
  }
  return {true, true, "global_consensus_accepted"};
}

}  // namespace glim_prior_map_localizer
