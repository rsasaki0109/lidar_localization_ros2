#include "glim_prior_map_localizer/prior_map_factor_policy.hpp"

#include <cassert>
#include <limits>

namespace gp = glim_prior_map_localizer;

void test_rejects_invalid_and_non_finite_solutions()
{
  const gp::PriorMapFactorGateParams params;
  auto input = gp::PriorMapFactorGateInput{};
  auto decision = gp::decidePriorMapFactor(params, input);
  assert(!decision.accepted);
  assert(decision.reason == "solution_invalid");

  input.solution_valid = true;
  input.transform_finite = false;
  decision = gp::decidePriorMapFactor(params, input);
  assert(!decision.accepted);
  assert(decision.reason == "transform_non_finite");
}

void test_rejects_weak_correspondence_support()
{
  const gp::PriorMapFactorGateParams params;
  gp::PriorMapFactorGateInput input;
  input.solution_valid = true;
  input.transform_finite = true;
  input.final_inliers = params.min_final_inliers - 1;

  const auto decision = gp::decidePriorMapFactor(params, input);
  assert(!decision.accepted);
  assert(decision.reason == "insufficient_final_inliers");
}

void test_accepts_a_bounded_local_map_correction()
{
  const gp::PriorMapFactorGateParams params;
  gp::PriorMapFactorGateInput input;
  input.solution_valid = true;
  input.transform_finite = true;
  input.final_inliers = 50;
  input.local_correction_translation_m = 1.2;
  input.local_correction_rotation_deg = 4.0;

  const auto decision = gp::decidePriorMapFactor(params, input);
  assert(decision.accepted);
  assert(!decision.accepted_as_global_relocalization);
  assert(decision.reason == "local_correction_consistent");
}

void test_global_relocalization_requires_two_consistent_anchors()
{
  const gp::PriorMapFactorGateParams params;
  gp::PriorMapFactorGateInput input;
  input.solution_valid = true;
  input.transform_finite = true;
  input.final_inliers = 50;
  input.local_correction_translation_m = 30.0;
  input.local_correction_rotation_deg = 90.0;

  auto decision = gp::decidePriorMapFactor(params, input);
  assert(!decision.accepted);
  assert(decision.reason == "global_consensus_pending");

  input.previous_global_anchor_available = true;
  input.global_anchor_translation_disagreement_m = 0.8;
  input.global_anchor_rotation_disagreement_deg = 3.0;
  decision = gp::decidePriorMapFactor(params, input);
  assert(decision.accepted);
  assert(decision.accepted_as_global_relocalization);
  assert(decision.reason == "global_consensus_accepted");

  input.global_anchor_translation_disagreement_m = 4.0;
  decision = gp::decidePriorMapFactor(params, input);
  assert(!decision.accepted);
  assert(decision.reason == "global_consensus_rejected");
}

int main()
{
  test_rejects_invalid_and_non_finite_solutions();
  test_rejects_weak_correspondence_support();
  test_accepts_a_bounded_local_map_correction();
  test_global_relocalization_requires_two_consistent_anchors();
  return 0;
}
