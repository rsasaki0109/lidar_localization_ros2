#include "lidar_localization/alignment_retry_policy.hpp"

#include <cassert>
#include <limits>

namespace ll = lidar_localization;

ll::RecoveryRetryFromLastPoseParams default_params()
{
  return ll::makeRecoveryRetryFromLastPoseParams(true, 2, 1.0, 5.0);
}

ll::RecoveryRetryFromLastPoseInput default_input()
{
  return ll::makeRecoveryRetryFromLastPoseInput(true, 2, 0.5, 0.7, 4.0);
}

void test_retry_builders_and_fallback_gap()
{
  const auto params = ll::makeRecoveryRetryFromLastPoseParams(true, 3, 2.5, 6.0);
  assert(params.enable);
  assert(params.min_rejections == 3);
  assert(params.max_accepted_gap_sec == 2.5);
  assert(params.max_seed_translation_m == 6.0);

  const auto input = ll::makeRecoveryRetryFromLastPoseInput(true, 4, 1.0, 1.5, 2.0);
  assert(input.have_last_accepted_pose);
  assert(input.consecutive_rejected_updates == 4);
  assert(input.selected_accepted_gap_sec == 1.0);
  assert(input.fallback_accepted_gap_sec == 1.5);
  assert(input.selected_seed_translation_since_accept_m == 2.0);

  assert(ll::computeRecoveryRetryFallbackAcceptedGapSec(true, 12.0, 10.5) == 1.5);
  assert(ll::computeRecoveryRetryFallbackAcceptedGapSec(true, 9.0, 10.5) == 0.0);
  assert(!std::isfinite(ll::computeRecoveryRetryFallbackAcceptedGapSec(false, 12.0, 10.5)));
}

void test_disabled_and_missing_state_block_retry()
{
  auto params = default_params();
  auto input = default_input();

  params.enable = false;
  auto decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(!decision.should_retry);
  assert(decision.reason == "disabled");

  params.enable = true;
  input.have_last_accepted_pose = false;
  decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(!decision.should_retry);
  assert(decision.reason == "no_last_accepted_pose");
}

void test_min_rejections_and_gap_gate()
{
  auto params = default_params();
  auto input = default_input();

  input.consecutive_rejected_updates = 1;
  auto decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(!decision.should_retry);
  assert(decision.reason == "not_enough_rejections");

  input = default_input();
  input.selected_accepted_gap_sec = 1.1;
  decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(!decision.should_retry);
  assert(decision.reason == "accepted_gap_too_large");
}

void test_fallback_gap_and_unavailable_gap()
{
  auto params = default_params();
  auto input = default_input();
  input.selected_accepted_gap_sec = std::numeric_limits<double>::quiet_NaN();

  auto decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(decision.should_retry);
  assert(decision.accepted_gap_sec == input.fallback_accepted_gap_sec);

  input.fallback_accepted_gap_sec = std::numeric_limits<double>::quiet_NaN();
  decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(!decision.should_retry);
  assert(decision.reason == "accepted_gap_unavailable");
}

void test_seed_translation_gate_ignores_nan()
{
  auto params = default_params();
  auto input = default_input();

  input.selected_seed_translation_since_accept_m = 5.1;
  auto decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(!decision.should_retry);
  assert(decision.reason == "seed_translation_too_large");

  input.selected_seed_translation_since_accept_m =
    std::numeric_limits<double>::quiet_NaN();
  decision = ll::decideRecoveryRetryFromLastPose(params, input);
  assert(decision.should_retry);
}

void test_retry_result_requires_target_convergence_and_gate_acceptance()
{
  assert(ll::isRecoveryRetryAlignmentUsable(true, true));
  assert(!ll::isRecoveryRetryAlignmentUsable(false, true));
  assert(!ll::isRecoveryRetryAlignmentUsable(true, false));

  assert(ll::shouldUseRecoveryRetryResult(true, true, false));
  assert(!ll::shouldUseRecoveryRetryResult(true, true, true));
  assert(!ll::shouldUseRecoveryRetryResult(false, true, false));
  assert(!ll::shouldUseRecoveryRetryResult(true, false, false));
}

void test_recovered_gate_decision_clears_rejection_state()
{
  ll::MeasurementGateDecision gate;
  gate.status_level = ll::kMeasurementGateOk;
  gate.status_message = "fitness_score_over_threshold_rejected_seeded";
  gate.reject_measurement = true;
  gate.rejected_seed_update_applied = true;

  const auto recovered_gate = ll::makeRecoveryRetryRecoveredGateDecision(gate);
  assert(recovered_gate.status_level == ll::kMeasurementGateWarn);
  assert(recovered_gate.status_message == "recovery_retry_from_last_pose_recovered");
  assert(!recovered_gate.reject_measurement);
  assert(!recovered_gate.rejected_seed_update_applied);
}

int main()
{
  test_retry_builders_and_fallback_gap();
  test_disabled_and_missing_state_block_retry();
  test_min_rejections_and_gap_gate();
  test_fallback_gap_and_unavailable_gap();
  test_seed_translation_gate_ignores_nan();
  test_retry_result_requires_target_convergence_and_gate_acceptance();
  test_recovered_gate_decision_clears_rejection_state();
  return 0;
}
