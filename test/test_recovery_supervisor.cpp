#include "lidar_localization/recovery_supervisor.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace ll = lidar_localization;

void test_reinitialization_builders_map_fields()
{
  const auto params = ll::makeReinitializationTriggerParams(0.8, 10.0, 20.0, 30.0, 40.0);
  assert(params.threshold == 0.8);
  assert(params.gap_scale_sec == 10.0);
  assert(params.seed_translation_scale_m == 20.0);
  assert(params.reject_streak_scale == 30.0);
  assert(params.fitness_explosion_threshold == 40.0);

  const auto input = ll::makeReinitializationTriggerInput("registration_not_converged",
      1.0, 2.0, 3.0, 4);
  assert(input.status_message == "registration_not_converged");
  assert(input.fitness_score == 1.0);
  assert(input.seed_translation_since_accept_m == 2.0);
  assert(input.accepted_gap_sec == 3.0);
  assert(input.consecutive_rejected_updates == 4);
}

void test_non_failure_does_not_request_reinitialization()
{
  const auto decision = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "ok",
      1.0,
      0.0,
      0.0,
      0});

  assert(!decision.requested);
  assert(decision.reason == "not_failure");
  assert(decision.score == 0.0);
}

void test_target_unavailable_can_request_reinitialization()
{
  const auto decision = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "local_map_crop_too_small",
      std::numeric_limits<double>::quiet_NaN(),
      100.0,
      30.0,
      200});

  assert(decision.requested);
  assert(decision.reason == "target_unavailable_reinit_requested");
  assert(decision.score >= 0.95);
}

void test_fitness_explosion_reason_wins_for_rejected_measurement()
{
  const auto decision = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "fitness_score_over_6_rejected",
      1000.0,
      100.0,
      30.0,
      200});

  assert(decision.requested);
  assert(decision.reason == "fitness_exploded_reinit_requested");
}

void test_sustained_gap_and_streak_alone_requests_reinitialization()
{
  // No fitness spike (fitness stays "merely bad", well under the explosion
  // threshold) and a well-predicted seed (seed_translation_since_accept_m
  // stays small -- e.g. an odom-TF-bridged prediction that never drifts far):
  // gap and streak alone, both fully saturated, must still be enough. This is
  // the "silent death" fix -- without it 0.45*1.0 + 0.20*1.0 = 0.65 < 0.95
  // threshold and the request would never re-assert no matter how long the
  // dropout lasts.
  const auto decision = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "fitness_score_over_6_rejected",
      50.0,     // fitness: bad, but nowhere near fitness_explosion_threshold (1000)
      0.5,      // seed drift: tiny (e.g. a well-predicted odom-bridge seed)
      30.0,     // accepted_gap_sec: exactly at gap_scale_sec -> saturated
      200});    // consecutive_rejected_updates: exactly at reject_streak_scale -> saturated

  assert(decision.requested);
  assert(decision.reason == "sustained_failure_reinit_requested");
  assert(decision.score >= 0.95);
}

void test_gap_or_streak_alone_without_the_other_does_not_force_a_request()
{
  // Gap saturated but streak far from saturated (e.g. many advance-without-
  // measurement ticks reset consecutive_rejected_updates in some other path):
  // must not trigger the new sustained_failure path.
  const auto gap_only = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "fitness_score_over_6_rejected", 50.0, 0.5, 30.0, 10});
  assert(!gap_only.requested);

  // Streak saturated but gap far from saturated.
  const auto streak_only = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "fitness_score_over_6_rejected", 50.0, 0.5, 2.0, 200});
  assert(!streak_only.requested);
}

void test_target_unavailable_reason_still_wins_over_sustained_failure()
{
  // When multiple conditions saturate at once, the more specific/informative
  // reasons still take priority over the generic sustained_failure fallback.
  const auto decision = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "local_map_crop_too_small",
      std::numeric_limits<double>::quiet_NaN(),
      0.5,
      30.0,
      200});
  assert(decision.requested);
  assert(decision.reason == "target_unavailable_reinit_requested");
}

void test_recovery_state_and_action_classification()
{
  const ll::ReinitializationRequestDecision no_request{};
  const ll::ReinitializationRequestDecision request{true, "test", 1.0};

  assert(
    ll::classifyRecoverySupervisorState(0, "ok", no_request, 0) ==
    ll::RecoverySupervisorState::kTracking);
  assert(
    ll::classifyRecoverySupervisorState(1, "ok", no_request, 0) ==
    ll::RecoverySupervisorState::kDegraded);
  assert(
    ll::classifyRecoverySupervisorState(0, "registration_not_converged", no_request, 0) ==
    ll::RecoverySupervisorState::kRecovering);
  assert(
    ll::classifyRecoverySupervisorState(0, "ok", request, 0) ==
    ll::RecoverySupervisorState::kReinitializationRequested);

  assert(
    ll::classifyRecoverySupervisorAction(0, "fitness_score_over_6_rejected", no_request, 0) ==
    "reject_measurement");
  assert(
    ll::classifyRecoverySupervisorState(
      1, "odom_tf_prediction_correction_guard_rejected", no_request, 0) ==
    ll::RecoverySupervisorState::kRecovering);
  assert(
    ll::classifyRecoverySupervisorAction(
      1, "odom_tf_prediction_correction_guard_rejected", no_request, 0) ==
    "reject_measurement");
  assert(
    ll::classifyRecoverySupervisorAction(
      1, "imu_prediction_correction_guard_rejected", no_request, 0) ==
    "reject_measurement");
  assert(
    ll::classifyRecoverySupervisorAction(0, "fitness_score_over_6_rejected_seeded", no_request, 0) ==
    "reuse_rejected_seed_for_prediction");
  assert(
    ll::classifyRecoverySupervisorAction(0, "ok", no_request, 1) ==
    "accept_measurement_after_recovery");
  assert(
    ll::classifyRecoverySupervisorAction(0, "ok", request, 0) ==
    "request_reinitialization");
}

void test_prediction_guard_rejection_counts_as_reinitialization_failure()
{
  const auto decision = ll::computeReinitializationRequest(
    ll::ReinitializationTriggerParams{},
    ll::ReinitializationTriggerInput{
      "odom_tf_prediction_correction_guard_rejected", 0.1, 0.5, 2.0, 1});

  assert(!decision.requested);
  assert(decision.reason == "reinit_not_requested");
  assert(decision.score > 0.0);
}

void test_recovery_supervisor_evaluator_pairs_state_and_action()
{
  const ll::ReinitializationRequestDecision no_request{};
  const ll::ReinitializationRequestDecision request{true, "test", 1.0};

  auto evaluation = ll::evaluateRecoverySupervisor(
    0, "fitness_score_over_6_rejected", no_request, 2);
  assert(evaluation.state == ll::RecoverySupervisorState::kRecovering);
  assert(evaluation.action == "reject_measurement");

  evaluation = ll::evaluateRecoverySupervisor(0, "ok", request, 0);
  assert(evaluation.state == ll::RecoverySupervisorState::kReinitializationRequested);
  assert(evaluation.action == "request_reinitialization");
}

void test_state_names_are_stable_diagnostic_contract()
{
  assert(std::string(ll::recoverySupervisorStateName(ll::RecoverySupervisorState::kTracking)) ==
    "tracking");
  assert(std::string(ll::recoverySupervisorStateName(ll::RecoverySupervisorState::kDegraded)) ==
    "degraded");
  assert(std::string(ll::recoverySupervisorStateName(ll::RecoverySupervisorState::kRecovering)) ==
    "recovering");
  assert(
    std::string(
      ll::recoverySupervisorStateName(ll::RecoverySupervisorState::kReinitializationRequested)) ==
    "reinitialization_requested");
}

int main()
{
  test_reinitialization_builders_map_fields();
  test_non_failure_does_not_request_reinitialization();
  test_target_unavailable_can_request_reinitialization();
  test_fitness_explosion_reason_wins_for_rejected_measurement();
  test_sustained_gap_and_streak_alone_requests_reinitialization();
  test_gap_or_streak_alone_without_the_other_does_not_force_a_request();
  test_target_unavailable_reason_still_wins_over_sustained_failure();
  test_recovery_state_and_action_classification();
  test_prediction_guard_rejection_counts_as_reinitialization_failure();
  test_recovery_supervisor_evaluator_pairs_state_and_action();
  test_state_names_are_stable_diagnostic_contract();
  return 0;
}
