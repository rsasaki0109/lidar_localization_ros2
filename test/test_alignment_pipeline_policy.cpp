#include "lidar_localization/alignment_pipeline_policy.hpp"

#include <cassert>
#include <limits>
#include <string>

namespace ll = lidar_localization;

ll::AlignmentAttempt make_attempt(
  bool target_ready,
  bool has_converged,
  double fitness_score = 1.0)
{
  ll::AlignmentAttempt attempt;
  attempt.target_ready = target_ready;
  attempt.has_converged = has_converged;
  attempt.alignment_time_sec = 0.25;
  attempt.fitness_score = fitness_score;
  attempt.seed_translation_since_accept_m = 0.5;
  attempt.seed_yaw_since_accept_deg = 1.0;
  attempt.accepted_gap_sec = 0.4;
  if (has_converged) {
    attempt.final_transformation(0, 3) = static_cast<float>(fitness_score);
    attempt.correction_translation_m = 0.1;
    attempt.correction_yaw_deg = 0.2;
  }
  return attempt;
}

ll::AlignmentPipelineInput default_input()
{
  return {
    true,
    2,
    12.0,
    10.0,
    ll::makeRecoveryRetryFromLastPoseParams(true, 2, 5.0, 10.0)};
}

ll::MeasurementGateDecision accepted_gate()
{
  ll::MeasurementGateDecision gate;
  gate.status_level = ll::kMeasurementGateOk;
  gate.status_message = "ok";
  gate.reject_measurement = false;
  return gate;
}

ll::MeasurementGateDecision rejected_gate()
{
  ll::MeasurementGateDecision gate;
  gate.status_level = ll::kMeasurementGateWarn;
  gate.status_message = "fitness_score_over_threshold_rejected";
  gate.reject_measurement = true;
  return gate;
}

void test_target_missing_terminal_failure_when_retry_unavailable()
{
  auto input = default_input();
  input.have_last_accepted_pose = false;
  int retry_calls = 0;
  int gate_calls = 0;

  const auto result = ll::runAlignmentPipeline(
    make_attempt(false, false, std::numeric_limits<double>::quiet_NaN()),
    input,
    [&]() {
      ++retry_calls;
      return make_attempt(true, true);
    },
    [&](const ll::AlignmentAttempt &) {
      ++gate_calls;
      return accepted_gate();
    });

  assert(!result.should_continue);
  assert(result.should_advance_prediction_without_measurement);
  assert(result.status_level == ll::kAlignmentPipelineError);
  assert(result.status_message == "local_map_crop_too_small");
  assert(retry_calls == 0);
  assert(gate_calls == 0);
}

void test_not_converged_terminal_failure_when_retry_disabled()
{
  auto input = default_input();
  input.recovery_retry_params.enable = false;
  int retry_calls = 0;

  const auto result = ll::runAlignmentPipeline(
    make_attempt(true, false, 3.0),
    input,
    [&]() {
      ++retry_calls;
      return make_attempt(true, true);
    },
    [](const ll::AlignmentAttempt &) {
      return accepted_gate();
    });

  assert(!result.should_continue);
  assert(result.status_message == "registration_not_converged");
  assert(result.selected_attempt.target_ready);
  assert(!result.selected_attempt.has_converged);
  assert(retry_calls == 0);
}

void test_fitness_reject_remains_rejected_when_retry_denied()
{
  auto input = default_input();
  input.recovery_retry_params.min_rejections = 3;
  int retry_calls = 0;
  int gate_calls = 0;

  const auto result = ll::runAlignmentPipeline(
    make_attempt(true, true, 9.0),
    input,
    [&]() {
      ++retry_calls;
      return make_attempt(true, true, 1.0);
    },
    [&](const ll::AlignmentAttempt &) {
      ++gate_calls;
      return rejected_gate();
    });

  assert(result.should_continue);
  assert(!result.recovered_by_retry_from_last_pose);
  assert(result.gate_result.reject_measurement);
  assert(result.status_message == "fitness_score_over_threshold_rejected");
  assert(retry_calls == 0);
  assert(gate_calls == 1);
}

void test_retry_success_replaces_rejected_primary_attempt()
{
  int retry_calls = 0;
  int gate_calls = 0;

  const auto result = ll::runAlignmentPipeline(
    make_attempt(true, true, 9.0),
    default_input(),
    [&]() {
      ++retry_calls;
      return make_attempt(true, true, 1.0);
    },
    [&](const ll::AlignmentAttempt & attempt) {
      ++gate_calls;
      return attempt.fitness_score > 5.0 ? rejected_gate() : accepted_gate();
    });

  assert(result.should_continue);
  assert(result.recovered_by_retry_from_last_pose);
  assert(result.selected_attempt.fitness_score == 1.0);
  assert(!result.gate_result.reject_measurement);
  assert(result.gate_result.status_level == ll::kMeasurementGateWarn);
  assert(result.gate_result.status_message == "recovery_retry_from_last_pose_recovered");
  assert(retry_calls == 1);
  assert(gate_calls == 2);
}

void test_target_missing_can_recover_by_retry()
{
  int retry_calls = 0;
  int gate_calls = 0;

  const auto result = ll::runAlignmentPipeline(
    make_attempt(false, false, std::numeric_limits<double>::quiet_NaN()),
    default_input(),
    [&]() {
      ++retry_calls;
      return make_attempt(true, true, 1.5);
    },
    [&](const ll::AlignmentAttempt &) {
      ++gate_calls;
      return accepted_gate();
    });

  assert(result.should_continue);
  assert(result.recovered_by_retry_from_last_pose);
  assert(result.selected_attempt.target_ready);
  assert(result.selected_attempt.has_converged);
  assert(result.status_message == "recovery_retry_from_last_pose_recovered");
  assert(retry_calls == 1);
  assert(gate_calls == 1);
}

void test_terminal_handling_publishes_and_advances()
{
  ll::AlignmentPipelineResult result =
    ll::makeTerminalAlignmentPipelineResult(
    make_attempt(true, false, 3.0), "registration_not_converged");

  const auto handling = ll::decideAlignmentPipelineHandling(result);

  assert(!handling.log_recovery_retry_success);
  assert(handling.publish_terminal_status);
  assert(handling.warn_registration_not_converged);
  assert(handling.advance_prediction_without_measurement);
  assert(!handling.continue_to_backend);
}

void test_continuing_recovery_handling_logs_and_uses_backend()
{
  ll::AlignmentPipelineResult result;
  result.should_continue = true;
  result.recovered_by_retry_from_last_pose = true;
  result.status_message = "recovery_retry_from_last_pose_recovered";

  const auto handling = ll::decideAlignmentPipelineHandling(result);

  assert(handling.log_recovery_retry_success);
  assert(!handling.publish_terminal_status);
  assert(!handling.warn_registration_not_converged);
  assert(!handling.advance_prediction_without_measurement);
  assert(handling.continue_to_backend);
}

int main()
{
  test_target_missing_terminal_failure_when_retry_unavailable();
  test_not_converged_terminal_failure_when_retry_disabled();
  test_fitness_reject_remains_rejected_when_retry_denied();
  test_retry_success_replaces_rejected_primary_attempt();
  test_target_missing_can_recover_by_retry();
  test_terminal_handling_publishes_and_advances();
  test_continuing_recovery_handling_logs_and_uses_backend();
  return 0;
}
