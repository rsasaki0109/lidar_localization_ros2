#include "lidar_localization/alignment_status_policy.hpp"

#include <cassert>
#include <cmath>
#include <string>

namespace ll = lidar_localization;

ll::AlignmentStatusInput default_status_input()
{
  ll::AlignmentStatusInput input;
  input.registration_method = "NDT";
  input.level = ll::kMeasurementGateWarn;
  input.message = "fitness_score_over_threshold_rejected";
  input.has_converged = true;
  input.fitness_score = 6.0;
  input.alignment_time_sec = 0.05;
  input.filtered_point_count = 1200;
  input.correction_translation_m = 0.2;
  input.correction_yaw_deg = 0.3;
  input.seed_translation_since_accept_m = 4.0;
  input.seed_yaw_since_accept_deg = 2.0;
  input.accepted_gap_sec = 8.0;
  input.consecutive_rejected_updates = 5;
  input.have_last_accepted_pose = true;
  input.stamp_sec = 20.0;
  input.last_accepted_pose_time_sec = 12.0;
  input.fallback_seed_translation_since_accept_m = 9.0;
  input.imu_prediction_active = true;
  input.map_received = true;
  input.initialpose_received = true;
  input.gate_params.score_threshold = 7.0;
  input.gate_params.enable_post_reject_strict_score_threshold = true;
  input.gate_params.post_reject_strict_min_rejections = 5;
  input.gate_params.post_reject_strict_score_threshold = 5.0;
  input.gate_params.enable_borderline_seed_rejection_gate = true;
  input.gate_params.borderline_seed_gate_score_threshold = 4.5;
  input.gate_params.borderline_seed_gate_min_seed_translation_m = 1.0;
  input.reinitialization_params = ll::makeReinitializationTriggerParams(
    0.5, 8.0, 4.0, 5.0, 1000.0);
  return input;
}

void test_prepare_alignment_status_computes_gate_and_reinit()
{
  const auto input = default_status_input();
  const auto preparation = ll::prepareAlignmentStatus(input);

  assert(preparation.threshold_decision.post_reject_strict_active);
  assert(preparation.threshold_decision.effective_score_threshold == 5.0);
  assert(!preparation.borderline_seed_gate_active);
  assert(preparation.effective_reinitialization_metrics.accepted_gap_sec == 8.0);
  assert(preparation.effective_reinitialization_metrics.seed_translation_since_accept_m == 4.0);
  assert(preparation.reinitialization_request.requested);
  assert(preparation.reinitialization_request.reason == "accepted_gap_reinit_requested");
}

void test_make_alignment_status_input_combines_observation_and_context()
{
  ll::AlignmentStatusObservation observation;
  observation.level = ll::kMeasurementGateWarn;
  observation.message = "registration_not_converged";
  observation.has_converged = false;
  observation.fitness_score = 9.0;
  observation.alignment_time_sec = 0.2;
  observation.filtered_point_count = 345;
  observation.correction_translation_m = 1.2;
  observation.correction_yaw_deg = 3.4;
  observation.seed_translation_since_accept_m = 5.6;
  observation.seed_yaw_since_accept_deg = 7.8;
  observation.accepted_gap_sec = 9.1;
  observation.imu_prediction_active = true;

  ll::AlignmentStatusRuntimeContext context;
  context.registration_method = "GICP";
  context.consecutive_rejected_updates = 12;
  context.have_last_accepted_pose = true;
  context.stamp_sec = 44.0;
  context.last_accepted_pose_time_sec = 40.0;
  context.fallback_seed_translation_since_accept_m = 2.5;
  context.map_received = true;
  context.initialpose_received = false;
  context.gate_params.score_threshold = 4.0;
  context.reinitialization_params.threshold = 0.8;

  const auto input = ll::makeAlignmentStatusInput(observation, context);

  assert(input.registration_method == "GICP");
  assert(input.level == ll::kMeasurementGateWarn);
  assert(input.message == "registration_not_converged");
  assert(!input.has_converged);
  assert(input.fitness_score == 9.0);
  assert(input.alignment_time_sec == 0.2);
  assert(input.filtered_point_count == 345);
  assert(input.correction_translation_m == 1.2);
  assert(input.correction_yaw_deg == 3.4);
  assert(input.seed_translation_since_accept_m == 5.6);
  assert(input.seed_yaw_since_accept_deg == 7.8);
  assert(input.accepted_gap_sec == 9.1);
  assert(input.consecutive_rejected_updates == 12);
  assert(input.have_last_accepted_pose);
  assert(input.stamp_sec == 44.0);
  assert(input.last_accepted_pose_time_sec == 40.0);
  assert(input.fallback_seed_translation_since_accept_m == 2.5);
  assert(input.imu_prediction_active);
  assert(input.map_received);
  assert(!input.initialpose_received);
  assert(input.gate_params.score_threshold == 4.0);
  assert(input.reinitialization_params.threshold == 0.8);
}

void test_prepare_alignment_status_uses_fallback_reinit_metrics()
{
  auto input = default_status_input();
  input.accepted_gap_sec = NAN;
  input.seed_translation_since_accept_m = NAN;
  input.message = "registration_not_converged";

  const auto preparation = ll::prepareAlignmentStatus(input);

  assert(preparation.effective_reinitialization_metrics.accepted_gap_sec == 8.0);
  assert(preparation.effective_reinitialization_metrics.seed_translation_since_accept_m == 9.0);
  assert(preparation.reinitialization_request.requested);
}

void test_recovery_evaluation_uses_latched_reinit_request()
{
  const auto input = default_status_input();
  const ll::ReinitializationRequestDecision latched_request{
    true, "latched_request", 1.0};

  const auto recovery = ll::evaluateAlignmentStatusRecovery(input, latched_request);

  assert(recovery.state == ll::RecoverySupervisorState::kReinitializationRequested);
  assert(recovery.action == "request_reinitialization");
}

void test_make_diagnostic_input_preserves_status_context()
{
  const auto input = default_status_input();
  const auto preparation = ll::prepareAlignmentStatus(input);
  const ll::ReinitializationRequestDecision reinit_request{
    true, "latched_request", 0.75};
  const auto diagnostic_input = ll::makeAlignmentStatusDiagnosticValuesInput(
    ll::AlignmentStatusDiagnosticInput{
      input,
      preparation,
      reinit_request,
      "reinitialization_requested",
      "request_reinitialization",
      3.0,
      2,
      true,
      4.0});

  assert(diagnostic_input.registration_method == "NDT");
  assert(diagnostic_input.has_converged);
  assert(diagnostic_input.effective_score_threshold == 5.0);
  assert(diagnostic_input.recovery_state == "reinitialization_requested");
  assert(diagnostic_input.recovery_action == "request_reinitialization");
  assert(diagnostic_input.reinitialization_requested);
  assert(diagnostic_input.reinitialization_request_reason == "latched_request");
  assert(diagnostic_input.reinitialization_request_score == 0.75);
  assert(diagnostic_input.reinitialization_request_latched);
  assert(diagnostic_input.map_received);
  assert(diagnostic_input.initialpose_received);
}

void test_make_diagnostic_input_computes_runtime_ages()
{
  auto input = default_status_input();
  input.stamp_sec = 25.0;
  const auto preparation = ll::prepareAlignmentStatus(input);
  const ll::ReinitializationRequestDecision reinit_request{
    true, "latched_request", 0.75};

  const auto diagnostic_input = ll::makeAlignmentStatusDiagnosticInput(
    input,
    preparation,
    reinit_request,
    ll::AlignmentStatusDiagnosticRuntimeContext{
      "recovering",
      "reject_measurement",
      20.0,
      6,
      true,
      21.5});

  assert(diagnostic_input.recovery_state == "recovering");
  assert(diagnostic_input.recovery_action == "reject_measurement");
  assert(diagnostic_input.recovery_state_age_sec == 5.0);
  assert(diagnostic_input.recovery_state_transition_count == 6);
  assert(diagnostic_input.reinitialization_request_latched);
  assert(diagnostic_input.reinitialization_request_latch_age_sec == 3.5);
}

void test_make_diagnostic_input_zeros_unlatched_request_age()
{
  auto input = default_status_input();
  input.stamp_sec = 25.0;
  const auto preparation = ll::prepareAlignmentStatus(input);

  const auto diagnostic_input = ll::makeAlignmentStatusDiagnosticInput(
    input,
    preparation,
    ll::ReinitializationRequestDecision{},
    ll::AlignmentStatusDiagnosticRuntimeContext{
      "tracking",
      "accept_measurement",
      30.0,
      1,
      false,
      21.5});

  assert(diagnostic_input.recovery_state_age_sec == 0.0);
  assert(!diagnostic_input.reinitialization_request_latched);
  assert(diagnostic_input.reinitialization_request_latch_age_sec == 0.0);
}

int main()
{
  test_make_alignment_status_input_combines_observation_and_context();
  test_prepare_alignment_status_computes_gate_and_reinit();
  test_prepare_alignment_status_uses_fallback_reinit_metrics();
  test_recovery_evaluation_uses_latched_reinit_request();
  test_make_diagnostic_input_preserves_status_context();
  test_make_diagnostic_input_computes_runtime_ages();
  test_make_diagnostic_input_zeros_unlatched_request_age();
  return 0;
}
