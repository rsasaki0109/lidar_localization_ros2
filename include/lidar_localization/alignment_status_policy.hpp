#ifndef LIDAR_LOCALIZATION_ALIGNMENT_STATUS_POLICY_HPP_
#define LIDAR_LOCALIZATION_ALIGNMENT_STATUS_POLICY_HPP_

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <string>

#include "lidar_localization/alignment_diagnostics_policy.hpp"
#include "lidar_localization/measurement_gate_policy.hpp"
#include "lidar_localization/recovery_supervisor.hpp"

namespace lidar_localization
{

struct AlignmentStatusInput
{
  std::string registration_method;
  std::uint8_t level{0};
  std::string message{"ok"};
  bool has_converged{false};
  double fitness_score{0.0};
  double alignment_time_sec{0.0};
  std::size_t filtered_point_count{0};
  double correction_translation_m{NAN};
  double correction_yaw_deg{NAN};
  double seed_translation_since_accept_m{NAN};
  double seed_yaw_since_accept_deg{NAN};
  double accepted_gap_sec{NAN};
  std::size_t consecutive_rejected_updates{0};
  bool have_last_accepted_pose{false};
  double stamp_sec{0.0};
  double last_accepted_pose_time_sec{0.0};
  double fallback_seed_translation_since_accept_m{NAN};
  bool imu_prediction_active{false};
  bool map_received{false};
  bool initialpose_received{false};
  MeasurementGateParams gate_params;
  ReinitializationTriggerParams reinitialization_params;
};

struct AlignmentStatusObservation
{
  std::uint8_t level{0};
  std::string message{"ok"};
  bool has_converged{false};
  double fitness_score{0.0};
  double alignment_time_sec{0.0};
  std::size_t filtered_point_count{0};
  double correction_translation_m{NAN};
  double correction_yaw_deg{NAN};
  double seed_translation_since_accept_m{NAN};
  double seed_yaw_since_accept_deg{NAN};
  double accepted_gap_sec{NAN};
  bool imu_prediction_active{false};
};

struct AlignmentStatusRuntimeContext
{
  std::string registration_method;
  std::size_t consecutive_rejected_updates{0};
  bool have_last_accepted_pose{false};
  double stamp_sec{0.0};
  double last_accepted_pose_time_sec{0.0};
  double fallback_seed_translation_since_accept_m{NAN};
  bool map_received{false};
  bool initialpose_received{false};
  MeasurementGateParams gate_params;
  ReinitializationTriggerParams reinitialization_params;
};

struct AlignmentStatusPreparation
{
  MeasurementGateInput gate_input;
  EffectiveScoreThresholdDecision threshold_decision;
  bool borderline_seed_gate_active{false};
  EffectiveReinitializationMetrics effective_reinitialization_metrics;
  ReinitializationRequestDecision reinitialization_request;
};

struct AlignmentStatusDiagnosticInput
{
  AlignmentStatusInput status_input;
  AlignmentStatusPreparation preparation;
  ReinitializationRequestDecision reinitialization_request;
  std::string recovery_state;
  std::string recovery_action;
  double recovery_state_age_sec{0.0};
  std::size_t recovery_state_transition_count{0};
  bool reinitialization_request_latched{false};
  double reinitialization_request_latch_age_sec{0.0};
};

struct AlignmentStatusDiagnosticRuntimeContext
{
  std::string recovery_state;
  std::string recovery_action;
  double recovery_state_entered_stamp_sec{0.0};
  std::size_t recovery_state_transition_count{0};
  bool reinitialization_request_latched{false};
  double reinitialization_request_latch_stamp_sec{0.0};
};

inline AlignmentStatusInput makeAlignmentStatusInput(
  const AlignmentStatusObservation & observation,
  const AlignmentStatusRuntimeContext & context)
{
  AlignmentStatusInput input;
  input.registration_method = context.registration_method;
  input.level = observation.level;
  input.message = observation.message;
  input.has_converged = observation.has_converged;
  input.fitness_score = observation.fitness_score;
  input.alignment_time_sec = observation.alignment_time_sec;
  input.filtered_point_count = observation.filtered_point_count;
  input.correction_translation_m = observation.correction_translation_m;
  input.correction_yaw_deg = observation.correction_yaw_deg;
  input.seed_translation_since_accept_m = observation.seed_translation_since_accept_m;
  input.seed_yaw_since_accept_deg = observation.seed_yaw_since_accept_deg;
  input.accepted_gap_sec = observation.accepted_gap_sec;
  input.consecutive_rejected_updates = context.consecutive_rejected_updates;
  input.have_last_accepted_pose = context.have_last_accepted_pose;
  input.stamp_sec = context.stamp_sec;
  input.last_accepted_pose_time_sec = context.last_accepted_pose_time_sec;
  input.fallback_seed_translation_since_accept_m =
    context.fallback_seed_translation_since_accept_m;
  input.imu_prediction_active = observation.imu_prediction_active;
  input.map_received = context.map_received;
  input.initialpose_received = context.initialpose_received;
  input.gate_params = context.gate_params;
  input.reinitialization_params = context.reinitialization_params;
  return input;
}

inline AlignmentStatusPreparation prepareAlignmentStatus(
  const AlignmentStatusInput & input)
{
  AlignmentStatusPreparation preparation;
  preparation.gate_input = makeMeasurementGateInput(
    input.fitness_score,
    input.accepted_gap_sec,
    input.seed_translation_since_accept_m,
    input.correction_translation_m,
    input.correction_yaw_deg,
    input.consecutive_rejected_updates);
  preparation.threshold_decision =
    computeEffectiveScoreThreshold(input.gate_params, preparation.gate_input);
  preparation.borderline_seed_gate_active = isBorderlineSeedGateActive(
    input.gate_params,
    preparation.gate_input,
    preparation.threshold_decision.effective_score_threshold);

  preparation.effective_reinitialization_metrics =
    resolveEffectiveReinitializationMetrics(
      EffectiveReinitializationMetricsInput{
        input.accepted_gap_sec,
        input.seed_translation_since_accept_m,
        input.have_last_accepted_pose,
        input.stamp_sec,
        input.last_accepted_pose_time_sec,
        input.fallback_seed_translation_since_accept_m});

  const auto reinit_input = makeReinitializationTriggerInput(
    input.message,
    input.fitness_score,
    preparation.effective_reinitialization_metrics.seed_translation_since_accept_m,
    preparation.effective_reinitialization_metrics.accepted_gap_sec,
    input.consecutive_rejected_updates);
  preparation.reinitialization_request =
    computeReinitializationRequest(input.reinitialization_params, reinit_input);
  return preparation;
}

inline RecoverySupervisorEvaluation evaluateAlignmentStatusRecovery(
  const AlignmentStatusInput & input,
  const ReinitializationRequestDecision & reinitialization_request)
{
  return evaluateRecoverySupervisor(
    input.level,
    input.message,
    reinitialization_request,
    input.consecutive_rejected_updates);
}

inline AlignmentStatusDiagnosticInput makeAlignmentStatusDiagnosticInput(
  const AlignmentStatusInput & status_input,
  const AlignmentStatusPreparation & preparation,
  const ReinitializationRequestDecision & reinitialization_request,
  const AlignmentStatusDiagnosticRuntimeContext & context)
{
  const double recovery_state_age_sec =
    nonnegativeElapsedOrZero(
      status_input.stamp_sec,
      context.recovery_state_entered_stamp_sec);
  const double reinitialization_request_latch_age_sec =
    context.reinitialization_request_latched &&
    context.reinitialization_request_latch_stamp_sec > 0.0 ?
    nonnegativeElapsedOrZero(
      status_input.stamp_sec,
      context.reinitialization_request_latch_stamp_sec) : 0.0;

  return AlignmentStatusDiagnosticInput{
    status_input,
    preparation,
    reinitialization_request,
    context.recovery_state,
    context.recovery_action,
    recovery_state_age_sec,
    context.recovery_state_transition_count,
    context.reinitialization_request_latched,
    reinitialization_request_latch_age_sec};
}

inline AlignmentDiagnosticValuesInput makeAlignmentStatusDiagnosticValuesInput(
  const AlignmentStatusDiagnosticInput & input)
{
  const auto & status = input.status_input;
  const auto & preparation = input.preparation;
  AlignmentDiagnosticValuesInput diagnostic_input;
  diagnostic_input.registration_method = status.registration_method;
  diagnostic_input.has_converged = status.has_converged;
  diagnostic_input.fitness_score = status.fitness_score;
  diagnostic_input.score_threshold = status.gate_params.score_threshold;
  diagnostic_input.effective_score_threshold =
    preparation.threshold_decision.effective_score_threshold;
  diagnostic_input.post_reject_strict_active =
    preparation.threshold_decision.post_reject_strict_active;
  diagnostic_input.open_loop_strict_active =
    preparation.threshold_decision.open_loop_strict_active;
  diagnostic_input.borderline_seed_gate_active =
    preparation.borderline_seed_gate_active;
  diagnostic_input.alignment_time_sec = status.alignment_time_sec;
  diagnostic_input.filtered_point_count = status.filtered_point_count;
  diagnostic_input.correction_translation_m = status.correction_translation_m;
  diagnostic_input.correction_yaw_deg = status.correction_yaw_deg;
  diagnostic_input.seed_translation_since_accept_m =
    status.seed_translation_since_accept_m;
  diagnostic_input.seed_yaw_since_accept_deg = status.seed_yaw_since_accept_deg;
  diagnostic_input.accepted_gap_sec = status.accepted_gap_sec;
  diagnostic_input.consecutive_rejected_updates =
    status.consecutive_rejected_updates;
  diagnostic_input.recovery_state = input.recovery_state;
  diagnostic_input.recovery_action = input.recovery_action;
  diagnostic_input.recovery_state_age_sec = input.recovery_state_age_sec;
  diagnostic_input.recovery_state_transition_count =
    input.recovery_state_transition_count;
  diagnostic_input.imu_prediction_active = status.imu_prediction_active;
  diagnostic_input.reinitialization_requested =
    input.reinitialization_request.requested;
  diagnostic_input.reinitialization_request_reason =
    input.reinitialization_request.reason;
  diagnostic_input.reinitialization_request_score =
    input.reinitialization_request.score;
  diagnostic_input.reinitialization_request_latched =
    input.reinitialization_request_latched;
  diagnostic_input.reinitialization_request_latch_age_sec =
    input.reinitialization_request_latch_age_sec;
  diagnostic_input.map_received = status.map_received;
  diagnostic_input.initialpose_received = status.initialpose_received;
  return diagnostic_input;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_STATUS_POLICY_HPP_
