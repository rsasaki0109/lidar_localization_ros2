#ifndef LIDAR_LOCALIZATION_ALIGNMENT_PIPELINE_POLICY_HPP_
#define LIDAR_LOCALIZATION_ALIGNMENT_PIPELINE_POLICY_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include "lidar_localization/alignment_retry_policy.hpp"
#include "lidar_localization/measurement_gate_policy.hpp"

namespace lidar_localization
{

constexpr std::uint8_t kAlignmentPipelineError = 2;

struct AlignmentAttempt
{
  bool target_ready{false};
  bool has_converged{false};
  double alignment_time_sec{0.0};
  double fitness_score{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Matrix4f init_guess{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f final_transformation{Eigen::Matrix4f::Identity()};
  double correction_translation_m{std::numeric_limits<double>::quiet_NaN()};
  double correction_yaw_deg{std::numeric_limits<double>::quiet_NaN()};
  double seed_translation_since_accept_m{std::numeric_limits<double>::quiet_NaN()};
  double seed_yaw_since_accept_deg{std::numeric_limits<double>::quiet_NaN()};
  double accepted_gap_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct AlignmentPipelineInput
{
  bool have_last_accepted_pose{false};
  std::size_t consecutive_rejected_updates{0};
  double scan_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double last_accepted_pose_time_sec{std::numeric_limits<double>::quiet_NaN()};
  RecoveryRetryFromLastPoseParams recovery_retry_params;
};

struct AlignmentPipelineResult
{
  AlignmentAttempt selected_attempt;
  MeasurementGateDecision gate_result;
  bool recovered_by_retry_from_last_pose{false};
  bool should_continue{true};
  bool should_advance_prediction_without_measurement{false};
  std::uint8_t status_level{kMeasurementGateOk};
  std::string status_message{"ok"};
};

struct AlignmentPipelineHandlingDecision
{
  bool log_recovery_retry_success{false};
  bool publish_terminal_status{false};
  bool warn_registration_not_converged{false};
  bool advance_prediction_without_measurement{false};
  bool continue_to_backend{true};
};

inline AlignmentPipelineResult makeTerminalAlignmentPipelineResult(
  const AlignmentAttempt & attempt,
  const std::string & status_message)
{
  AlignmentPipelineResult result;
  result.selected_attempt = attempt;
  result.should_continue = false;
  result.should_advance_prediction_without_measurement = true;
  result.status_level = kAlignmentPipelineError;
  result.status_message = status_message;
  return result;
}

inline AlignmentPipelineHandlingDecision decideAlignmentPipelineHandling(
  const AlignmentPipelineResult & result)
{
  AlignmentPipelineHandlingDecision decision;
  decision.log_recovery_retry_success = result.recovered_by_retry_from_last_pose;
  decision.publish_terminal_status = !result.should_continue;
  decision.warn_registration_not_converged =
    decision.publish_terminal_status && result.status_message == "registration_not_converged";
  decision.advance_prediction_without_measurement =
    decision.publish_terminal_status && result.should_advance_prediction_without_measurement;
  decision.continue_to_backend = result.should_continue;
  return decision;
}

inline void syncPipelineStatusFromGate(AlignmentPipelineResult & result)
{
  result.status_level = result.gate_result.status_level;
  result.status_message = result.gate_result.status_message;
}

template<typename RetryAttemptFn, typename GateEvaluatorFn>
AlignmentPipelineResult runAlignmentPipeline(
  const AlignmentAttempt & primary_attempt,
  const AlignmentPipelineInput & input,
  RetryAttemptFn retry_attempt_fn,
  GateEvaluatorFn evaluate_gate)
{
  AlignmentPipelineResult result;
  result.selected_attempt = primary_attempt;

  auto try_recovery_retry = [&]() {
      const double fallback_accepted_gap_sec =
        computeRecoveryRetryFallbackAcceptedGapSec(
          input.have_last_accepted_pose,
          input.scan_stamp_sec,
          input.last_accepted_pose_time_sec);
      const auto retry_decision = decideRecoveryRetryFromLastPose(
        input.recovery_retry_params,
        makeRecoveryRetryFromLastPoseInput(
          input.have_last_accepted_pose,
          input.consecutive_rejected_updates,
          result.selected_attempt.accepted_gap_sec,
          fallback_accepted_gap_sec,
          result.selected_attempt.seed_translation_since_accept_m));
      if (!retry_decision.should_retry) {
        return false;
      }

      const AlignmentAttempt retry_attempt = retry_attempt_fn();
      if (!isRecoveryRetryAlignmentUsable(
          retry_attempt.target_ready, retry_attempt.has_converged))
      {
        return false;
      }

      const MeasurementGateDecision retry_gate = evaluate_gate(retry_attempt);
      if (!shouldUseRecoveryRetryResult(
          retry_attempt.target_ready,
          retry_attempt.has_converged,
          retry_gate.reject_measurement))
      {
        return false;
      }

      result.selected_attempt = retry_attempt;
      result.gate_result = makeRecoveryRetryRecoveredGateDecision(retry_gate);
      result.recovered_by_retry_from_last_pose = true;
      syncPipelineStatusFromGate(result);
      return true;
    };

  if (!result.selected_attempt.target_ready) {
    if (try_recovery_retry()) {
      return result;
    }
    return makeTerminalAlignmentPipelineResult(
      result.selected_attempt, "local_map_crop_too_small");
  }

  if (!result.selected_attempt.has_converged) {
    if (try_recovery_retry()) {
      return result;
    }
    return makeTerminalAlignmentPipelineResult(
      result.selected_attempt, "registration_not_converged");
  }

  result.gate_result = evaluate_gate(result.selected_attempt);
  syncPipelineStatusFromGate(result);
  if (result.gate_result.reject_measurement) {
    (void)try_recovery_retry();
  }
  return result;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_PIPELINE_POLICY_HPP_
