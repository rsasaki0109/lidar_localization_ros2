#ifndef LIDAR_LOCALIZATION_RECOVERY_SUPERVISOR_HPP_
#define LIDAR_LOCALIZATION_RECOVERY_SUPERVISOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace lidar_localization
{

enum class RecoverySupervisorState : uint8_t
{
  kTracking = 0,
  kDegraded = 1,
  kRecovering = 2,
  kReinitializationRequested = 3,
};

struct ReinitializationRequestDecision
{
  bool requested{false};
  std::string reason{"not_requested"};
  double score{0.0};
};

struct ReinitializationTriggerParams
{
  double threshold{0.95};
  double gap_scale_sec{30.0};
  double seed_translation_scale_m{100.0};
  double reject_streak_scale{200.0};
  double fitness_explosion_threshold{1000.0};
};

struct ReinitializationTriggerInput
{
  std::string status_message;
  double fitness_score{std::numeric_limits<double>::quiet_NaN()};
  double seed_translation_since_accept_m{std::numeric_limits<double>::quiet_NaN()};
  double accepted_gap_sec{std::numeric_limits<double>::quiet_NaN()};
  std::size_t consecutive_rejected_updates{0};
};

struct RecoverySupervisorEvaluation
{
  RecoverySupervisorState state{RecoverySupervisorState::kTracking};
  std::string action{"accept_measurement"};
};

inline ReinitializationTriggerParams makeReinitializationTriggerParams(
  double threshold,
  double gap_scale_sec,
  double seed_translation_scale_m,
  double reject_streak_scale,
  double fitness_explosion_threshold)
{
  return {
    threshold,
    gap_scale_sec,
    seed_translation_scale_m,
    reject_streak_scale,
    fitness_explosion_threshold};
}

inline ReinitializationTriggerInput makeReinitializationTriggerInput(
  const std::string & status_message,
  double fitness_score,
  double seed_translation_since_accept_m,
  double accepted_gap_sec,
  std::size_t consecutive_rejected_updates)
{
  return {
    status_message,
    fitness_score,
    seed_translation_since_accept_m,
    accepted_gap_sec,
    consecutive_rejected_updates};
}

inline bool isRecoveryFailureStatus(const std::string & status_message)
{
  return status_message == "local_map_crop_too_small" ||
         status_message == "registration_not_converged" ||
         status_message == "filtered_scan_empty" ||
         status_message == "scan_missing_xyz_field" ||
         status_message.rfind("fitness_score_over_", 0) == 0;
}

inline ReinitializationRequestDecision computeReinitializationRequest(
  const ReinitializationTriggerParams & params,
  const ReinitializationTriggerInput & input)
{
  ReinitializationRequestDecision decision;
  const bool target_unavailable = input.status_message == "local_map_crop_too_small";
  const bool not_converged = input.status_message == "registration_not_converged";
  const bool rejected_measurement = input.status_message.rfind("fitness_score_over_", 0) == 0;
  const bool failure = target_unavailable || not_converged || rejected_measurement;

  if (!failure) {
    decision.reason = "not_failure";
    return decision;
  }

  const double gap_component =
    std::isfinite(input.accepted_gap_sec) && params.gap_scale_sec > 0.0 ?
    std::min(1.0, input.accepted_gap_sec / params.gap_scale_sec) : 0.0;
  const double seed_component =
    std::isfinite(input.seed_translation_since_accept_m) &&
    params.seed_translation_scale_m > 0.0 ?
    std::min(1.0, input.seed_translation_since_accept_m / params.seed_translation_scale_m) : 0.0;
  const double streak_component =
    params.reject_streak_scale > 0.0 ?
    std::min(
      1.0,
      static_cast<double>(input.consecutive_rejected_updates) /
      params.reject_streak_scale) : 0.0;
  const bool fitness_exploded =
    std::isfinite(input.fitness_score) &&
    input.fitness_score >= params.fitness_explosion_threshold;

  decision.score =
    0.45 * gap_component +
    0.30 * seed_component +
    0.20 * streak_component +
    (target_unavailable ? 0.15 : 0.0) +
    (fitness_exploded ? 0.15 : 0.0);

  if (decision.score < params.threshold) {
    decision.reason = "reinit_not_requested";
    return decision;
  }

  decision.requested = true;
  if (target_unavailable) {
    decision.reason = "target_unavailable_reinit_requested";
  } else if (fitness_exploded) {
    decision.reason = "fitness_exploded_reinit_requested";
  } else if (gap_component >= 1.0) {
    decision.reason = "accepted_gap_reinit_requested";
  } else if (streak_component >= 1.0) {
    decision.reason = "reject_streak_reinit_requested";
  } else {
    decision.reason = "reinit_score_exceeded";
  }
  return decision;
}

inline const char * recoverySupervisorStateName(RecoverySupervisorState state)
{
  switch (state) {
    case RecoverySupervisorState::kTracking:
      return "tracking";
    case RecoverySupervisorState::kDegraded:
      return "degraded";
    case RecoverySupervisorState::kRecovering:
      return "recovering";
    case RecoverySupervisorState::kReinitializationRequested:
      return "reinitialization_requested";
  }
  return "unknown";
}

inline RecoverySupervisorState classifyRecoverySupervisorState(
  uint8_t level,
  const std::string & status_message,
  const ReinitializationRequestDecision & reinitialization_request,
  std::size_t consecutive_rejected_updates)
{
  constexpr uint8_t kDiagnosticOk = 0;
  if (reinitialization_request.requested) {
    return RecoverySupervisorState::kReinitializationRequested;
  }
  if (
    status_message == "recovery_retry_from_last_pose_recovered" ||
    isRecoveryFailureStatus(status_message) ||
    consecutive_rejected_updates > 0)
  {
    return RecoverySupervisorState::kRecovering;
  }
  if (level != kDiagnosticOk) {
    return RecoverySupervisorState::kDegraded;
  }
  return RecoverySupervisorState::kTracking;
}

inline std::string classifyRecoverySupervisorAction(
  uint8_t level,
  const std::string & status_message,
  const ReinitializationRequestDecision & reinitialization_request,
  std::size_t consecutive_rejected_updates)
{
  constexpr uint8_t kDiagnosticOk = 0;
  if (reinitialization_request.requested) {
    return "request_reinitialization";
  }
  if (status_message == "recovery_retry_from_last_pose_recovered") {
    return "retry_from_last_pose";
  }
  if (status_message.find("_seeded") != std::string::npos) {
    return "reuse_rejected_seed_for_prediction";
  }
  if (
    status_message == "local_map_crop_too_small" ||
    status_message == "registration_not_converged" ||
    status_message == "filtered_scan_empty" ||
    status_message == "scan_missing_xyz_field")
  {
    return "advance_prediction_without_measurement";
  }
  if (
    status_message.rfind("fitness_score_over_", 0) == 0 &&
    status_message.find("_rejected") != std::string::npos)
  {
    return "reject_measurement";
  }
  if (consecutive_rejected_updates > 0) {
    return "accept_measurement_after_recovery";
  }
  if (level != kDiagnosticOk) {
    return "accept_measurement_with_warning";
  }
  return "accept_measurement";
}

inline RecoverySupervisorEvaluation evaluateRecoverySupervisor(
  uint8_t level,
  const std::string & status_message,
  const ReinitializationRequestDecision & reinitialization_request,
  std::size_t consecutive_rejected_updates)
{
  return {
    classifyRecoverySupervisorState(
      level,
      status_message,
      reinitialization_request,
      consecutive_rejected_updates),
    classifyRecoverySupervisorAction(
      level,
      status_message,
      reinitialization_request,
      consecutive_rejected_updates)};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_RECOVERY_SUPERVISOR_HPP_
