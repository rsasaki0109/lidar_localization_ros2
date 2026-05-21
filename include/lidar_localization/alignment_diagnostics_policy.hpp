#ifndef LIDAR_LOCALIZATION_ALIGNMENT_DIAGNOSTICS_POLICY_HPP_
#define LIDAR_LOCALIZATION_ALIGNMENT_DIAGNOSTICS_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace lidar_localization
{

using DiagnosticKeyValue = std::pair<std::string, std::string>;

struct EffectiveReinitializationMetricsInput
{
  double accepted_gap_sec{NAN};
  double seed_translation_since_accept_m{NAN};
  bool have_last_accepted_pose{false};
  double stamp_sec{0.0};
  double last_accepted_pose_time_sec{0.0};
  double fallback_seed_translation_since_accept_m{NAN};
};

struct EffectiveReinitializationMetrics
{
  double accepted_gap_sec{NAN};
  double seed_translation_since_accept_m{NAN};
};

struct AlignmentDiagnosticValuesInput
{
  std::string registration_method;
  bool has_converged{false};
  double fitness_score{NAN};
  double score_threshold{NAN};
  double effective_score_threshold{NAN};
  bool post_reject_strict_active{false};
  bool open_loop_strict_active{false};
  bool borderline_seed_gate_active{false};
  double alignment_time_sec{0.0};
  std::size_t filtered_point_count{0};
  double correction_translation_m{NAN};
  double correction_yaw_deg{NAN};
  double seed_translation_since_accept_m{NAN};
  double seed_yaw_since_accept_deg{NAN};
  double accepted_gap_sec{NAN};
  std::size_t consecutive_rejected_updates{0};
  std::string recovery_state;
  std::string recovery_action;
  double recovery_state_age_sec{0.0};
  std::size_t recovery_state_transition_count{0};
  bool imu_prediction_active{false};
  bool reinitialization_requested{false};
  std::string reinitialization_request_reason;
  double reinitialization_request_score{0.0};
  bool reinitialization_request_latched{false};
  double reinitialization_request_latch_age_sec{0.0};
  bool map_received{false};
  bool initialpose_received{false};
};

inline const char * boolString(bool value)
{
  return value ? "true" : "false";
}

inline double nonnegativeElapsedOrZero(double stamp_sec, double start_sec)
{
  return start_sec > 0.0 ? std::max(0.0, stamp_sec - start_sec) : 0.0;
}

inline EffectiveReinitializationMetrics resolveEffectiveReinitializationMetrics(
  const EffectiveReinitializationMetricsInput & input)
{
  EffectiveReinitializationMetrics metrics;
  metrics.accepted_gap_sec = input.accepted_gap_sec;
  if (!std::isfinite(metrics.accepted_gap_sec) && input.have_last_accepted_pose) {
    metrics.accepted_gap_sec =
      std::max(0.0, input.stamp_sec - input.last_accepted_pose_time_sec);
  }

  metrics.seed_translation_since_accept_m = input.seed_translation_since_accept_m;
  if (
    !std::isfinite(metrics.seed_translation_since_accept_m) &&
    input.have_last_accepted_pose)
  {
    metrics.seed_translation_since_accept_m =
      input.fallback_seed_translation_since_accept_m;
  }
  return metrics;
}

inline std::vector<DiagnosticKeyValue> buildAlignmentDiagnosticValues(
  const AlignmentDiagnosticValuesInput & input)
{
  std::vector<DiagnosticKeyValue> values;
  values.reserve(31);
  auto append = [&values](const std::string & key, const std::string & value) {
      values.emplace_back(key, value);
    };

  append("registration_method", input.registration_method);
  append("has_converged", boolString(input.has_converged));
  append("fitness_score", std::to_string(input.fitness_score));
  append("score_threshold", std::to_string(input.score_threshold));
  append("effective_score_threshold", std::to_string(input.effective_score_threshold));
  append(
    "post_reject_strict_score_threshold_active",
    boolString(input.post_reject_strict_active));
  append(
    "open_loop_strict_score_threshold_active",
    boolString(input.open_loop_strict_active));
  append(
    "borderline_seed_rejection_gate_active",
    boolString(input.borderline_seed_gate_active));
  append("alignment_time_sec", std::to_string(input.alignment_time_sec));
  append("filtered_point_count", std::to_string(input.filtered_point_count));
  append("correction_translation_m", std::to_string(input.correction_translation_m));
  append("correction_yaw_deg", std::to_string(input.correction_yaw_deg));
  append(
    "seed_translation_since_accept_m",
    std::to_string(input.seed_translation_since_accept_m));
  append("seed_yaw_since_accept_deg", std::to_string(input.seed_yaw_since_accept_deg));
  append("accepted_gap_sec", std::to_string(input.accepted_gap_sec));
  append(
    "consecutive_rejected_updates",
    std::to_string(input.consecutive_rejected_updates));
  append("recovery_state", input.recovery_state);
  append("recovery_action", input.recovery_action);
  append("recovery_state_age_sec", std::to_string(input.recovery_state_age_sec));
  append(
    "recovery_state_transition_count",
    std::to_string(input.recovery_state_transition_count));
  append("imu_prediction_active", boolString(input.imu_prediction_active));
  append("reinitialization_requested", boolString(input.reinitialization_requested));
  append("reinitialization_request_reason", input.reinitialization_request_reason);
  append(
    "reinitialization_request_score",
    std::to_string(input.reinitialization_request_score));
  append(
    "reinitialization_request_latched",
    boolString(input.reinitialization_request_latched));
  append(
    "reinitialization_request_latch_age_sec",
    std::to_string(input.reinitialization_request_latch_age_sec));
  append("map_received", boolString(input.map_received));
  append("initialpose_received", boolString(input.initialpose_received));
  return values;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_DIAGNOSTICS_POLICY_HPP_
