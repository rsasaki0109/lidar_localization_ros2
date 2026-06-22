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
  std::string registration_seed_source{"not_selected"};
  bool imu_preintegration_enabled{false};
  std::string imu_preintegration_status{"imu_preintegration_disabled"};
  bool imu_preintegration_fallback_mode{false};
  bool imu_smoother_initialized{false};
  bool imu_has_new_samples{false};
  std::size_t imu_received_sample_count{0};
  std::size_t imu_integrated_sample_count{0};
  std::size_t imu_skipped_sample_count{0};
  std::size_t imu_transform_failure_count{0};
  std::size_t imu_non_finite_sample_count{0};
  std::size_t imu_invalid_dt_count{0};
  double imu_last_dt_sec{NAN};
  double imu_last_sample_age_sec{NAN};
  double imu_integration_window_sec{0.0};
  std::string scan_time_status{"scan_time_field_missing"};
  std::string scan_time_field{"none"};
  double scan_time_duration_sec{NAN};
  std::size_t scan_time_valid_point_count{0};
  std::size_t scan_time_invalid_point_count{0};
  bool deskew_ready{false};
  std::string deskew_readiness_status{"deskew_scan_time_field_missing"};
  bool continuous_time_deskew_enabled{false};
  bool continuous_time_deskew_applied{false};
  std::string continuous_time_deskew_status{"continuous_time_deskew_disabled"};
  std::size_t continuous_time_deskew_point_count{0};
  std::size_t continuous_time_deskew_skipped_invalid_time_count{0};
  std::size_t continuous_time_deskew_clamped_time_count{0};
  bool reinitialization_requested{false};
  std::string reinitialization_request_reason;
  double reinitialization_request_score{0.0};
  bool reinitialization_request_latched{false};
  double reinitialization_request_latch_age_sec{0.0};
  bool map_received{false};
  bool initialpose_received{false};
  std::string failure_category;
  bool weak_overlap_active{false};
  bool bad_match_active{false};
  bool stale_prediction_active{false};
  bool overload_active{false};
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
  values.reserve(61);
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
  append("imu_preintegration_enabled", boolString(input.imu_preintegration_enabled));
  append("imu_preintegration_status", input.imu_preintegration_status);
  append(
    "imu_preintegration_fallback_mode",
    boolString(input.imu_preintegration_fallback_mode));
  append("imu_smoother_initialized", boolString(input.imu_smoother_initialized));
  append("imu_has_new_samples", boolString(input.imu_has_new_samples));
  append("imu_received_sample_count", std::to_string(input.imu_received_sample_count));
  append("imu_integrated_sample_count", std::to_string(input.imu_integrated_sample_count));
  append("imu_skipped_sample_count", std::to_string(input.imu_skipped_sample_count));
  append("imu_transform_failure_count", std::to_string(input.imu_transform_failure_count));
  append("imu_non_finite_sample_count", std::to_string(input.imu_non_finite_sample_count));
  append("imu_invalid_dt_count", std::to_string(input.imu_invalid_dt_count));
  append("imu_last_dt_sec", std::to_string(input.imu_last_dt_sec));
  append("imu_last_sample_age_sec", std::to_string(input.imu_last_sample_age_sec));
  append("imu_integration_window_sec", std::to_string(input.imu_integration_window_sec));
  append("scan_time_status", input.scan_time_status);
  append("scan_time_field", input.scan_time_field);
  append("scan_time_duration_sec", std::to_string(input.scan_time_duration_sec));
  append(
    "scan_time_valid_point_count",
    std::to_string(input.scan_time_valid_point_count));
  append(
    "scan_time_invalid_point_count",
    std::to_string(input.scan_time_invalid_point_count));
  append("deskew_ready", boolString(input.deskew_ready));
  append("deskew_readiness_status", input.deskew_readiness_status);
  append("continuous_time_deskew_enabled", boolString(input.continuous_time_deskew_enabled));
  append("continuous_time_deskew_applied", boolString(input.continuous_time_deskew_applied));
  append("continuous_time_deskew_status", input.continuous_time_deskew_status);
  append(
    "continuous_time_deskew_point_count",
    std::to_string(input.continuous_time_deskew_point_count));
  append(
    "continuous_time_deskew_skipped_invalid_time_count",
    std::to_string(input.continuous_time_deskew_skipped_invalid_time_count));
  append(
    "continuous_time_deskew_clamped_time_count",
    std::to_string(input.continuous_time_deskew_clamped_time_count));
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
  append("failure_category", input.failure_category);
  append("weak_overlap_active", boolString(input.weak_overlap_active));
  append("bad_match_active", boolString(input.bad_match_active));
  append("stale_prediction_active", boolString(input.stale_prediction_active));
  append("overload_active", boolString(input.overload_active));
  append("registration_seed_source", input.registration_seed_source);
  return values;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_DIAGNOSTICS_POLICY_HPP_
