#include "lidar_localization/alignment_diagnostics_policy.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace ll = lidar_localization;

void test_bool_and_elapsed_helpers()
{
  assert(std::string(ll::boolString(true)) == "true");
  assert(std::string(ll::boolString(false)) == "false");
  assert(ll::nonnegativeElapsedOrZero(10.0, 7.0) == 3.0);
  assert(ll::nonnegativeElapsedOrZero(7.0, 10.0) == 0.0);
  assert(ll::nonnegativeElapsedOrZero(10.0, 0.0) == 0.0);
}

void test_effective_reinitialization_metrics_use_direct_values_when_finite()
{
  const auto metrics = ll::resolveEffectiveReinitializationMetrics(
    ll::EffectiveReinitializationMetricsInput{
      2.0,
      3.0,
      true,
      10.0,
      1.0,
      99.0});

  assert(metrics.accepted_gap_sec == 2.0);
  assert(metrics.seed_translation_since_accept_m == 3.0);
}

void test_effective_reinitialization_metrics_fallbacks()
{
  const auto metrics = ll::resolveEffectiveReinitializationMetrics(
    ll::EffectiveReinitializationMetricsInput{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      true,
      10.0,
      7.0,
      4.0});

  assert(metrics.accepted_gap_sec == 3.0);
  assert(metrics.seed_translation_since_accept_m == 4.0);

  const auto no_pose = ll::resolveEffectiveReinitializationMetrics(
    ll::EffectiveReinitializationMetricsInput{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      false,
      10.0,
      7.0,
      4.0});
  assert(std::isnan(no_pose.accepted_gap_sec));
  assert(std::isnan(no_pose.seed_translation_since_accept_m));
}

void test_build_alignment_diagnostic_values_order_and_strings()
{
  ll::AlignmentDiagnosticValuesInput input;
  input.registration_method = "NDT";
  input.has_converged = true;
  input.fitness_score = 1.25;
  input.score_threshold = 2.0;
  input.effective_score_threshold = 1.5;
  input.post_reject_strict_active = true;
  input.open_loop_strict_active = false;
  input.borderline_seed_gate_active = true;
  input.alignment_time_sec = 0.01;
  input.filtered_point_count = 123;
  input.correction_translation_m = 0.2;
  input.correction_yaw_deg = 0.3;
  input.seed_translation_since_accept_m = 4.0;
  input.seed_yaw_since_accept_deg = 5.0;
  input.accepted_gap_sec = 6.0;
  input.consecutive_rejected_updates = 7;
  input.recovery_state = "recovering";
  input.recovery_action = "reject_measurement";
  input.recovery_state_age_sec = 8.0;
  input.recovery_state_transition_count = 9;
  input.imu_prediction_active = true;
  input.registration_seed_source = "imu_preintegration";
  input.imu_preintegration_enabled = true;
  input.imu_preintegration_status = "imu_preintegration_prediction_active";
  input.imu_preintegration_fallback_mode = false;
  input.imu_smoother_initialized = true;
  input.imu_has_new_samples = true;
  input.imu_received_sample_count = 20;
  input.imu_integrated_sample_count = 19;
  input.imu_skipped_sample_count = 1;
  input.imu_transform_failure_count = 0;
  input.imu_non_finite_sample_count = 0;
  input.imu_invalid_dt_count = 1;
  input.imu_last_dt_sec = 0.005;
  input.imu_last_sample_age_sec = 0.01;
  input.imu_integration_window_sec = 0.09;
  input.scan_time_status = "scan_time_range_ready";
  input.scan_time_field = "offset_time";
  input.scan_time_duration_sec = 0.095;
  input.scan_time_valid_point_count = 120;
  input.scan_time_invalid_point_count = 3;
  input.deskew_ready = true;
  input.deskew_readiness_status = "deskew_ready";
  input.continuous_time_deskew_enabled = true;
  input.continuous_time_deskew_applied = true;
  input.continuous_time_deskew_status = "continuous_time_deskew_applied";
  input.continuous_time_deskew_point_count = 115;
  input.continuous_time_deskew_skipped_invalid_time_count = 2;
  input.continuous_time_deskew_clamped_time_count = 1;
  input.reinitialization_requested = false;
  input.reinitialization_request_reason = "not_requested";
  input.reinitialization_request_score = 0.4;
  input.reinitialization_request_latched = true;
  input.reinitialization_request_latch_age_sec = 10.0;
  input.map_received = true;
  input.initialpose_received = false;
  input.failure_category = "bad_match";
  input.weak_overlap_active = false;
  input.bad_match_active = true;
  input.stale_prediction_active = true;
  input.overload_active = false;
  input.continuous_time_deskew_mode = "imu_pose_history";
  input.continuous_time_deskew_pose_history_coverage_ratio = 1.0;
  input.localizability_guard_enabled = true;
  input.localizability_valid = true;
  input.horizontal_localizability_eigenvalue_ratio = 0.25;
  input.localizability_guard_active = false;
  input.registration_localizability.enabled = true;
  input.registration_localizability.valid = true;
  input.registration_localizability.status = "ok";
  input.registration_localizability.correspondence_count = 456;
  input.registration_localizability.eigenvalues[0] = 0.125;
  input.registration_localizability.weakest_eigenvector[0] = -0.5;

  const auto values = ll::buildAlignmentDiagnosticValues(input);
  assert(values.size() == 88);
  assert(values[0].first == "registration_method");
  assert(values[0].second == "NDT");
  assert(values[1].first == "has_converged");
  assert(values[1].second == "true");
  assert(values[5].first == "post_reject_strict_score_threshold_active");
  assert(values[5].second == "true");
  assert(values[7].first == "borderline_seed_rejection_gate_active");
  assert(values[7].second == "true");
  assert(values[9].first == "filtered_point_count");
  assert(values[9].second == "123");
  assert(values[16].first == "recovery_state");
  assert(values[16].second == "recovering");
  assert(values[20].first == "imu_prediction_active");
  assert(values[20].second == "true");
  assert(values[21].first == "imu_preintegration_enabled");
  assert(values[21].second == "true");
  assert(values[22].first == "imu_preintegration_status");
  assert(values[22].second == "imu_preintegration_prediction_active");
  assert(values[23].first == "imu_preintegration_fallback_mode");
  assert(values[23].second == "false");
  assert(values[24].first == "imu_smoother_initialized");
  assert(values[24].second == "true");
  assert(values[25].first == "imu_has_new_samples");
  assert(values[25].second == "true");
  assert(values[26].first == "imu_received_sample_count");
  assert(values[26].second == "20");
  assert(values[27].first == "imu_integrated_sample_count");
  assert(values[27].second == "19");
  assert(values[28].first == "imu_skipped_sample_count");
  assert(values[28].second == "1");
  assert(values[29].first == "imu_transform_failure_count");
  assert(values[29].second == "0");
  assert(values[30].first == "imu_non_finite_sample_count");
  assert(values[30].second == "0");
  assert(values[31].first == "imu_invalid_dt_count");
  assert(values[31].second == "1");
  assert(values[32].first == "imu_last_dt_sec");
  assert(values[32].second.find("0.005") == 0);
  assert(values[33].first == "imu_last_sample_age_sec");
  assert(values[33].second.find("0.010") == 0);
  assert(values[34].first == "imu_integration_window_sec");
  assert(values[34].second.find("0.090") == 0);
  assert(values[35].first == "scan_time_status");
  assert(values[35].second == "scan_time_range_ready");
  assert(values[36].first == "scan_time_field");
  assert(values[36].second == "offset_time");
  assert(values[37].first == "scan_time_duration_sec");
  assert(values[37].second.find("0.095") == 0);
  assert(values[38].first == "scan_time_valid_point_count");
  assert(values[38].second == "120");
  assert(values[39].first == "scan_time_invalid_point_count");
  assert(values[39].second == "3");
  assert(values[40].first == "deskew_ready");
  assert(values[40].second == "true");
  assert(values[41].first == "deskew_readiness_status");
  assert(values[41].second == "deskew_ready");
  assert(values[42].first == "continuous_time_deskew_enabled");
  assert(values[42].second == "true");
  assert(values[43].first == "continuous_time_deskew_applied");
  assert(values[43].second == "true");
  assert(values[44].first == "continuous_time_deskew_status");
  assert(values[44].second == "continuous_time_deskew_applied");
  assert(values[45].first == "continuous_time_deskew_point_count");
  assert(values[45].second == "115");
  assert(values[46].first == "continuous_time_deskew_skipped_invalid_time_count");
  assert(values[46].second == "2");
  assert(values[47].first == "continuous_time_deskew_clamped_time_count");
  assert(values[47].second == "1");
  assert(values[48].first == "registration_localizability_enabled");
  assert(values[48].second == "true");
  assert(values[51].first == "registration_localizability_correspondence_count");
  assert(values[51].second == "456");
  assert(values[57].first == "registration_localizability_eigenvalue_0");
  assert(values[57].second.find("0.125") == 0);
  assert(values[58].first == "registration_localizability_weakest_vector_x");
  assert(values[58].second.find("-0.500") == 0);
  assert(values[74].first == "map_received");
  assert(values[74].second == "true");
  assert(values[75].first == "initialpose_received");
  assert(values[75].second == "false");
  assert(values[76].first == "failure_category");
  assert(values[76].second == "bad_match");
  assert(values[77].first == "weak_overlap_active");
  assert(values[77].second == "false");
  assert(values[78].first == "bad_match_active");
  assert(values[78].second == "true");
  assert(values[79].first == "stale_prediction_active");
  assert(values[79].second == "true");
  assert(values[80].first == "overload_active");
  assert(values[80].second == "false");
  assert(values[81].first == "registration_seed_source");
  assert(values[81].second == "imu_preintegration");
  assert(values[82].first == "continuous_time_deskew_mode");
  assert(values[82].second == "imu_pose_history");
  assert(values[83].first == "continuous_time_deskew_pose_history_coverage_ratio");
  assert(values[83].second.find("1.000") == 0);
  assert(values[84].first == "localizability_guard_enabled");
  assert(values[84].second == "true");
  assert(values[85].first == "localizability_valid");
  assert(values[85].second == "true");
  assert(values[86].first == "horizontal_localizability_eigenvalue_ratio");
  assert(values[86].second.find("0.250") == 0);
  assert(values[87].first == "localizability_guard_active");
  assert(values[87].second == "false");
}

int main()
{
  test_bool_and_elapsed_helpers();
  test_effective_reinitialization_metrics_use_direct_values_when_finite();
  test_effective_reinitialization_metrics_fallbacks();
  test_build_alignment_diagnostic_values_order_and_strings();
  return 0;
}
