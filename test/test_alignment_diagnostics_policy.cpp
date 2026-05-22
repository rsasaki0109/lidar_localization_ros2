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
  input.reinitialization_requested = false;
  input.reinitialization_request_reason = "not_requested";
  input.reinitialization_request_score = 0.4;
  input.reinitialization_request_latched = true;
  input.reinitialization_request_latch_age_sec = 10.0;
  input.map_received = true;
  input.initialpose_received = false;

  const auto values = ll::buildAlignmentDiagnosticValues(input);
  assert(values.size() == 28);
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
  assert(values[26].first == "map_received");
  assert(values[26].second == "true");
  assert(values[27].first == "initialpose_received");
  assert(values[27].second == "false");
}

int main()
{
  test_bool_and_elapsed_helpers();
  test_effective_reinitialization_metrics_use_direct_values_when_finite();
  test_effective_reinitialization_metrics_fallbacks();
  test_build_alignment_diagnostic_values_order_and_strings();
  return 0;
}
