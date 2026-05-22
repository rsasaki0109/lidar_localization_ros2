#include "lidar_localization/measurement_gate_policy.hpp"

#include <cassert>
#include <limits>
#include <string>

namespace ll = lidar_localization;

ll::MeasurementGateParams default_params()
{
  ll::MeasurementGateParams params;
  params.score_threshold = 10.0;
  params.reject_above_score_threshold = true;
  return params;
}

ll::MeasurementGateInput default_input()
{
  return ll::makeMeasurementGateInput(1.0, 0.0, 0.0, 0.0, 0.0, 0);
}

void test_param_and_input_builders_map_fields()
{
  ll::MeasurementGateParamConfig config;
  config.score_threshold = 9.0;
  config.reject_above_score_threshold = false;
  config.enable_consistency_recovery_gate = true;
  config.consistency_recovery_min_rejections = 4;
  config.consistency_recovery_score_margin = 1.5;
  config.consistency_recovery_max_translation_m = 0.2;
  config.consistency_recovery_max_yaw_deg = 3.0;
  config.enable_post_reject_strict_score_threshold = true;
  config.post_reject_strict_min_rejections = 5;
  config.post_reject_strict_score_threshold = 7.0;
  config.enable_open_loop_strict_score_threshold = true;
  config.open_loop_strict_min_accepted_gap_sec = 6.0;
  config.open_loop_strict_min_seed_translation_m = 2.5;
  config.open_loop_strict_score_threshold = 6.5;
  config.enable_borderline_seed_rejection_gate = true;
  config.borderline_seed_gate_score_threshold = 6.25;
  config.borderline_seed_gate_min_seed_translation_m = 1.25;
  config.enable_rejected_seed_update = true;
  config.rejected_seed_update_min_rejections = 8;
  config.rejected_seed_update_max_fitness = 10.5;
  config.rejected_seed_update_max_correction_translation_m = 1.75;
  config.rejected_seed_update_max_correction_yaw_deg = 2.25;

  const auto params = ll::makeMeasurementGateParams(config);
  assert(params.score_threshold == 9.0);
  assert(!params.reject_above_score_threshold);
  assert(params.enable_consistency_recovery_gate);
  assert(params.consistency_recovery_min_rejections == 4);
  assert(params.consistency_recovery_score_margin == 1.5);
  assert(params.consistency_recovery_max_translation_m == 0.2);
  assert(params.consistency_recovery_max_yaw_deg == 3.0);
  assert(params.enable_post_reject_strict_score_threshold);
  assert(params.post_reject_strict_min_rejections == 5);
  assert(params.post_reject_strict_score_threshold == 7.0);
  assert(params.enable_open_loop_strict_score_threshold);
  assert(params.open_loop_strict_min_accepted_gap_sec == 6.0);
  assert(params.open_loop_strict_min_seed_translation_m == 2.5);
  assert(params.open_loop_strict_score_threshold == 6.5);
  assert(params.enable_borderline_seed_rejection_gate);
  assert(params.borderline_seed_gate_score_threshold == 6.25);
  assert(params.borderline_seed_gate_min_seed_translation_m == 1.25);
  assert(params.enable_rejected_seed_update);
  assert(params.rejected_seed_update_min_rejections == 8);
  assert(params.rejected_seed_update_max_fitness == 10.5);
  assert(params.rejected_seed_update_max_correction_translation_m == 1.75);
  assert(params.rejected_seed_update_max_correction_yaw_deg == 2.25);

  const auto input = ll::makeMeasurementGateInput(1.0, 2.0, 3.0, 4.0, 5.0, 6);
  assert(input.fitness_score == 1.0);
  assert(input.accepted_gap_sec == 2.0);
  assert(input.seed_translation_since_accept_m == 3.0);
  assert(input.correction_translation_m == 4.0);
  assert(input.correction_yaw_deg == 5.0);
  assert(input.consecutive_rejected_updates == 6);
}

void test_default_ok_gate()
{
  const auto gate = ll::evaluateMeasurementGate(default_params(), default_input());
  assert(gate.status_level == ll::kMeasurementGateOk);
  assert(gate.status_message == "ok");
  assert(!gate.reject_measurement);
  assert(gate.effective_score_threshold == 10.0);

  auto nan_input = default_input();
  nan_input.fitness_score = std::numeric_limits<double>::quiet_NaN();
  const auto nan_gate = ll::evaluateMeasurementGate(default_params(), nan_input);
  assert(nan_gate.status_level == ll::kMeasurementGateOk);
}

void test_effective_threshold_uses_lowest_active_strict_threshold()
{
  auto params = default_params();
  params.enable_post_reject_strict_score_threshold = true;
  params.post_reject_strict_min_rejections = 3;
  params.post_reject_strict_score_threshold = 8.0;
  params.enable_open_loop_strict_score_threshold = true;
  params.open_loop_strict_min_accepted_gap_sec = 5.0;
  params.open_loop_strict_min_seed_translation_m = 2.0;
  params.open_loop_strict_score_threshold = 6.0;

  auto input = default_input();
  input.consecutive_rejected_updates = 3;
  input.accepted_gap_sec = 5.0;
  input.seed_translation_since_accept_m = 2.0;

  const auto threshold = ll::computeEffectiveScoreThreshold(params, input);
  assert(threshold.post_reject_strict_active);
  assert(threshold.open_loop_strict_active);
  assert(threshold.effective_score_threshold == 6.0);
}

void test_threshold_rejection_messages()
{
  auto params = default_params();
  auto input = default_input();
  input.fitness_score = 11.0;

  auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.status_level == ll::kMeasurementGateWarn);
  assert(gate.reject_measurement);
  assert(gate.status_message == "fitness_score_over_threshold_rejected");

  params.reject_above_score_threshold = false;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.status_level == ll::kMeasurementGateWarn);
  assert(!gate.reject_measurement);
  assert(gate.status_message == "fitness_score_over_threshold");
}

void test_open_loop_and_post_reject_messages()
{
  auto params = default_params();
  params.enable_open_loop_strict_score_threshold = true;
  params.open_loop_strict_min_accepted_gap_sec = 5.0;
  params.open_loop_strict_min_seed_translation_m = 2.0;
  params.open_loop_strict_score_threshold = 6.0;

  auto input = default_input();
  input.accepted_gap_sec = 5.0;
  input.seed_translation_since_accept_m = 2.0;
  input.fitness_score = 7.0;

  auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.open_loop_strict_active);
  assert(gate.status_message == "fitness_score_over_open_loop_strict_threshold_rejected");

  params.enable_post_reject_strict_score_threshold = true;
  params.post_reject_strict_min_rejections = 2;
  params.post_reject_strict_score_threshold = 5.0;
  input.consecutive_rejected_updates = 2;
  gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.post_reject_strict_active);
  assert(gate.status_message == "fitness_score_over_post_reject_strict_threshold_rejected");
}

void test_borderline_seed_gate()
{
  auto params = default_params();
  params.enable_borderline_seed_rejection_gate = true;
  params.borderline_seed_gate_score_threshold = 8.0;
  params.borderline_seed_gate_min_seed_translation_m = 1.0;

  auto input = default_input();
  input.fitness_score = 9.0;
  input.seed_translation_since_accept_m = 1.0;

  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.borderline_seed_gate_active);
  assert(gate.reject_measurement);
  assert(gate.status_message == "fitness_score_over_borderline_seed_gate_rejected");
}

void test_consistency_recovery_clears_rejection()
{
  auto params = default_params();
  params.enable_consistency_recovery_gate = true;
  params.consistency_recovery_min_rejections = 3;
  params.consistency_recovery_score_margin = 2.0;
  params.consistency_recovery_max_translation_m = 0.1;
  params.consistency_recovery_max_yaw_deg = 1.0;

  auto input = default_input();
  input.fitness_score = 11.0;
  input.consecutive_rejected_updates = 3;
  input.correction_translation_m = 0.05;
  input.correction_yaw_deg = 0.5;

  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.status_level == ll::kMeasurementGateWarn);
  assert(!gate.reject_measurement);
  assert(gate.status_message == "fitness_score_over_threshold_consistency_recovered");
}

void test_rejected_seed_update_marks_rejection_for_prediction_update()
{
  auto params = default_params();
  params.enable_rejected_seed_update = true;
  params.rejected_seed_update_min_rejections = 2;
  params.rejected_seed_update_max_fitness = 12.0;
  params.rejected_seed_update_max_correction_translation_m = 1.0;
  params.rejected_seed_update_max_correction_yaw_deg = 2.0;

  auto input = default_input();
  input.fitness_score = 11.0;
  input.consecutive_rejected_updates = 2;
  input.correction_translation_m = 0.5;
  input.correction_yaw_deg = 1.0;

  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);
  assert(gate.rejected_seed_update_applied);
  assert(gate.status_message == "fitness_score_over_threshold_rejected_seeded");
}

int main()
{
  test_param_and_input_builders_map_fields();
  test_default_ok_gate();
  test_effective_threshold_uses_lowest_active_strict_threshold();
  test_threshold_rejection_messages();
  test_open_loop_and_post_reject_messages();
  test_borderline_seed_gate();
  test_consistency_recovery_clears_rejection();
  test_rejected_seed_update_marks_rejection_for_prediction_update();
  return 0;
}
