#include "lidar_localization/imu_preintegration_guard_policy.hpp"

#include <cassert>
#include <limits>
#include <string>

namespace ll = lidar_localization;

ll::ImuPreintegrationGuardParams default_params()
{
  ll::ImuPreintegrationGuardParams params;
  params.prediction_correction_guard_translation_m = 2.0;
  params.prediction_correction_guard_yaw_deg = 4.0;
  params.smoother_measurement_translation_guard_m = 2.0;
  params.smoother_measurement_rotation_guard_deg = 10.0;
  return params;
}

void test_prediction_correction_guard_requires_active_prediction()
{
  const auto params = default_params();
  ll::ImuPredictionCorrectionGuardInput input;
  input.fallback_mode = false;
  input.imu_prediction_ready = true;
  input.correction_translation_m = 2.1;
  input.correction_yaw_deg = 0.0;
  assert(ll::isImuPredictionCorrectionGuardTripped(params, input));

  input.fallback_mode = true;
  assert(!ll::isImuPredictionCorrectionGuardTripped(params, input));

  input.fallback_mode = false;
  input.imu_prediction_ready = false;
  assert(!ll::isImuPredictionCorrectionGuardTripped(params, input));
}

void test_prediction_correction_guard_checks_translation_or_yaw()
{
  const auto params = default_params();
  ll::ImuPredictionCorrectionGuardInput input;
  input.imu_prediction_ready = true;
  input.correction_translation_m = 2.0;
  input.correction_yaw_deg = 4.0;
  assert(!ll::isImuPredictionCorrectionGuardTripped(params, input));

  input.correction_translation_m = 2.1;
  assert(ll::isImuPredictionCorrectionGuardTripped(params, input));

  input.correction_translation_m = std::numeric_limits<double>::quiet_NaN();
  input.correction_yaw_deg = 4.1;
  assert(ll::isImuPredictionCorrectionGuardTripped(params, input));

  input.correction_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  assert(!ll::isImuPredictionCorrectionGuardTripped(params, input));
}

void test_smoother_divergence_rejects_non_finite_or_large_delta()
{
  const auto params = default_params();
  ll::ImuSmootherDivergenceInput input;
  input.pose_all_finite = true;
  input.measurement_translation_delta_m = 2.0;
  input.measurement_rotation_delta_deg = 10.0;
  assert(!ll::isImuSmootherDiverged(params, input));

  input.measurement_translation_delta_m = 2.1;
  assert(ll::isImuSmootherDiverged(params, input));

  input.measurement_translation_delta_m = 0.0;
  input.measurement_rotation_delta_deg = 10.1;
  assert(ll::isImuSmootherDiverged(params, input));

  input.measurement_rotation_delta_deg = 0.0;
  input.pose_all_finite = false;
  assert(ll::isImuSmootherDiverged(params, input));

  input.pose_all_finite = true;
  input.measurement_translation_delta_m = std::numeric_limits<double>::quiet_NaN();
  assert(ll::isImuSmootherDiverged(params, input));
}

void test_backend_state_skips_smoother_when_already_in_fallback()
{
  const auto params = default_params();
  ll::ImuPredictionCorrectionGuardInput input;
  input.fallback_mode = true;
  input.imu_prediction_ready = true;
  input.correction_translation_m = 10.0;
  input.correction_yaw_deg = 10.0;

  const auto state = ll::beginImuPreintegrationBackendState(params, input);

  assert(state.fallback_mode);
  assert(!state.state_reset);
  assert(!state.correction_guard_tripped);
  assert(!state.should_update_smoother);
  assert(ll::shouldUseImuMeasurementPose(state));
}

void test_backend_state_enters_fallback_on_prediction_guard()
{
  const auto params = default_params();
  ll::ImuPredictionCorrectionGuardInput input;
  input.imu_prediction_ready = true;
  input.correction_translation_m = 2.1;

  const auto state = ll::beginImuPreintegrationBackendState(params, input);

  assert(state.fallback_mode);
  assert(state.state_reset);
  assert(state.correction_guard_tripped);
  assert(!state.smoother_diverged);
  assert(!state.should_update_smoother);
}

void test_backend_state_enters_fallback_on_smoother_divergence()
{
  const auto params = default_params();
  ll::ImuPredictionCorrectionGuardInput correction_input;
  correction_input.imu_prediction_ready = true;

  auto state = ll::beginImuPreintegrationBackendState(params, correction_input);
  assert(!state.fallback_mode);
  assert(state.should_update_smoother);

  ll::ImuSmootherDivergenceInput divergence_input;
  divergence_input.pose_all_finite = true;
  divergence_input.measurement_translation_delta_m = 2.1;
  divergence_input.measurement_rotation_delta_deg = 0.0;
  state = ll::applyImuSmootherDivergenceDecision(state, params, divergence_input);

  assert(state.fallback_mode);
  assert(state.state_reset);
  assert(!state.correction_guard_tripped);
  assert(state.smoother_diverged);
  assert(!state.should_update_smoother);
}

void test_backend_status_overload_uses_backend_state()
{
  ll::ImuPreintegrationBackendState state;
  state.state_reset = true;
  state.correction_guard_tripped = true;

  auto status = ll::decideImuPreintegrationStatus(state, true);
  assert(status.warning);
  assert(std::string(status.status_message) ==
         "imu_prediction_correction_guard_imu_disabled");

  state.correction_guard_tripped = false;
  state.smoother_diverged = true;
  status = ll::decideImuPreintegrationStatus(state, true);
  assert(status.warning);
  assert(std::string(status.status_message) == "imu_smoother_diverged_imu_disabled");
}

void test_status_decision_prioritizes_reset_reasons()
{
  auto status = ll::decideImuPreintegrationStatus(true, true, true);
  assert(status.warning);
  assert(std::string(status.status_message) ==
         "imu_prediction_correction_guard_imu_disabled");

  status = ll::decideImuPreintegrationStatus(true, false, true);
  assert(status.warning);
  assert(std::string(status.status_message) == "imu_smoother_diverged_imu_disabled");

  status = ll::decideImuPreintegrationStatus(false, false, false);
  assert(status.warning);
  assert(std::string(status.status_message) == "imu_smoother_update_rejected");

  status = ll::decideImuPreintegrationStatus(false, false, true);
  assert(!status.warning);
  assert(std::string(status.status_message) == "ok");
}

int main()
{
  test_prediction_correction_guard_requires_active_prediction();
  test_prediction_correction_guard_checks_translation_or_yaw();
  test_smoother_divergence_rejects_non_finite_or_large_delta();
  test_backend_state_skips_smoother_when_already_in_fallback();
  test_backend_state_enters_fallback_on_prediction_guard();
  test_backend_state_enters_fallback_on_smoother_divergence();
  test_backend_status_overload_uses_backend_state();
  test_status_decision_prioritizes_reset_reasons();
  return 0;
}
