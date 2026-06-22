#include "lidar_localization/imu_preintegration_diagnostics_policy.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace ll = lidar_localization;

void test_disabled_status()
{
  ll::ImuPreintegrationDiagnosticsInput input;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kDisabled);
  assert(std::string(ll::imuPreintegrationDiagnosticStatusMessage(diagnostics.status)) ==
         "imu_preintegration_disabled");
  assert(!diagnostics.enabled);
}

void test_fallback_status_has_priority_after_enabled()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.fallback_mode = true;
  input.has_imu_stamp = true;
  input.smoother_initialized = true;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kFallbackMode);
  assert(diagnostics.fallback_mode);
}

void test_waiting_for_imu_before_smoother_initialization()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kWaitingForImu);
}

void test_waiting_for_smoother_initialization()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.has_imu_stamp = true;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(
    diagnostics.status ==
    ll::ImuPreintegrationDiagnosticStatus::kWaitingForSmootherInitialization);
}

void test_transform_unavailable_when_all_samples_fail_transform()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.received_sample_count = 3;
  input.transform_failure_count = 3;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kTransformUnavailable);
  assert(diagnostics.received_sample_count == 3);
  assert(diagnostics.transform_failure_count == 3);
}

void test_non_finite_sample_when_all_samples_are_invalid()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.received_sample_count = 2;
  input.non_finite_sample_count = 2;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kNonFiniteSample);
  assert(diagnostics.non_finite_sample_count == 2);
}

void test_invalid_dt_when_no_samples_integrated()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.has_imu_stamp = true;
  input.smoother_initialized = true;
  input.has_new_samples = true;
  input.received_sample_count = 2;
  input.invalid_dt_count = 2;
  input.last_dt_sec = -0.01;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kInvalidDeltaTime);
  assert(diagnostics.invalid_dt_count == 2);
  assert(diagnostics.last_dt_sec == -0.01);
}

void test_stale_imu_overrides_waiting_for_new_samples()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.has_imu_stamp = true;
  input.smoother_initialized = true;
  input.has_new_samples = false;
  input.last_sample_age_sec = 0.5;
  input.stale_sample_age_threshold_sec = 0.2;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kStaleImu);
  assert(diagnostics.last_sample_age_sec == 0.5);
}

void test_integration_window_too_large_overrides_prediction_available()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.has_imu_stamp = true;
  input.smoother_initialized = true;
  input.has_new_samples = true;
  input.integration_window_sec = 1.2;
  input.max_integration_window_sec = 1.0;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(
    diagnostics.status ==
    ll::ImuPreintegrationDiagnosticStatus::kIntegrationWindowTooLarge);
  assert(std::string(ll::imuPreintegrationDiagnosticStatusMessage(diagnostics.status)) ==
         "imu_preintegration_integration_window_too_large");
  assert(diagnostics.has_new_samples);
}

void test_waiting_for_new_imu_when_recent_but_no_new_samples()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.has_imu_stamp = true;
  input.smoother_initialized = true;
  input.has_new_samples = false;
  input.last_sample_age_sec = 0.01;
  input.stale_sample_age_threshold_sec = 0.2;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kWaitingForNewImu);
}

void test_prediction_non_finite()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.has_imu_stamp = true;
  input.smoother_initialized = true;
  input.has_new_samples = true;
  input.prediction_finite = false;

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kPredictionNonFinite);
}

void test_prediction_active_and_helpers()
{
  ll::ImuPreintegrationDiagnosticsInput input;
  input.enabled = true;
  input.has_imu_stamp = true;
  input.smoother_initialized = true;
  input.has_new_samples = true;
  input.prediction_active = true;
  input.received_sample_count = 20;
  input.integrated_sample_count = 19;
  input.skipped_sample_count = 1;
  input.invalid_dt_count = 1;
  input.last_dt_sec = 0.005;
  input.last_sample_age_sec = ll::imuLastSampleAgeSec(10.0, 9.9);
  input.integration_window_sec = ll::imuIntegrationWindowSec(9.9, 9.8);

  const auto diagnostics = ll::makeImuPreintegrationDiagnostics(input);

  assert(diagnostics.status == ll::ImuPreintegrationDiagnosticStatus::kPredictionActive);
  assert(diagnostics.has_new_samples);
  assert(diagnostics.received_sample_count == 20);
  assert(diagnostics.integrated_sample_count == 19);
  assert(diagnostics.skipped_sample_count == 1);
  assert(diagnostics.invalid_dt_count == 1);
  assert(diagnostics.last_dt_sec == 0.005);
  assert(std::abs(diagnostics.last_sample_age_sec - 0.1) < 1e-9);
  assert(std::abs(diagnostics.integration_window_sec - 0.1) < 1e-9);
  assert(ll::imuIntegrationWindowSec(9.9, 10.0) == 0.0);
  assert(std::isnan(ll::imuLastSampleAgeSec(10.0, 0.0)));
  assert(ll::imuStaleSampleAgeThresholdSec(0.05) == 0.2);
  assert(std::abs(ll::imuStaleSampleAgeThresholdSec(0.2) - 0.4) < 1e-9);
  assert(ll::imuMaximumIntegrationWindowSec(0.05) == 1.0);
  assert(ll::imuMaximumIntegrationWindowSec(0.2) == 1.0);
  assert(std::abs(ll::imuMaximumIntegrationWindowSec(0.3) - 1.5) < 1e-9);
  assert(ll::isImuIntegrationWindowTooLarge(1.01, 1.0));
  assert(!ll::isImuIntegrationWindowTooLarge(1.0, 1.0));
  assert(ll::isValidImuPreintegrationDt(0.001));
  assert(!ll::isValidImuPreintegrationDt(0.0));
  assert(!ll::isValidImuPreintegrationDt(0.5));
}

int main()
{
  test_disabled_status();
  test_fallback_status_has_priority_after_enabled();
  test_waiting_for_imu_before_smoother_initialization();
  test_waiting_for_smoother_initialization();
  test_transform_unavailable_when_all_samples_fail_transform();
  test_non_finite_sample_when_all_samples_are_invalid();
  test_invalid_dt_when_no_samples_integrated();
  test_stale_imu_overrides_waiting_for_new_samples();
  test_integration_window_too_large_overrides_prediction_available();
  test_waiting_for_new_imu_when_recent_but_no_new_samples();
  test_prediction_non_finite();
  test_prediction_active_and_helpers();
  return 0;
}
