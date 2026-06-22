#include "lidar_localization/deskew_readiness_policy.hpp"

#include <cassert>
#include <string>

namespace ll = lidar_localization;

void test_ready_when_scan_time_and_imu_prediction_are_available()
{
  const auto diagnostics = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kReady,
      ll::ImuPreintegrationDiagnosticStatus::kPredictionAvailable});

  assert(diagnostics.ready);
  assert(diagnostics.status == ll::DeskewReadinessStatus::kReady);
  assert(std::string(ll::deskewReadinessStatusMessage(diagnostics.status)) ==
         "deskew_ready");
}

void test_active_imu_prediction_is_ready()
{
  const auto diagnostics = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kReady,
      ll::ImuPreintegrationDiagnosticStatus::kPredictionActive});

  assert(diagnostics.ready);
  assert(diagnostics.status == ll::DeskewReadinessStatus::kReady);
}

void test_scan_time_blockers_are_reported_before_imu_blockers()
{
  const auto diagnostics = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kNoTimeField,
      ll::ImuPreintegrationDiagnosticStatus::kPredictionActive});

  assert(!diagnostics.ready);
  assert(diagnostics.status == ll::DeskewReadinessStatus::kScanTimeMissing);
  assert(std::string(ll::deskewReadinessStatusMessage(diagnostics.status)) ==
         "deskew_scan_time_field_missing");
}

void test_scan_time_invalid_and_too_large_are_distinct()
{
  const auto invalid = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kInvalid,
      ll::ImuPreintegrationDiagnosticStatus::kPredictionActive});
  assert(!invalid.ready);
  assert(invalid.status == ll::DeskewReadinessStatus::kScanTimeInvalid);

  const auto too_large = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kDurationTooLarge,
      ll::ImuPreintegrationDiagnosticStatus::kPredictionActive});
  assert(!too_large.ready);
  assert(too_large.status == ll::DeskewReadinessStatus::kScanTimeRangeTooLarge);
}

void test_imu_preintegration_disabled_blocks_ready_scan()
{
  const auto diagnostics = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kReady,
      ll::ImuPreintegrationDiagnosticStatus::kDisabled});

  assert(!diagnostics.ready);
  assert(diagnostics.status == ll::DeskewReadinessStatus::kImuPreintegrationDisabled);
  assert(std::string(ll::deskewReadinessStatusMessage(diagnostics.status)) ==
         "deskew_imu_preintegration_disabled");
}

void test_imu_health_blockers_are_explicit()
{
  const auto transform = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kReady,
      ll::ImuPreintegrationDiagnosticStatus::kTransformUnavailable});
  assert(!transform.ready);
  assert(transform.status == ll::DeskewReadinessStatus::kImuTransformUnavailable);

  const auto invalid_dt = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kReady,
      ll::ImuPreintegrationDiagnosticStatus::kInvalidDeltaTime});
  assert(!invalid_dt.ready);
  assert(invalid_dt.status == ll::DeskewReadinessStatus::kImuInvalidDeltaTime);

  const auto stale = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kReady,
      ll::ImuPreintegrationDiagnosticStatus::kStaleImu});
  assert(!stale.ready);
  assert(stale.status == ll::DeskewReadinessStatus::kImuStale);

  const auto window_too_large = ll::makeDeskewReadinessDiagnostics(
    ll::DeskewReadinessInput{
      ll::ScanTimeRangeStatus::kReady,
      ll::ImuPreintegrationDiagnosticStatus::kIntegrationWindowTooLarge});
  assert(!window_too_large.ready);
  assert(
    window_too_large.status ==
    ll::DeskewReadinessStatus::kImuIntegrationWindowTooLarge);
  assert(std::string(ll::deskewReadinessStatusMessage(window_too_large.status)) ==
         "deskew_imu_integration_window_too_large");
}

int main()
{
  test_ready_when_scan_time_and_imu_prediction_are_available();
  test_active_imu_prediction_is_ready();
  test_scan_time_blockers_are_reported_before_imu_blockers();
  test_scan_time_invalid_and_too_large_are_distinct();
  test_imu_preintegration_disabled_blocks_ready_scan();
  test_imu_health_blockers_are_explicit();
  return 0;
}
