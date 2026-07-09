#include "lidar_localization/scan_preprocessing_policy.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace ll = lidar_localization;

void test_direct_range_filter_path_requires_no_voxel_no_imu_and_same_frame()
{
  assert(ll::shouldUseDirectRangeFilter({false, false, "base_link", "base_link"}));
  assert(!ll::shouldUseDirectRangeFilter({true, false, "base_link", "base_link"}));
  assert(!ll::shouldUseDirectRangeFilter({false, true, "base_link", "base_link"}));
  assert(!ll::shouldUseDirectRangeFilter({false, false, "livox_frame", "base_link"}));
}

void test_scan_xyz_field_availability()
{
  assert(ll::hasRequiredXyzFields({true, true, true}));
  assert(!ll::hasRequiredXyzFields({false, true, true}));
  assert(!ll::hasRequiredXyzFields({true, false, true}));
  assert(!ll::hasRequiredXyzFields({true, true, false}));
}

void test_scan_time_range_statuses_are_explicit()
{
  assert(
    ll::classifyScanTimeRange({false, false, 0.0, 0, 0, 0.1, 2.0}) ==
    ll::ScanTimeRangeStatus::kNoTimeField);
  assert(
    ll::classifyScanTimeRange({true, false, 0.0, 0, 10, 0.1, 2.0}) ==
    ll::ScanTimeRangeStatus::kInvalid);
  assert(
    ll::classifyScanTimeRange({true, true, 0.25, 100, 0, 0.1, 2.0}) ==
    ll::ScanTimeRangeStatus::kDurationTooLarge);
  assert(
    ll::classifyScanTimeRange({true, true, 0.08, 100, 0, 0.1, 2.0}) ==
    ll::ScanTimeRangeStatus::kReady);

  assert(ll::isScanTimeRangeReady(ll::ScanTimeRangeStatus::kReady));
  assert(!ll::isScanTimeRangeReady(ll::ScanTimeRangeStatus::kInvalid));
  assert(
    std::string(ll::scanTimeRangeStatusMessage(
      ll::ScanTimeRangeStatus::kDurationTooLarge)) == "scan_time_range_too_large");
}

void test_scan_time_range_ratio_is_configurable()
{
  // Real Livox multi-frame spans measured on Koide outdoor_hard_01a: the driver
  // occasionally fuses two or three 0.1s integration windows into one message
  // (~0.2005s double, up to ~0.3003s triple). With scan_period 0.1 these trip
  // the default 2.0 ratio and must pass the widened preset ratio (4.0) so
  // continuous-time deskew is not dropped for these frames.
  const double kLivoxDoubleFrameSpanSec = 0.2005;
  const double kLivoxTripleFrameSpanSec = 0.3003;
  for (double span : {kLivoxDoubleFrameSpanSec, kLivoxTripleFrameSpanSec}) {
    assert(
      ll::classifyScanTimeRange({true, true, span, 40000, 0, 0.1, 2.0}) ==
      ll::ScanTimeRangeStatus::kDurationTooLarge);
    assert(
      ll::classifyScanTimeRange({true, true, span, 40000, 0, 0.1, 4.0}) ==
      ll::ScanTimeRangeStatus::kReady);
  }
  // A genuine unit-misparse (e.g. nanoseconds read as seconds) is still rejected
  // even at the widened ratio.
  assert(
    ll::classifyScanTimeRange({true, true, 2.0e5, 40000, 0, 0.1, 4.0}) ==
    ll::ScanTimeRangeStatus::kDurationTooLarge);
}

void test_scan_preparation_status_priority_and_actions()
{
  assert(
    ll::classifyPreparedScan(false, false, true) ==
    ll::ScanPreparationStatus::kMissingXyzField);
  assert(
    ll::classifyPreparedScan(true, false, true) ==
    ll::ScanPreparationStatus::kTransformUnavailable);
  assert(
    ll::classifyPreparedScan(true, true, true) ==
    ll::ScanPreparationStatus::kFilteredScanEmpty);
  assert(ll::classifyPreparedScan(true, true, false) == ll::ScanPreparationStatus::kReady);

  assert(!ll::isPreparedScanReady(ll::ScanPreparationStatus::kFilteredScanEmpty));
  assert(ll::isPreparedScanReady(ll::ScanPreparationStatus::kReady));
  assert(ll::shouldAdvancePredictionAfterScanPreparationFailure(
    ll::ScanPreparationStatus::kMissingXyzField));
  assert(ll::shouldAdvancePredictionAfterScanPreparationFailure(
    ll::ScanPreparationStatus::kFilteredScanEmpty));
  assert(!ll::shouldAdvancePredictionAfterScanPreparationFailure(
    ll::ScanPreparationStatus::kTransformUnavailable));
  assert(
    std::string(ll::scanPreparationStatusMessage(
      ll::ScanPreparationStatus::kMissingXyzField)) == "scan_missing_xyz_field");
}

void test_range_is_horizontal_and_exclusive()
{
  assert(std::abs(ll::horizontalRange(3.0f, 4.0f) - 5.0) < 1.0e-9);
  assert(ll::isPointInScanRange(3.0f, 4.0f, 100.0f, 1.0, 6.0));
  assert(!ll::isPointInScanRange(3.0f, 4.0f, 0.0f, 5.0, 6.0));
  assert(!ll::isPointInScanRange(3.0f, 4.0f, 0.0f, 1.0, 5.0));
}

void test_non_finite_points_are_rejected()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  assert(!ll::isPointInScanRange(nan, 0.0f, 0.0f, 0.0, 10.0));
  assert(!ll::isPointInScanRange(0.0f, inf, 0.0f, 0.0, 10.0));
  assert(!ll::isPointInScanRange(3.0f, 4.0f, nan, 0.0, 10.0));
}

int main()
{
  test_direct_range_filter_path_requires_no_voxel_no_imu_and_same_frame();
  test_scan_xyz_field_availability();
  test_scan_time_range_statuses_are_explicit();
  test_scan_time_range_ratio_is_configurable();
  test_scan_preparation_status_priority_and_actions();
  test_range_is_horizontal_and_exclusive();
  test_non_finite_points_are_rejected();
  return 0;
}
