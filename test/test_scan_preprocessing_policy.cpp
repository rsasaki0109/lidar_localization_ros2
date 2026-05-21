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
  test_scan_preparation_status_priority_and_actions();
  test_range_is_horizontal_and_exclusive();
  test_non_finite_points_are_rejected();
  return 0;
}
