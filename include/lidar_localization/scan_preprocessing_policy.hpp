#ifndef LIDAR_LOCALIZATION_SCAN_PREPROCESSING_POLICY_HPP_
#define LIDAR_LOCALIZATION_SCAN_PREPROCESSING_POLICY_HPP_

#include <cmath>
#include <string>

namespace lidar_localization
{

struct ScanPreprocessingPathInput
{
  bool enable_scan_voxel_filter{true};
  bool use_imu_undistortion{false};
  std::string scan_frame_id;
  std::string base_frame_id;
};

struct ScanXyzFieldAvailability
{
  bool has_x{false};
  bool has_y{false};
  bool has_z{false};
};

enum class ScanPreparationStatus
{
  kReady = 0,
  kMissingXyzField,
  kTransformUnavailable,
  kFilteredScanEmpty,
};

inline bool shouldUseDirectRangeFilter(const ScanPreprocessingPathInput & input)
{
  return !input.enable_scan_voxel_filter &&
         !input.use_imu_undistortion &&
         input.scan_frame_id == input.base_frame_id;
}

inline bool hasRequiredXyzFields(const ScanXyzFieldAvailability & fields)
{
  return fields.has_x && fields.has_y && fields.has_z;
}

inline ScanPreparationStatus classifyPreparedScan(
  bool has_required_xyz_fields,
  bool transform_available,
  bool filtered_scan_empty)
{
  if (!has_required_xyz_fields) {
    return ScanPreparationStatus::kMissingXyzField;
  }
  if (!transform_available) {
    return ScanPreparationStatus::kTransformUnavailable;
  }
  if (filtered_scan_empty) {
    return ScanPreparationStatus::kFilteredScanEmpty;
  }
  return ScanPreparationStatus::kReady;
}

inline bool isPreparedScanReady(ScanPreparationStatus status)
{
  return status == ScanPreparationStatus::kReady;
}

inline bool shouldAdvancePredictionAfterScanPreparationFailure(ScanPreparationStatus status)
{
  return status == ScanPreparationStatus::kMissingXyzField ||
         status == ScanPreparationStatus::kFilteredScanEmpty;
}

inline const char * scanPreparationStatusMessage(ScanPreparationStatus status)
{
  switch (status) {
    case ScanPreparationStatus::kReady:
      return "ok";
    case ScanPreparationStatus::kMissingXyzField:
      return "scan_missing_xyz_field";
    case ScanPreparationStatus::kTransformUnavailable:
      return "scan_transform_unavailable";
    case ScanPreparationStatus::kFilteredScanEmpty:
      return "filtered_scan_empty";
  }
  return "unknown_scan_preparation_status";
}

inline bool isFinitePoint(float x, float y, float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline double horizontalRange(float x, float y)
{
  return std::hypot(static_cast<double>(x), static_cast<double>(y));
}

inline bool isPointInScanRange(
  float x,
  float y,
  float z,
  double scan_min_range,
  double scan_max_range)
{
  if (!isFinitePoint(x, y, z)) {
    return false;
  }
  const double range = horizontalRange(x, y);
  return scan_min_range < range && range < scan_max_range;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_SCAN_PREPROCESSING_POLICY_HPP_
