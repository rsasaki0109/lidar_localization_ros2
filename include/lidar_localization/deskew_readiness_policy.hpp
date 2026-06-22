#ifndef LIDAR_LOCALIZATION_DESKEW_READINESS_POLICY_HPP_
#define LIDAR_LOCALIZATION_DESKEW_READINESS_POLICY_HPP_

#include "lidar_localization/imu_preintegration_diagnostics_policy.hpp"
#include "lidar_localization/scan_preprocessing_policy.hpp"

namespace lidar_localization
{

enum class DeskewReadinessStatus
{
  kReady = 0,
  kScanTimeMissing,
  kScanTimeInvalid,
  kScanTimeRangeTooLarge,
  kImuPreintegrationDisabled,
  kImuPreintegrationFallbackMode,
  kWaitingForImu,
  kWaitingForSmootherInitialization,
  kImuTransformUnavailable,
  kImuNonFiniteSample,
  kImuInvalidDeltaTime,
  kImuIntegrationWindowTooLarge,
  kImuStale,
  kWaitingForNewImu,
  kImuPredictionNonFinite,
};

struct DeskewReadinessInput
{
  ScanTimeRangeStatus scan_time_status{ScanTimeRangeStatus::kNoTimeField};
  ImuPreintegrationDiagnosticStatus imu_status{
    ImuPreintegrationDiagnosticStatus::kDisabled};
};

struct DeskewReadinessDiagnostics
{
  DeskewReadinessStatus status{DeskewReadinessStatus::kScanTimeMissing};
  bool ready{false};
};

inline DeskewReadinessStatus classifyDeskewReadiness(
  const DeskewReadinessInput & input)
{
  switch (input.scan_time_status) {
    case ScanTimeRangeStatus::kNoTimeField:
      return DeskewReadinessStatus::kScanTimeMissing;
    case ScanTimeRangeStatus::kInvalid:
      return DeskewReadinessStatus::kScanTimeInvalid;
    case ScanTimeRangeStatus::kDurationTooLarge:
      return DeskewReadinessStatus::kScanTimeRangeTooLarge;
    case ScanTimeRangeStatus::kReady:
      break;
  }

  switch (input.imu_status) {
    case ImuPreintegrationDiagnosticStatus::kDisabled:
      return DeskewReadinessStatus::kImuPreintegrationDisabled;
    case ImuPreintegrationDiagnosticStatus::kFallbackMode:
      return DeskewReadinessStatus::kImuPreintegrationFallbackMode;
    case ImuPreintegrationDiagnosticStatus::kWaitingForImu:
      return DeskewReadinessStatus::kWaitingForImu;
    case ImuPreintegrationDiagnosticStatus::kWaitingForSmootherInitialization:
      return DeskewReadinessStatus::kWaitingForSmootherInitialization;
    case ImuPreintegrationDiagnosticStatus::kTransformUnavailable:
      return DeskewReadinessStatus::kImuTransformUnavailable;
    case ImuPreintegrationDiagnosticStatus::kNonFiniteSample:
      return DeskewReadinessStatus::kImuNonFiniteSample;
    case ImuPreintegrationDiagnosticStatus::kInvalidDeltaTime:
      return DeskewReadinessStatus::kImuInvalidDeltaTime;
    case ImuPreintegrationDiagnosticStatus::kIntegrationWindowTooLarge:
      return DeskewReadinessStatus::kImuIntegrationWindowTooLarge;
    case ImuPreintegrationDiagnosticStatus::kStaleImu:
      return DeskewReadinessStatus::kImuStale;
    case ImuPreintegrationDiagnosticStatus::kWaitingForNewImu:
      return DeskewReadinessStatus::kWaitingForNewImu;
    case ImuPreintegrationDiagnosticStatus::kPredictionNonFinite:
      return DeskewReadinessStatus::kImuPredictionNonFinite;
    case ImuPreintegrationDiagnosticStatus::kPredictionAvailable:
    case ImuPreintegrationDiagnosticStatus::kPredictionActive:
      return DeskewReadinessStatus::kReady;
  }

  return DeskewReadinessStatus::kScanTimeMissing;
}

inline bool isDeskewReady(DeskewReadinessStatus status)
{
  return status == DeskewReadinessStatus::kReady;
}

inline const char * deskewReadinessStatusMessage(DeskewReadinessStatus status)
{
  switch (status) {
    case DeskewReadinessStatus::kReady:
      return "deskew_ready";
    case DeskewReadinessStatus::kScanTimeMissing:
      return "deskew_scan_time_field_missing";
    case DeskewReadinessStatus::kScanTimeInvalid:
      return "deskew_scan_time_field_invalid";
    case DeskewReadinessStatus::kScanTimeRangeTooLarge:
      return "deskew_scan_time_range_too_large";
    case DeskewReadinessStatus::kImuPreintegrationDisabled:
      return "deskew_imu_preintegration_disabled";
    case DeskewReadinessStatus::kImuPreintegrationFallbackMode:
      return "deskew_imu_preintegration_fallback_mode";
    case DeskewReadinessStatus::kWaitingForImu:
      return "deskew_waiting_for_imu";
    case DeskewReadinessStatus::kWaitingForSmootherInitialization:
      return "deskew_waiting_for_smoother_initialization";
    case DeskewReadinessStatus::kImuTransformUnavailable:
      return "deskew_imu_transform_unavailable";
    case DeskewReadinessStatus::kImuNonFiniteSample:
      return "deskew_imu_non_finite_sample";
    case DeskewReadinessStatus::kImuInvalidDeltaTime:
      return "deskew_imu_invalid_delta_time";
    case DeskewReadinessStatus::kImuIntegrationWindowTooLarge:
      return "deskew_imu_integration_window_too_large";
    case DeskewReadinessStatus::kImuStale:
      return "deskew_imu_stale";
    case DeskewReadinessStatus::kWaitingForNewImu:
      return "deskew_waiting_for_new_imu";
    case DeskewReadinessStatus::kImuPredictionNonFinite:
      return "deskew_imu_prediction_non_finite";
  }
  return "deskew_unknown";
}

inline DeskewReadinessDiagnostics makeDeskewReadinessDiagnostics(
  const DeskewReadinessInput & input)
{
  const auto status = classifyDeskewReadiness(input);
  return {status, isDeskewReady(status)};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_DESKEW_READINESS_POLICY_HPP_
