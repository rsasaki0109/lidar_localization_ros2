#ifndef LIDAR_LOCALIZATION_IMU_PREINTEGRATION_DIAGNOSTICS_POLICY_HPP_
#define LIDAR_LOCALIZATION_IMU_PREINTEGRATION_DIAGNOSTICS_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace lidar_localization
{

enum class ImuPreintegrationDiagnosticStatus
{
  kDisabled = 0,
  kFallbackMode,
  kWaitingForImu,
  kWaitingForSmootherInitialization,
  kTransformUnavailable,
  kNonFiniteSample,
  kInvalidDeltaTime,
  kIntegrationWindowTooLarge,
  kStaleImu,
  kWaitingForNewImu,
  kPredictionNonFinite,
  kPredictionAvailable,
  kPredictionActive,
};

struct ImuPreintegrationDiagnosticsInput
{
  bool enabled{false};
  bool fallback_mode{false};
  bool smoother_initialized{false};
  bool has_imu_stamp{false};
  bool has_new_samples{false};
  bool prediction_active{false};
  bool prediction_finite{true};
  std::size_t received_sample_count{0};
  std::size_t integrated_sample_count{0};
  std::size_t skipped_sample_count{0};
  std::size_t transform_failure_count{0};
  std::size_t non_finite_sample_count{0};
  std::size_t invalid_dt_count{0};
  double last_dt_sec{std::numeric_limits<double>::quiet_NaN()};
  double last_sample_age_sec{std::numeric_limits<double>::quiet_NaN()};
  double integration_window_sec{0.0};
  double stale_sample_age_threshold_sec{0.2};
  double max_integration_window_sec{1.0};
  bool seed_consistency_gate_enabled{false};
  bool seed_consistency_seed_allowed{false};
  std::size_t seed_consistency_valid_comparison_count{0};
  std::size_t seed_consistency_consecutive_pass_count{0};
  double seed_consistency_translation_error_m{std::numeric_limits<double>::quiet_NaN()};
  double seed_consistency_rotation_error_deg{std::numeric_limits<double>::quiet_NaN()};
  bool seed_consistency_sample_passed{false};
};

struct ImuPreintegrationDiagnostics
{
  ImuPreintegrationDiagnosticStatus status{
    ImuPreintegrationDiagnosticStatus::kDisabled};
  bool enabled{false};
  bool fallback_mode{false};
  bool smoother_initialized{false};
  bool has_new_samples{false};
  std::size_t received_sample_count{0};
  std::size_t integrated_sample_count{0};
  std::size_t skipped_sample_count{0};
  std::size_t transform_failure_count{0};
  std::size_t non_finite_sample_count{0};
  std::size_t invalid_dt_count{0};
  double last_dt_sec{std::numeric_limits<double>::quiet_NaN()};
  double last_sample_age_sec{std::numeric_limits<double>::quiet_NaN()};
  double integration_window_sec{0.0};
  bool seed_consistency_gate_enabled{false};
  bool seed_consistency_seed_allowed{false};
  std::size_t seed_consistency_valid_comparison_count{0};
  std::size_t seed_consistency_consecutive_pass_count{0};
  double seed_consistency_translation_error_m{std::numeric_limits<double>::quiet_NaN()};
  double seed_consistency_rotation_error_deg{std::numeric_limits<double>::quiet_NaN()};
  bool seed_consistency_sample_passed{false};
};

constexpr double kMaximumImuPreintegrationSampleDtSec = 0.5;
// ROS timestamps and callback scheduling can put an otherwise complete scan
// interval a fraction of an IMU tick beyond the nominal one-second guard.
// This tolerance is tiny relative to the guard and does not permit a genuine
// missing scan interval (for example 1.1 s) to seed registration.
constexpr double kImuIntegrationWindowJitterToleranceSec = 0.01;

inline double finiteNonnegativeOrNaN(double value)
{
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(0.0, value);
}

inline double imuLastSampleAgeSec(double status_stamp_sec, double last_imu_stamp_sec)
{
  if (
    !std::isfinite(status_stamp_sec) ||
    !std::isfinite(last_imu_stamp_sec) ||
    last_imu_stamp_sec <= 0.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return finiteNonnegativeOrNaN(status_stamp_sec - last_imu_stamp_sec);
}

inline double imuIntegrationWindowSec(
  double last_imu_stamp_sec,
  double last_scan_stamp_for_imu_sec)
{
  if (
    !std::isfinite(last_imu_stamp_sec) ||
    !std::isfinite(last_scan_stamp_for_imu_sec) ||
    last_imu_stamp_sec <= 0.0 ||
    last_scan_stamp_for_imu_sec <= 0.0)
  {
    return 0.0;
  }
  return std::max(0.0, last_imu_stamp_sec - last_scan_stamp_for_imu_sec);
}

inline double imuStaleSampleAgeThresholdSec(double scan_period_sec)
{
  if (!std::isfinite(scan_period_sec) || scan_period_sec <= 0.0) {
    return 0.2;
  }
  return std::max(0.2, scan_period_sec * 2.0);
}

inline double imuMaximumIntegrationWindowSec(double scan_period_sec)
{
  if (!std::isfinite(scan_period_sec) || scan_period_sec <= 0.0) {
    return 1.0;
  }
  return std::max(1.0, scan_period_sec * 5.0);
}

inline bool isValidImuPreintegrationDt(double dt)
{
  return std::isfinite(dt) &&
         dt > 0.0 &&
         dt < kMaximumImuPreintegrationSampleDtSec;
}

inline bool isImuSampleStale(const ImuPreintegrationDiagnosticsInput & input)
{
  return std::isfinite(input.last_sample_age_sec) &&
         input.stale_sample_age_threshold_sec > 0.0 &&
         input.last_sample_age_sec > input.stale_sample_age_threshold_sec;
}

inline bool isImuIntegrationWindowTooLarge(double integration_window_sec, double max_window_sec)
{
  return std::isfinite(integration_window_sec) &&
         std::isfinite(max_window_sec) &&
         max_window_sec > 0.0 &&
         integration_window_sec >
         max_window_sec + kImuIntegrationWindowJitterToleranceSec;
}

inline bool isImuIntegrationWindowTooLarge(
  const ImuPreintegrationDiagnosticsInput & input)
{
  return isImuIntegrationWindowTooLarge(
    input.integration_window_sec, input.max_integration_window_sec);
}

inline ImuPreintegrationDiagnosticStatus classifyImuPreintegrationDiagnostics(
  const ImuPreintegrationDiagnosticsInput & input)
{
  if (!input.enabled) {
    return ImuPreintegrationDiagnosticStatus::kDisabled;
  }
  if (input.fallback_mode) {
    return ImuPreintegrationDiagnosticStatus::kFallbackMode;
  }
  if (
    input.transform_failure_count > 0 &&
    input.transform_failure_count >= input.received_sample_count)
  {
    return ImuPreintegrationDiagnosticStatus::kTransformUnavailable;
  }
  if (
    input.non_finite_sample_count > 0 &&
    input.non_finite_sample_count >= input.received_sample_count)
  {
    return ImuPreintegrationDiagnosticStatus::kNonFiniteSample;
  }
  if (!input.has_imu_stamp) {
    return ImuPreintegrationDiagnosticStatus::kWaitingForImu;
  }
  if (!input.smoother_initialized) {
    return ImuPreintegrationDiagnosticStatus::kWaitingForSmootherInitialization;
  }
  if (input.invalid_dt_count > 0 && input.integrated_sample_count == 0) {
    return ImuPreintegrationDiagnosticStatus::kInvalidDeltaTime;
  }
  if (isImuIntegrationWindowTooLarge(input)) {
    return ImuPreintegrationDiagnosticStatus::kIntegrationWindowTooLarge;
  }
  if (isImuSampleStale(input)) {
    return ImuPreintegrationDiagnosticStatus::kStaleImu;
  }
  if (!input.has_new_samples) {
    return ImuPreintegrationDiagnosticStatus::kWaitingForNewImu;
  }
  if (!input.prediction_finite) {
    return ImuPreintegrationDiagnosticStatus::kPredictionNonFinite;
  }
  if (input.prediction_active) {
    return ImuPreintegrationDiagnosticStatus::kPredictionActive;
  }
  return ImuPreintegrationDiagnosticStatus::kPredictionAvailable;
}

inline const char * imuPreintegrationDiagnosticStatusMessage(
  ImuPreintegrationDiagnosticStatus status)
{
  switch (status) {
    case ImuPreintegrationDiagnosticStatus::kDisabled:
      return "imu_preintegration_disabled";
    case ImuPreintegrationDiagnosticStatus::kFallbackMode:
      return "imu_preintegration_fallback_mode";
    case ImuPreintegrationDiagnosticStatus::kWaitingForImu:
      return "imu_preintegration_waiting_for_imu";
    case ImuPreintegrationDiagnosticStatus::kWaitingForSmootherInitialization:
      return "imu_preintegration_waiting_for_smoother_initialization";
    case ImuPreintegrationDiagnosticStatus::kTransformUnavailable:
      return "imu_preintegration_transform_unavailable";
    case ImuPreintegrationDiagnosticStatus::kNonFiniteSample:
      return "imu_preintegration_non_finite_sample";
    case ImuPreintegrationDiagnosticStatus::kInvalidDeltaTime:
      return "imu_preintegration_invalid_delta_time";
    case ImuPreintegrationDiagnosticStatus::kIntegrationWindowTooLarge:
      return "imu_preintegration_integration_window_too_large";
    case ImuPreintegrationDiagnosticStatus::kStaleImu:
      return "imu_preintegration_stale_imu";
    case ImuPreintegrationDiagnosticStatus::kWaitingForNewImu:
      return "imu_preintegration_waiting_for_new_imu";
    case ImuPreintegrationDiagnosticStatus::kPredictionNonFinite:
      return "imu_preintegration_prediction_non_finite";
    case ImuPreintegrationDiagnosticStatus::kPredictionAvailable:
      return "imu_preintegration_prediction_available";
    case ImuPreintegrationDiagnosticStatus::kPredictionActive:
      return "imu_preintegration_prediction_active";
  }
  return "imu_preintegration_unknown";
}

inline ImuPreintegrationDiagnostics makeImuPreintegrationDiagnostics(
  const ImuPreintegrationDiagnosticsInput & input)
{
  return {
    classifyImuPreintegrationDiagnostics(input),
    input.enabled,
    input.fallback_mode,
    input.smoother_initialized,
    input.has_new_samples,
    input.received_sample_count,
    input.integrated_sample_count,
    input.skipped_sample_count,
    input.transform_failure_count,
    input.non_finite_sample_count,
    input.invalid_dt_count,
    input.last_dt_sec,
    input.last_sample_age_sec,
    input.integration_window_sec,
    input.seed_consistency_gate_enabled,
    input.seed_consistency_seed_allowed,
    input.seed_consistency_valid_comparison_count,
    input.seed_consistency_consecutive_pass_count,
    input.seed_consistency_translation_error_m,
    input.seed_consistency_rotation_error_deg,
    input.seed_consistency_sample_passed};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_IMU_PREINTEGRATION_DIAGNOSTICS_POLICY_HPP_
