#ifndef LIDAR_LOCALIZATION_CONTINUOUS_TIME_DESKEW_POLICY_HPP_
#define LIDAR_LOCALIZATION_CONTINUOUS_TIME_DESKEW_POLICY_HPP_

#include "lidar_localization/scan_preprocessing_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization
{

enum class ContinuousTimeDeskewStatus
{
  kDisabled = 0,
  kEmptyScan,
  kScanTimeNotReady,
  kTimesNotAligned,
  kImuPreintegrationDisabled,
  kImuFallbackMode,
  kWaitingForSmoother,
  kWaitingForNewImu,
  kPredictionNonFinite,
  kReady,
  kApplied,
  kNotApplied,
  kScanNotPrepared,
};

struct ContinuousTimeDeskewDecisionInput
{
  bool enabled{false};
  bool cloud_empty{false};
  ScanTimeRangeStatus scan_time_status{ScanTimeRangeStatus::kNoTimeField};
  bool relative_times_aligned_with_cloud{false};
  std::size_t relative_time_count{0};
  std::size_t cloud_point_count{0};
  bool use_imu_preintegration{false};
  bool imu_preintegration_fallback_mode{false};
  bool smoother_initialized{false};
  bool has_new_imu_samples{false};
  bool prediction_finite{false};
  bool requires_smoother{true};
  bool requires_imu_preintegration{true};
};

struct ContinuousTimeDeskewDecision
{
  ContinuousTimeDeskewStatus status{ContinuousTimeDeskewStatus::kDisabled};
  bool should_apply{false};
};

struct ContinuousTimeDeskewResult
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  bool applied{false};
  std::size_t deskewed_point_count{0};
  std::size_t skipped_invalid_time_count{0};
  std::size_t clamped_time_count{0};
};

inline const char * continuousTimeDeskewStatusMessage(ContinuousTimeDeskewStatus status)
{
  switch (status) {
    case ContinuousTimeDeskewStatus::kDisabled:
      return "continuous_time_deskew_disabled";
    case ContinuousTimeDeskewStatus::kEmptyScan:
      return "continuous_time_deskew_empty_scan";
    case ContinuousTimeDeskewStatus::kScanTimeNotReady:
      return "continuous_time_deskew_scan_time_not_ready";
    case ContinuousTimeDeskewStatus::kTimesNotAligned:
      return "continuous_time_deskew_times_not_aligned";
    case ContinuousTimeDeskewStatus::kImuPreintegrationDisabled:
      return "continuous_time_deskew_imu_preintegration_disabled";
    case ContinuousTimeDeskewStatus::kImuFallbackMode:
      return "continuous_time_deskew_imu_fallback_mode";
    case ContinuousTimeDeskewStatus::kWaitingForSmoother:
      return "continuous_time_deskew_waiting_for_smoother";
    case ContinuousTimeDeskewStatus::kWaitingForNewImu:
      return "continuous_time_deskew_waiting_for_new_imu";
    case ContinuousTimeDeskewStatus::kPredictionNonFinite:
      return "continuous_time_deskew_prediction_non_finite";
    case ContinuousTimeDeskewStatus::kReady:
      return "continuous_time_deskew_ready";
    case ContinuousTimeDeskewStatus::kApplied:
      return "continuous_time_deskew_applied";
    case ContinuousTimeDeskewStatus::kNotApplied:
      return "continuous_time_deskew_not_applied";
    case ContinuousTimeDeskewStatus::kScanNotPrepared:
      return "continuous_time_deskew_scan_not_prepared";
  }
  return "continuous_time_deskew_unknown";
}

inline ContinuousTimeDeskewDecision decideContinuousTimeDeskew(
  const ContinuousTimeDeskewDecisionInput & input)
{
  if (!input.enabled) {
    return {ContinuousTimeDeskewStatus::kDisabled, false};
  }
  if (input.cloud_empty) {
    return {ContinuousTimeDeskewStatus::kEmptyScan, false};
  }
  if (input.scan_time_status != ScanTimeRangeStatus::kReady) {
    return {ContinuousTimeDeskewStatus::kScanTimeNotReady, false};
  }
  if (
    !input.relative_times_aligned_with_cloud ||
    input.relative_time_count != input.cloud_point_count)
  {
    return {ContinuousTimeDeskewStatus::kTimesNotAligned, false};
  }
  if (input.requires_imu_preintegration && !input.use_imu_preintegration) {
    return {ContinuousTimeDeskewStatus::kImuPreintegrationDisabled, false};
  }
  if (input.requires_imu_preintegration && input.imu_preintegration_fallback_mode) {
    return {ContinuousTimeDeskewStatus::kImuFallbackMode, false};
  }
  if (input.requires_smoother && !input.smoother_initialized) {
    return {ContinuousTimeDeskewStatus::kWaitingForSmoother, false};
  }
  if (!input.has_new_imu_samples) {
    return {ContinuousTimeDeskewStatus::kWaitingForNewImu, false};
  }
  if (!input.prediction_finite) {
    return {ContinuousTimeDeskewStatus::kPredictionNonFinite, false};
  }
  return {ContinuousTimeDeskewStatus::kReady, true};
}

inline bool isFiniteMatrix4f(const Eigen::Matrix4f & matrix)
{
  return matrix.allFinite();
}

inline double normalizedScanTime(double relative_time_sec, double scan_duration_sec)
{
  if (
    !std::isfinite(relative_time_sec) ||
    !std::isfinite(scan_duration_sec) ||
    scan_duration_sec <= 0.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return relative_time_sec / scan_duration_sec;
}

inline double clampUnitInterval(double value, bool * clamped = nullptr)
{
  if (clamped) {
    *clamped = value < 0.0 || value > 1.0;
  }
  return std::min(1.0, std::max(0.0, value));
}

inline Eigen::Matrix4f interpolateRelativeMotion(
  const Eigen::Matrix4f & start_to_end_motion,
  double alpha)
{
  const double clamped_alpha = clampUnitInterval(alpha);
  Eigen::Matrix4f interpolated = Eigen::Matrix4f::Identity();
  interpolated.block<3, 1>(0, 3) =
    start_to_end_motion.block<3, 1>(0, 3) * static_cast<float>(clamped_alpha);

  Eigen::Quaternionf q_start = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf q_end(start_to_end_motion.block<3, 3>(0, 0));
  q_end.normalize();
  const Eigen::Quaternionf q_interp =
    q_start.slerp(static_cast<float>(clamped_alpha), q_end).normalized();
  interpolated.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
  return interpolated;
}

inline Eigen::Matrix4f scaleRelativeMotion(
  const Eigen::Matrix4f & motion,
  double scale)
{
  Eigen::Matrix4f scaled = Eigen::Matrix4f::Identity();
  if (!motion.allFinite() || !std::isfinite(scale)) {
    scaled.setConstant(std::numeric_limits<float>::quiet_NaN());
    return scaled;
  }
  scaled.block<3, 1>(0, 3) =
    motion.block<3, 1>(0, 3) * static_cast<float>(scale);
  Eigen::Quaternionf end(motion.block<3, 3>(0, 0));
  end.normalize();
  scaled.block<3, 3>(0, 0) = Eigen::Quaternionf::Identity().slerp(
    static_cast<float>(scale), end).normalized().toRotationMatrix();
  return scaled;
}

inline pcl::PointXYZI transformPointXyzi(
  const pcl::PointXYZI & point,
  const Eigen::Matrix4f & transform)
{
  const Eigen::Vector4f source(point.x, point.y, point.z, 1.0f);
  const Eigen::Vector4f transformed = transform * source;
  pcl::PointXYZI output = point;
  output.x = transformed.x();
  output.y = transformed.y();
  output.z = transformed.z();
  return output;
}

inline ContinuousTimeDeskewResult deskewPointCloudWithRelativeMotion(
  const pcl::PointCloud<pcl::PointXYZI> & input,
  const std::vector<double> & relative_times_sec,
  double scan_duration_sec,
  const Eigen::Matrix4f & start_to_end_motion,
  double reference_relative_time_sec = 0.0)
{
  ContinuousTimeDeskewResult result;
  result.cloud = input;
  if (
    input.empty() ||
    relative_times_sec.size() != input.size() ||
    !std::isfinite(scan_duration_sec) ||
    scan_duration_sec <= 0.0 ||
    !isFiniteMatrix4f(start_to_end_motion))
  {
    return result;
  }

  bool reference_clamped = false;
  const double reference_alpha_raw =
    normalizedScanTime(reference_relative_time_sec, scan_duration_sec);
  if (!std::isfinite(reference_alpha_raw)) {
    return result;
  }
  const double reference_alpha = clampUnitInterval(reference_alpha_raw, &reference_clamped);
  if (reference_clamped) {
    ++result.clamped_time_count;
  }
  const Eigen::Matrix4f start_to_reference =
    interpolateRelativeMotion(start_to_end_motion, reference_alpha);
  const Eigen::Matrix4f reference_to_start = start_to_reference.inverse();

  result.applied = true;
  for (std::size_t point_idx = 0; point_idx < input.size(); ++point_idx) {
    const double alpha_raw =
      normalizedScanTime(relative_times_sec[point_idx], scan_duration_sec);
    if (!std::isfinite(alpha_raw)) {
      ++result.skipped_invalid_time_count;
      continue;
    }
    bool point_clamped = false;
    const double alpha = clampUnitInterval(alpha_raw, &point_clamped);
    if (point_clamped) {
      ++result.clamped_time_count;
    }
    const Eigen::Matrix4f start_to_point =
      interpolateRelativeMotion(start_to_end_motion, alpha);
    const Eigen::Matrix4f reference_to_point = reference_to_start * start_to_point;
    result.cloud[point_idx] = transformPointXyzi(input[point_idx], reference_to_point);
    ++result.deskewed_point_count;
  }
  return result;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_CONTINUOUS_TIME_DESKEW_POLICY_HPP_
