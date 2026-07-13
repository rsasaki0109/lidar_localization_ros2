#ifndef LIDAR_LOCALIZATION_IMU_POSE_HISTORY_DESKEW_HPP_
#define LIDAR_LOCALIZATION_IMU_POSE_HISTORY_DESKEW_HPP_

#include "lidar_localization/continuous_time_deskew_policy.hpp"

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

struct TimestampedPose
{
  double stamp_sec{0.0};
  Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
};

enum class PoseHistoryDeskewStatus
{
  kReady = 0,
  kEmptyHistory,
  kInvalidInterval,
  kHistoryNotOrdered,
  kHistoryStartsTooLate,
  kHistoryEndsTooEarly,
  kReferenceOutsideScan,
  kInvalidPose,
  kNotApplied,
  kApplied,
};

inline const char * poseHistoryDeskewStatusMessage(PoseHistoryDeskewStatus status)
{
  switch (status) {
    case PoseHistoryDeskewStatus::kReady:
      return "continuous_time_deskew_pose_history_ready";
    case PoseHistoryDeskewStatus::kEmptyHistory:
      return "continuous_time_deskew_pose_history_empty";
    case PoseHistoryDeskewStatus::kInvalidInterval:
      return "continuous_time_deskew_pose_history_invalid_interval";
    case PoseHistoryDeskewStatus::kHistoryNotOrdered:
      return "continuous_time_deskew_pose_history_not_ordered";
    case PoseHistoryDeskewStatus::kHistoryStartsTooLate:
      return "continuous_time_deskew_pose_history_starts_too_late";
    case PoseHistoryDeskewStatus::kHistoryEndsTooEarly:
      return "continuous_time_deskew_pose_history_ends_too_early";
    case PoseHistoryDeskewStatus::kReferenceOutsideScan:
      return "continuous_time_deskew_reference_outside_scan";
    case PoseHistoryDeskewStatus::kInvalidPose:
      return "continuous_time_deskew_pose_history_invalid_pose";
    case PoseHistoryDeskewStatus::kNotApplied:
      return "continuous_time_deskew_pose_history_not_applied";
    case PoseHistoryDeskewStatus::kApplied:
      return "continuous_time_deskew_pose_history_applied";
  }
  return "continuous_time_deskew_pose_history_unknown";
}

struct PoseHistoryCoverage
{
  PoseHistoryDeskewStatus status{PoseHistoryDeskewStatus::kEmptyHistory};
  bool full_coverage{false};
  double ratio{0.0};
};

inline PoseHistoryCoverage evaluatePoseHistoryCoverage(
  const std::vector<TimestampedPose> & history,
  double scan_start_sec,
  double scan_end_sec)
{
  PoseHistoryCoverage result;
  if (history.empty()) {
    return result;
  }
  if (
    !std::isfinite(scan_start_sec) || !std::isfinite(scan_end_sec) ||
    scan_end_sec <= scan_start_sec)
  {
    result.status = PoseHistoryDeskewStatus::kInvalidInterval;
    return result;
  }
  for (std::size_t i = 0; i < history.size(); ++i) {
    if (!std::isfinite(history[i].stamp_sec) || !history[i].pose.allFinite()) {
      result.status = PoseHistoryDeskewStatus::kInvalidPose;
      return result;
    }
    if (i > 0 && history[i].stamp_sec <= history[i - 1].stamp_sec) {
      result.status = PoseHistoryDeskewStatus::kHistoryNotOrdered;
      return result;
    }
  }

  const double overlap_start = std::max(scan_start_sec, history.front().stamp_sec);
  const double overlap_end = std::min(scan_end_sec, history.back().stamp_sec);
  result.ratio = std::max(0.0, overlap_end - overlap_start) /
    (scan_end_sec - scan_start_sec);
  result.ratio = std::min(1.0, result.ratio);
  if (history.front().stamp_sec > scan_start_sec) {
    result.status = PoseHistoryDeskewStatus::kHistoryStartsTooLate;
    return result;
  }
  if (history.back().stamp_sec < scan_end_sec) {
    result.status = PoseHistoryDeskewStatus::kHistoryEndsTooEarly;
    return result;
  }
  result.status = PoseHistoryDeskewStatus::kReady;
  result.full_coverage = true;
  result.ratio = 1.0;
  return result;
}

inline bool interpolatePoseHistory(
  const std::vector<TimestampedPose> & history,
  double stamp_sec,
  Eigen::Matrix4f * pose)
{
  if (!pose || history.empty() || !std::isfinite(stamp_sec) ||
    stamp_sec < history.front().stamp_sec || stamp_sec > history.back().stamp_sec)
  {
    return false;
  }
  const auto upper = std::lower_bound(
    history.begin(), history.end(), stamp_sec,
    [](const TimestampedPose & sample, double stamp) {
      return sample.stamp_sec < stamp;
    });
  if (upper == history.begin()) {
    *pose = upper->pose;
    return pose->allFinite();
  }
  if (upper == history.end()) {
    *pose = history.back().pose;
    return pose->allFinite();
  }
  if (upper->stamp_sec == stamp_sec) {
    *pose = upper->pose;
    return pose->allFinite();
  }

  const auto lower = std::prev(upper);
  const double interval_sec = upper->stamp_sec - lower->stamp_sec;
  if (!std::isfinite(interval_sec) || interval_sec <= 0.0) {
    return false;
  }
  const float alpha = static_cast<float>((stamp_sec - lower->stamp_sec) / interval_sec);
  Eigen::Quaternionf q0(lower->pose.block<3, 3>(0, 0));
  Eigen::Quaternionf q1(upper->pose.block<3, 3>(0, 0));
  if (!q0.coeffs().allFinite() || !q1.coeffs().allFinite()) {
    return false;
  }
  q0.normalize();
  q1.normalize();
  pose->setIdentity();
  pose->block<3, 3>(0, 0) = q0.slerp(alpha, q1).normalized().toRotationMatrix();
  pose->block<3, 1>(0, 3) =
    (1.0f - alpha) * lower->pose.block<3, 1>(0, 3) +
    alpha * upper->pose.block<3, 1>(0, 3);
  return pose->allFinite();
}

struct PoseHistoryDeskewResult
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  PoseHistoryDeskewStatus status{PoseHistoryDeskewStatus::kNotApplied};
  bool applied{false};
  double coverage_ratio{0.0};
  std::size_t deskewed_point_count{0};
  std::size_t skipped_invalid_time_count{0};
};

inline PoseHistoryDeskewResult deskewPointCloudWithPoseHistory(
  const pcl::PointCloud<pcl::PointXYZI> & input,
  const std::vector<double> & relative_times_sec,
  double scan_start_sec,
  double scan_duration_sec,
  const std::vector<TimestampedPose> & history,
  double reference_relative_time_sec = 0.0)
{
  PoseHistoryDeskewResult result;
  result.cloud = input;
  if (
    input.empty() || relative_times_sec.size() != input.size() ||
    !std::isfinite(scan_start_sec) || !std::isfinite(scan_duration_sec) ||
    scan_duration_sec <= 0.0)
  {
    result.status = PoseHistoryDeskewStatus::kInvalidInterval;
    return result;
  }
  if (
    !std::isfinite(reference_relative_time_sec) || reference_relative_time_sec < 0.0 ||
    reference_relative_time_sec > scan_duration_sec)
  {
    result.status = PoseHistoryDeskewStatus::kReferenceOutsideScan;
    return result;
  }

  const double scan_end_sec = scan_start_sec + scan_duration_sec;
  const auto coverage = evaluatePoseHistoryCoverage(history, scan_start_sec, scan_end_sec);
  result.coverage_ratio = coverage.ratio;
  if (!coverage.full_coverage) {
    result.status = coverage.status;
    return result;
  }

  Eigen::Matrix4f reference_pose = Eigen::Matrix4f::Identity();
  if (!interpolatePoseHistory(
      history, scan_start_sec + reference_relative_time_sec, &reference_pose))
  {
    result.status = PoseHistoryDeskewStatus::kInvalidPose;
    return result;
  }
  const Eigen::Matrix4f reference_inverse = reference_pose.inverse();
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (!std::isfinite(relative_times_sec[i])) {
      ++result.skipped_invalid_time_count;
      continue;
    }
    const double point_stamp_sec = scan_start_sec + relative_times_sec[i];
    Eigen::Matrix4f point_pose = Eigen::Matrix4f::Identity();
    if (!interpolatePoseHistory(history, point_stamp_sec, &point_pose)) {
      result.status = PoseHistoryDeskewStatus::kInvalidPose;
      return result;
    }
    result.cloud[i] = transformPointXyzi(input[i], reference_inverse * point_pose);
    ++result.deskewed_point_count;
  }
  result.status = PoseHistoryDeskewStatus::kApplied;
  result.applied = true;
  return result;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_IMU_POSE_HISTORY_DESKEW_HPP_
