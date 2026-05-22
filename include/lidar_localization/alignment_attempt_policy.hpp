#ifndef LIDAR_LOCALIZATION_ALIGNMENT_ATTEMPT_POLICY_HPP_
#define LIDAR_LOCALIZATION_ALIGNMENT_ATTEMPT_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace lidar_localization
{

struct AlignmentSeedMetrics
{
  double translation_since_accept_m{std::numeric_limits<double>::quiet_NaN()};
  double yaw_since_accept_deg{std::numeric_limits<double>::quiet_NaN()};
  double accepted_gap_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct AlignmentCorrectionMetrics
{
  double translation_m{std::numeric_limits<double>::quiet_NaN()};
  double yaw_deg{std::numeric_limits<double>::quiet_NaN()};
};

inline double poseDeltaTranslationNormM(const Eigen::Matrix4f & delta_matrix)
{
  return static_cast<double>(delta_matrix.block<3, 1>(0, 3).norm());
}

inline double poseDeltaYawAbsDeg(const Eigen::Matrix4f & delta_matrix)
{
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  return std::abs(
    std::atan2(
      static_cast<double>(delta_matrix(1, 0)),
      static_cast<double>(delta_matrix(0, 0)))) * kRadiansToDegrees;
}

inline AlignmentSeedMetrics computeAlignmentSeedMetrics(
  bool have_last_accepted_pose,
  const Eigen::Matrix4f & last_accepted_pose_matrix,
  const Eigen::Matrix4f & init_guess,
  double scan_stamp_sec,
  double last_accepted_pose_time_sec)
{
  AlignmentSeedMetrics metrics;
  if (!have_last_accepted_pose) {
    return metrics;
  }

  const Eigen::Matrix4f seed_delta_matrix =
    last_accepted_pose_matrix.inverse() * init_guess;
  metrics.translation_since_accept_m = poseDeltaTranslationNormM(seed_delta_matrix);
  metrics.yaw_since_accept_deg = poseDeltaYawAbsDeg(seed_delta_matrix);
  metrics.accepted_gap_sec = std::max(0.0, scan_stamp_sec - last_accepted_pose_time_sec);
  return metrics;
}

inline AlignmentCorrectionMetrics computeAlignmentCorrectionMetrics(
  const Eigen::Matrix4f & init_guess,
  const Eigen::Matrix4f & final_transformation)
{
  const Eigen::Matrix4f correction_matrix = init_guess.inverse() * final_transformation;
  return {
    poseDeltaTranslationNormM(correction_matrix),
    poseDeltaYawAbsDeg(correction_matrix)};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_ATTEMPT_POLICY_HPP_
