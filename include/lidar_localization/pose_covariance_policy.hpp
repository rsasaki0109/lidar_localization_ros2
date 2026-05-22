#ifndef LIDAR_LOCALIZATION_POSE_COVARIANCE_POLICY_HPP_
#define LIDAR_LOCALIZATION_POSE_COVARIANCE_POLICY_HPP_

#include <algorithm>
#include <array>

#include <Eigen/Core>

namespace lidar_localization
{

using PoseCovariance = std::array<double, 36>;
using EkfStateCovariance = Eigen::Matrix<double, 9, 9>;

inline double poseCovarianceScale(double fitness_score)
{
  return std::max(1.0, fitness_score);
}

inline PoseCovariance makeFitnessPoseCovariance(double fitness_score)
{
  PoseCovariance covariance{};
  const double scale = poseCovarianceScale(fitness_score);
  covariance[0] = 0.01 * scale;
  covariance[7] = 0.01 * scale;
  covariance[14] = 0.05 * scale;
  covariance[21] = 0.001 * scale;
  covariance[28] = 0.001 * scale;
  covariance[35] = 0.0005 * scale;
  return covariance;
}

inline PoseCovariance makeEkfPoseCovariance(
  const EkfStateCovariance & ekf_covariance,
  double fitness_score)
{
  PoseCovariance covariance{};
  covariance[0] = ekf_covariance(0, 0);
  covariance[1] = ekf_covariance(0, 1);
  covariance[6] = ekf_covariance(1, 0);
  covariance[7] = ekf_covariance(1, 1);
  covariance[14] = ekf_covariance(2, 2);
  covariance[35] = ekf_covariance(6, 6);

  const double scale = poseCovarianceScale(fitness_score);
  covariance[21] = 0.001 * scale;
  covariance[28] = 0.001 * scale;
  return covariance;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POSE_COVARIANCE_POLICY_HPP_
