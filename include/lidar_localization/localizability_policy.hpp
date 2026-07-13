#ifndef LIDAR_LOCALIZATION_LOCALIZABILITY_POLICY_HPP_
#define LIDAR_LOCALIZATION_LOCALIZABILITY_POLICY_HPP_

#include <cmath>
#include <cstddef>

#include <Eigen/Eigenvalues>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization
{

struct HorizontalLocalizability
{
  bool valid{false};
  std::size_t point_count{0};
  double minimum_eigenvalue{0.0};
  double maximum_eigenvalue{0.0};
  double eigenvalue_ratio{0.0};
};

inline HorizontalLocalizability evaluateHorizontalLocalizability(
  const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  HorizontalLocalizability result;
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto & point : cloud) {
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      mean += Eigen::Vector2d(point.x, point.y);
      ++result.point_count;
    }
  }
  if (result.point_count < 3) {
    return result;
  }
  mean /= static_cast<double>(result.point_count);
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto & point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    const Eigen::Vector2d centered(point.x - mean.x(), point.y - mean.y());
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(result.point_count - 1);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return result;
  }
  result.minimum_eigenvalue = solver.eigenvalues().x();
  result.maximum_eigenvalue = solver.eigenvalues().y();
  if (result.maximum_eigenvalue <= 1e-9 || result.minimum_eigenvalue < 0.0) {
    return result;
  }
  result.eigenvalue_ratio = result.minimum_eigenvalue / result.maximum_eigenvalue;
  result.valid = std::isfinite(result.eigenvalue_ratio);
  return result;
}

inline bool shouldSuppressPreviousDeltaSeed(
  bool enabled,
  const HorizontalLocalizability & localizability,
  double minimum_eigenvalue_ratio)
{
  return enabled && localizability.valid &&
         std::isfinite(minimum_eigenvalue_ratio) && minimum_eigenvalue_ratio > 0.0 &&
         localizability.eigenvalue_ratio < minimum_eigenvalue_ratio;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_LOCALIZABILITY_POLICY_HPP_
