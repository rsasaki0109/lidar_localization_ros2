#ifndef LIDAR_LOCALIZATION_REGISTRATION_LOCALIZABILITY_POLICY_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_LOCALIZABILITY_POLICY_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include <Eigen/Eigenvalues>

namespace lidar_localization
{

struct RegistrationLocalizabilityMetrics
{
  bool enabled{false};
  bool valid{false};
  std::string status{"disabled"};
  std::size_t correspondence_count{0};
  std::array<double, 6> eigenvalues{{NAN, NAN, NAN, NAN, NAN, NAN}};
  std::array<double, 6> weakest_eigenvector{{NAN, NAN, NAN, NAN, NAN, NAN}};
  double weakest_absolute_eigenvalue{NAN};
  double strongest_absolute_eigenvalue{NAN};
  double weak_ratio{NAN};
  double absolute_condition_number{NAN};
  std::size_t nonpositive_eigenvalue_count{0};
};

inline RegistrationLocalizabilityMetrics analyzeRegistrationHessian(
  const Eigen::Matrix<double, 6, 6> & score_hessian,
  std::size_t correspondence_count,
  bool enabled = true)
{
  RegistrationLocalizabilityMetrics metrics;
  metrics.enabled = enabled;
  if (!enabled) {
    return metrics;
  }
  metrics.correspondence_count = correspondence_count;
  if (correspondence_count == 0) {
    metrics.status = "no_correspondences";
    return metrics;
  }
  if (!score_hessian.allFinite()) {
    metrics.status = "non_finite_hessian";
    return metrics;
  }

  // NDT_OMP maximizes its score. Negate the symmetric Hessian and normalize by
  // the number of actual voxel correspondences to obtain a per-correspondence
  // curvature matrix. Translation and rotation retain different units.
  const Eigen::Matrix<double, 6, 6> information =
    -0.5 * (score_hessian + score_hessian.transpose()) /
    static_cast<double>(correspondence_count);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    metrics.status = "eigendecomposition_failed";
    return metrics;
  }

  std::size_t weakest_index = 0;
  double weakest_abs = std::numeric_limits<double>::infinity();
  double strongest_abs = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    const double value = solver.eigenvalues()(static_cast<Eigen::Index>(i));
    metrics.eigenvalues[i] = value;
    if (value <= 0.0) {
      ++metrics.nonpositive_eigenvalue_count;
    }
    const double magnitude = std::abs(value);
    if (magnitude < weakest_abs) {
      weakest_abs = magnitude;
      weakest_index = i;
    }
    strongest_abs = std::max(strongest_abs, magnitude);
  }
  for (std::size_t row = 0; row < 6; ++row) {
    metrics.weakest_eigenvector[row] = solver.eigenvectors()(
      static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(weakest_index));
  }
  metrics.weakest_absolute_eigenvalue = weakest_abs;
  metrics.strongest_absolute_eigenvalue = strongest_abs;
  metrics.weak_ratio = strongest_abs > 0.0 ? weakest_abs / strongest_abs : 0.0;
  metrics.absolute_condition_number = weakest_abs > 0.0 ?
    strongest_abs / weakest_abs : std::numeric_limits<double>::infinity();
  metrics.valid = true;
  metrics.status = "ok";
  return metrics;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_LOCALIZABILITY_POLICY_HPP_
