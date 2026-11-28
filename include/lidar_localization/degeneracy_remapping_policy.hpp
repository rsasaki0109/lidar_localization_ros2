// Solution remapping for degenerate scans: where the NDT score's Hessian is
// weakest along a horizontal translation it barely constrains (a corridor's or
// a tunnel's axis), NDT's correction in that direction is noise the scan cannot
// confirm, and kept scan after scan it slides the pose along the corridor below
// the seed correction guards' limits. The correction's component along that
// direction is dropped, so the pose keeps the seed's (the prediction's) position
// along it (after Zhang, Kaess and Singh, "On degeneracy of optimization-based
// state estimation problems", ICRA 2016).
#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "lidar_localization/registration_localizability_policy.hpp"

namespace lidar_localization
{

struct DegeneracyRemappingParams
{
  bool enable{false};
  // The Hessian's weakest over strongest absolute eigenvalue below which its
  // weakest direction counts as unconstrained.
  double weak_ratio{0.05};
  // The weakest direction's share in horizontal translation (its x, y
  // components' norm) from which it is taken as a horizontal translation.
  double min_horizontal_share{0.7};
  // Above 0: also drop the correction along every horizontal direction the
  // map's surfaces under the scan pin less than this (the eigenvalues of
  // HorizontalConstraint::information, registration_fitness.hpp: the share of
  // the scan's points whose surface faces that way). Bare ground pins neither
  // direction, though NDT's Hessian says it does.
  double min_horizontal_constraint{0.0};
};

struct DegeneracyRemapping
{
  bool applied{false};
  // The correction dropped, meters (signed, along `direction`).
  double dropped_m{0.0};
  Eigen::Vector2d direction{Eigen::Vector2d::Zero()};
};

// Drops from `aligned` the part of its translation from `seed` along the
// Hessian's weakest direction when that direction is a weakly constrained
// horizontal translation. `metrics` must be the localizability of this
// alignment (RegistrationEngine::localizability()).
inline DegeneracyRemapping remapDegenerateCorrection(
  const Eigen::Matrix4f & seed, Eigen::Matrix4f & aligned,
  const RegistrationLocalizabilityMetrics & metrics, const DegeneracyRemappingParams & params)
{
  DegeneracyRemapping result;
  if (!params.enable || !metrics.valid || !(metrics.weak_ratio < params.weak_ratio)) {
    return result;
  }
  const Eigen::Vector2d weakest(metrics.weakest_eigenvector[0], metrics.weakest_eigenvector[1]);
  const double horizontal = weakest.norm();
  if (!std::isfinite(horizontal) || horizontal < params.min_horizontal_share) {
    return result;
  }
  result.direction = weakest / horizontal;
  const Eigen::Vector2d correction(
    static_cast<double>(aligned(0, 3) - seed(0, 3)), static_cast<double>(aligned(1, 3) - seed(1, 3)));
  result.dropped_m = correction.dot(result.direction);
  aligned(0, 3) -= static_cast<float>(result.dropped_m * result.direction.x());
  aligned(1, 3) -= static_cast<float>(result.dropped_m * result.direction.y());
  result.applied = true;
  return result;
}

struct ConstraintRemapping
{
  // How many horizontal directions were dropped (0, 1 or 2), the correction
  // dropped (m), and the information's eigenvalues (ascending).
  int dropped_directions{0};
  double dropped_m{0.0};
  Eigen::Vector2d eigenvalues{Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};
};

// Drops from `aligned` the part of its translation from `seed` along every
// horizontal direction whose `information` eigenvalue is below
// params.min_horizontal_constraint (`information`: HorizontalConstraint's).
inline ConstraintRemapping remapUnconstrainedCorrection(
  const Eigen::Matrix4f & seed, Eigen::Matrix4f & aligned, const Eigen::Matrix2d & information,
  const DegeneracyRemappingParams & params)
{
  ConstraintRemapping result;
  if (!params.enable || !(params.min_horizontal_constraint > 0.0) || !information.allFinite()) {
    return result;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(information);
  result.eigenvalues = solver.eigenvalues();
  Eigen::Vector2d correction(
    static_cast<double>(aligned(0, 3) - seed(0, 3)), static_cast<double>(aligned(1, 3) - seed(1, 3)));
  Eigen::Vector2d dropped = Eigen::Vector2d::Zero();
  for (int k = 0; k < 2; ++k) {
    if (result.eigenvalues(k) < params.min_horizontal_constraint) {
      const Eigen::Vector2d v = solver.eigenvectors().col(k);
      dropped += correction.dot(v) * v;
      ++result.dropped_directions;
    }
  }
  aligned(0, 3) -= static_cast<float>(dropped.x());
  aligned(1, 3) -= static_cast<float>(dropped.y());
  result.dropped_m = dropped.norm();
  return result;
}

}  // namespace lidar_localization
