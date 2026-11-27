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

#include <Eigen/Core>

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

}  // namespace lidar_localization
