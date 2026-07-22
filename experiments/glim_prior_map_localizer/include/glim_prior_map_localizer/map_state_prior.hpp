#pragma once

#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

namespace glim_prior_map_localizer
{

inline void addMapStateKeepAlivePrior(
  gtsam::NonlinearFactorGraph & factors,
  gtsam::Key key,
  const gtsam::Pose3 & pose,
  double precision)
{
  if (!(precision > 0.0)) {
    throw std::invalid_argument("map-state prior precision must be positive");
  }
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    key, pose, gtsam::noiseModel::Isotropic::Precision(6, precision));
}

}  // namespace glim_prior_map_localizer
