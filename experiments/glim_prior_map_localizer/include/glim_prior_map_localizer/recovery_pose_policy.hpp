#pragma once

#include <Eigen/Geometry>

namespace glim_prior_map_localizer
{

inline Eigen::Isometry3d recoveryPoseCorrection(
  const Eigen::Isometry3d & predicted,
  const Eigen::Isometry3d & observed)
{
  return predicted.inverse() * observed;
}

}  // namespace glim_prior_map_localizer
