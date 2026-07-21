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

// A global-search pose belongs to the scan timestamp carried by the result.
// Preserve that map/odom relationship and advance it with odometry while the
// candidate batch is ranked and verified.
inline Eigen::Isometry3d propagateRecoveryPose(
  const Eigen::Isometry3d & odom_from_map_at_candidate,
  const Eigen::Isometry3d & odom_from_sensor_now)
{
  return odom_from_map_at_candidate.inverse() * odom_from_sensor_now;
}

}  // namespace glim_prior_map_localizer
