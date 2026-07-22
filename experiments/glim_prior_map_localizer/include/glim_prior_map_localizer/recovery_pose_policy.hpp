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

inline bool recoveryRearmAllowed(
  double last_verified_stamp,
  double current_stamp,
  double cooldown_sec)
{
  return last_verified_stamp < 0.0 ||
         current_stamp - last_verified_stamp >= cooldown_sec;
}

}  // namespace glim_prior_map_localizer
