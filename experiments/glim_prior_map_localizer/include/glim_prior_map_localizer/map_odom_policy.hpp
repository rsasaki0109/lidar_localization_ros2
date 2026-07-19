#pragma once

#include <algorithm>

#include <Eigen/Geometry>

namespace glim_prior_map_localizer
{

inline Eigen::Isometry3d observeMapOdomWithFixedRotation(
  const Eigen::Isometry3d & map_from_sensor,
  const Eigen::Isometry3d & odom_from_sensor,
  const Eigen::Matrix3d & fixed_map_odom_rotation)
{
  Eigen::Isometry3d observation = Eigen::Isometry3d::Identity();
  observation.linear() = fixed_map_odom_rotation;
  observation.translation() = map_from_sensor.translation() -
    fixed_map_odom_rotation * odom_from_sensor.translation();
  return observation;
}

inline Eigen::Isometry3d updateMapOdomTranslation(
  const Eigen::Isometry3d & reference,
  const Eigen::Isometry3d & observed,
  double gain)
{
  Eigen::Isometry3d updated = reference;
  const double bounded_gain = std::clamp(gain, 0.0, 1.0);
  updated.translation() +=
    bounded_gain * (observed.translation() - reference.translation());
  return updated;
}

}  // namespace glim_prior_map_localizer
