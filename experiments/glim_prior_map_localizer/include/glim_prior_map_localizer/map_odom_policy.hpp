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
  double horizontal_gain,
  double vertical_gain)
{
  Eigen::Isometry3d updated = reference;
  const double bounded_horizontal_gain = std::clamp(horizontal_gain, 0.0, 1.0);
  const double bounded_vertical_gain = std::clamp(vertical_gain, 0.0, 1.0);
  const Eigen::Vector3d displacement =
    observed.translation() - reference.translation();
  updated.translation().head<2>() +=
    bounded_horizontal_gain * displacement.head<2>();
  updated.translation().z() += bounded_vertical_gain * displacement.z();
  return updated;
}

inline Eigen::Isometry3d updateMapOdomTranslation(
  const Eigen::Isometry3d & reference,
  const Eigen::Isometry3d & observed,
  double gain)
{
  return updateMapOdomTranslation(reference, observed, gain, gain);
}

inline Eigen::Isometry3d updateMapOdomVertical(
  const Eigen::Isometry3d & reference,
  const Eigen::Isometry3d & current,
  const Eigen::Isometry3d & observed,
  double vertical_gain)
{
  Eigen::Isometry3d updated = current;
  const double bounded_vertical_gain = std::clamp(vertical_gain, 0.0, 1.0);
  updated.translation().z() = reference.translation().z() +
    bounded_vertical_gain *
    (observed.translation().z() - reference.translation().z());
  return updated;
}

}  // namespace glim_prior_map_localizer
