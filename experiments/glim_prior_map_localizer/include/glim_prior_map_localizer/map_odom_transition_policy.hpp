#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace glim_prior_map_localizer
{

struct BoundedMapOdomStep
{
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  bool translation_limited{false};
  bool rotation_limited{false};
};

inline Eigen::Isometry3d mapFromOdomHoldingSensorPose(
  const Eigen::Isometry3d & frozen_map_from_sensor,
  const Eigen::Isometry3d & odom_from_sensor)
{
  return frozen_map_from_sensor * odom_from_sensor.inverse();
}

inline BoundedMapOdomStep boundMapOdomStep(
  const Eigen::Isometry3d & current,
  const Eigen::Isometry3d & target,
  double max_translation_m,
  double max_rotation_deg)
{
  BoundedMapOdomStep result;
  result.transform = target;

  const Eigen::Vector3d delta = target.translation() - current.translation();
  const double distance = delta.norm();
  if (distance > max_translation_m) {
    result.transform.translation() =
      current.translation() + delta * (max_translation_m / distance);
    result.translation_limited = true;
  }

  Eigen::Quaterniond current_rotation(current.linear());
  Eigen::Quaterniond target_rotation(target.linear());
  current_rotation.normalize();
  target_rotation.normalize();
  const double cosine = std::clamp(
    std::abs(current_rotation.dot(target_rotation)), 0.0, 1.0);
  const double angle = 2.0 * std::acos(cosine);
  constexpr double degrees_to_radians = 0.01745329251994329577;
  const double max_rotation_rad = max_rotation_deg * degrees_to_radians;
  if (angle > max_rotation_rad) {
    const double fraction = max_rotation_rad / angle;
    result.transform.linear() =
      current_rotation.slerp(fraction, target_rotation).normalized().toRotationMatrix();
    result.rotation_limited = true;
  }
  return result;
}

}  // namespace glim_prior_map_localizer
