#ifndef LIDAR_LOCALIZATION_POSE_FINITE_HPP_
#define LIDAR_LOCALIZATION_POSE_FINITE_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"

namespace lidar_localization
{

inline bool isPoseFinite(const geometry_msgs::msg::Pose & pose)
{
  const double values[] = {
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
  };
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POSE_FINITE_HPP_
