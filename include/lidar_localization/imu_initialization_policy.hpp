#ifndef LIDAR_LOCALIZATION_IMU_INITIALIZATION_POLICY_HPP_
#define LIDAR_LOCALIZATION_IMU_INITIALIZATION_POLICY_HPP_

#include <Eigen/Core>

#include <cmath>

namespace lidar_localization
{

struct ImuInitialVelocityEstimate
{
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double interval_sec{0.0};
  bool usable{false};
};

inline Eigen::Vector3d scaleImuAcceleration(
  const Eigen::Vector3d & acceleration, double scale)
{
  return acceleration * scale;
}

inline ImuInitialVelocityEstimate estimateImuInitialVelocity(
  const Eigen::Matrix4f & previous_pose,
  double previous_stamp_sec,
  const Eigen::Matrix4f & current_pose,
  double current_stamp_sec,
  double max_interval_sec = 2.0,
  double max_speed_mps = 50.0)
{
  ImuInitialVelocityEstimate estimate;
  estimate.interval_sec = current_stamp_sec - previous_stamp_sec;
  if (!previous_pose.allFinite() || !current_pose.allFinite() ||
    !std::isfinite(previous_stamp_sec) || !std::isfinite(current_stamp_sec) ||
    !std::isfinite(max_interval_sec) || max_interval_sec <= 0.0 ||
    !std::isfinite(max_speed_mps) || max_speed_mps <= 0.0 ||
    estimate.interval_sec <= 0.0 || estimate.interval_sec > max_interval_sec)
  {
    return estimate;
  }

  estimate.velocity =
    (current_pose.block<3, 1>(0, 3) - previous_pose.block<3, 1>(0, 3)).cast<double>() /
    estimate.interval_sec;
  estimate.usable =
    estimate.velocity.allFinite() && estimate.velocity.norm() <= max_speed_mps;
  if (!estimate.usable) {
    estimate.velocity.setZero();
  }
  return estimate;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_IMU_INITIALIZATION_POLICY_HPP_
