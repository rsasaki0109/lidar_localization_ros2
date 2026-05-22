#ifndef LIDAR_LOCALIZATION_REGISTRATION_OBSERVATION_POLICY_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_OBSERVATION_POLICY_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace lidar_localization
{

constexpr double kRegistrationObservationPi = 3.14159265358979323846;

struct RegistrationObservation
{
  Eigen::Matrix4f pose_matrix{Eigen::Matrix4f::Identity()};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

inline RegistrationObservation makeRegistrationObservation(const Eigen::Matrix4f & pose_matrix)
{
  const Eigen::Matrix3f rotation = pose_matrix.block<3, 3>(0, 0);

  RegistrationObservation observation;
  observation.pose_matrix = pose_matrix;
  observation.x = static_cast<double>(pose_matrix(0, 3));
  observation.y = static_cast<double>(pose_matrix(1, 3));
  observation.z = static_cast<double>(pose_matrix(2, 3));
  observation.roll = static_cast<double>(std::atan2(rotation(2, 1), rotation(2, 2)));
  observation.pitch = static_cast<double>(
    std::asin(-std::clamp(rotation(2, 0), -1.0f, 1.0f)));
  observation.yaw = static_cast<double>(std::atan2(rotation(1, 0), rotation(0, 0)));
  return observation;
}

inline double rotationDeltaDeg(
  const Eigen::Matrix3f & reference_rotation,
  const Eigen::Matrix3f & candidate_rotation)
{
  const Eigen::Matrix3d relative_rotation =
    reference_rotation.cast<double>().transpose() * candidate_rotation.cast<double>();
  const double trace =
    std::clamp((relative_rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(trace) * 180.0 / kRegistrationObservationPi;
}

inline double rotationDeltaDeg(
  const Eigen::Matrix4f & reference_pose,
  const Eigen::Matrix4f & candidate_pose)
{
  const Eigen::Matrix3f reference_rotation = reference_pose.block<3, 3>(0, 0);
  const Eigen::Matrix3f candidate_rotation = candidate_pose.block<3, 3>(0, 0);
  return rotationDeltaDeg(reference_rotation, candidate_rotation);
}

inline Eigen::Matrix4f makePoseMatrix(
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw)
{
  Eigen::Affine3f affine = Eigen::Affine3f::Identity();
  affine.translation() = Eigen::Vector3f(
    static_cast<float>(x),
    static_cast<float>(y),
    static_cast<float>(z));
  affine.linear() = (
    Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(static_cast<float>(roll), Eigen::Vector3f::UnitX())
  ).toRotationMatrix();
  return affine.matrix();
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_OBSERVATION_POLICY_HPP_
