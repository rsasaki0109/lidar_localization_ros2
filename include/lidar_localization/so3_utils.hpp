#ifndef SO3_UTILS_HPP_
#define SO3_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace so3 {

/// Skew-symmetric matrix from 3-vector
inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S <<    0, -v(2),  v(1),
       v(2),     0, -v(0),
      -v(1),  v(0),     0;
  return S;
}

/// Exponential map: R^3 -> SO(3) via Rodrigues formula
inline Eigen::Matrix3d Exp(const Eigen::Vector3d & v)
{
  double theta = v.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() + skew(v);
  }
  Eigen::Vector3d a = v / theta;
  double s = std::sin(theta);
  double c = std::cos(theta);
  return c * Eigen::Matrix3d::Identity() +
         (1.0 - c) * a * a.transpose() +
         s * skew(a);
}

/// Logarithmic map: SO(3) -> R^3
inline Eigen::Vector3d Log(const Eigen::Matrix3d & R)
{
  double cos_theta = 0.5 * (R.trace() - 1.0);
  cos_theta = std::clamp(cos_theta, -1.0, 1.0);
  double theta = std::acos(cos_theta);

  if (theta < 1e-10) {
    return Eigen::Vector3d(R(2, 1) - R(1, 2),
                           R(0, 2) - R(2, 0),
                           R(1, 0) - R(0, 1)) * 0.5;
  }
  double half_theta_sin = theta / (2.0 * std::sin(theta));
  return Eigen::Vector3d(R(2, 1) - R(1, 2),
                         R(0, 2) - R(2, 0),
                         R(1, 0) - R(0, 1)) * half_theta_sin;
}

/// Right Jacobian of SO(3)
inline Eigen::Matrix3d Jr(const Eigen::Vector3d & v)
{
  double theta = v.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() - 0.5 * skew(v);
  }
  Eigen::Vector3d a = v / theta;
  double s = std::sin(theta);
  double c = std::cos(theta);
  return (s / theta) * Eigen::Matrix3d::Identity() +
         (1.0 - s / theta) * a * a.transpose() -
         ((1.0 - c) / theta) * skew(a);
}

/// Inverse right Jacobian of SO(3)
inline Eigen::Matrix3d Jr_inv(const Eigen::Vector3d & v)
{
  double theta = v.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() + 0.5 * skew(v);
  }
  Eigen::Vector3d a = v / theta;
  double half_theta = 0.5 * theta;
  double cot_half = std::cos(half_theta) / std::sin(half_theta);
  return half_theta * cot_half * Eigen::Matrix3d::Identity() +
         (1.0 - half_theta * cot_half) * a * a.transpose() +
         half_theta * skew(a);
}

/// Build rotation matrix from Euler angles (roll, pitch, yaw) in ZYX order
inline Eigen::Matrix3d eulerToRotation(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// Extract Euler angles (roll, pitch, yaw) from rotation matrix (ZYX order)
inline Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d & R)
{
  double pitch = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
  double roll = std::atan2(R(2, 1), R(2, 2));
  double yaw = std::atan2(R(1, 0), R(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace so3

#endif  // SO3_UTILS_HPP_
