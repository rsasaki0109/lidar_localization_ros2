#include "glim_prior_map_localizer/map_odom_policy.hpp"

#include <cassert>
#include <cmath>

#include <Eigen/Geometry>

namespace gp = glim_prior_map_localizer;

int main()
{
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  current.translation() = Eigen::Vector3d(10.0, -2.0, 1.0);
  current.linear() = Eigen::AngleAxisd(
    0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  Eigen::Isometry3d observed = Eigen::Isometry3d::Identity();
  observed.translation() = Eigen::Vector3d(20.0, 8.0, -4.0);
  observed.linear() = Eigen::AngleAxisd(
    -1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto updated = gp::updateMapOdomTranslation(current, observed, 0.2);
  assert(updated.translation().isApprox(Eigen::Vector3d(12.0, 0.0, 0.0)));
  assert(updated.linear().isApprox(current.linear()));

  const auto clamped_high = gp::updateMapOdomTranslation(current, observed, 2.0);
  assert(clamped_high.translation().isApprox(observed.translation()));
  assert(clamped_high.linear().isApprox(current.linear()));

  const auto clamped_low = gp::updateMapOdomTranslation(current, observed, -1.0);
  assert(clamped_low.matrix().isApprox(current.matrix()));

  Eigen::Isometry3d odom_from_sensor = Eigen::Isometry3d::Identity();
  odom_from_sensor.translation() = Eigen::Vector3d(3.0, -2.0, 1.0);
  const Eigen::Matrix3d fixed_rotation = Eigen::AngleAxisd(
    0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d expected_map_odom = Eigen::Isometry3d::Identity();
  expected_map_odom.linear() = fixed_rotation;
  expected_map_odom.translation() = Eigen::Vector3d(-8.0, 4.0, 2.0);
  const Eigen::Isometry3d map_from_sensor = expected_map_odom * odom_from_sensor;
  const auto observation = gp::observeMapOdomWithFixedRotation(
    map_from_sensor, odom_from_sensor,
    fixed_rotation);
  assert(observation.matrix().isApprox(expected_map_odom.matrix()));
  return 0;
}
