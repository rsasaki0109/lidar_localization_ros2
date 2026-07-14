#include "lidar_localization/imu_initialization_policy.hpp"

#include <cassert>
#include <limits>

namespace ll = lidar_localization;

Eigen::Matrix4f pose(double x, double y, double z)
{
  Eigen::Matrix4f value = Eigen::Matrix4f::Identity();
  value.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
  return value;
}

void test_estimates_world_velocity_from_two_accepted_poses()
{
  const auto estimate = ll::estimateImuInitialVelocity(
    pose(1.0, 2.0, 3.0), 10.0,
    pose(2.0, 1.5, 3.25), 10.5);

  assert(estimate.usable);
  assert((estimate.velocity - Eigen::Vector3d(2.0, -1.0, 0.5)).norm() < 1e-12);
  assert(estimate.interval_sec == 0.5);
}

void test_scales_g_units_to_si_acceleration()
{
  const Eigen::Vector3d in_g(0.1, -0.2, 1.0);
  const Eigen::Vector3d expected(0.980665, -1.96133, 9.80665);
  assert((ll::scaleImuAcceleration(in_g, 9.80665) - expected).norm() < 1e-12);
}

void test_rejects_nonpositive_or_excessive_interval()
{
  assert(!ll::estimateImuInitialVelocity(
    pose(0, 0, 0), 1.0, pose(1, 0, 0), 1.0).usable);
  assert(!ll::estimateImuInitialVelocity(
    pose(0, 0, 0), 1.0, pose(1, 0, 0), 4.0, 2.0).usable);
}

void test_rejects_implausible_speed_and_non_finite_pose()
{
  assert(!ll::estimateImuInitialVelocity(
    pose(0, 0, 0), 1.0, pose(100, 0, 0), 2.0, 2.0, 50.0).usable);

  auto invalid = pose(1, 0, 0);
  invalid(0, 3) = std::numeric_limits<float>::quiet_NaN();
  assert(!ll::estimateImuInitialVelocity(
    pose(0, 0, 0), 1.0, invalid, 2.0).usable);
}

int main()
{
  test_estimates_world_velocity_from_two_accepted_poses();
  test_scales_g_units_to_si_acceleration();
  test_rejects_nonpositive_or_excessive_interval();
  test_rejects_implausible_speed_and_non_finite_pose();
  return 0;
}
