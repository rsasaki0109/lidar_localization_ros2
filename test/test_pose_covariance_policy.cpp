#include "lidar_localization/pose_covariance_policy.hpp"

#include <cassert>
#include <cmath>
#include <limits>

namespace ll = lidar_localization;

bool near(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1.0e-12;
}

void test_fitness_covariance_uses_fitness_floor()
{
  const auto covariance = ll::makeFitnessPoseCovariance(0.2);

  assert(near(covariance[0], 0.01));
  assert(near(covariance[7], 0.01));
  assert(near(covariance[14], 0.05));
  assert(near(covariance[21], 0.001));
  assert(near(covariance[28], 0.001));
  assert(near(covariance[35], 0.0005));
}

void test_fitness_covariance_scales_pose_axes()
{
  const auto covariance = ll::makeFitnessPoseCovariance(3.0);

  assert(near(covariance[0], 0.03));
  assert(near(covariance[7], 0.03));
  assert(near(covariance[14], 0.15));
  assert(near(covariance[21], 0.003));
  assert(near(covariance[28], 0.003));
  assert(near(covariance[35], 0.0015));
}

void test_ekf_covariance_maps_estimated_terms()
{
  ll::EkfStateCovariance ekf_covariance = ll::EkfStateCovariance::Zero();
  ekf_covariance(0, 0) = 1.0;
  ekf_covariance(0, 1) = 0.2;
  ekf_covariance(1, 0) = 0.3;
  ekf_covariance(1, 1) = 2.0;
  ekf_covariance(2, 2) = 3.0;
  ekf_covariance(6, 6) = 4.0;

  const auto covariance = ll::makeEkfPoseCovariance(ekf_covariance, 5.0);

  assert(near(covariance[0], 1.0));
  assert(near(covariance[1], 0.2));
  assert(near(covariance[6], 0.3));
  assert(near(covariance[7], 2.0));
  assert(near(covariance[14], 3.0));
  assert(near(covariance[35], 4.0));
  assert(near(covariance[21], 0.005));
  assert(near(covariance[28], 0.005));
}

void test_nan_fitness_uses_floor_like_previous_std_max_behavior()
{
  const auto covariance =
    ll::makeFitnessPoseCovariance(std::numeric_limits<double>::quiet_NaN());

  assert(near(covariance[0], 0.01));
  assert(near(covariance[35], 0.0005));
}

int main()
{
  test_fitness_covariance_uses_fitness_floor();
  test_fitness_covariance_scales_pose_axes();
  test_ekf_covariance_maps_estimated_terms();
  test_nan_fitness_uses_floor_like_previous_std_max_behavior();
  return 0;
}
