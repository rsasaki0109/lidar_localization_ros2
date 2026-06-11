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

bool near_loose(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1.0e-9;
}

void test_error_floor_std_clamps_and_handles_nan()
{
  assert(near(ll::errorFloorStd(0.2, 0.1, 5.0, 0.0), 0.2));
  assert(near(ll::errorFloorStd(0.2, 0.1, 5.0, 3.0), 0.5));
  assert(near(ll::errorFloorStd(0.2, 0.1, 5.0, 1000.0), 5.0));
  assert(near(ll::errorFloorStd(0.2, 0.1, 5.0, -4.0), 0.2));
  assert(
    near(ll::errorFloorStd(0.2, 0.1, 5.0, std::numeric_limits<double>::quiet_NaN()), 0.2));
}

void test_error_floor_covariance_uses_calibrated_defaults()
{
  const ll::ErrorFloorCovarianceParams params;
  const auto covariance = ll::makeErrorFloorPoseCovariance(params, 0.0);

  // floors: xy 0.2 m, z 0.3 m, yaw 2 deg, roll/pitch 1.5 deg
  assert(near_loose(covariance[0], 0.04));
  assert(near_loose(covariance[7], 0.04));
  assert(near_loose(covariance[14], 0.09));
  const double roll_pitch_floor = 1.5 * M_PI / 180.0;
  const double yaw_floor = 2.0 * M_PI / 180.0;
  assert(near_loose(covariance[21], roll_pitch_floor * roll_pitch_floor));
  assert(near_loose(covariance[28], roll_pitch_floor * roll_pitch_floor));
  assert(near_loose(covariance[35], yaw_floor * yaw_floor));
}

void test_error_floor_covariance_grows_with_fitness()
{
  const ll::ErrorFloorCovarianceParams params;
  const auto covariance = ll::makeErrorFloorPoseCovariance(params, 4.0);

  // xy std 0.2 + 0.1*4 = 0.6, z std 0.3 + 0.1*4 = 0.7, yaw std 2 + 1*4 = 6 deg
  assert(near_loose(covariance[0], 0.36));
  assert(near_loose(covariance[14], 0.49));
  const double yaw_std = 6.0 * M_PI / 180.0;
  assert(near_loose(covariance[35], yaw_std * yaw_std));
}

void test_ekf_error_floor_covariance_keeps_ekf_translation_and_yaw()
{
  ll::EkfStateCovariance ekf_covariance = ll::EkfStateCovariance::Zero();
  ekf_covariance(0, 0) = 1.0;
  ekf_covariance(0, 1) = 0.2;
  ekf_covariance(1, 0) = 0.3;
  ekf_covariance(1, 1) = 2.0;
  ekf_covariance(2, 2) = 3.0;
  ekf_covariance(6, 6) = 4.0;

  const ll::ErrorFloorCovarianceParams params;
  const auto covariance =
    ll::makeEkfErrorFloorPoseCovariance(ekf_covariance, params, 4.0);

  assert(near(covariance[0], 1.0));
  assert(near(covariance[1], 0.2));
  assert(near(covariance[6], 0.3));
  assert(near(covariance[7], 2.0));
  assert(near(covariance[14], 3.0));
  assert(near(covariance[35], 4.0));
  // roll/pitch come from the error-floor model: std 1.5 + 1*4 = 5.5 deg
  const double roll_pitch_std = 5.5 * M_PI / 180.0;
  assert(near_loose(covariance[21], roll_pitch_std * roll_pitch_std));
  assert(near_loose(covariance[28], roll_pitch_std * roll_pitch_std));
}

int main()
{
  test_fitness_covariance_uses_fitness_floor();
  test_fitness_covariance_scales_pose_axes();
  test_ekf_covariance_maps_estimated_terms();
  test_nan_fitness_uses_floor_like_previous_std_max_behavior();
  test_error_floor_std_clamps_and_handles_nan();
  test_error_floor_covariance_uses_calibrated_defaults();
  test_error_floor_covariance_grows_with_fitness();
  test_ekf_error_floor_covariance_keeps_ekf_translation_and_yaw();
  return 0;
}
