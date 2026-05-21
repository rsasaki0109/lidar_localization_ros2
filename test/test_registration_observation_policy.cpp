#include "lidar_localization/registration_observation_policy.hpp"

#include <cassert>
#include <cmath>

namespace ll = lidar_localization;

bool near(double lhs, double rhs, double tolerance = 1.0e-5)
{
  return std::abs(lhs - rhs) < tolerance;
}

void test_registration_observation_extracts_translation_and_rpy()
{
  const double roll = 0.12;
  const double pitch = -0.08;
  const double yaw = 0.35;
  const Eigen::Matrix4f pose = ll::makePoseMatrix(1.25, -2.5, 0.75, roll, pitch, yaw);

  const auto observation = ll::makeRegistrationObservation(pose);

  assert(near(observation.x, 1.25));
  assert(near(observation.y, -2.5));
  assert(near(observation.z, 0.75));
  assert(near(observation.roll, roll));
  assert(near(observation.pitch, pitch));
  assert(near(observation.yaw, yaw));
}

void test_rotation_delta_deg_reports_relative_rotation()
{
  const auto reference = ll::makePoseMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto candidate = ll::makePoseMatrix(
    0.0, 0.0, 0.0, 0.0, 0.0, ll::kRegistrationObservationPi / 2.0);

  assert(near(ll::rotationDeltaDeg(reference, candidate), 90.0, 1.0e-4));
}

void test_pitch_extraction_clamps_small_numeric_overshoot()
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose(2, 0) = -1.00001f;

  const auto observation = ll::makeRegistrationObservation(pose);

  assert(near(observation.pitch, ll::kRegistrationObservationPi / 2.0, 1.0e-5));
}

int main()
{
  test_registration_observation_extracts_translation_and_rpy();
  test_rotation_delta_deg_reports_relative_rotation();
  test_pitch_extraction_clamps_small_numeric_overshoot();
  return 0;
}
