#include "lidar_localization/alignment_attempt_policy.hpp"

#include <cassert>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

namespace ll = lidar_localization;

constexpr double kPi = 3.14159265358979323846;

Eigen::Matrix4f make_pose(float x, float y, float yaw_rad)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  pose(0, 3) = x;
  pose(1, 3) = y;
  return pose;
}

void test_pose_delta_helpers()
{
  const Eigen::Matrix4f delta = make_pose(3.0f, 4.0f, static_cast<float>(kPi / 2.0));
  assert(std::abs(ll::poseDeltaTranslationNormM(delta) - 5.0) < 1.0e-6);
  assert(std::abs(ll::poseDeltaYawAbsDeg(delta) - 90.0) < 1.0e-4);

  const Eigen::Matrix4f negative_yaw =
    make_pose(0.0f, 0.0f, static_cast<float>(-kPi / 4.0));
  assert(std::abs(ll::poseDeltaYawAbsDeg(negative_yaw) - 45.0) < 1.0e-4);
}

void test_seed_metrics_are_nan_without_last_accepted_pose()
{
  const auto metrics = ll::computeAlignmentSeedMetrics(
    false,
    Eigen::Matrix4f::Identity(),
    make_pose(1.0f, 2.0f, 0.1f),
    10.0,
    5.0);

  assert(std::isnan(metrics.translation_since_accept_m));
  assert(std::isnan(metrics.yaw_since_accept_deg));
  assert(std::isnan(metrics.accepted_gap_sec));
}

void test_seed_metrics_use_relative_pose_and_nonnegative_gap()
{
  const Eigen::Matrix4f last_pose = make_pose(10.0f, 20.0f, static_cast<float>(kPi / 2.0));
  const Eigen::Matrix4f init_guess =
    last_pose * make_pose(3.0f, 4.0f, static_cast<float>(kPi / 6.0));

  auto metrics = ll::computeAlignmentSeedMetrics(true, last_pose, init_guess, 12.0, 10.5);
  assert(std::abs(metrics.translation_since_accept_m - 5.0) < 1.0e-5);
  assert(std::abs(metrics.yaw_since_accept_deg - 30.0) < 1.0e-4);
  assert(std::abs(metrics.accepted_gap_sec - 1.5) < 1.0e-9);

  metrics = ll::computeAlignmentSeedMetrics(true, last_pose, init_guess, 9.0, 10.5);
  assert(metrics.accepted_gap_sec == 0.0);
}

void test_correction_metrics_use_init_to_final_delta()
{
  const Eigen::Matrix4f init_guess = make_pose(-2.0f, 5.0f, static_cast<float>(kPi / 3.0));
  const Eigen::Matrix4f final_pose =
    init_guess * make_pose(0.3f, 0.4f, static_cast<float>(-kPi / 9.0));

  const auto metrics = ll::computeAlignmentCorrectionMetrics(init_guess, final_pose);
  assert(std::abs(metrics.translation_m - 0.5) < 1.0e-5);
  assert(std::abs(metrics.yaw_deg - 20.0) < 1.0e-4);
}

int main()
{
  test_pose_delta_helpers();
  test_seed_metrics_are_nan_without_last_accepted_pose();
  test_seed_metrics_use_relative_pose_and_nonnegative_gap();
  test_correction_metrics_use_init_to_final_delta();
  return 0;
}
