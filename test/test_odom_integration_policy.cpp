#include "lidar_localization/odom_integration_policy.hpp"

#include <cassert>
#include <cmath>
#include <limits>

namespace ll = lidar_localization;

ll::OdomAdmissionInput ready_input()
{
  ll::OdomAdmissionInput input;
  input.use_odom = true;
  input.initial_pose_received = true;
  input.has_current_pose = true;
  input.current_pose_finite = true;
  input.dt_odom_sec = 0.1;
  return input;
}

void test_disabled_and_waiting_states()
{
  auto input = ready_input();
  input.use_odom = false;
  const auto disabled = ll::decideOdomAdmission(input);
  assert(!disabled.accepted);
  assert(disabled.status == ll::OdomAdmissionStatus::kDisabled);

  input = ready_input();
  input.initial_pose_received = false;
  const auto waiting = ll::decideOdomAdmission(input);
  assert(!waiting.accepted);
  assert(waiting.status == ll::OdomAdmissionStatus::kWaitingForInitialPose);

  input = ready_input();
  input.has_current_pose = false;
  const auto missing_pose = ll::decideOdomAdmission(input);
  assert(!missing_pose.accepted);
  assert(missing_pose.status == ll::OdomAdmissionStatus::kMissingCurrentPose);
}

void test_interval_and_pose_guards()
{
  auto input = ready_input();
  input.dt_odom_sec = 2.0;
  const auto too_large = ll::decideOdomAdmission(input);
  assert(!too_large.accepted);
  assert(too_large.status == ll::OdomAdmissionStatus::kIntervalTooLarge);

  input = ready_input();
  input.dt_odom_sec = -0.1;
  const auto negative = ll::decideOdomAdmission(input);
  assert(!negative.accepted);
  assert(negative.status == ll::OdomAdmissionStatus::kIntervalNegative);

  input = ready_input();
  input.current_pose_finite = false;
  const auto non_finite = ll::decideOdomAdmission(input);
  assert(!non_finite.accepted);
  assert(non_finite.status == ll::OdomAdmissionStatus::kCurrentPoseNonFinite);
}

void test_pose_finite_helper()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.orientation.w = 1.0;
  assert(ll::isPoseFinite(pose));

  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  assert(!ll::isPoseFinite(pose));
}

Eigen::Quaterniond rpy(double roll, double pitch, double yaw)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

bool near(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  return a.angularDistance(b) < 1e-9;
}

bool near(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return (a - b).norm() < 1e-9;
}

void test_zero_twist_keeps_tilted_pose()
{
  const Eigen::Quaterniond start = rpy(0.3, -0.2, 1.5);
  const auto result = ll::integrateOdomTwist(
    start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.1);
  assert(near(result.orientation, start));
  assert(near(result.delta_position, Eigen::Vector3d::Zero()));
}

void test_level_yaw_rate_matches_planar_integration()
{
  const auto result = ll::integrateOdomTwist(
    rpy(0.0, 0.0, 0.4), Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.5), 0.2);
  assert(near(result.orientation, rpy(0.0, 0.0, 0.5)));
  assert(near(
    result.delta_position,
    Eigen::Vector3d(0.2 * std::cos(0.5), 0.2 * std::sin(0.5), 0.0)));
}

void test_body_rate_rotates_about_tilted_body_axis()
{
  // Rolled 90 deg, the body z axis is world -y: a body yaw rate pitches the
  // pose in the world, where adding it to yaw would turn it about world z.
  const Eigen::Quaterniond start = rpy(M_PI_2, 0.0, 0.0);
  Eigen::Quaterniond orientation = start;
  for (int i = 0; i < 10; ++i) {
    orientation = ll::integrateOdomTwist(
      orientation, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 0.3), 0.1).orientation;
  }
  const Eigen::Quaterniond expected =
    start * Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  assert(near(orientation, expected));
  assert(near(orientation * Eigen::Vector3d::UnitX(),
    Eigen::Vector3d(std::cos(0.3), 0.0, std::sin(0.3))));
}

void test_linear_velocity_follows_pitch()
{
  // Negative pitch tilts the nose up.
  const auto result = ll::integrateOdomTwist(
    rpy(0.0, -M_PI / 6.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0),
    Eigen::Vector3d::Zero(), 0.5);
  assert(near(result.delta_position, Eigen::Vector3d(std::cos(M_PI / 6.0), 0.0, 0.5)));
}

void test_unnormalized_orientation_is_normalized()
{
  const Eigen::Quaterniond start = rpy(0.1, 0.2, 0.3);
  const Eigen::Quaterniond scaled(
    2.0 * start.w(), 2.0 * start.x(), 2.0 * start.y(), 2.0 * start.z());
  const auto result = ll::integrateOdomTwist(
    scaled, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), 1.0);
  assert(std::abs(result.orientation.norm() - 1.0) < 1e-12);
  assert(near(result.orientation, start));
  assert(near(result.delta_position, start * Eigen::Vector3d::UnitX()));
}

int main()
{
  test_disabled_and_waiting_states();
  test_interval_and_pose_guards();
  test_pose_finite_helper();
  test_zero_twist_keeps_tilted_pose();
  test_level_yaw_rate_matches_planar_integration();
  test_body_rate_rotates_about_tilted_body_axis();
  test_linear_velocity_follows_pitch();
  test_unnormalized_orientation_is_normalized();
  return 0;
}
