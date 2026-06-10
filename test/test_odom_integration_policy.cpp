#include "lidar_localization/odom_integration_policy.hpp"

#include <cassert>
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

int main()
{
  test_disabled_and_waiting_states();
  test_interval_and_pose_guards();
  test_pose_finite_helper();
  return 0;
}
