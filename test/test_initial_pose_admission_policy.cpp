#include "lidar_localization/initial_pose_admission_policy.hpp"

#include <cassert>
#include <limits>

namespace ll = lidar_localization;

ll::InitialPoseAdmissionInput ready_input()
{
  ll::InitialPoseAdmissionInput input;
  input.expected_frame_id = "map";
  input.received_frame_id = "map";
  input.pose.position.x = 1.0;
  input.pose.orientation.w = 1.0;
  input.map_received = true;
  return input;
}

void test_frame_and_pose_guards()
{
  auto input = ready_input();
  const auto accepted = ll::decideInitialPoseAdmission(input);
  assert(accepted.accepted);
  assert(!accepted.warn_map_not_ready);

  input = ready_input();
  input.received_frame_id = "odom";
  const auto frame_mismatch = ll::decideInitialPoseAdmission(input);
  assert(!frame_mismatch.accepted);
  assert(frame_mismatch.status == ll::InitialPoseAdmissionStatus::kRejectedFrameMismatch);

  input = ready_input();
  input.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  const auto non_finite = ll::decideInitialPoseAdmission(input);
  assert(!non_finite.accepted);
  assert(non_finite.status == ll::InitialPoseAdmissionStatus::kRejectedNonFinitePose);
}

void test_map_not_ready_warning()
{
  auto input = ready_input();
  input.map_received = false;
  const auto early_pose = ll::decideInitialPoseAdmission(input);
  assert(early_pose.accepted);
  assert(early_pose.warn_map_not_ready);
}

int main()
{
  test_frame_and_pose_guards();
  test_map_not_ready_warning();
  return 0;
}
