#include "lidar_localization/pose_backend_selection_policy.hpp"

#include <cassert>

namespace ll = lidar_localization;

void test_raw_registration_is_default_backend()
{
  const auto selected = ll::selectPoseBackend({});

  assert(selected == ll::PoseBackendKind::kRawRegistration);
}

void test_twist_ekf_selected_when_only_twist_enabled()
{
  ll::PoseBackendSelectionInput input;
  input.use_twist_ekf = true;

  const auto selected = ll::selectPoseBackend(input);

  assert(selected == ll::PoseBackendKind::kTwistEkf);
}

void test_imu_requires_stamp_before_it_can_win()
{
  ll::PoseBackendSelectionInput input;
  input.use_imu_preintegration = true;
  input.use_twist_ekf = true;
  input.has_imu_stamp = false;

  assert(ll::selectPoseBackend(input) == ll::PoseBackendKind::kTwistEkf);

  input.has_imu_stamp = true;
  assert(ll::selectPoseBackend(input) == ll::PoseBackendKind::kImuPreintegration);
}

void test_gtsam_has_highest_priority()
{
  ll::PoseBackendSelectionInput input;
  input.use_gtsam_smoother = true;
  input.use_imu_preintegration = true;
  input.has_imu_stamp = true;
  input.use_twist_ekf = true;

  const auto selected = ll::selectPoseBackend(input);

  assert(selected == ll::PoseBackendKind::kGtsamSmoother);
}

int main()
{
  test_raw_registration_is_default_backend();
  test_twist_ekf_selected_when_only_twist_enabled();
  test_imu_requires_stamp_before_it_can_win();
  test_gtsam_has_highest_priority();
  return 0;
}
