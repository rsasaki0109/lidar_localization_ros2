#include "lidar_localization/pose_backend_result_policy.hpp"

#include <cassert>

namespace ll = lidar_localization;

Eigen::Matrix4f pose_x(float x)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose(0, 3) = x;
  return pose;
}

void test_pose_backend_result_carries_pose_and_status()
{
  const auto result = ll::makePoseBackendResult(pose_x(3.0f), 0, "ok");

  assert(result.pose_matrix(0, 3) == 3.0f);
  assert(result.status_level == 0);
  assert(result.status_message == "ok");
  assert(result.update_current_pose);
  assert(result.update_prediction_state);
  assert(!result.advance_prediction_without_measurement);
  assert(!result.update_prediction_from_rejected_measurement);
  assert(result.fill_pose_covariance);
  assert(result.continue_to_pose_publish);
}

void test_backend_update_failure_overrides_status_only()
{
  const auto result = ll::applyPoseBackendUpdateStatus(
    ll::makePoseBackendResult(pose_x(5.0f), 0, "ok"),
    false,
    1,
    "backend_update_rejected");

  assert(result.pose_matrix(0, 3) == 5.0f);
  assert(result.status_level == 1);
  assert(result.status_message == "backend_update_rejected");
  assert(result.update_current_pose);
  assert(result.update_prediction_state);
}

void test_backend_update_success_preserves_existing_status()
{
  const auto result = ll::applyPoseBackendUpdateStatus(
    ll::makePoseBackendResult(pose_x(7.0f), 2, "upstream_warning"),
    true,
    1,
    "backend_update_rejected");

  assert(result.pose_matrix(0, 3) == 7.0f);
  assert(result.status_level == 2);
  assert(result.status_message == "upstream_warning");
}

void test_warning_status_applies_only_when_requested()
{
  auto result = ll::applyPoseBackendWarningStatus(
    ll::makePoseBackendResult(pose_x(1.0f), 0, "ok"),
    false,
    1,
    "warning");
  assert(result.status_level == 0);
  assert(result.status_message == "ok");

  result = ll::applyPoseBackendWarningStatus(result, true, 1, "warning");
  assert(result.status_level == 1);
  assert(result.status_message == "warning");
}

void test_localization_accept_maps_to_pose_backend_result()
{
  const auto result = ll::makePoseBackendResultFromLocalizationUpdate(
    pose_x(2.0f),
    0,
    "ok",
    ll::makeAcceptedLocalizationUpdateDecision());

  assert(result.pose_matrix(0, 3) == 2.0f);
  assert(result.update_current_pose);
  assert(result.update_prediction_state);
  assert(!result.advance_prediction_without_measurement);
  assert(!result.update_prediction_from_rejected_measurement);
  assert(result.fill_pose_covariance);
  assert(result.continue_to_pose_publish);
}

void test_localization_reject_maps_to_prediction_advance_result()
{
  const auto result = ll::makePoseBackendResultFromLocalizationUpdate(
    pose_x(4.0f),
    1,
    "fitness_score_over_threshold_rejected",
    ll::makePredictionAdvanceLocalizationUpdateDecision());

  assert(!result.update_current_pose);
  assert(!result.update_prediction_state);
  assert(result.advance_prediction_without_measurement);
  assert(!result.update_prediction_from_rejected_measurement);
  assert(!result.fill_pose_covariance);
  assert(!result.continue_to_pose_publish);
}

void test_localization_rejected_seed_maps_to_prediction_update_result()
{
  const auto result = ll::makePoseBackendResultFromLocalizationUpdate(
    pose_x(6.0f),
    1,
    "fitness_score_over_threshold_rejected",
    ll::makeRejectedSeedLocalizationUpdateDecision());

  assert(!result.update_current_pose);
  assert(!result.update_prediction_state);
  assert(!result.advance_prediction_without_measurement);
  assert(result.update_prediction_from_rejected_measurement);
  assert(!result.fill_pose_covariance);
  assert(!result.continue_to_pose_publish);
}

int main()
{
  test_pose_backend_result_carries_pose_and_status();
  test_backend_update_failure_overrides_status_only();
  test_backend_update_success_preserves_existing_status();
  test_warning_status_applies_only_when_requested();
  test_localization_accept_maps_to_pose_backend_result();
  test_localization_reject_maps_to_prediction_advance_result();
  test_localization_rejected_seed_maps_to_prediction_update_result();
  return 0;
}
