#include "lidar_localization/prediction_state_policy.hpp"

#include <cassert>
#include <cmath>

namespace ll = lidar_localization;

Eigen::Matrix4f pose_x(float x)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose(0, 3) = x;
  return pose;
}

bool near(float lhs, float rhs)
{
  return std::abs(lhs - rhs) < 1.0e-5f;
}

void test_reset_prediction_state()
{
  const auto pose = pose_x(3.0f);
  const auto state = ll::resetPredictionState(pose, 10.0);

  assert(state.have_last_accepted_pose);
  assert(near(state.last_accepted_pose_matrix(0, 3), 3.0f));
  assert(near(state.predicted_pose_matrix(0, 3), 3.0f));
  assert(state.last_relative_motion_matrix.isApprox(Eigen::Matrix4f::Identity()));
  assert(state.consecutive_rejected_updates == 0);
  assert(state.last_accepted_pose_time_sec == 10.0);
  assert(state.predicted_pose_time_sec == 10.0);
}

void test_accepted_measurement_initializes_when_empty()
{
  ll::PredictionStateSnapshot empty;
  const auto state =
    ll::updatePredictionStateFromAcceptedMeasurement(empty, pose_x(2.0f), 5.0);

  assert(state.have_last_accepted_pose);
  assert(near(state.last_accepted_pose_matrix(0, 3), 2.0f));
  assert(near(state.predicted_pose_matrix(0, 3), 2.0f));
}

void test_accepted_measurement_updates_relative_motion_without_reject_streak()
{
  auto state = ll::resetPredictionState(pose_x(1.0f), 1.0);
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(3.0f), 2.0);

  assert(near(state.last_relative_motion_matrix(0, 3), 2.0f));
  assert(near(state.last_accepted_pose_matrix(0, 3), 3.0f));
  assert(near(state.predicted_pose_matrix(0, 3), 5.0f));
  assert(state.consecutive_rejected_updates == 0);
  assert(state.last_accepted_pose_time_sec == 2.0);
  assert(state.predicted_pose_time_sec == 2.0);
}

void test_accepted_measurement_preserves_relative_motion_after_reject_streak()
{
  auto state = ll::resetPredictionState(pose_x(1.0f), 1.0);
  state.last_relative_motion_matrix = pose_x(0.5f);
  state.consecutive_rejected_updates = 3;

  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(3.0f), 2.0);

  assert(near(state.last_relative_motion_matrix(0, 3), 0.5f));
  assert(near(state.predicted_pose_matrix(0, 3), 3.5f));
  assert(state.consecutive_rejected_updates == 0);
}

void test_choose_prediction_advance_mode()
{
  assert(
    ll::choosePredictionAdvanceMode(false, true, true, true) ==
    ll::PredictionAdvanceMode::kNone);
  assert(
    ll::choosePredictionAdvanceMode(true, true, true, true) ==
    ll::PredictionAdvanceMode::kTwistPrediction);
  assert(
    ll::choosePredictionAdvanceMode(true, true, false, true) ==
    ll::PredictionAdvanceMode::kPreviousDelta);
  assert(
    ll::choosePredictionAdvanceMode(true, false, false, false) ==
    ll::PredictionAdvanceMode::kNone);
}

void test_advance_prediction_without_measurement()
{
  auto state = ll::resetPredictionState(pose_x(1.0f), 1.0);
  state.predicted_pose_matrix = pose_x(3.0f);
  state.last_relative_motion_matrix = pose_x(2.0f);

  auto advanced = ll::advancePredictionWithoutMeasurement(
    state, 2.0, ll::PredictionAdvanceMode::kPreviousDelta);
  assert(near(advanced.predicted_pose_matrix(0, 3), 5.0f));
  assert(advanced.predicted_pose_time_sec == 2.0);
  assert(advanced.consecutive_rejected_updates == 1);

  advanced = ll::advancePredictionWithoutMeasurement(
    state, 3.0, ll::PredictionAdvanceMode::kTwistPrediction, pose_x(7.0f));
  assert(near(advanced.predicted_pose_matrix(0, 3), 7.0f));
  assert(advanced.predicted_pose_time_sec == 3.0);
  assert(advanced.consecutive_rejected_updates == 1);

  const auto unchanged = ll::advancePredictionWithoutMeasurement(
    state, 4.0, ll::PredictionAdvanceMode::kNone);
  assert(near(unchanged.predicted_pose_matrix(0, 3), 3.0f));
  assert(unchanged.consecutive_rejected_updates == 0);
}

void test_rejected_measurement_updates_prediction_only()
{
  auto state = ll::resetPredictionState(pose_x(1.0f), 1.0);
  state.last_relative_motion_matrix = pose_x(0.25f);

  const auto updated = ll::updatePredictionFromRejectedMeasurement(state, pose_x(4.0f), 2.0);
  assert(near(updated.last_accepted_pose_matrix(0, 3), 1.0f));
  assert(near(updated.predicted_pose_matrix(0, 3), 4.0f));
  assert(near(updated.last_relative_motion_matrix(0, 3), 0.25f));
  assert(updated.predicted_pose_time_sec == 2.0);
  assert(updated.consecutive_rejected_updates == 1);

  ll::PredictionStateSnapshot empty;
  const auto unchanged = ll::updatePredictionFromRejectedMeasurement(empty, pose_x(9.0f), 3.0);
  assert(!unchanged.have_last_accepted_pose);
  assert(near(unchanged.predicted_pose_matrix(0, 3), 0.0f));
}

int main()
{
  test_reset_prediction_state();
  test_accepted_measurement_initializes_when_empty();
  test_accepted_measurement_updates_relative_motion_without_reject_streak();
  test_accepted_measurement_preserves_relative_motion_after_reject_streak();
  test_choose_prediction_advance_mode();
  test_advance_prediction_without_measurement();
  test_rejected_measurement_updates_prediction_only();
  return 0;
}
