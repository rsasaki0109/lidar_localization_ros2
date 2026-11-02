#include "lidar_localization/prediction_state_policy.hpp"
#include "lidar_localization/alignment_retry_policy.hpp"
#include "lidar_localization/registration_seed_policy.hpp"

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
  assert(unchanged.consecutive_rejected_updates == 1);
  assert(unchanged.predicted_pose_time_sec == state.predicted_pose_time_sec);
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

void test_accepted_measurement_without_previous_delta_extrapolation()
{
  Eigen::Matrix4f first = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f second = Eigen::Matrix4f::Identity();
  second(0, 3) = 1.0f;
  auto state = ll::resetPredictionState(first, 0.0);
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, second, 0.1, false);
  // Twist prediction starts from the accepted pose; the delta is still kept.
  assert(near(state.predicted_pose_matrix(0, 3), 1.0f));
  assert(near(state.last_relative_motion_matrix(0, 3), 1.0f));
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, second, 0.2);
  assert(near(state.predicted_pose_matrix(0, 3), 1.0f));  // zero delta now
}

void test_rejections_without_prediction_enable_retry_and_preserve_motion()
{
  auto state = ll::resetPredictionState(pose_x(0.0f), 10.0);
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(0.1f), 10.1, false);
  for (int i = 1; i <= 3; ++i) {
    state = ll::advancePredictionWithoutMeasurement(
      state, 10.1 + 0.1 * i, ll::PredictionAdvanceMode::kNone);
    const auto retry = ll::decideRecoveryRetryFromLastPose(
      {true, 3, 1.0, 10.0},
      {true, state.consecutive_rejected_updates, 0.4, 0.4, 0.0});
    assert(state.consecutive_rejected_updates == static_cast<std::size_t>(i));
    assert(retry.should_retry == (i == 3));
    assert(near(state.predicted_pose_matrix(0, 3), 0.1f));
    assert(state.predicted_pose_time_sec == 10.1);
    assert(state.last_accepted_pose_time_sec == 10.1);
  }
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(0.5f), 10.5, false);
  assert(state.consecutive_rejected_updates == 0);
  assert(near(state.last_relative_motion_matrix(0, 3), 0.1f));
  assert(near(state.predicted_pose_matrix(0, 3), 0.5f));

  const ll::PredictionStateSnapshot empty;
  const auto unchanged = ll::advancePredictionWithoutMeasurement(
    empty, 10.0, ll::PredictionAdvanceMode::kNone);
  assert(!unchanged.have_last_accepted_pose);
  assert(unchanged.consecutive_rejected_updates == 0);
}

void test_twist_dropout_and_return_keep_a_single_time_anchor()
{
  // Constant 1 m/s motion. Keep the stored pose at the last processed scan,
  // even when the next seed must use previous-delta rather than twist.
  auto state = ll::resetPredictionState(pose_x(0.0f), 10.0);
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(0.1f), 10.1, false);
  const Eigen::Matrix4f delta_seed =
    state.predicted_pose_matrix * state.last_relative_motion_matrix;
  assert(near(delta_seed(0, 3), 0.2f));
  state = ll::advancePredictionWithoutMeasurement(
    state, 10.2, ll::PredictionAdvanceMode::kPreviousDelta);
  assert(near(state.predicted_pose_matrix(0, 3), 0.2f));
  // Twist returns after a rejected dropout scan: advance only 10.2 -> 10.3.
  double dt = ll::clampPredictionDt(10.3, state.predicted_pose_time_sec, 1.5);
  assert(near(state.predicted_pose_matrix(0, 3) + dt, 0.3f));
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(0.3f), 10.3, false);
  // Twist also returns after an accepted fallback scan: no stored future delta.
  dt = ll::clampPredictionDt(10.4, state.predicted_pose_time_sec, 1.5);
  assert(near(state.predicted_pose_matrix(0, 3) + dt, 0.4f));
  const Eigen::Matrix4f next_delta_seed =
    state.predicted_pose_matrix * state.last_relative_motion_matrix;
  assert(near(next_delta_seed(0, 3), 0.4f));
}

// The component stores the accepted pose at its own time (no extrapolation) and
// seeds the next scan with predicted_pose_matrix * previousDeltaMotionAt(...).
void test_previous_delta_motion_scales_with_elapsed_time()
{
  auto state = ll::resetPredictionState(pose_x(0.0f), 10.0);
  // 1 m in 0.1 s.
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(1.0f), 10.1, false);
  const double duration = 0.1;

  // The next scan on time: one delta.
  assert(near(ll::previousDeltaMotionAt(state, duration, 10.2, 1.0)(0, 3), 1.0f));
  // Six scans dropped: 0.7 s, seven deltas, where one delta per scan gave 1 m.
  assert(near(ll::previousDeltaMotionAt(state, duration, 10.8, 1.0)(0, 3), 7.0f));
  // Extrapolated for at most max_extrapolation_sec.
  assert(near(ll::previousDeltaMotionAt(state, duration, 12.0, 1.0)(0, 3), 10.0f));
  // The seed of the first scan after the gap, and the advance when it is rejected.
  const Eigen::Matrix4f seed =
    state.predicted_pose_matrix * ll::previousDeltaMotionAt(state, duration, 10.8, 1.0);
  assert(near(seed(0, 3), 8.0f));
  const auto advanced = ll::advancePredictionWithoutMeasurement(
    state, 10.8, ll::PredictionAdvanceMode::kPreviousDelta, Eigen::Matrix4f::Identity(),
    ll::previousDeltaMotionAt(state, duration, 10.8, 1.0));
  assert(near(advanced.predicted_pose_matrix(0, 3), 8.0f));
  assert(advanced.predicted_pose_time_sec == 10.8);

  // After the gap the delta spans 0.7 s: the next scan, 0.1 s later, gets a seventh
  // of it instead of all of it.
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, pose_x(8.0f), 10.8, false);
  assert(near(ll::previousDeltaMotionAt(state, 0.7, 10.9, 1.0)(0, 3), 1.0f));

  // No usable duration, time anchor or limit: the stored motion, as before.
  assert(near(ll::previousDeltaMotionAt(state, 0.0, 11.5, 1.0)(0, 3), 7.0f));
  assert(near(ll::previousDeltaMotionAt(state, 0.7, 11.5, 0.0)(0, 3), 7.0f));
  assert(near(ll::previousDeltaMotionAt(state, 0.7, 10.8, 1.0)(0, 3), 7.0f));
  auto unanchored = state;
  unanchored.predicted_pose_time_sec = 0.0;
  assert(near(ll::previousDeltaMotionAt(unanchored, 0.7, 11.5, 1.0)(0, 3), 7.0f));
}

void test_previous_delta_rotation_scales_about_its_axis()
{
  auto state = ll::resetPredictionState(Eigen::Matrix4f::Identity(), 10.0);
  Eigen::Matrix4f turned = Eigen::Matrix4f::Identity();
  turned.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(0.02f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  turned(0, 3) = 0.5f;
  state = ll::updatePredictionStateFromAcceptedMeasurement(state, turned, 10.1, false);
  const Eigen::Matrix4f motion = ll::previousDeltaMotionAt(state, 0.1, 10.6, 1.0);
  const Eigen::AngleAxisf rotation(Eigen::Matrix3f(motion.block<3, 3>(0, 0)));
  assert(std::abs(rotation.angle() - 0.1f) < 1.0e-5f);
  assert(std::abs(rotation.axis().z() - 1.0f) < 1.0e-5f);
  assert(near(motion(0, 3), 2.5f));
  // No motion stays no motion.
  auto still = ll::resetPredictionState(Eigen::Matrix4f::Identity(), 10.0);
  still = ll::updatePredictionStateFromAcceptedMeasurement(
    still, Eigen::Matrix4f::Identity(), 10.1, false);
  assert(ll::previousDeltaMotionAt(still, 0.1, 10.9, 1.0).isApprox(Eigen::Matrix4f::Identity()));
}

int main()
{
  test_rejections_without_prediction_enable_retry_and_preserve_motion();
  test_twist_dropout_and_return_keep_a_single_time_anchor();
  test_accepted_measurement_without_previous_delta_extrapolation();
  test_reset_prediction_state();
  test_accepted_measurement_initializes_when_empty();
  test_accepted_measurement_updates_relative_motion_without_reject_streak();
  test_accepted_measurement_preserves_relative_motion_after_reject_streak();
  test_choose_prediction_advance_mode();
  test_advance_prediction_without_measurement();
  test_rejected_measurement_updates_prediction_only();
  test_previous_delta_motion_scales_with_elapsed_time();
  test_previous_delta_rotation_scales_about_its_axis();
  return 0;
}
