#include "lidar_localization/registration_seed_policy.hpp"

#include <cassert>
#include <cmath>
#include <string>

namespace ll = lidar_localization;

void test_imu_preintegration_has_highest_priority_when_finite()
{
  const auto decision = ll::chooseRegistrationSeed(
    ll::RegistrationSeedPolicyInput{
      true,
      false,
      true,
      true,
      true,
      true,
      true,
      true,
      true,
      true,
      true,
      true,
      true});

  assert(decision.source == ll::RegistrationSeedSource::kImuPreintegration);
  assert(decision.imu_prediction_ready);
  assert(!decision.uses_prediction_state);
}

void test_non_finite_imu_prediction_blocks_lower_priority_sources()
{
  const auto decision = ll::chooseRegistrationSeed(
    ll::RegistrationSeedPolicyInput{
      true,
      false,
      true,
      true,
      false,
      true,
      true,
      true,
      true,
      true,
      true,
      true,
      true});

  assert(decision.source == ll::RegistrationSeedSource::kCurrentPose);
  assert(!decision.imu_prediction_ready);
  assert(decision.ignored_non_finite_imu_prediction);
}

void test_fallback_order_without_imu_candidate()
{
  ll::RegistrationSeedPolicyInput input;
  input.use_gtsam_smoother = true;
  input.gtsam_smoother_initialized = true;
  input.use_twist_ekf = true;
  input.twist_ekf_initialized = true;
  input.use_twist_prediction = true;
  input.have_last_accepted_pose = true;
  input.has_latest_twist = true;
  input.predict_pose_from_previous_delta = true;
  assert(ll::chooseRegistrationSeed(input).source == ll::RegistrationSeedSource::kGtsamSmoother);

  input.gtsam_smoother_initialized = false;
  assert(ll::chooseRegistrationSeed(input).source == ll::RegistrationSeedSource::kTwistEkf);

  input.twist_ekf_initialized = false;
  assert(ll::chooseRegistrationSeed(input).source == ll::RegistrationSeedSource::kTwistPrediction);
  assert(ll::chooseRegistrationSeed(input).uses_prediction_state);

  input.has_latest_twist = false;
  assert(ll::chooseRegistrationSeed(input).source == ll::RegistrationSeedSource::kPreviousDelta);
  assert(ll::chooseRegistrationSeed(input).uses_prediction_state);

  input.have_last_accepted_pose = false;
  assert(ll::chooseRegistrationSeed(input).source == ll::RegistrationSeedSource::kCurrentPose);
}

void test_odom_tf_prediction_preempts_everything_while_lost()
{
  // Tracking is lost and the odom bridge is available and opted in: it must
  // win even over a fully-ready, finite IMU preintegration candidate -- the
  // whole point is that IMU preintegration itself can be diverging through
  // the same dropout, so the external front end takes over as the anchor.
  ll::RegistrationSeedPolicyInput input;
  input.use_imu_preintegration = true;
  input.imu_smoother_initialized = true;
  input.imu_has_new_samples = true;
  input.imu_prediction_finite = true;
  input.use_odom_tf_prediction = true;
  input.odom_tf_bridge_available = true;

  const auto decision = ll::chooseRegistrationSeed(input);
  assert(decision.source == ll::RegistrationSeedSource::kOdomTfPrediction);
  assert(decision.uses_prediction_state);
}

void test_odom_tf_prediction_off_by_default_preserves_imu_priority()
{
  // Same scenario as above but use_odom_tf_prediction is left at its default
  // (false): behavior must be unchanged -- IMU preintegration still wins.
  ll::RegistrationSeedPolicyInput input;
  input.use_imu_preintegration = true;
  input.imu_smoother_initialized = true;
  input.imu_has_new_samples = true;
  input.imu_prediction_finite = true;
  input.odom_tf_bridge_available = true;
  assert(!input.use_odom_tf_prediction);

  const auto decision = ll::chooseRegistrationSeed(input);
  assert(decision.source == ll::RegistrationSeedSource::kImuPreintegration);
}

void test_odom_tf_prediction_requires_bridge_available_even_when_lost()
{
  // Opted in and lost, but the bridge itself has nothing (no accepted match
  // has ever frozen the offset, or the external front end's odom is stale):
  // must fall through to the normal priority chain, not seed from nothing.
  ll::RegistrationSeedPolicyInput input;
  input.use_imu_preintegration = true;
  input.imu_smoother_initialized = true;
  input.imu_has_new_samples = true;
  input.imu_prediction_finite = true;
  input.use_odom_tf_prediction = true;
  input.odom_tf_bridge_available = false;

  const auto decision = ll::chooseRegistrationSeed(input);
  assert(decision.source == ll::RegistrationSeedSource::kImuPreintegration);
}

void test_odom_tf_prediction_preempts_imu_while_tracking_and_opted_in()
{
  // Normal tracking uses the external odometry as the motion model too. It
  // must win over a ready IMU seed instead of switching only after a reject.
  ll::RegistrationSeedPolicyInput input;
  input.use_imu_preintegration = true;
  input.imu_smoother_initialized = true;
  input.imu_has_new_samples = true;
  input.imu_prediction_finite = true;
  input.use_odom_tf_prediction = true;
  input.odom_tf_bridge_available = true;
  input.use_twist_prediction = true;
  input.have_last_accepted_pose = true;
  input.has_latest_twist = true;
  input.predict_pose_from_previous_delta = true;

  const auto decision = ll::chooseRegistrationSeed(input);
  assert(decision.source == ll::RegistrationSeedSource::kOdomTfPrediction);
  assert(decision.uses_prediction_state);
}

void test_odom_tf_prediction_falls_back_to_twist_when_unavailable_and_tracking()
{
  ll::RegistrationSeedPolicyInput input;
  input.use_odom_tf_prediction = true;
  input.odom_tf_bridge_available = false;
  input.use_twist_prediction = true;
  input.have_last_accepted_pose = true;
  input.has_latest_twist = true;

  const auto decision = ll::chooseRegistrationSeed(input);
  assert(decision.source == ll::RegistrationSeedSource::kTwistPrediction);
}

void test_imu_sample_and_dt_helpers()
{
  assert(ll::hasNewImuSamples(2.0, 1.0));
  assert(!ll::hasNewImuSamples(1.0, 1.0));

  assert(std::abs(ll::clampPredictionDt(10.0, 9.8, 0.5) - 0.2) < 1.0e-9);
  assert(ll::clampPredictionDt(10.0, 9.0, 0.5) == 0.5);
  assert(ll::clampPredictionDt(9.0, 10.0, 0.5) == 0.0);
  assert(ll::clampPredictionDt(NAN, 10.0, 0.5) == 0.0);
  assert(ll::clampPredictionDt(10.0, 9.0, -1.0) == 0.0);
}

void test_registration_seed_source_names_are_stable_for_diagnostics()
{
  assert(
    std::string(ll::registrationSeedSourceName(ll::RegistrationSeedSource::kCurrentPose)) ==
    "current_pose");
  assert(
    std::string(ll::registrationSeedSourceName(ll::RegistrationSeedSource::kImuPreintegration)) ==
    "imu_preintegration");
  assert(
    std::string(ll::registrationSeedSourceName(ll::RegistrationSeedSource::kGtsamSmoother)) ==
    "gtsam_smoother");
  assert(
    std::string(ll::registrationSeedSourceName(ll::RegistrationSeedSource::kTwistEkf)) ==
    "twist_ekf");
  assert(
    std::string(ll::registrationSeedSourceName(ll::RegistrationSeedSource::kTwistPrediction)) ==
    "twist_prediction");
  assert(
    std::string(ll::registrationSeedSourceName(ll::RegistrationSeedSource::kPreviousDelta)) ==
    "previous_delta");
  assert(
    std::string(ll::registrationSeedSourceName(ll::RegistrationSeedSource::kOdomTfPrediction)) ==
    "odom_tf_prediction");
}

int main()
{
  test_imu_preintegration_has_highest_priority_when_finite();
  test_non_finite_imu_prediction_blocks_lower_priority_sources();
  test_fallback_order_without_imu_candidate();
  test_odom_tf_prediction_preempts_everything_while_lost();
  test_odom_tf_prediction_off_by_default_preserves_imu_priority();
  test_odom_tf_prediction_requires_bridge_available_even_when_lost();
  test_odom_tf_prediction_preempts_imu_while_tracking_and_opted_in();
  test_odom_tf_prediction_falls_back_to_twist_when_unavailable_and_tracking();
  test_imu_sample_and_dt_helpers();
  test_registration_seed_source_names_are_stable_for_diagnostics();
  return 0;
}
