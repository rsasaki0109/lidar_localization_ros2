#include "lidar_localization/registration_seed_policy.hpp"

#include <cassert>
#include <cmath>

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

int main()
{
  test_imu_preintegration_has_highest_priority_when_finite();
  test_non_finite_imu_prediction_blocks_lower_priority_sources();
  test_fallback_order_without_imu_candidate();
  test_imu_sample_and_dt_helpers();
  return 0;
}
