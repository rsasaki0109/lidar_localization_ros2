#ifndef LIDAR_LOCALIZATION_REGISTRATION_SEED_POLICY_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_SEED_POLICY_HPP_

#include <algorithm>
#include <cmath>

namespace lidar_localization
{

enum class RegistrationSeedSource
{
  kCurrentPose = 0,
  kImuPreintegration,
  kGtsamSmoother,
  kTwistEkf,
  kTwistPrediction,
  kPreviousDelta,
};

struct RegistrationSeedPolicyInput
{
  bool use_imu_preintegration{false};
  bool imu_preintegration_fallback_mode{false};
  bool imu_smoother_initialized{false};
  bool imu_has_new_samples{false};
  bool imu_prediction_finite{false};
  bool use_gtsam_smoother{false};
  bool gtsam_smoother_initialized{false};
  bool use_twist_ekf{false};
  bool twist_ekf_initialized{false};
  bool use_twist_prediction{false};
  bool have_last_accepted_pose{false};
  bool has_latest_twist{false};
  bool predict_pose_from_previous_delta{false};
};

struct RegistrationSeedPolicyDecision
{
  RegistrationSeedSource source{RegistrationSeedSource::kCurrentPose};
  bool uses_prediction_state{false};
  bool imu_prediction_ready{false};
  bool ignored_non_finite_imu_prediction{false};
};

inline bool hasNewImuSamples(double last_imu_stamp, double last_scan_stamp_for_imu)
{
  return last_imu_stamp > last_scan_stamp_for_imu;
}

inline bool isImuSeedCandidateReady(const RegistrationSeedPolicyInput & input)
{
  return input.use_imu_preintegration &&
         !input.imu_preintegration_fallback_mode &&
         input.imu_smoother_initialized &&
         input.imu_has_new_samples;
}

inline double clampPredictionDt(
  double scan_stamp_sec,
  double predicted_pose_time_sec,
  double max_prediction_dt_sec)
{
  const double raw_dt = scan_stamp_sec - predicted_pose_time_sec;
  if (!std::isfinite(raw_dt) || max_prediction_dt_sec <= 0.0) {
    return 0.0;
  }
  return std::clamp(raw_dt, 0.0, max_prediction_dt_sec);
}

inline RegistrationSeedPolicyDecision chooseRegistrationSeed(
  const RegistrationSeedPolicyInput & input)
{
  const bool imu_candidate_ready = isImuSeedCandidateReady(input);
  if (imu_candidate_ready) {
    if (input.imu_prediction_finite) {
      return {
        RegistrationSeedSource::kImuPreintegration,
        false,
        true,
        false};
    }
    return {
      RegistrationSeedSource::kCurrentPose,
      false,
      false,
      true};
  }
  if (input.use_gtsam_smoother && input.gtsam_smoother_initialized) {
    return {
      RegistrationSeedSource::kGtsamSmoother,
      false,
      false,
      false};
  }
  if (input.use_twist_ekf && input.twist_ekf_initialized) {
    return {
      RegistrationSeedSource::kTwistEkf,
      false,
      false,
      false};
  }
  if (
    input.use_twist_prediction &&
    input.have_last_accepted_pose &&
    input.has_latest_twist)
  {
    return {
      RegistrationSeedSource::kTwistPrediction,
      true,
      false,
      false};
  }
  if (input.predict_pose_from_previous_delta && input.have_last_accepted_pose) {
    return {
      RegistrationSeedSource::kPreviousDelta,
      true,
      false,
      false};
  }
  return {};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_SEED_POLICY_HPP_
