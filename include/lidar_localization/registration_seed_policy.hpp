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
  kLocalizabilityGuard,
  kOdomTfPrediction,
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
  // Odom-TF-bridge prediction (opt-in, see PCLLocalization::use_odom_tf_prediction_):
  // the composed map -> odom(last accepted) x odom -> base_link(now) pose from an
  // external LIO front end (e.g. GLIM), same math as publishOdomBridgePose.
  // Equivalently "last accepted pose x odom delta since that accept" -- both
  // formulations compose through the same frozen offset and are identical.
  bool use_odom_tf_prediction{false};
  // Whether that composed pose is available this scan (an accepted match has
  // frozen the offset at least once, and the live odom -> base_link edge
  // resolved at the current scan stamp).
  bool odom_tf_bridge_available{false};
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

// The IMU prediction no longer seeds a scan once its window, from the last accepted
// scan, passes imuMaximumIntegrationWindowSec (1 s): on Koide outdoor_hard_01a, with a
// handheld MID-360's IMU, the scan's other seed (the previous scan's motion) recovers
// from short rejection streaks where the IMU's has drifted. It can still seed a second
// alignment, up to this window, when the first is rejected: on the looped Boreas
// fixture, with its Applanix IMU, it carries the pose through LiDAR gaps of 2-5 s where
// the previous scan's motion, advanced at most 1 s, is metres short.
constexpr double kMaximumImuRetrySeedWindowSec = 10.0;

struct ImuRetrySeedInput
{
  bool use_imu_preintegration{false};
  bool imu_preintegration_fallback_mode{false};
  bool imu_smoother_initialized{false};
  // The IMU samples cover the whole window.
  bool imu_has_new_samples{false};
  bool imu_window_too_large{false};
  double imu_integration_window_sec{0.0};
  // The IMU seed consistency gate allows IMU seeds (true when it is off).
  bool imu_seed_allowed{true};
};

inline bool isImuRetrySeedReady(const ImuRetrySeedInput & input)
{
  return input.use_imu_preintegration &&
         !input.imu_preintegration_fallback_mode &&
         input.imu_smoother_initialized &&
         input.imu_has_new_samples &&
         input.imu_window_too_large &&
         std::isfinite(input.imu_integration_window_sec) &&
         input.imu_integration_window_sec <= kMaximumImuRetrySeedWindowSec &&
         input.imu_seed_allowed;
}

inline double clampPredictionDt(
  double scan_stamp_sec,
  double predicted_pose_time_sec,
  double max_prediction_dt_sec)
{
  const double raw_dt = scan_stamp_sec - predicted_pose_time_sec;
  // An initial pose stamped before the first simulated clock has no time
  // anchor. Do not extrapolate it by the maximum interval on the first scan.
  if (predicted_pose_time_sec <= 0.0 || !std::isfinite(raw_dt) ||
    !std::isfinite(max_prediction_dt_sec) || max_prediction_dt_sec <= 0.0)
  {
    return 0.0;
  }
  return std::clamp(raw_dt, 0.0, max_prediction_dt_sec);
}

inline RegistrationSeedPolicyDecision chooseRegistrationSeed(
  const RegistrationSeedPolicyInput & input)
{
  const bool odom_tf_seed_ready =
    input.use_odom_tf_prediction && input.odom_tf_bridge_available;

  // The opt-in external odometry is the map tracker's motion model, not merely
  // a last-resort recovery seed. Prefer it on every scan while available so
  // the seed is the last accepted map pose advanced by the front end's motion
  // since that accept. This also avoids a discontinuous source switch at the
  // first rejected scan. Default-off and unavailable cases fall through to
  // the legacy priority chain unchanged.
  if (odom_tf_seed_ready) {
    return {
      RegistrationSeedSource::kOdomTfPrediction,
      true,
      false,
      false};
  }

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

inline const char * registrationSeedSourceName(RegistrationSeedSource source)
{
  switch (source) {
    case RegistrationSeedSource::kCurrentPose:
      return "current_pose";
    case RegistrationSeedSource::kImuPreintegration:
      return "imu_preintegration";
    case RegistrationSeedSource::kGtsamSmoother:
      return "gtsam_smoother";
    case RegistrationSeedSource::kTwistEkf:
      return "twist_ekf";
    case RegistrationSeedSource::kTwistPrediction:
      return "twist_prediction";
    case RegistrationSeedSource::kPreviousDelta:
      return "previous_delta";
    case RegistrationSeedSource::kLocalizabilityGuard:
      return "localizability_guard_current_pose";
    case RegistrationSeedSource::kOdomTfPrediction:
      return "odom_tf_prediction";
  }
  return "unknown";
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_SEED_POLICY_HPP_
