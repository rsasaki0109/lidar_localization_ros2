#ifndef LIDAR_LOCALIZATION_IMU_PREINTEGRATION_GUARD_POLICY_HPP_
#define LIDAR_LOCALIZATION_IMU_PREINTEGRATION_GUARD_POLICY_HPP_

#include <cmath>

namespace lidar_localization
{

constexpr double kDefaultImuSmootherMeasurementTranslationGuardM = 2.0;
constexpr double kDefaultImuSmootherMeasurementRotationGuardDeg = 10.0;

struct ImuPreintegrationGuardParams
{
  double prediction_correction_guard_translation_m{2.0};
  double prediction_correction_guard_yaw_deg{4.0};
  double smoother_measurement_translation_guard_m{
    kDefaultImuSmootherMeasurementTranslationGuardM};
  double smoother_measurement_rotation_guard_deg{
    kDefaultImuSmootherMeasurementRotationGuardDeg};
};

struct ImuPredictionCorrectionGuardInput
{
  bool fallback_mode{false};
  bool imu_prediction_ready{false};
  double correction_translation_m{0.0};
  double correction_yaw_deg{0.0};
};

struct ImuSmootherDivergenceInput
{
  bool pose_all_finite{true};
  double measurement_translation_delta_m{0.0};
  double measurement_rotation_delta_deg{0.0};
};

struct ImuPreintegrationStatusDecision
{
  bool warning{false};
  const char * status_message{"ok"};
};

struct ImuPreintegrationBackendState
{
  bool fallback_mode{false};
  bool state_reset{false};
  bool correction_guard_tripped{false};
  bool smoother_diverged{false};
  bool should_update_smoother{true};
};

inline bool isImuPredictionCorrectionGuardTripped(
  const ImuPreintegrationGuardParams & params,
  const ImuPredictionCorrectionGuardInput & input)
{
  if (input.fallback_mode || !input.imu_prediction_ready) {
    return false;
  }
  const bool translation_guard_tripped =
    std::isfinite(input.correction_translation_m) &&
    input.correction_translation_m > params.prediction_correction_guard_translation_m;
  const bool yaw_guard_tripped =
    std::isfinite(input.correction_yaw_deg) &&
    input.correction_yaw_deg > params.prediction_correction_guard_yaw_deg;
  return translation_guard_tripped || yaw_guard_tripped;
}

inline ImuPreintegrationBackendState beginImuPreintegrationBackendState(
  const ImuPreintegrationGuardParams & params,
  const ImuPredictionCorrectionGuardInput & input)
{
  ImuPreintegrationBackendState state;
  state.correction_guard_tripped =
    isImuPredictionCorrectionGuardTripped(params, input);
  state.fallback_mode = input.fallback_mode || state.correction_guard_tripped;
  state.state_reset = state.correction_guard_tripped;
  state.should_update_smoother = !state.fallback_mode;
  return state;
}

inline bool isImuSmootherDiverged(
  const ImuPreintegrationGuardParams & params,
  const ImuSmootherDivergenceInput & input)
{
  return !input.pose_all_finite ||
         !std::isfinite(input.measurement_translation_delta_m) ||
         !std::isfinite(input.measurement_rotation_delta_deg) ||
         input.measurement_translation_delta_m >
         params.smoother_measurement_translation_guard_m ||
         input.measurement_rotation_delta_deg >
         params.smoother_measurement_rotation_guard_deg;
}

inline ImuPreintegrationBackendState applyImuSmootherDivergenceDecision(
  const ImuPreintegrationBackendState & current,
  const ImuPreintegrationGuardParams & params,
  const ImuSmootherDivergenceInput & input)
{
  if (current.fallback_mode) {
    return current;
  }

  ImuPreintegrationBackendState next = current;
  next.smoother_diverged = isImuSmootherDiverged(params, input);
  if (next.smoother_diverged) {
    next.fallback_mode = true;
    next.state_reset = true;
    next.should_update_smoother = false;
  }
  return next;
}

inline bool shouldUseImuMeasurementPose(const ImuPreintegrationBackendState & state)
{
  return state.fallback_mode;
}

inline ImuPreintegrationStatusDecision decideImuPreintegrationStatus(
  bool imu_state_reset,
  bool imu_correction_guard_tripped,
  bool imu_updated)
{
  if (imu_state_reset) {
    return {
      true,
      imu_correction_guard_tripped ?
      "imu_prediction_correction_guard_imu_disabled" :
      "imu_smoother_diverged_imu_disabled"};
  }
  if (!imu_updated) {
    return {true, "imu_smoother_update_rejected"};
  }
  return {};
}

inline ImuPreintegrationStatusDecision decideImuPreintegrationStatus(
  const ImuPreintegrationBackendState & state,
  bool imu_updated)
{
  return decideImuPreintegrationStatus(
    state.state_reset, state.correction_guard_tripped, imu_updated);
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_IMU_PREINTEGRATION_GUARD_POLICY_HPP_
