#include "component_internal.hpp"
bool PCLLocalization::applyRegistrationPoseBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  const lidar_localization::MeasurementGateDecision & gate_result,
  const builtin_interfaces::msg::Time & stamp,
  std::size_t filtered_point_count,
  double stamp_sec,
  bool imu_prediction_ready,
  const std::string & registration_seed_source)
{
  const PoseBackendApplyContext context{
    observation,
    attempt,
    gate_result,
    stamp,
    filtered_point_count,
    stamp_sec,
    imu_prediction_ready,
    registration_seed_source};
  return dispatchRegistrationPoseBackend(selectRegistrationPoseBackend(), context);
}

lidar_localization::PoseBackendKind PCLLocalization::selectRegistrationPoseBackend() const
{
  bool has_imu_stamp = false;
  {
    std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
    has_imu_stamp = last_imu_stamp_ > 0.0;
  }
  return lidar_localization::selectPoseBackend(
    lidar_localization::PoseBackendSelectionInput{
      use_gtsam_smoother_,
      use_imu_preintegration_,
      has_imu_stamp,
      use_twist_ekf_});
}

bool PCLLocalization::dispatchRegistrationPoseBackend(
  lidar_localization::PoseBackendKind backend,
  const PoseBackendApplyContext & context)
{
  switch (backend) {
    case lidar_localization::PoseBackendKind::kGtsamSmoother:
      return applyGtsamPoseBackend(context);

    case lidar_localization::PoseBackendKind::kImuPreintegration:
      return applyImuPreintegrationPoseBackend(context);

    case lidar_localization::PoseBackendKind::kTwistEkf:
      return applyTwistEkfPoseBackend(context);

    case lidar_localization::PoseBackendKind::kRawRegistration:
      return applyRawRegistrationPoseBackend(context);
  }

  return applyRawRegistrationPoseBackend(context);
}

bool PCLLocalization::applyGtsamPoseBackend(const PoseBackendApplyContext & context)
{
  updateBackendRollPitch(context.observation);
  return applyPlanarSmootherPoseBackendUpdate(
    updateGtsamPoseBackend(context.observation, context.attempt, context.stamp_sec),
    context);
}

void PCLLocalization::updateBackendRollPitch(
  const lidar_localization::RegistrationObservation & observation)
{
  last_ndt_roll_ = static_cast<float>(observation.roll);
  last_ndt_pitch_ = static_cast<float>(observation.pitch);
}

PCLLocalization::PlanarSmootherBackendUpdate
PCLLocalization::updateGtsamPoseBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  double stamp_sec)
{
  if (!gtsam_smoother_.isInitialized()) {
    gtsam_smoother_.initialize(
      observation.x, observation.y, observation.z, observation.yaw, stamp_sec);
  }

  PlanarSmootherBackendUpdate update;
  update.updated = gtsam_smoother_.update(
    observation.x, observation.y, observation.z, observation.yaw,
    attempt.fitness_score, stamp_sec);
  update.pose = gtsam_smoother_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
  update.rejected_status_message = "gtsam_update_rejected";
  return update;
}

PCLLocalization::PlanarSmootherBackendUpdate
PCLLocalization::updateTwistEkfPoseBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  double stamp_sec)
{
  if (!twist_ekf_.isInitialized()) {
    twist_ekf_.initialize(
      observation.x, observation.y, observation.z, observation.yaw, stamp_sec);
  }

  PlanarSmootherBackendUpdate update;
  update.updated = twist_ekf_.update(
    observation.x, observation.y, observation.z, observation.yaw, attempt.fitness_score);
  update.pose = twist_ekf_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
  update.rejected_status_message = "ekf_update_rejected";
  return update;
}

bool PCLLocalization::applyPlanarSmootherPoseBackendUpdate(
  const PlanarSmootherBackendUpdate & update,
  const PoseBackendApplyContext & context)
{
  const auto backend_result = lidar_localization::applyPoseBackendUpdateStatus(
    lidar_localization::makePoseBackendResult(
      update.pose, context.gate_result.status_level, context.gate_result.status_message),
    update.updated,
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    update.rejected_status_message);
  applyPoseBackendResult(
    backend_result, context.stamp, context.stamp_sec, context.attempt.fitness_score);
  publishAlignmentStatusForAttempt(
    context.stamp,
    backend_result.status_level,
    backend_result.status_message,
    context.attempt,
    context.filtered_point_count,
    context.imu_prediction_ready,
    context.registration_seed_source);
  return true;
}

bool PCLLocalization::applyImuPreintegrationPoseBackend(
  const PoseBackendApplyContext & context)
{
  const auto localization_update =
    lidar_localization::decideLocalizationUpdate(context.gate_result);
  if (localization_update.action != lidar_localization::LocalizationUpdateAction::kAcceptMeasurement) {
    const auto backend_result =
      lidar_localization::makePoseBackendResultFromLocalizationUpdate(
      context.attempt.final_transformation,
      context.gate_result.status_level,
      context.gate_result.status_message,
      localization_update);
    publishAlignmentStatusForAttempt(
      context.stamp,
      backend_result.status_level,
      backend_result.status_message,
      context.attempt,
      context.filtered_point_count,
      context.imu_prediction_ready,
      context.registration_seed_source);
    return applyPoseBackendResult(
      backend_result, context.stamp, context.stamp_sec, context.attempt.fitness_score);
  }

  const auto imu_update = updateImuPreintegrationBackend(
    context.observation, context.attempt, context.stamp_sec, context.imu_prediction_ready);
  logImuPreintegrationBackendWarnings(imu_update, context.attempt);

  const auto backend_result = makeImuPreintegrationPoseBackendResult(
    imu_update, context.gate_result.status_level, context.gate_result.status_message);
  applyPoseBackendResult(
    backend_result, context.stamp, context.stamp_sec, context.attempt.fitness_score);
  publishAlignmentStatusForAttempt(
    context.stamp,
    backend_result.status_level,
    backend_result.status_message,
    context.attempt,
    context.filtered_point_count,
    context.imu_prediction_ready,
    context.registration_seed_source);
  return true;
}

lidar_localization::ImuPreintegrationGuardParams
PCLLocalization::imuPreintegrationGuardParams() const
{
  return {
    imu_prediction_correction_guard_translation_m_,
    imu_prediction_correction_guard_yaw_deg_,
    lidar_localization::kDefaultImuSmootherMeasurementTranslationGuardM,
    lidar_localization::kDefaultImuSmootherMeasurementRotationGuardDeg};
}

void PCLLocalization::initializeImuPreintegrationSmootherIfNeeded(
  const lidar_localization::RegistrationObservation & observation,
  double stamp_sec)
{
  if (imu_smoother_.isInitialized()) {
    return;
  }

  resetImuPreintegrationSmootherToObservation(observation, stamp_sec);
}

void PCLLocalization::resetImuPreintegrationSmootherToObservation(
  const lidar_localization::RegistrationObservation & observation,
  double stamp_sec)
{
  Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
  if (imu_dual_queue_enabled_ && have_last_accepted_pose_) {
    const auto estimate = lidar_localization::estimateImuInitialVelocity(
      last_accepted_pose_matrix_, last_accepted_pose_time_sec_,
      observation.pose_matrix, stamp_sec);
    if (estimate.usable) {
      initial_velocity = estimate.velocity;
    }
  }
  imu_smoother_.initialize(
    observation.x, observation.y, observation.z,
    observation.roll, observation.pitch, observation.yaw,
    initial_velocity.x(), initial_velocity.y(), initial_velocity.z(), stamp_sec);
}

PCLLocalization::ImuPreintegrationBackendUpdate
PCLLocalization::updateImuPreintegrationBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  double stamp_sec,
  bool imu_prediction_ready)
{
  std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
  initializeImuPreintegrationSmootherIfNeeded(observation, stamp_sec);

  if (imu_seed_consistency_gate_enabled_) {
    const bool prediction_available = latest_imu_open_loop_prediction_available_;
    const double translation_error_m = prediction_available ?
      static_cast<double>(
      (latest_imu_open_loop_prediction_.block<3, 1>(0, 3) -
      observation.pose_matrix.block<3, 1>(0, 3)).norm()) :
      std::numeric_limits<double>::quiet_NaN();
    const double rotation_error_deg = prediction_available ?
      lidar_localization::rotationDeltaDeg(
      latest_imu_open_loop_prediction_, observation.pose_matrix) :
      std::numeric_limits<double>::quiet_NaN();
    const bool was_allowed = imu_seed_consistency_state_.seed_allowed;
    const auto consistency_update = lidar_localization::updateImuSeedConsistency(
      imu_seed_consistency_state_,
      imu_seed_consistency_params_,
      lidar_localization::ImuSeedConsistencyInput{
        true, prediction_available, translation_error_m, rotation_error_deg});
    imu_seed_consistency_state_ = consistency_update.state;
    latest_imu_seed_consistency_translation_error_m_ = translation_error_m;
    latest_imu_seed_consistency_rotation_error_deg_ = rotation_error_deg;
    latest_imu_seed_consistency_sample_passed_ = consistency_update.sample_passed;
    if (was_allowed != imu_seed_consistency_state_.seed_allowed) {
      RCLCPP_WARN(
        get_logger(),
        "IMU seed consistency gate %s after %zu consecutive passes "
        "(translation error %.3f m, rotation error %.3f deg)",
        imu_seed_consistency_state_.seed_allowed ? "enabled" : "disabled",
        imu_seed_consistency_state_.consecutive_pass_count,
        translation_error_m,
        rotation_error_deg);
    }
  }

  ImuPreintegrationBackendUpdate update;
  update.pose = observation.pose_matrix;

  const auto guard_params = imuPreintegrationGuardParams();
  update.state = lidar_localization::beginImuPreintegrationBackendState(
    guard_params,
    lidar_localization::ImuPredictionCorrectionGuardInput{
      imu_preintegration_fallback_mode_,
      imu_prediction_ready,
      attempt.correction_translation_m,
      attempt.correction_yaw_deg,
      imu_guard_warmup_accepts_remaining_.load(std::memory_order_acquire)});
  if (
    imu_dual_queue_enabled_ &&
    !update.state.fallback_mode &&
    !update.state.correction_guard_tripped)
  {
    // The consistency gate controls whether prediction may seed NDT, not
    // whether accepted LiDAR poses may teach the optimization-only smoother.
    // If causal IMU coverage is incomplete, resetPendingIntegration() has
    // already removed that factor. A pose-only update then preserves the
    // optimized velocity and biases instead of resetting them from a noisy
    // finite difference across the scheduling gap.
    update.state.should_update_smoother = true;
  }

  if (update.state.should_update_smoother) {
    if (imu_dual_queue_enabled_ && !latest_imu_open_loop_prediction_available_) {
      update.updated = imu_smoother_.updatePoseOnly(
        observation.x, observation.y, observation.z,
        observation.roll, observation.pitch, observation.yaw,
        attempt.fitness_score, stamp_sec);
    } else {
      update.updated = imu_smoother_.update(
        observation.x, observation.y, observation.z,
        observation.roll, observation.pitch, observation.yaw,
        attempt.fitness_score, stamp_sec);
    }

    update.pose = imu_smoother_.poseMatrix();
    if (update.pose.allFinite()) {
      update.smoother_measurement_translation_delta_m = static_cast<double>(
        (update.pose.block<3, 1>(0, 3) -
        observation.pose_matrix.block<3, 1>(0, 3)).norm());
      update.smoother_measurement_rotation_delta_deg =
        lidar_localization::rotationDeltaDeg(observation.pose_matrix, update.pose);
    }

    update.state = lidar_localization::applyImuSmootherDivergenceDecision(
      update.state,
      guard_params,
      lidar_localization::ImuSmootherDivergenceInput{
        update.pose.allFinite(),
        update.smoother_measurement_translation_delta_m,
        update.smoother_measurement_rotation_delta_deg});
  }

  if (
    imu_dual_queue_enabled_ &&
    (update.state.state_reset || !update.updated))
  {
    resetImuPreintegrationSmootherToObservation(observation, stamp_sec);
  } else if (!imu_dual_queue_enabled_ && !update.state.fallback_mode) {
    resetImuPreintegrationSmootherToObservation(observation, stamp_sec);
  }
  imu_preintegration_fallback_mode_ = update.state.fallback_mode;
  if (
    !update.state.fallback_mode ||
    lidar_localization::shouldUseImuMeasurementPose(update.state))
  {
    update.pose = observation.pose_matrix;
    update.updated = true;
  }

  if (imu_guard_warmup_accepts_remaining_.load(std::memory_order_acquire) > 0) {
    if (imu_guard_warmup_accepts_remaining_.fetch_sub(
        1, std::memory_order_acq_rel) == 1)
    {
      RCLCPP_INFO(get_logger(), "post-reset IMU correction guard warmup complete");
    }
  }

  if (imu_dual_queue_enabled_) {
    const auto consumed_optimization =
      imu_dual_queue_.consumeOptimizationThrough(stamp_sec);
    if (!consumed_optimization.empty()) {
      imu_optimization_anchor_sample_ = consumed_optimization.back();
    }
    const auto repropagation_window =
      imu_dual_queue_.preparePredictionRepropagation(stamp_sec);
    if (repropagation_window.anchor_sample.has_value()) {
      imu_prediction_anchor_sample_ = repropagation_window.anchor_sample;
    }
    imu_prediction_smoother_.initializeState(
      imu_smoother_.position(), imu_smoother_.rotation(), imu_smoother_.velocity(),
      imu_smoother_.gyroBias(), imu_smoother_.accelBias(), stamp_sec);
    latest_dual_queue_integrated_stamp_ = stamp_sec;
  }
  last_scan_stamp_for_imu_ = stamp_sec;
  return update;
}

void PCLLocalization::logImuPreintegrationBackendWarnings(
  const ImuPreintegrationBackendUpdate & update,
  const lidar_localization::AlignmentAttempt & attempt)
{
  if (update.state.correction_guard_tripped) {
    RCLCPP_WARN(
      get_logger(),
      "IMU prediction required a large measurement correction (translation=%.3f m, yaw=%.3f deg). Resetting IMU preintegration to the latest LiDAR measurement.",
      attempt.correction_translation_m,
      attempt.correction_yaw_deg);
  }

  if (update.state.smoother_diverged) {
    RCLCPP_WARN(
      get_logger(),
      "IMU smoother diverged from measurement (translation=%.3f m, rotation=%.3f deg). Resetting IMU preintegration to the latest LiDAR measurement.",
      update.smoother_measurement_translation_delta_m,
      update.smoother_measurement_rotation_delta_deg);
  }
}

lidar_localization::PoseBackendResult
PCLLocalization::makeImuPreintegrationPoseBackendResult(
  const ImuPreintegrationBackendUpdate & update,
  uint8_t status_level,
  const std::string & status_message) const
{
  const auto imu_status = lidar_localization::decideImuPreintegrationStatus(
    update.state, update.updated);
  return lidar_localization::applyPoseBackendWarningStatus(
    lidar_localization::makePoseBackendResult(
      update.pose, status_level, status_message),
    imu_status.warning,
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    imu_status.status_message);
}

bool PCLLocalization::applyTwistEkfPoseBackend(
  const PoseBackendApplyContext & context)
{
  updateBackendRollPitch(context.observation);
  return applyPlanarSmootherPoseBackendUpdate(
    updateTwistEkfPoseBackend(context.observation, context.attempt, context.stamp_sec),
    context);
}

bool PCLLocalization::applyRawRegistrationPoseBackend(
  const PoseBackendApplyContext & context)
{
  const auto localization_update =
    lidar_localization::decideLocalizationUpdate(context.gate_result);
  const auto backend_result =
    lidar_localization::makePoseBackendResultFromLocalizationUpdate(
      context.attempt.final_transformation,
      context.gate_result.status_level,
      context.gate_result.status_message,
      localization_update);
  publishAlignmentStatusForAttempt(
    context.stamp,
    backend_result.status_level,
    backend_result.status_message,
    context.attempt,
    context.filtered_point_count,
    context.imu_prediction_ready,
    context.registration_seed_source);

  return applyPoseBackendResult(
    backend_result, context.stamp, context.stamp_sec, context.attempt.fitness_score);
}

bool PCLLocalization::applyPoseBackendResult(
  const lidar_localization::PoseBackendResult & result,
  const builtin_interfaces::msg::Time & stamp,
  double stamp_sec,
  double fitness_score)
{
  if (
    result.advance_prediction_without_measurement ||
    result.update_prediction_from_rejected_measurement)
  {
    return applyPoseBackendPredictionOnlyResult(result, stamp_sec);
  }

  applyAcceptedPoseBackendResult(result, stamp, stamp_sec, fitness_score);
  return result.continue_to_pose_publish;
}

bool PCLLocalization::applyPoseBackendPredictionOnlyResult(
  const lidar_localization::PoseBackendResult & result,
  double stamp_sec)
{
  if (result.advance_prediction_without_measurement) {
    advancePredictionWithoutMeasurement(stamp_sec);
    return result.continue_to_pose_publish;
  }
  if (result.update_prediction_from_rejected_measurement) {
    updatePredictionFromRejectedMeasurement(result.pose_matrix, stamp_sec);
    return result.continue_to_pose_publish;
  }
  return result.continue_to_pose_publish;
}

void PCLLocalization::applyAcceptedPoseBackendResult(
  const lidar_localization::PoseBackendResult & result,
  const builtin_interfaces::msg::Time & stamp,
  double stamp_sec,
  double fitness_score)
{
  if (result.update_current_pose) {
    setCurrentPoseFromMatrix(result.pose_matrix, stamp);
  }
  if (result.fill_pose_covariance) {
    fillPoseCovariance(fitness_score);
  }
  if (result.update_prediction_state) {
    updatePredictionState(result.pose_matrix, stamp_sec);
  }
}

