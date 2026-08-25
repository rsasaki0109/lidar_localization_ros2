#include "component_internal.hpp"
void PCLLocalization::publishAlignmentStatusForAttempt(
  const builtin_interfaces::msg::Time & stamp,
  uint8_t level,
  const std::string & message,
  const lidar_localization::AlignmentAttempt & attempt,
  std::size_t filtered_point_count,
  bool imu_prediction_active,
  const std::string & registration_seed_source)
{
  publishAlignmentStatus(
    stamp,
    level,
    message,
    attempt.has_converged,
    attempt.fitness_score,
    attempt.alignment_time_sec,
    filtered_point_count,
    attempt.correction_translation_m,
    attempt.correction_yaw_deg,
    attempt.seed_translation_since_accept_m,
    attempt.seed_yaw_since_accept_deg,
    attempt.accepted_gap_sec,
    imu_prediction_active,
    registration_seed_source,
    attempt.registration_localizability);
}

void PCLLocalization::publishAlignmentStatus(
  const builtin_interfaces::msg::Time & stamp,
  uint8_t level,
  const std::string & message,
  bool has_converged,
  double fitness_score,
  double alignment_time_sec,
  std::size_t filtered_point_count,
  double correction_translation_m,
  double correction_yaw_deg,
  double seed_translation_since_accept_m,
  double seed_yaw_since_accept_deg,
  double accepted_gap_sec,
  bool imu_prediction_active,
  const std::string & registration_seed_source,
  const lidar_localization::RegistrationLocalizabilityMetrics & registration_localizability)
{
  if (!status_pub_) {return;}

  const AlignmentStatusPublishInput publish_input{
    stamp,
    level,
    message,
    has_converged,
    fitness_score,
    alignment_time_sec,
    filtered_point_count,
    correction_translation_m,
    correction_yaw_deg,
    seed_translation_since_accept_m,
    seed_yaw_since_accept_deg,
    accepted_gap_sec,
    imu_prediction_active,
    registration_seed_source,
    registration_localizability};

  const auto evaluation = evaluateAlignmentStatus(stamp, publish_input);
  auto status = makeAlignmentDiagnosticStatus(evaluation.status_input);

  appendAlignmentDiagnosticValues(
    status,
    prepareAlignmentDiagnosticValuesInput(
      evaluation.status_input,
      evaluation.status_preparation,
      evaluation.reinitialization_request));
  publishAlignmentDiagnosticStatus(stamp, status);
  publishReinitializationRequest(stamp, evaluation.reinitialization_request);
}

lidar_localization::AlignmentStatusInput PCLLocalization::makeAlignmentStatusInput(
  const AlignmentStatusPublishInput & input) const
{
  const double stamp_sec = stamp_to_sec(input.stamp);
  return lidar_localization::makeAlignmentStatusInput(
    lidar_localization::AlignmentStatusObservation{
      input.level,
      input.message,
      input.has_converged,
      input.fitness_score,
      input.alignment_time_sec,
      input.filtered_point_count,
      input.correction_translation_m,
      input.correction_yaw_deg,
      input.seed_translation_since_accept_m,
      input.seed_yaw_since_accept_deg,
      input.accepted_gap_sec,
      input.imu_prediction_active,
      input.registration_seed_source,
      input.registration_localizability},
    makeAlignmentStatusRuntimeContext(stamp_sec));
}

lidar_localization::AlignmentStatusRuntimeContext
PCLLocalization::makeAlignmentStatusRuntimeContext(double stamp_sec) const
{
  const auto fallback_seed_metrics = lidar_localization::computeAlignmentSeedMetrics(
    have_last_accepted_pose_,
    last_accepted_pose_matrix_,
    predicted_pose_matrix_,
    stamp_sec,
    last_accepted_pose_time_sec_);

  return lidar_localization::AlignmentStatusRuntimeContext{
    registration_method_,
    consecutive_rejected_updates_,
    have_last_accepted_pose_,
    stamp_sec,
    last_accepted_pose_time_sec_,
    fallback_seed_metrics.translation_since_accept_m,
    map_recieved_,
    initialpose_recieved_,
    measurementGateParams(),
    reinitializationTriggerParams(),
    failure_taxonomy_params_};
}

diagnostic_msgs::msg::DiagnosticStatus PCLLocalization::makeAlignmentDiagnosticStatus(
  const lidar_localization::AlignmentStatusInput & status_input) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = status_input.level;
  status.name = "lidar_localization_ros2/alignment";
  status.message = status_input.message;
  status.hardware_id = status_input.registration_method;
  return status;
}

PCLLocalization::AlignmentStatusEvaluation PCLLocalization::evaluateAlignmentStatus(
  const builtin_interfaces::msg::Time & stamp,
  const AlignmentStatusPublishInput & publish_input)
{
  AlignmentStatusEvaluation evaluation;
  evaluation.status_input = makeAlignmentStatusInput(publish_input);
  evaluation.status_preparation =
    lidar_localization::prepareAlignmentStatus(evaluation.status_input);
  evaluation.reinitialization_request =
    applyReinitializationRequestLatch(
      stamp,
      evaluation.status_preparation.reinitialization_request,
      evaluation.status_input.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
      evaluation.status_input.message == "ok" &&
      std::isfinite(evaluation.status_input.fitness_score) &&
      evaluation.status_input.fitness_score <= reinitialization_request_clear_max_fitness_ &&
      std::isfinite(evaluation.status_input.correction_translation_m) &&
      evaluation.status_input.correction_translation_m <=
      reinitialization_request_clear_max_correction_translation_m_);

  const auto recovery_evaluation =
    lidar_localization::evaluateAlignmentStatusRecovery(
      evaluation.status_input,
      evaluation.reinitialization_request);
  updateRecoverySupervisorState(
    stamp,
    recovery_evaluation.state,
    recovery_evaluation.action);
  return evaluation;
}

lidar_localization::AlignmentDiagnosticValuesInput
PCLLocalization::prepareAlignmentDiagnosticValuesInput(
  const lidar_localization::AlignmentStatusInput & status_input,
  const lidar_localization::AlignmentStatusPreparation & status_preparation,
  const ReinitializationRequestDecision & reinitialization_request) const
{
  auto diagnostic_input = lidar_localization::makeAlignmentStatusDiagnosticValuesInput(
    lidar_localization::makeAlignmentStatusDiagnosticInput(
      status_input,
      status_preparation,
      reinitialization_request,
      lidar_localization::AlignmentStatusDiagnosticRuntimeContext{
        lidar_localization::recoverySupervisorStateName(recovery_supervisor_state_),
        recovery_supervisor_action_,
        recovery_supervisor_state_entered_stamp_sec_,
        recovery_supervisor_transition_count_,
        reinitialization_request_latched_,
        reinitialization_request_latch_stamp_sec_}));
  const auto imu_diagnostics = lidar_localization::makeImuPreintegrationDiagnostics(
    makeImuPreintegrationDiagnosticsInput(status_input));
  diagnostic_input.imu_preintegration_enabled = imu_diagnostics.enabled;
  diagnostic_input.imu_preintegration_status =
    lidar_localization::imuPreintegrationDiagnosticStatusMessage(imu_diagnostics.status);
  diagnostic_input.imu_preintegration_fallback_mode = imu_diagnostics.fallback_mode;
  diagnostic_input.imu_smoother_initialized = imu_diagnostics.smoother_initialized;
  diagnostic_input.imu_has_new_samples = imu_diagnostics.has_new_samples;
  diagnostic_input.imu_received_sample_count = imu_diagnostics.received_sample_count;
  diagnostic_input.imu_integrated_sample_count = imu_diagnostics.integrated_sample_count;
  diagnostic_input.imu_skipped_sample_count = imu_diagnostics.skipped_sample_count;
  diagnostic_input.imu_transform_failure_count = imu_diagnostics.transform_failure_count;
  diagnostic_input.imu_non_finite_sample_count = imu_diagnostics.non_finite_sample_count;
  diagnostic_input.imu_invalid_dt_count = imu_diagnostics.invalid_dt_count;
  diagnostic_input.imu_last_dt_sec = imu_diagnostics.last_dt_sec;
  diagnostic_input.imu_last_sample_age_sec = imu_diagnostics.last_sample_age_sec;
  diagnostic_input.imu_integration_window_sec = imu_diagnostics.integration_window_sec;
  diagnostic_input.imu_seed_consistency_gate_enabled =
    imu_diagnostics.seed_consistency_gate_enabled;
  diagnostic_input.imu_seed_consistency_seed_allowed =
    imu_diagnostics.seed_consistency_seed_allowed;
  diagnostic_input.imu_seed_consistency_valid_comparison_count =
    imu_diagnostics.seed_consistency_valid_comparison_count;
  diagnostic_input.imu_seed_consistency_consecutive_pass_count =
    imu_diagnostics.seed_consistency_consecutive_pass_count;
  diagnostic_input.imu_seed_consistency_translation_error_m =
    imu_diagnostics.seed_consistency_translation_error_m;
  diagnostic_input.imu_seed_consistency_rotation_error_deg =
    imu_diagnostics.seed_consistency_rotation_error_deg;
  diagnostic_input.imu_seed_consistency_sample_passed =
    imu_diagnostics.seed_consistency_sample_passed;
  diagnostic_input.scan_time_status =
    lidar_localization::scanTimeRangeStatusMessage(latest_scan_time_status_);
  diagnostic_input.scan_time_field = latest_scan_time_field_;
  diagnostic_input.scan_time_duration_sec = latest_scan_time_duration_sec_;
  diagnostic_input.scan_time_valid_point_count = latest_scan_time_valid_point_count_;
  diagnostic_input.scan_time_invalid_point_count = latest_scan_time_invalid_point_count_;
  const auto deskew_readiness = lidar_localization::makeDeskewReadinessDiagnostics(
    lidar_localization::DeskewReadinessInput{
      latest_scan_time_status_,
      imu_diagnostics.status});
  diagnostic_input.deskew_ready = deskew_readiness.ready;
  diagnostic_input.deskew_readiness_status =
    lidar_localization::deskewReadinessStatusMessage(deskew_readiness.status);
  diagnostic_input.continuous_time_deskew_enabled = use_continuous_time_deskew_;
  diagnostic_input.continuous_time_deskew_applied = latest_continuous_time_deskew_applied_;
  diagnostic_input.continuous_time_deskew_status = latest_continuous_time_deskew_status_;
  diagnostic_input.continuous_time_deskew_point_count =
    latest_continuous_time_deskew_point_count_;
  diagnostic_input.continuous_time_deskew_skipped_invalid_time_count =
    latest_continuous_time_deskew_skipped_invalid_time_count_;
  diagnostic_input.continuous_time_deskew_clamped_time_count =
    latest_continuous_time_deskew_clamped_time_count_;
  diagnostic_input.continuous_time_deskew_mode = continuous_time_deskew_mode_;
  diagnostic_input.continuous_time_deskew_pose_history_coverage_ratio =
    latest_continuous_time_deskew_pose_history_coverage_ratio_;
  diagnostic_input.localizability_guard_enabled = enable_localizability_guard_;
  diagnostic_input.localizability_valid = latest_horizontal_localizability_.valid;
  diagnostic_input.horizontal_localizability_eigenvalue_ratio =
    latest_horizontal_localizability_.eigenvalue_ratio;
  diagnostic_input.localizability_guard_active = latest_localizability_guard_active_;
  return diagnostic_input;
}

lidar_localization::ImuPreintegrationDiagnosticsInput
PCLLocalization::makeImuPreintegrationDiagnosticsInput(
  const lidar_localization::AlignmentStatusInput & status_input) const
{
  std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
  return lidar_localization::ImuPreintegrationDiagnosticsInput{
    use_imu_preintegration_,
    imu_preintegration_fallback_mode_,
    imu_smoother_.isInitialized(),
    last_imu_stamp_ > 0.0,
    latest_imu_seed_has_new_samples_,
    status_input.imu_prediction_active,
    latest_imu_seed_prediction_finite_,
    latest_imu_seed_received_sample_count_,
    latest_imu_seed_integrated_sample_count_,
    latest_imu_seed_skipped_sample_count_,
    latest_imu_seed_transform_failure_count_,
    latest_imu_seed_non_finite_sample_count_,
    latest_imu_seed_invalid_dt_count_,
    latest_imu_seed_last_dt_sec_,
    latest_imu_seed_last_sample_age_sec_,
    latest_imu_seed_integration_window_sec_,
    lidar_localization::imuStaleSampleAgeThresholdSec(scan_period_),
    lidar_localization::imuMaximumIntegrationWindowSec(scan_period_),
    imu_seed_consistency_gate_enabled_,
    imu_seed_consistency_state_.seed_allowed,
    imu_seed_consistency_state_.valid_comparison_count,
    imu_seed_consistency_state_.consecutive_pass_count,
    latest_imu_seed_consistency_translation_error_m_,
    latest_imu_seed_consistency_rotation_error_deg_,
    latest_imu_seed_consistency_sample_passed_};
}

void PCLLocalization::appendAlignmentDiagnosticValues(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const lidar_localization::AlignmentDiagnosticValuesInput & diagnostic_values_input) const
{
  for (const auto & value :
    lidar_localization::makeRosAlignmentDiagnosticKeyValues(diagnostic_values_input))
  {
    status.values.push_back(value);
  }
}

diagnostic_msgs::msg::DiagnosticArray PCLLocalization::makeAlignmentDiagnosticArray(
  const builtin_interfaces::msg::Time & stamp,
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  diagnostic_msgs::msg::DiagnosticArray status_array;
  status_array.header.stamp = stamp;
  status_array.header.frame_id = base_frame_id_;
  status_array.status.push_back(status);
  return status_array;
}

void PCLLocalization::publishAlignmentDiagnosticStatus(
  const builtin_interfaces::msg::Time & stamp,
  const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  status_pub_->publish(makeAlignmentDiagnosticArray(stamp, status));
}

