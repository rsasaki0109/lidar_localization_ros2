#include "component_internal.hpp"
PCLLocalization::SelectedRegistrationSeed PCLLocalization::selectRegistrationSeed(
  const builtin_interfaces::msg::Time & stamp, double scan_stamp_sec)
{
  SelectedRegistrationSeed selected_seed;
  selected_seed.init_guess = currentPoseMatrix();
  Eigen::Matrix4f imu_init_guess = Eigen::Matrix4f::Identity();
  bool imu_preintegration_fallback_mode = false;
  bool imu_smoother_initialized = false;
  bool imu_has_usable_new_samples = false;
  bool imu_prediction_finite = false;
  {
    std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
    bool dual_queue_prediction_ready = false;
    Eigen::Matrix4f dual_queue_prediction = Eigen::Matrix4f::Identity();
    if (imu_dual_queue_enabled_ && imu_smoother_.isInitialized()) {
      std::vector<lidar_localization::TimestampedImuSample> optimization_samples;
      if (imu_optimization_anchor_sample_.has_value()) {
        optimization_samples.push_back(*imu_optimization_anchor_sample_);
      }
      const auto queued_optimization_samples =
        imu_dual_queue_.optimizationSamplesThrough(scan_stamp_sec);
      optimization_samples.insert(
        optimization_samples.end(),
        queued_optimization_samples.begin(), queued_optimization_samples.end());
      const auto optimization_plan = lidar_localization::planCausalImuIntervals(
        last_scan_stamp_for_imu_, scan_stamp_sec, optimization_samples,
        lidar_localization::kMaximumImuPreintegrationSampleDtSec);
      imu_smoother_.resetPendingIntegration();
      if (optimization_plan.complete_coverage) {
        for (const auto & interval : optimization_plan.intervals) {
          imu_smoother_.integrateImu(interval.gyro, interval.accel, interval.dt());
          ++imu_preintegration_integrated_sample_count_since_scan_;
          imu_preintegration_last_dt_sec_ = interval.dt();
        }
      } else {
        imu_smoother_.resetPendingIntegration();
        imu_preintegration_skipped_sample_count_since_scan_ +=
          optimization_plan.skipped_gap_count;
      }

      std::vector<lidar_localization::TimestampedImuSample> prediction_samples;
      if (imu_prediction_anchor_sample_.has_value()) {
        prediction_samples.push_back(*imu_prediction_anchor_sample_);
      }
      const auto queued_prediction_samples = imu_dual_queue_.predictionSamples();
      prediction_samples.insert(
        prediction_samples.end(),
        queued_prediction_samples.begin(), queued_prediction_samples.end());
      const auto prediction_plan = lidar_localization::planCausalImuIntervals(
        last_scan_stamp_for_imu_, scan_stamp_sec, prediction_samples,
        lidar_localization::kMaximumImuPreintegrationSampleDtSec);
      imu_prediction_smoother_.initializeState(
        imu_smoother_.position(), imu_smoother_.rotation(), imu_smoother_.velocity(),
        imu_smoother_.gyroBias(), imu_smoother_.accelBias(),
        last_scan_stamp_for_imu_);
      if (prediction_plan.complete_coverage && !prediction_plan.intervals.empty()) {
        for (const auto & interval : prediction_plan.intervals) {
          imu_prediction_smoother_.integrateImu(
            interval.gyro, interval.accel, interval.dt());
        }
        dual_queue_prediction = imu_prediction_smoother_.predictedPoseMatrix();
        dual_queue_prediction_ready = dual_queue_prediction.allFinite();
        latest_dual_queue_integrated_stamp_ = scan_stamp_sec;
      }
    }
    snapshotImuPreintegrationSampleCountersForScan();

    const bool imu_has_new_samples = imu_dual_queue_enabled_ ?
      dual_queue_prediction_ready :
      lidar_localization::hasNewImuSamples(last_imu_stamp_, last_scan_stamp_for_imu_);
    latest_imu_seed_has_new_samples_ = imu_has_new_samples;
    latest_imu_seed_last_sample_age_sec_ =
      lidar_localization::imuLastSampleAgeSec(
      scan_stamp_sec,
      imu_dual_queue_enabled_ ? latest_dual_queue_integrated_stamp_ : last_imu_stamp_);
    latest_imu_seed_integration_window_sec_ =
      imu_dual_queue_enabled_ ?
      std::max(0.0, scan_stamp_sec - last_scan_stamp_for_imu_) :
      lidar_localization::imuIntegrationWindowSec(last_imu_stamp_, last_scan_stamp_for_imu_);
    const double max_imu_integration_window_sec =
      lidar_localization::imuMaximumIntegrationWindowSec(scan_period_);
    const bool imu_window_too_large =
      lidar_localization::isImuIntegrationWindowTooLarge(
        latest_imu_seed_integration_window_sec_, max_imu_integration_window_sec);
    if (imu_has_new_samples && imu_window_too_large) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring IMU preintegration prediction because its integration window %.3f sec exceeds %.3f sec.",
        latest_imu_seed_integration_window_sec_,
        max_imu_integration_window_sec);
    }
    imu_preintegration_fallback_mode = imu_preintegration_fallback_mode_;
    imu_smoother_initialized = imu_smoother_.isInitialized();
    imu_has_usable_new_samples = imu_has_new_samples && !imu_window_too_large;
    const bool imu_candidate_ready =
      use_imu_preintegration_ &&
      !imu_preintegration_fallback_mode &&
      imu_smoother_initialized &&
      imu_has_usable_new_samples;
    if (imu_candidate_ready) {
      imu_init_guess = imu_dual_queue_enabled_ ?
        dual_queue_prediction : imu_smoother_.predictedPoseMatrix();
      imu_prediction_finite = imu_init_guess.allFinite();
    }
    latest_imu_open_loop_prediction_ = imu_init_guess;
    latest_imu_open_loop_prediction_available_ =
      imu_candidate_ready && imu_prediction_finite;
    latest_imu_seed_prediction_finite_ = !imu_candidate_ready || imu_prediction_finite;
    imu_has_usable_new_samples =
      imu_has_usable_new_samples &&
      (!imu_seed_consistency_gate_enabled_ || imu_seed_consistency_state_.seed_allowed);
  }

  Eigen::Matrix4f odom_tf_prediction = Eigen::Matrix4f::Identity();
  const bool odom_tf_bridge_available =
    use_odom_tf_prediction_ && lookupOdomBridgePoseMatrix(stamp, odom_tf_prediction);

  const lidar_localization::RegistrationSeedPolicyDecision seed_decision =
    lidar_localization::chooseRegistrationSeed(
    lidar_localization::RegistrationSeedPolicyInput{
      use_imu_preintegration_,
      imu_preintegration_fallback_mode,
      imu_smoother_initialized,
      imu_has_usable_new_samples,
      imu_prediction_finite,
      use_gtsam_smoother_,
      gtsam_smoother_.isInitialized(),
      use_twist_ekf_,
      twist_ekf_.isInitialized(),
      use_twist_prediction_,
      have_last_accepted_pose_,
      static_cast<bool>(latest_twist_msg_),
      predict_pose_from_previous_delta_,
      use_odom_tf_prediction_,
      odom_tf_bridge_available});
  selected_seed.source = seed_decision.source;
  selected_seed.imu_prediction_ready = seed_decision.imu_prediction_ready;

  switch (seed_decision.source) {
    case lidar_localization::RegistrationSeedSource::kImuPreintegration:
      selected_seed.init_guess = imu_init_guess;
      break;
    case lidar_localization::RegistrationSeedSource::kGtsamSmoother:
      selected_seed.init_guess =
        gtsam_smoother_.predictedPoseMatrix(last_ndt_roll_, last_ndt_pitch_);
      break;
    case lidar_localization::RegistrationSeedSource::kTwistEkf:
      selected_seed.init_guess = twist_ekf_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
      break;
    case lidar_localization::RegistrationSeedSource::kTwistPrediction: {
      const double dt = lidar_localization::clampPredictionDt(
        scan_stamp_sec, predicted_pose_time_sec_, max_twist_prediction_dt_);
      selected_seed.init_guess = applyTwistPrediction(predicted_pose_matrix_, dt);
      break;
    }
    case lidar_localization::RegistrationSeedSource::kPreviousDelta:
      if (latest_localizability_guard_active_) {
        selected_seed.source =
          lidar_localization::RegistrationSeedSource::kLocalizabilityGuard;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Suppressing previous-delta seed: horizontal localizability ratio %.4f < %.4f.",
          latest_horizontal_localizability_.eigenvalue_ratio,
          localizability_min_xy_eigen_ratio_);
      } else {
        selected_seed.init_guess = predicted_pose_matrix_;
      }
      break;
    case lidar_localization::RegistrationSeedSource::kLocalizabilityGuard:
    case lidar_localization::RegistrationSeedSource::kCurrentPose:
      break;
    case lidar_localization::RegistrationSeedSource::kOdomTfPrediction:
      selected_seed.init_guess = odom_tf_prediction;
      break;
  }

  if (seed_decision.ignored_non_finite_imu_prediction) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring non-finite IMU predicted pose and falling back to non-IMU seed.");
  }
  return selected_seed;
}

Eigen::Matrix4f PCLLocalization::refineSeedWithNdtInitializer(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud,
  const Eigen::Matrix4f & init_guess)
{
  const auto run_input = lidar_localization::NdtInitializerRunInput{
    use_ndt_initializer_,
    static_cast<bool>(ndt_initializer_) && static_cast<bool>(source_cloud),
    ndt_init_scan_count_,
    ndt_init_scans_required_};
  if (!lidar_localization::shouldRunNdtInitializer(run_input)) {
    return init_guess;
  }

  ndt_initializer_->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ndt_output(new pcl::PointCloud<pcl::PointXYZI>);
  ndt_initializer_->align(*ndt_output, init_guess);
  const auto progress_decision =
    lidar_localization::updateNdtInitializerProgress(
    lidar_localization::NdtInitializerProgressInput{
      run_input,
      ndt_initializer_->hasConverged()});
  if (!progress_decision.should_accept_refined_seed) {
    return init_guess;
  }

  Eigen::Matrix4f refined_seed = ndt_initializer_->getFinalTransformation();
  ndt_init_scan_count_ = progress_decision.next_scan_count;
  RCLCPP_INFO(
    get_logger(), "NDT init scan %d/%d fitness=%.3f",
    ndt_init_scan_count_, ndt_init_scans_required_, ndt_initializer_->getFitnessScore());
  if (progress_decision.should_reset_initializer) {
    RCLCPP_INFO(get_logger(), "NDT init complete, switching to %s", registration_method_.c_str());
    ndt_initializer_.reset();
  }
  return refined_seed;
}

void PCLLocalization::warmUpRegistrationTarget(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & target)
{
  // pcl::Registration builds its target search tree lazily inside the first
  // align() call (even for NDT, which does not use it).  On a large map that
  // takes about a second and stalls the first scan, dropping the following
  // ones while the robot keeps moving.  Pay that cost at map load instead by
  // aligning a small probe cloud once.
  if (!registration_ || !target || target->empty()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr probe(new pcl::PointCloud<pcl::PointXYZI>());
  const std::size_t stride = std::max<std::size_t>(1, target->size() / 100);
  for (std::size_t i = 0; i < target->size(); i += stride) {
    probe->push_back(target->points[i]);
  }
  const auto start = std::chrono::steady_clock::now();
  registration_->setInputSource(probe);
  lidar_localization::keepRegistrationCloudAlive(
    recent_source_clouds_, probe, registration_source_cloud_keep_alive_count_);
  pcl::PointCloud<pcl::PointXYZI> output;
  registration_->align(output, Eigen::Matrix4f::Identity());
  RCLCPP_INFO(
    get_logger(), "Registration target warm-up took %.3f s",
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
}

bool PCLLocalization::setInputTargetForPose(const Eigen::Matrix4f & center_pose_matrix)
{
  if (!use_local_map_crop_ || !full_map_cloud_ptr_) {
    return true;
  }

  const float cx = center_pose_matrix(0, 3);
  const float cy = center_pose_matrix(1, 3);
  const lidar_localization::LocalMapCropRequest crop_request{
    cx,
    cy,
    local_map_radius_,
    local_map_min_points_,
    lidar_localization::LocalMapBounds2d{
      map_bounds_valid_,
      map_min_pt_.x,
      map_max_pt_.x,
      map_min_pt_.y,
      map_max_pt_.y}};
  const auto steady_now = std::chrono::steady_clock::now();

  auto handle_crop_failure = [&](lidar_localization::LocalMapTargetFailure failure) {
      const bool has_last_streak_log =
        last_crop_failure_streak_log_time_ != std::chrono::steady_clock::time_point{};
      const auto elapsed_since_streak_log =
        has_last_streak_log ?
        steady_now - last_crop_failure_streak_log_time_ :
        std::chrono::steady_clock::duration::zero();
      const bool has_last_bounds_log =
        last_crop_out_of_bounds_log_time_ != std::chrono::steady_clock::time_point{};
      const auto elapsed_since_bounds_log =
        has_last_bounds_log ?
        steady_now - last_crop_out_of_bounds_log_time_ :
        std::chrono::steady_clock::duration::zero();
      const auto decision = lidar_localization::handleLocalMapTargetFailure(
        lidar_localization::LocalMapTargetFailureHandlingInput{
          failure,
          consecutive_crop_failures_,
          has_last_streak_log,
          elapsed_since_streak_log,
          has_last_bounds_log,
          elapsed_since_bounds_log});
      consecutive_crop_failures_ = decision.consecutive_crop_failures;
      if (decision.should_log_failure_streak) {
        last_crop_failure_streak_log_time_ = steady_now;
        RCLCPP_WARN(
          get_logger(),
          "Crop failure streak reached %d while target setup keeps failing.",
          consecutive_crop_failures_);
      }
      if (decision.should_log_out_of_bounds) {
        last_crop_out_of_bounds_log_time_ = steady_now;
      }
      return decision;
    };

  const auto crop_validation =
    lidar_localization::validateLocalMapCropRequest(crop_request);
  if (!crop_validation.can_crop) {
    const auto failure_decision = handle_crop_failure(crop_validation.failure);
    if (failure_decision.should_log_out_of_bounds) {
      RCLCPP_WARN(
        get_logger(),
        "Crop center (%.1f, %.1f) is outside map bounds + radius, skipping alignment",
        static_cast<double>(cx), static_cast<double>(cy));
    }
    return false;
  }

  if (lidar_localization::shouldReuseLocalMapTarget(
      local_map_target_cached_, local_map_target_center_x_, local_map_target_center_y_,
      cx, cy, local_map_update_distance_))
  {
    const auto reuse_decision = lidar_localization::handleLocalMapTargetSuccess();
    consecutive_crop_failures_ = reuse_decision.consecutive_crop_failures;
    crop_failure_guard_active_ = reuse_decision.crop_failure_guard_active;
    return true;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map =
    lidar_localization::cropLocalMapByRadius(
      *full_map_cloud_ptr_,
      crop_request.center_x,
      crop_request.center_y,
      crop_request.radius_m);
  const auto crop_size_validation =
    lidar_localization::validateLocalMapCropSize(
      local_map->size(), crop_request.min_points);
  if (!crop_size_validation.target_ready) {
    handle_crop_failure(crop_size_validation.failure);
    RCLCPP_WARN(
      get_logger(),
      "Local map crop too small (%zu points, min %zu) around (%.3f, %.3f) with radius %.1fm.",
      local_map->size(),
      local_map_min_points_,
      static_cast<double>(cx),
      static_cast<double>(cy),
      local_map_radius_);
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_local_map(
    new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(local_map);
  voxel_grid_filter_.filter(*filtered_local_map);
  const auto target_selection = lidar_localization::chooseTargetAfterLocalMapCrop(
    local_map->size(),
    filtered_local_map->size(),
    crop_request.min_points);
  if (target_selection.target_cloud ==
    lidar_localization::LocalMapTargetCloud::kFilteredLocalMap)
  {
    registration_->setInputTarget(filtered_local_map);
    lidar_localization::keepRegistrationCloudAlive(
      recent_target_clouds_, filtered_local_map, registration_target_cloud_keep_alive_count_);
  } else {
    registration_->setInputTarget(local_map);
    lidar_localization::keepRegistrationCloudAlive(
      recent_target_clouds_, local_map, registration_target_cloud_keep_alive_count_);
  }
  local_map_target_cached_ = true;
  local_map_target_center_x_ = cx;
  local_map_target_center_y_ = cy;

  const auto success_decision = lidar_localization::handleLocalMapTargetSuccess();
  consecutive_crop_failures_ = success_decision.consecutive_crop_failures;
  crop_failure_guard_active_ = success_decision.crop_failure_guard_active;
  return true;
}

lidar_localization::AlignmentAttempt PCLLocalization::runAlignmentAttempt(
  const Eigen::Matrix4f & attempt_init_guess,
  const Eigen::Matrix4f & crop_center_pose_matrix,
  double scan_stamp_sec,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation)
{
  lidar_localization::AlignmentAttempt attempt;
  attempt.init_guess = attempt_init_guess;
  if (!setInputTargetForPose(crop_center_pose_matrix)) {
    return attempt;
  }
  attempt.target_ready = true;

  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  rclcpp::Clock system_clock;
  const rclcpp::Time time_align_start = system_clock.now();
  // Only the backend call runs outside the shared-state lock. All seed, crop,
  // recovery, and pose fields remain protected, while /initialpose can acquire
  // the lock and reset state during this expensive operation.
  state_lock.unlock();
  try {
    auto registration_execution_lock =
      callback_state_coordinator_.lockRegistrationExecution();
    registration_->align(output_cloud, attempt_init_guess);
  } catch (...) {
    state_lock.lock();
    throw;
  }
  state_lock.lock();
  const rclcpp::Time time_align_end = system_clock.now();
  attempt.alignment_time_sec = time_align_end.seconds() - time_align_start.seconds();
  if (
    shutting_down_.load(std::memory_order_acquire) ||
    !callback_state_coordinator_.initialPoseGenerationMatches(seed_generation))
  {
    attempt.target_ready = false;
    return attempt;
  }
  attempt.has_converged = registration_->hasConverged();
  attempt.fitness_score = registration_->getFitnessScore();

  if (enable_registration_localizability_diagnostics_ && ndt_omp_registration_) {
    Eigen::Matrix<double, 6, 6> hessian;
    std::size_t correspondence_count = 0;
    ndt_omp_registration_->evaluateFinalScoreHessian(
      hessian, correspondence_count);
    attempt.registration_localizability =
      lidar_localization::analyzeRegistrationHessian(hessian, correspondence_count);
  }

  const auto seed_metrics = lidar_localization::computeAlignmentSeedMetrics(
    have_last_accepted_pose_,
    last_accepted_pose_matrix_,
    attempt_init_guess,
    scan_stamp_sec,
    last_accepted_pose_time_sec_);
  attempt.seed_translation_since_accept_m = seed_metrics.translation_since_accept_m;
  attempt.seed_yaw_since_accept_deg = seed_metrics.yaw_since_accept_deg;
  attempt.accepted_gap_sec = seed_metrics.accepted_gap_sec;

  if (attempt.has_converged) {
    attempt.final_transformation = registration_->getFinalTransformation();
    const auto correction_metrics =
      lidar_localization::computeAlignmentCorrectionMetrics(
      attempt_init_guess,
      attempt.final_transformation);
    attempt.correction_translation_m = correction_metrics.translation_m;
    attempt.correction_yaw_deg = correction_metrics.yaw_deg;
  }

  return attempt;
}

lidar_localization::AlignmentPipelineResult PCLLocalization::runAlignmentPipelineForScan(
  const Eigen::Matrix4f & init_guess,
  double scan_stamp_sec,
  lidar_localization::RegistrationSeedSource seed_source,
  bool imu_prediction_ready,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation)
{
  const lidar_localization::AlignmentAttempt primary_attempt =
    runAlignmentAttempt(init_guess, init_guess, scan_stamp_sec, state_lock, seed_generation);
  if (!callback_state_coordinator_.initialPoseGenerationMatches(seed_generation)) {
    lidar_localization::AlignmentPipelineResult interrupted_result;
    interrupted_result.selected_attempt = primary_attempt;
    return interrupted_result;
  }
  const int imu_guard_warmup_accepts_remaining =
    imu_guard_warmup_accepts_remaining_.load(std::memory_order_acquire);
  const bool force_retry_from_last_pose =
    seed_source == lidar_localization::RegistrationSeedSource::kImuPreintegration &&
    lidar_localization::isImuPredictionCorrectionGuardTripped(
      imuPreintegrationGuardParams(),
      lidar_localization::ImuPredictionCorrectionGuardInput{
        false,
        imu_prediction_ready,
        primary_attempt.correction_translation_m,
        primary_attempt.correction_yaw_deg,
        imu_guard_warmup_accepts_remaining});
  return lidar_localization::runAlignmentPipeline(
    primary_attempt,
    lidar_localization::AlignmentPipelineInput{
      have_last_accepted_pose_,
      consecutive_rejected_updates_,
      scan_stamp_sec,
      last_accepted_pose_time_sec_,
      recoveryRetryFromLastPoseParams(),
      force_retry_from_last_pose,
      "imu_prediction_correction_guard_rejected"},
    [&]() {
      return runAlignmentAttempt(
        last_accepted_pose_matrix_, last_accepted_pose_matrix_, scan_stamp_sec,
        state_lock, seed_generation);
    },
    [this, seed_source](const lidar_localization::AlignmentAttempt & attempt) {
      return evaluateMeasurementGateForAttempt(attempt, seed_source);
    });
}

lidar_localization::MeasurementGateDecision PCLLocalization::evaluateMeasurementGateForAttempt(
  const lidar_localization::AlignmentAttempt & attempt,
  lidar_localization::RegistrationSeedSource seed_source)
{
  const auto gate_input = lidar_localization::makeMeasurementGateInput(
    attempt.fitness_score,
    attempt.accepted_gap_sec,
    attempt.seed_translation_since_accept_m,
    attempt.correction_translation_m,
    attempt.correction_yaw_deg,
    consecutive_rejected_updates_,
    use_odom_tf_prediction_ && has_last_good_map_to_odom_,
    accepted_updates_since_reset_);
  auto gate =
    lidar_localization::evaluateMeasurementGate(measurementGateParams(), gate_input);
  if (gate.status_level == lidar_localization::kMeasurementGateWarn) {
    RCLCPP_WARN(
      get_logger(), "The fitness score is over %lf.", gate.effective_score_threshold);
  }

  return gate;
}

void PCLLocalization::logAlignmentPipelineRecovery(
  const lidar_localization::AlignmentPipelineResult & pipeline_result)
{
  const auto handling = lidar_localization::decideAlignmentPipelineHandling(pipeline_result);
  if (!handling.log_recovery_retry_success) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Recovery retry from last pose succeeded after %zu rejects: fitness=%.6f",
    consecutive_rejected_updates_,
    pipeline_result.selected_attempt.fitness_score);
}

bool PCLLocalization::handleTerminalAlignmentPipelineResult(
  const builtin_interfaces::msg::Time & stamp,
  const lidar_localization::AlignmentPipelineResult & pipeline_result,
  std::size_t filtered_point_count,
  double scan_stamp_sec,
  bool imu_prediction_ready,
  const std::string & registration_seed_source)
{
  const auto handling = lidar_localization::decideAlignmentPipelineHandling(pipeline_result);
  if (!handling.publish_terminal_status) {
    return false;
  }

  publishAlignmentStatusForAttempt(
    stamp,
    pipeline_result.status_level,
    pipeline_result.status_message,
    pipeline_result.selected_attempt,
    filtered_point_count,
    imu_prediction_ready,
    registration_seed_source);
  if (handling.warn_registration_not_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
  }
  if (handling.advance_prediction_without_measurement) {
    advancePredictionWithoutMeasurement(scan_stamp_sec);
  }
  return true;
}

bool PCLLocalization::applyAcceptedAlignmentPipelineResult(
  const builtin_interfaces::msg::Time & stamp,
  const lidar_localization::AlignmentPipelineResult & pipeline_result,
  std::size_t filtered_point_count,
  double scan_stamp_sec,
  bool imu_prediction_ready,
  const std::string & registration_seed_source)
{
  const auto handling = lidar_localization::decideAlignmentPipelineHandling(pipeline_result);
  if (!handling.continue_to_backend) {
    return false;
  }

  const auto registration_observation =
    lidar_localization::makeRegistrationObservation(
    pipeline_result.selected_attempt.final_transformation);
  if (!applyRegistrationPoseBackend(
      registration_observation,
      pipeline_result.selected_attempt,
      pipeline_result.gate_result,
      stamp,
      filtered_point_count,
      scan_stamp_sec,
      imu_prediction_ready,
      registration_seed_source))
  {
    return false;
  }

  if (lidar_localization::shouldPublishAcceptedPoseImmediately(
      enable_timer_publishing_, publish_bridge_pose_when_lost_))
  {
    // In bridge mode the timer supplements accepted scan outputs; it does not
    // replace them.  Keeping the scan-stamped accepted pose guarantees an
    // initial output/anchor and fills any short gap in the upstream odom TF,
    // while timer ticks provide continuity when scans themselves pause.
    publishCurrentPose(
      stamp, pipeline_result.selected_attempt.fitness_score,
      lidar_localization::rotationDeltaDeg(
        pipeline_result.selected_attempt.init_guess,
        pipeline_result.selected_attempt.final_transformation));
  } else if (corrent_pose_with_cov_stamped_ptr_) {
    // Timer publication still needs an accepted map->odom anchor before it
    // can compose the live odom bridge.  publishCurrentPose normally creates
    // that anchor, so skipping it in timer mode must skip only the pose/path
    // messages, not the accepted transform update.
    const bool freeze_as_last_good = lidar_localization::shouldFreezeMapToOdomAnchor(
      enable_map_odom_anchor_fitness_gate_, map_odom_anchor_max_fitness_,
      pipeline_result.selected_attempt.fitness_score,
      map_odom_anchor_max_correction_rotation_deg_,
      lidar_localization::rotationDeltaDeg(
        pipeline_result.selected_attempt.init_guess,
        pipeline_result.selected_attempt.final_transformation));
    publishPoseTransform(
      stamp, corrent_pose_with_cov_stamped_ptr_->pose.pose, freeze_as_last_good);
  }
  return true;
}

void PCLLocalization::setRegistrationSourceCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud)
{
  registration_->setInputSource(source_cloud);
  lidar_localization::keepRegistrationCloudAlive(
    recent_source_clouds_, source_cloud, registration_source_cloud_keep_alive_count_);
}

void PCLLocalization::printAlignmentDebugInfo(
  const Eigen::Matrix4f & init_guess,
  const lidar_localization::AlignmentAttempt & selected_attempt,
  std::size_t filtered_point_count) const
{
  if (!enable_debug_) {
    return;
  }

  std::cout << "number of filtered cloud points: " << filtered_point_count << std::endl;
  std::cout << "align time:" << selected_attempt.alignment_time_sec <<
    "[sec]" << std::endl;
  std::cout << "has converged: " << selected_attempt.has_converged << std::endl;
  std::cout << "fitness score: " << selected_attempt.fitness_score << std::endl;
  std::cout << "final transformation:" << std::endl;
  std::cout << selected_attempt.final_transformation << std::endl;
  /* delta_angle check
   * trace(RotationMatrix) = 2(cos(theta) + 1)
   */
  double init_cos_angle = 0.5 *
    (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
  double cos_angle = 0.5 *
    (selected_attempt.final_transformation.coeff(0,
    0) + selected_attempt.final_transformation.coeff(1, 1) +
    selected_attempt.final_transformation.coeff(2, 2) - 1);
  double init_angle = acos(init_cos_angle);
  double angle = acos(cos_angle);
  // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
  double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
  std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
  std::cout << "-----------------------------------------------------" << std::endl;
}

