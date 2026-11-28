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
    bool bridged_prediction_ready = false;
    Eigen::Matrix4f bridged_prediction = Eigen::Matrix4f::Identity();
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
          ++imu_samples_since_scan_.integrated;
          imu_samples_since_scan_.last_dt_sec = interval.dt();
        }
      } else {
        imu_smoother_.resetPendingIntegration();
        imu_samples_since_scan_.skipped +=
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
      } else if (!prediction_plan.complete_coverage) {
        // While the node is stopped, the IMU samples queued for it overflow. The gap
        // stays in every window from the last accepted scan, so no prediction is made
        // again and the retry below never runs. For that retry only, hold the sample
        // before a gap across it, as the motion then.
        const auto bridged_plan = lidar_localization::planCausalImuIntervals(
          last_scan_stamp_for_imu_, scan_stamp_sec, prediction_samples,
          lidar_localization::kMaximumImuPreintegrationSampleDtSec,
          lidar_localization::kMaximumImuRetrySeedWindowSec);
        if (bridged_plan.complete_coverage && !bridged_plan.intervals.empty()) {
          for (const auto & interval : bridged_plan.intervals) {
            imu_prediction_smoother_.integrateImu(
              interval.gyro, interval.accel, interval.dt());
          }
          bridged_prediction = imu_prediction_smoother_.predictedPoseMatrix();
          bridged_prediction_ready = bridged_prediction.allFinite();
        }
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
    const bool imu_seed_allowed =
      !imu_seed_consistency_gate_enabled_ || imu_seed_consistency_state_.seed_allowed;
    imu_has_usable_new_samples = imu_has_usable_new_samples && imu_seed_allowed;
    if (
      lidar_localization::isImuRetrySeedReady(
        lidar_localization::ImuRetrySeedInput{
          use_imu_preintegration_,
          imu_preintegration_fallback_mode,
          imu_smoother_initialized,
          imu_has_new_samples || bridged_prediction_ready,
          imu_window_too_large || bridged_prediction_ready,
          latest_imu_seed_integration_window_sec_,
          imu_seed_allowed}))
    {
      const Eigen::Matrix4f imu_retry_guess = !imu_dual_queue_enabled_ ?
        imu_smoother_.predictedPoseMatrix() :
        (dual_queue_prediction_ready ? dual_queue_prediction : bridged_prediction);
      if (imu_retry_guess.allFinite()) {
        selected_seed.imu_retry_guess = imu_retry_guess;
        selected_seed.imu_retry_guess_bridged =
          imu_dual_queue_enabled_ && !dual_queue_prediction_ready &&
          lidar_localization::shouldSweepBridgedRetryYaw(
          bridged_prediction_ready, latest_imu_seed_integration_window_sec_);
      }
    }
  }

  Eigen::Matrix4f odom_tf_prediction = Eigen::Matrix4f::Identity();
  const bool odom_tf_bridge_available =
    use_odom_tf_prediction_ && lookupOdomBridgePoseMatrix(stamp, odom_tf_prediction);
  if (odom_tf_bridge_available) {
    if (lidar_localization::isOdomSeedGap(last_odom_seed_stamp_sec_, scan_stamp_sec)) {
      settled_accepts_since_reset_ = 0;
      seed_correction_window_.clear();
      seed_correction_release_history_.clear();
      scan_tracker_.seed_correction_agreement_history.clear();
      scan_tracker_.accepted_correction_history.clear();
    }
    last_odom_seed_stamp_sec_ = scan_stamp_sec;
  }
  local_reacquisition_confirmation_scan_ = false;
  scan_seeded_from_reacquisition_ = false;
  if (odom_tf_bridge_available && pending_reacquisition_map_to_odom_) {
    // Seed this scan from the proposed map -> odom instead of the frozen one.
    // The proposal is used once; the normal gate on this scan decides whether
    // it becomes the accepted anchor.
    const Eigen::Matrix4f frozen_map_to_odom =
      tf2::transformToEigen(last_good_map_to_odom_.transform).matrix().cast<float>();
    odom_tf_prediction =
      *pending_reacquisition_map_to_odom_ * frozen_map_to_odom.inverse() * odom_tf_prediction;
    pending_reacquisition_map_to_odom_.reset();
    local_reacquisition_confirmation_scan_ = true;
    scan_seeded_from_reacquisition_ = true;
  }

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
      use_twist_prediction_,
      prediction_state_.have_last_accepted_pose,
      static_cast<bool>(scan_twist_),
      predict_pose_from_previous_delta_,
      use_odom_tf_prediction_,
      odom_tf_bridge_available});
  selected_seed.source = seed_decision.source;
  selected_seed.imu_prediction_ready = seed_decision.imu_prediction_ready;
  if (trusted_start_pending_) {
    // Only the first scan after a trusted initial pose decides; without an
    // odometry seed then, the guards warm up as after any other pose.
    trusted_start_pending_ = false;
    settled_accepts_since_reset_ = lidar_localization::trustedStartSettledAccepts(
      settled_accepts_since_reset_, measurementGateParams(),
      seed_decision.source == lidar_localization::RegistrationSeedSource::kOdomTfPrediction &&
      !scan_seeded_from_reacquisition_);
  }
  if (!lidar_localization::keepsImuRetrySeed(selected_seed.source)) {
    selected_seed.imu_retry_guess.reset();
    selected_seed.imu_retry_guess_bridged = false;
  }

  switch (seed_decision.source) {
    case lidar_localization::RegistrationSeedSource::kImuPreintegration:
      selected_seed.init_guess = imu_init_guess;
      break;
    case lidar_localization::RegistrationSeedSource::kGtsamSmoother:
      selected_seed.init_guess = gtsam_smoother_.predictedPoseMatrixAt(
        last_ndt_roll_, last_ndt_pitch_, scan_stamp_sec);
      break;
    case lidar_localization::RegistrationSeedSource::kTwistPrediction: {
      const double dt = lidar_localization::clampPredictionDt(
        scan_stamp_sec, prediction_state_.predicted_pose_time_sec, max_twist_prediction_dt_);
      selected_seed.init_guess = applyTwistPrediction(prediction_state_.predicted_pose_matrix, dt);
      break;
    }
    case lidar_localization::RegistrationSeedSource::kPreviousDelta:
      selected_seed.init_guess =
        prediction_state_.predicted_pose_matrix * previousDeltaMotionAt(scan_stamp_sec);
      break;
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

lidar_localization::AlignmentAttempt PCLLocalization::runAlignmentAttempt(
  const Eigen::Matrix4f & attempt_init_guess,
  const Eigen::Matrix4f & crop_center_pose_matrix,
  double scan_stamp_sec,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation,
  bool filter_by_map_distance)
{
  lidar_localization::AlignmentAttempt attempt;
  attempt.init_guess = attempt_init_guess;
  if (!registration_engine_.setTargetForPose(crop_center_pose_matrix)) {
    return attempt;
  }
  attempt.target_ready = true;
  applyMapDistanceFilter(attempt, filter_by_map_distance);

  // Only the backend call runs outside the shared-state lock (the engine takes
  // the registration execution lock around it). All seed, crop, recovery, and
  // pose fields remain protected, while /initialpose can acquire the lock and
  // reset state during this expensive operation.
  state_lock.unlock();
  lidar_localization::RegistrationAlignResult aligned;
  try {
    aligned = registration_engine_.align(attempt_init_guess);
  } catch (...) {
    state_lock.lock();
    throw;
  }
  state_lock.lock();
  attempt.alignment_time_sec = aligned.alignment_time_sec;
  if (
    shutting_down_.load(std::memory_order_acquire) ||
    !callback_state_coordinator_.initialPoseGenerationMatches(seed_generation))
  {
    attempt.target_ready = false;
    return attempt;
  }
  lidar_localization::scoreAlignment(
    registration_engine_, aligned,
    lidar_localization::AlignmentScoring{
      inlier_ratio_distance_m_,
      static_cast<std::size_t>(inlier_ratio_min_points_),
      min_inlier_ratio_,
      measurement_gate_config_.score_threshold,
      enable_registration_localizability_diagnostics_},
    attempt);
  scan_tracker_.fillSeedMetrics(attempt, scan_stamp_sec);
  return attempt;
}

void PCLLocalization::applyMapDistanceFilter(
  lidar_localization::AlignmentAttempt & attempt, bool filter_by_map_distance)
{
  using lidar_localization::MapDistanceFilterStatus;
  const bool enabled = map_distance_filter_params_.distance_m > 0.0;
  if (filter_by_map_distance && map_distance_filter_decision_.apply && registration_full_source_) {
    const auto filtered = registration_engine_.setSourceNearTarget(
      registration_full_source_, attempt.init_guess, map_distance_filter_decision_.distance_m,
      map_distance_filter_params_.min_kept_ratio);
    registration_source_filtered_ = filtered.status == MapDistanceFilterStatus::kApplied;
    attempt.map_distance_filter_status =
      lidar_localization::mapDistanceFilterStatusName(filtered.status);
    attempt.map_distance_filter_m = filtered.distance_m;
    attempt.map_distance_filter_input_points = filtered.input_points;
    attempt.map_distance_filter_kept_points = filtered.kept_points;
    attempt.map_distance_filter_time_sec = filtered.time_sec;
    return;
  }
  // Retries from other seeds, and scans the filter skips, align the whole scan.
  if (registration_source_filtered_ && registration_full_source_) {
    registration_engine_.setSource(registration_full_source_);
    registration_source_filtered_ = false;
  }
  if (enabled) {
    attempt.map_distance_filter_status = lidar_localization::mapDistanceFilterStatusName(
      filter_by_map_distance ?
      map_distance_filter_decision_.status : MapDistanceFilterStatus::kRetrySeed);
    attempt.map_distance_filter_m = map_distance_filter_decision_.distance_m;
  }
}

lidar_localization::AlignmentPipelineResult PCLLocalization::runAlignmentPipelineForScan(
  const Eigen::Matrix4f & init_guess,
  const std::optional<Eigen::Matrix4f> & alternate_guess,
  bool sweep_alternate_yaw,
  double scan_stamp_sec,
  lidar_localization::RegistrationSeedSource seed_source,
  bool imu_prediction_ready,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation)
{
  gate_scan_stamp_sec_ = scan_stamp_sec;
  map_distance_filter_decision_ = lidar_localization::decideMapDistanceFilter(
    map_distance_filter_params_,
    lidar_localization::MapDistanceFilterInput{
      prediction_state_.have_last_accepted_pose,
      scan_stamp_sec - prediction_state_.last_accepted_pose_time_sec,
      scan_period_,
      static_cast<int>(std::min<std::size_t>(
        prediction_state_.accepted_updates_since_reset,
        static_cast<std::size_t>(std::numeric_limits<int>::max())))});
  const lidar_localization::AlignmentAttempt primary_attempt = runAlignmentAttempt(
    init_guess, init_guess, scan_stamp_sec, state_lock, seed_generation, true);
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
      prediction_state_.have_last_accepted_pose,
      prediction_state_.consecutive_rejected_updates,
      scan_stamp_sec,
      prediction_state_.last_accepted_pose_time_sec,
      recoveryRetryFromLastPoseParams(),
      force_retry_from_last_pose,
      "imu_prediction_correction_guard_rejected"},
    [&]() -> std::optional<lidar_localization::AlignmentAttempt> {
      if (
        !alternate_guess ||
        !callback_state_coordinator_.initialPoseGenerationMatches(seed_generation))
      {
        return std::nullopt;
      }
      const auto usable = [this](const lidar_localization::AlignmentAttempt & attempt) {
          return attempt.target_ready && attempt.has_converged &&
                 attempt.final_transformation.allFinite() &&
                 !measurementGateDecisionFor(attempt).reject_measurement;
        };
      auto attempt = runAlignmentAttempt(
        *alternate_guess, *alternate_guess, scan_stamp_sec, state_lock, seed_generation);
      if (!sweep_alternate_yaw || usable(attempt)) {
        return attempt;
      }
      // Bridged across an IMU gap, the seed is short of the turn: try it turned, and take
      // the best that passes the gate.
      std::optional<lidar_localization::AlignmentAttempt> best;
      for (const double offset_deg : lidar_localization::kBridgedRetryYawOffsetsDeg) {
        if (!callback_state_coordinator_.initialPoseGenerationMatches(seed_generation)) {
          break;
        }
        const Eigen::Matrix4f turned =
          lidar_localization::yawTurnedSeed(*alternate_guess, offset_deg);
        auto candidate =
          runAlignmentAttempt(turned, turned, scan_stamp_sec, state_lock, seed_generation);
        if (usable(candidate) && (!best || candidate.fitness_score < best->fitness_score)) {
          best = candidate;
        }
      }
      return best ? *best : attempt;
    },
    [&]() {
      return runAlignmentAttempt(
        prediction_state_.last_accepted_pose_matrix, prediction_state_.last_accepted_pose_matrix,
        scan_stamp_sec, state_lock, seed_generation);
    },
    [this, seed_source](const lidar_localization::AlignmentAttempt & attempt) {
      return evaluateMeasurementGateForAttempt(attempt, seed_source);
    });
}

double PCLLocalization::signedCorrectionYawDeg(
  const lidar_localization::AlignmentAttempt & attempt)
{
  return lidar_localization::ScanTracker::signedCorrectionYawDeg(attempt);
}

void PCLLocalization::recordSeedCorrectionReleaseHistory(
  const lidar_localization::AlignmentAttempt & attempt,
  const lidar_localization::MeasurementGateDecision & gate)
{
  scan_tracker_.recordReleaseHistory(attempt, gate, measurement_gate_config_);
}

lidar_localization::MeasurementGateDecision PCLLocalization::measurementGateDecisionFor(
  const lidar_localization::AlignmentAttempt & attempt) const
{
  return scan_tracker_.gateDecision(attempt, measurement_gate_config_);
}

lidar_localization::MeasurementGateDecision PCLLocalization::evaluateMeasurementGateForAttempt(
  const lidar_localization::AlignmentAttempt & attempt,
  lidar_localization::RegistrationSeedSource seed_source)
{
  auto gate = scan_tracker_.evaluateGate(attempt, measurement_gate_config_);
  if (gate.status_level == lidar_localization::kMeasurementGateWarn) {
    RCLCPP_WARN(
      get_logger(),
      "Measurement gate: %s (fitness=%.6f, threshold=%.6f, correction=%.3f m/%.3f deg)",
      gate.status_message.c_str(), attempt.fitness_score, gate.effective_score_threshold,
      attempt.correction_translation_m, attempt.correction_yaw_deg);
  }

  return gate;
}

void PCLLocalization::logAlignmentPipelineRecovery(
  const lidar_localization::AlignmentPipelineResult & pipeline_result)
{
  if (pipeline_result.recovered_by_alternate_seed) {
    RCLCPP_INFO(
      get_logger(),
      "IMU prediction retry succeeded after %zu rejects: fitness=%.6f",
      prediction_state_.consecutive_rejected_updates,
      pipeline_result.selected_attempt.fitness_score);
    return;
  }
  const auto handling = lidar_localization::decideAlignmentPipelineHandling(pipeline_result);
  if (!handling.log_recovery_retry_success) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Recovery retry from last pose succeeded after %zu rejects: fitness=%.6f",
    prediction_state_.consecutive_rejected_updates,
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
    publishCurrentPose(stamp);
  } else if (current_pose_with_cov_stamped_ptr_) {
    // Timer publication still needs an accepted map->odom anchor before it
    // can compose the live odom bridge.  publishCurrentPose normally creates
    // that anchor, so skipping it in timer mode must skip only the pose/path
    // messages, not the accepted transform update.
    publishPoseTransform(stamp, current_pose_with_cov_stamped_ptr_->pose.pose);
  }
  return true;
}

void PCLLocalization::setRegistrationSourceCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud)
{
  registration_engine_.setSource(source_cloud);
  registration_full_source_ = source_cloud;
  registration_source_filtered_ = false;
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

