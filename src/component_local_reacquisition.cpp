#include "component_internal.hpp"

void PCLLocalization::finishLocalReacquisitionConfirmation(bool accepted)
{
  if (!local_reacquisition_confirmation_scan_) {
    return;
  }
  local_reacquisition_confirmation_scan_ = false;
  if (accepted) {
    scans_since_local_reacquisition_attempt_ = 0;
    RCLCPP_INFO(get_logger(), "local re-acquisition confirmed by the measurement gate");
  } else {
    RCLCPP_WARN(
      get_logger(), "local re-acquisition proposal rejected by the measurement gate");
  }
}

void PCLLocalization::maybeRunLocalReacquisition(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud,
  const SelectedRegistrationSeed & selected_seed,
  double scan_stamp_sec,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation)
{
  ++scans_since_local_reacquisition_attempt_;
  const bool should_attempt = lidar_localization::shouldAttemptLocalReacquisition(
    local_reacquisition_params_,
    lidar_localization::LocalReacquisitionTriggerInput{
      selected_seed.source == lidar_localization::RegistrationSeedSource::kOdomTfPrediction,
      local_reacquisition_map_.has_value(),
      pending_reacquisition_map_to_odom_.has_value(),
      prediction_state_.consecutive_rejected_updates,
      scans_since_local_reacquisition_attempt_});
  if (!should_attempt || scan_seeded_from_reacquisition_ || !source_cloud ||
    !has_last_good_map_to_odom_)
  {
    return;
  }
  scans_since_local_reacquisition_attempt_ = 0;

  const Eigen::Matrix4f bridged = selected_seed.init_guess;
  const double bridged_yaw = std::atan2(bridged(1, 0), bridged(0, 0));
  std::vector<std::array<double, 3>> points;
  points.reserve(source_cloud->size());
  for (const auto & point : source_cloud->points) {
    points.push_back({point.x, point.y, point.z});
  }
  const auto scan_xy =
    lidar_localization::prepareReacquisitionScanXy(points, local_reacquisition_params_);
  const auto & map = *local_reacquisition_map_;
  const auto window = lidar_localization::occupancyWindowAround(
    map, bridged(0, 3), bridged(1, 3), local_reacquisition_params_.search_radius_m);
  if (scan_xy.empty() || window.width <= 0 || window.height <= 0) {
    return;
  }
  const auto bbs_candidates = lidar_localization::bbs::branch_and_bound_candidates(
    lidar_localization::cropGrid(map.grid, window),
    scan_xy,
    map.resolution_m,
    local_reacquisition_params_.angular_resolution_deg * M_PI / 180.0,
    local_reacquisition_params_.pyramid_depth,
    16,
    static_cast<int>(std::lround(1.0 / map.resolution_m)),
    bridged_yaw,
    local_reacquisition_params_.yaw_window_deg * M_PI / 180.0);
  const auto candidates = lidar_localization::selectHeadingConsistentCandidates(
    bbs_candidates, map, window, bridged_yaw, local_reacquisition_params_);

  std::vector<lidar_localization::RefinedReacquisitionCandidate> refined;
  std::vector<Eigen::Matrix4f> refined_poses;
  for (const auto & candidate : candidates) {
    Eigen::Matrix4f guess = bridged;
    const Eigen::Matrix3f yaw_delta = Eigen::AngleAxisf(
      static_cast<float>(lidar_localization::wrapAngleRad(candidate.yaw_rad - bridged_yaw)),
      Eigen::Vector3f::UnitZ()).toRotationMatrix();
    guess.block<3, 3>(0, 0) = yaw_delta * bridged.block<3, 3>(0, 0);
    guess(0, 3) = static_cast<float>(candidate.x_m);
    guess(1, 3) = static_cast<float>(candidate.y_m);
    // Candidates lie within search_radius_m of the bridged pose, so the target
    // this scan already cropped around it serves all of them without a re-crop.
    const auto attempt =
      runAlignmentAttempt(guess, bridged, scan_stamp_sec, state_lock, seed_generation);
    if (!callback_state_coordinator_.initialPoseGenerationMatches(seed_generation)) {
      return;
    }
    lidar_localization::RefinedReacquisitionCandidate result;
    result.converged =
      attempt.target_ready && attempt.has_converged && attempt.final_transformation.allFinite();
    result.fitness = result.converged ? attempt.fitness_score :
      std::numeric_limits<double>::infinity();
    result.x_m = attempt.final_transformation(0, 3);
    result.y_m = attempt.final_transformation(1, 3);
    refined.push_back(result);
    refined_poses.push_back(attempt.final_transformation);
  }

  const auto decision = lidar_localization::decideLocalReacquisition(
    refined, measurement_gate_config_.score_threshold, local_reacquisition_params_);
  if (!decision.propose) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "local re-acquisition: %zu BBS / %zu heading-consistent candidates, no proposal (%s)",
      bbs_candidates.size(), candidates.size(), decision.reason.c_str());
    return;
  }
  const Eigen::Matrix4f frozen_map_to_odom =
    tf2::transformToEigen(last_good_map_to_odom_.transform).matrix().cast<float>();
  // New map -> odom such that (map -> odom) x (odom -> base at this scan) equals
  // the refined pose; odom -> base = frozen^-1 x bridged.
  pending_reacquisition_map_to_odom_ =
    refined_poses[decision.index] * bridged.inverse() * frozen_map_to_odom;
  const Eigen::Matrix4f & proposal = refined_poses[decision.index];
  RCLCPP_INFO(
    get_logger(),
    "local re-acquisition proposed (%.2f, %.2f) fitness %.3f, %.2f m from the bridged pose, "
    "after %zu rejections; confirming on the next scan",
    proposal(0, 3), proposal(1, 3), refined[decision.index].fitness,
    std::hypot(proposal(0, 3) - bridged(0, 3), proposal(1, 3) - bridged(1, 3)),
    prediction_state_.consecutive_rejected_updates);
}

void PCLLocalization::keepScanForStaleReset(
  double scan_stamp_sec, const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  if (!stale_reset_replay_params_.enable || !cloud) {
    return;
  }
  stale_reset_scans_.emplace_back(scan_stamp_sec, cloud);
  while (
    !stale_reset_scans_.empty() &&
    stale_reset_scans_.front().first <
    scan_stamp_sec - stale_reset_replay_params_.max_age_sec - 1.0)
  {
    stale_reset_scans_.pop_front();
  }
}

bool PCLLocalization::advanceStaleResetReplay(
  const builtin_interfaces::msg::Time & stamp,
  double scan_stamp_sec,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation)
{
  const auto & params = stale_reset_replay_params_;
  const auto now = std::chrono::steady_clock::now();
  if (stale_reset_replay_pending_) {
    stale_reset_replay_pending_ = false;
    const double reset_stamp_sec = last_initial_pose_stamp_sec_;
    double first_kept_sec = std::numeric_limits<double>::quiet_NaN();
    for (const auto & kept : stale_reset_scans_) {
      if (kept.first >= reset_stamp_sec) {
        first_kept_sec = kept.first;
        break;
      }
    }
    const auto plan = lidar_localization::planStaleResetReplay(
      params, reset_stamp_sec, scan_stamp_sec, first_kept_sec);
    if (!plan.replay || !current_pose_with_cov_stamped_ptr_) {
      if (plan.reason != "fresh" && plan.reason != "disabled") {
        RCLCPP_WARN(
          get_logger(), "reset %.1f s older than the scan applied as it is (%s)",
          scan_stamp_sec - reset_stamp_sec, plan.reason.c_str());
      }
      return false;
    }
    StaleResetReplay replay;
    replay.track = lidar_localization::resetPredictionState(currentPoseMatrix(), reset_stamp_sec);
    replay.reset_stamp_sec = reset_stamp_sec;
    replay.last_replayed_sec = std::nextafter(reset_stamp_sec, -1.0);
    replay.started = now;
    stale_reset_replay_ = replay;
  }
  if (!stale_reset_replay_) {
    return false;
  }
  auto & replay = *stale_reset_replay_;
  const auto give_up = [&](const std::string & why) {
      RCLCPP_WARN(
        get_logger(),
        "reset %.1f s older than the scan applied as it is: replay %s "
        "(%d of %d scans accepted) in %.2f s",
        scan_stamp_sec - replay.reset_stamp_sec, why.c_str(), replay.progress.accepted,
        replay.progress.steps,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - replay.started).count());
      stale_reset_replay_.reset();
      setRegistrationSourceCloud(cloud);
      return false;
    };
  if (lidar_localization::staleResetReplayFallenBehind(
      params, replay.last_replayed_sec, scan_stamp_sec))
  {
    return give_up("fell behind");
  }

  std::vector<double> stamps_sec;
  stamps_sec.reserve(stale_reset_scans_.size());
  for (const auto & kept : stale_reset_scans_) {
    stamps_sec.push_back(kept.first);
  }
  for (int next = lidar_localization::nextStaleResetReplayScan(
      params, stamps_sec, replay.last_replayed_sec, replay.progress.steps);
    next >= 0;
    next = lidar_localization::nextStaleResetReplayScan(
      params, stamps_sec, replay.last_replayed_sec, replay.progress.steps))
  {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - now).count() >
      params.budget_sec)
    {
      // Go on with the next scan, which keeps its own scan for the replay.
      return true;
    }
    const auto & [replay_stamp_sec, replay_cloud] =
      stale_reset_scans_[static_cast<std::size_t>(next)];
    const Eigen::Matrix4f seed = replay.track.predicted_pose_matrix *
      lidar_localization::previousDeltaMotionAt(
      replay.track, replay_stamp_sec, previous_delta_max_extrapolation_sec_);
    setRegistrationSourceCloud(replay_cloud);
    const auto attempt =
      runAlignmentAttempt(seed, seed, replay_stamp_sec, state_lock, seed_generation);
    if (!callback_state_coordinator_.initialPoseGenerationMatches(seed_generation)) {
      stale_reset_replay_.reset();
      return true;
    }
    const bool accepted = lidar_localization::staleResetReplayStepAccepted(
      attempt.target_ready && attempt.has_converged,
      attempt.final_transformation.allFinite(), attempt.fitness_score,
      measurement_gate_config_.score_threshold);
    replay.progress = lidar_localization::recordStaleResetReplayStep(replay.progress, accepted);
    replay.last_replayed_sec = replay_stamp_sec;
    if (accepted) {
      replay.track = lidar_localization::updatePredictionStateFromAcceptedMeasurement(
        replay.track, attempt.final_transformation, replay_stamp_sec, false);
    } else {
      replay.track = lidar_localization::advancePredictionWithoutMeasurement(
        replay.track, replay_stamp_sec, lidar_localization::PredictionAdvanceMode::kPreviousDelta,
        Eigen::Matrix4f::Identity(),
        lidar_localization::previousDeltaMotionAt(
          replay.track, replay_stamp_sec, previous_delta_max_extrapolation_sec_));
    }
    if (lidar_localization::staleResetReplayLost(params, replay.progress)) {
      return give_up("lost_track");
    }
  }
  const std::string failure = lidar_localization::staleResetReplayFailure(params, replay.progress);
  if (!failure.empty()) {
    return give_up(failure);
  }

  const auto & track = replay.track;
  const Eigen::Matrix4f & pose = track.last_accepted_pose_matrix;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  if (track.last_relative_motion_spans_scans && track.last_relative_motion_duration_sec > 0.0) {
    // The last scan-to-scan motion, from the scan before into the map frame.
    const Eigen::Matrix4f before = pose * track.last_relative_motion_matrix.inverse();
    velocity = (pose.block<3, 1>(0, 3) - before.block<3, 1>(0, 3)).cast<double>() /
      track.last_relative_motion_duration_sec;
  }
  RCLCPP_WARN(
    get_logger(),
    "reset %.1f s older than the scan replayed through %d scans (%d accepted) in %.2f s: "
    "moved %.2f m to (%.2f, %.2f) at %.1f m/s",
    scan_stamp_sec - replay.reset_stamp_sec, replay.progress.steps, replay.progress.accepted,
    std::chrono::duration<double>(std::chrono::steady_clock::now() - replay.started).count(),
    (pose.block<3, 1>(0, 3) - currentPoseMatrix().block<3, 1>(0, 3)).norm(), pose(0, 3),
    pose(1, 3), velocity.norm());
  auto moved = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(
    *current_pose_with_cov_stamped_ptr_);
  moved->header.stamp = stamp;
  moved->pose.pose = tf2::toMsg(Eigen::Affine3d(pose.cast<double>()));
  const auto moved_track = track;
  stale_reset_replay_.reset();
  applyInitialPose(moved, false, velocity);
  // Keep the motion tracked on the way, so the next scan is seeded with it.
  prediction_state_ = moved_track;
  return true;
}
