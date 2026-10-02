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
      consecutive_rejected_updates_,
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
    consecutive_rejected_updates_);
}
