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

namespace
{

std::vector<lidar_localization::RefinedReacquisitionCandidate> refinedCandidates(
  const std::vector<lidar_localization::AlignmentAttempt> & attempts)
{
  std::vector<lidar_localization::RefinedReacquisitionCandidate> refined;
  refined.reserve(attempts.size());
  for (const auto & attempt : attempts) {
    lidar_localization::RefinedReacquisitionCandidate result;
    result.converged =
      attempt.target_ready && attempt.has_converged && attempt.final_transformation.allFinite();
    result.fitness = result.converged ? attempt.fitness_score :
      std::numeric_limits<double>::infinity();
    result.x_m = attempt.final_transformation(0, 3);
    result.y_m = attempt.final_transformation(1, 3);
    refined.push_back(result);
  }
  return refined;
}

}  // namespace

std::vector<lidar_localization::AlignmentAttempt>
PCLLocalization::alignLocalReacquisitionCandidates(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud,
  const Eigen::Matrix4f & center,
  double scan_stamp_sec,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation,
  std::size_t * bbs_candidate_count)
{
  std::vector<lidar_localization::AlignmentAttempt> attempts;
  if (!source_cloud || !local_reacquisition_map_) {
    return attempts;
  }
  const double center_yaw = std::atan2(center(1, 0), center(0, 0));
  std::vector<std::array<double, 3>> points;
  points.reserve(source_cloud->size());
  for (const auto & point : source_cloud->points) {
    points.push_back({point.x, point.y, point.z});
  }
  const auto scan_xy =
    lidar_localization::prepareReacquisitionScanXy(points, local_reacquisition_params_);
  const auto & map = *local_reacquisition_map_;
  const auto window = lidar_localization::occupancyWindowAround(
    map, center(0, 3), center(1, 3), local_reacquisition_params_.search_radius_m);
  if (scan_xy.empty() || window.width <= 0 || window.height <= 0) {
    return attempts;
  }
  const auto bbs_candidates = lidar_localization::bbs::branch_and_bound_candidates(
    lidar_localization::cropGrid(map.grid, window),
    scan_xy,
    map.resolution_m,
    local_reacquisition_params_.angular_resolution_deg * M_PI / 180.0,
    local_reacquisition_params_.pyramid_depth,
    16,
    static_cast<int>(std::lround(1.0 / map.resolution_m)),
    center_yaw,
    local_reacquisition_params_.yaw_window_deg * M_PI / 180.0);
  if (bbs_candidate_count) {
    *bbs_candidate_count = bbs_candidates.size();
  }
  const auto candidates = lidar_localization::selectHeadingConsistentCandidates(
    bbs_candidates, map, window, center_yaw, local_reacquisition_params_);
  for (const auto & candidate : candidates) {
    Eigen::Matrix4f guess = center;
    const Eigen::Matrix3f yaw_delta = Eigen::AngleAxisf(
      static_cast<float>(lidar_localization::wrapAngleRad(candidate.yaw_rad - center_yaw)),
      Eigen::Vector3f::UnitZ()).toRotationMatrix();
    guess.block<3, 3>(0, 0) = yaw_delta * center.block<3, 3>(0, 0);
    guess(0, 3) = static_cast<float>(candidate.x_m);
    guess(1, 3) = static_cast<float>(candidate.y_m);
    // Candidates lie within search_radius_m of the center, so the target cropped
    // around it serves all of them without a re-crop.
    attempts.push_back(
      runAlignmentAttempt(guess, center, scan_stamp_sec, state_lock, seed_generation));
    if (!callback_state_coordinator_.initialPoseGenerationMatches(seed_generation)) {
      return {};
    }
  }
  return attempts;
}

std::optional<lidar_localization::AlignmentAttempt> PCLLocalization::searchAroundPrediction(
  const Eigen::Matrix4f & prediction,
  double scan_stamp_sec,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation)
{
  ++scans_since_local_reacquisition_attempt_;
  if (!lidar_localization::shouldSearchAroundPrediction(
      local_reacquisition_params_, use_odom_tf_prediction_,
      local_reacquisition_map_.has_value(), prediction_state_.consecutive_rejected_updates,
      scans_since_local_reacquisition_attempt_))
  {
    return std::nullopt;
  }
  scans_since_local_reacquisition_attempt_ = 0;
  std::size_t bbs_candidate_count = 0;
  const auto attempts = alignLocalReacquisitionCandidates(
    registration_source_cloud_, prediction, scan_stamp_sec, state_lock, seed_generation,
    &bbs_candidate_count);
  const auto decision = lidar_localization::decideLocalReacquisition(
    refinedCandidates(attempts), measurement_gate_config_.score_threshold,
    local_reacquisition_params_);
  if (!decision.propose || measurementGateDecisionFor(attempts[decision.index]).reject_measurement) {
    RCLCPP_INFO(
      get_logger(),
      "local search around the prediction: %zu BBS / %zu aligned candidates, none taken (%s)",
      bbs_candidate_count, attempts.size(),
      decision.propose ? "gate_rejected" : decision.reason.c_str());
    return std::nullopt;
  }
  const Eigen::Matrix4f & found = attempts[decision.index].final_transformation;
  RCLCPP_INFO(
    get_logger(),
    "local search around the prediction found (%.2f, %.2f) fitness %.3f, %.2f m from it, "
    "after %zu rejections",
    found(0, 3), found(1, 3), attempts[decision.index].fitness_score,
    std::hypot(found(0, 3) - prediction(0, 3), found(1, 3) - prediction(1, 3)),
    prediction_state_.consecutive_rejected_updates);
  return attempts[decision.index];
}

void PCLLocalization::maybeRunLocalReacquisition(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud,
  const SelectedRegistrationSeed & selected_seed,
  double scan_stamp_sec,
  lidar_localization::CallbackStateCoordinator::StateLock & state_lock,
  std::uint64_t seed_generation)
{
  if (!use_odom_tf_prediction_) {
    // Without the bridge the search runs in the alignment (searchAroundPrediction).
    return;
  }
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
  std::size_t bbs_candidate_count = 0;
  const auto attempts = alignLocalReacquisitionCandidates(
    source_cloud, bridged, scan_stamp_sec, state_lock, seed_generation, &bbs_candidate_count);
  if (!callback_state_coordinator_.initialPoseGenerationMatches(seed_generation)) {
    return;
  }
  const auto refined = refinedCandidates(attempts);
  const auto decision = lidar_localization::decideLocalReacquisition(
    refined, measurement_gate_config_.score_threshold, local_reacquisition_params_);
  if (!decision.propose) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "local re-acquisition: %zu BBS / %zu heading-consistent candidates, no proposal (%s)",
      bbs_candidate_count, attempts.size(), decision.reason.c_str());
    return;
  }
  const Eigen::Matrix4f frozen_map_to_odom =
    tf2::transformToEigen(last_good_map_to_odom_.transform).matrix().cast<float>();
  // New map -> odom such that (map -> odom) x (odom -> base at this scan) equals
  // the refined pose; odom -> base = frozen^-1 x bridged.
  const Eigen::Matrix4f & proposal = attempts[decision.index].final_transformation;
  pending_reacquisition_map_to_odom_ = proposal * bridged.inverse() * frozen_map_to_odom;
  RCLCPP_INFO(
    get_logger(),
    "local re-acquisition proposed (%.2f, %.2f) fitness %.3f, %.2f m from the bridged pose, "
    "after %zu rejections; confirming on the next scan",
    proposal(0, 3), proposal(1, 3), refined[decision.index].fitness,
    std::hypot(proposal(0, 3) - bridged(0, 3), proposal(1, 3) - bridged(1, 3)),
    prediction_state_.consecutive_rejected_updates);
}
