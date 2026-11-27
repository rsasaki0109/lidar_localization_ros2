#include "lidar_localization/core_c_api.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>

#include "lidar_localization/local_reacquisition.hpp"
#include "lidar_localization/localization_update_policy.hpp"
#include "lidar_localization/map_distance_filter.hpp"
#include "lidar_localization/pose_backend_result_policy.hpp"
#include "lidar_localization/registration_engine.hpp"
#include "lidar_localization/scan_tracker.hpp"

struct lidarloc_core
{
  lidar_localization::RegistrationEngine engine;
  lidarloc_config config{};
  std::string registration_method;
  bool map_set{false};
  // lidarloc_localize's state: the tracking settings, the tracked pose and
  // the prediction and gate state (scan_tracker.hpp).
  lidarloc_tracking tracking{};
  lidar_localization::MeasurementGateParamConfig gate;
  lidar_localization::RecoveryRetryFromLastPoseParams retry;
  lidar_localization::ScanTracker tracker;
  bool tracking_started{false};
  Eigen::Matrix4f current_pose{Eigen::Matrix4f::Identity()};
  // Local re-acquisition: its settings, the map's points (for the occupancy
  // grid), the grid, and the node's state (component_local_reacquisition.cpp).
  lidarloc_reacquisition reacquisition_config{};
  lidar_localization::LocalReacquisitionParams reacquisition;
  std::vector<std::array<float, 3>> map_points;
  std::optional<lidar_localization::OccupancyGridMap> occupancy;
  std::size_t scans_since_reacquisition_attempt{0};
  // The proposal's correction (map frame, left of the seed), for the next
  // scan with a seed.
  std::optional<Eigen::Matrix4f> pending_reacquisition;
  mutable std::mutex log_mutex;
  std::string last_log;

  explicit lidarloc_core(const lidarloc_config & c)
  : engine([this](lidar_localization::LogLevel, const std::string & message) {
        std::lock_guard<std::mutex> lock(log_mutex);
        last_log = message;
      }),
    config(c),
    registration_method(c.registration_method ? c.registration_method : "NDT_OMP")
  {
    config.registration_method = registration_method.c_str();
  }
};

namespace
{

using Cloud = lidar_localization::RegistrationEngine::Cloud;

Cloud::Ptr toCloud(const float * xyz, std::size_t count)
{
  Cloud::Ptr cloud(new Cloud());
  cloud->reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    pcl::PointXYZI p;
    p.x = xyz[3 * i];
    p.y = xyz[3 * i + 1];
    p.z = xyz[3 * i + 2];
    p.intensity = 0.0f;
    cloud->push_back(p);
  }
  return cloud;
}

Eigen::Matrix4f toMatrix(const double m[16])
{
  Eigen::Matrix4f out;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out(r, c) = static_cast<float>(m[4 * r + c]);
    }
  }
  return out;
}

void fromMatrix(const Eigen::Matrix4f & m, double out[16])
{
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out[4 * r + c] = static_cast<double>(m(r, c));
    }
  }
}

void writeError(char * error, std::size_t size, const std::string & message)
{
  if (error && size > 0) {
    const std::size_t n = std::min(size - 1, message.size());
    std::memcpy(error, message.data(), n);
    error[n] = '\0';
  }
}

// The occupancy grid for local re-acquisition from the map's points.
void buildOccupancy(lidarloc_core & core)
{
  const auto & r = core.reacquisition_config;
  lidar_localization::OccupancyMapOptions options;
  options.resolution_m = r.occupancy_resolution_m;
  options.obstacle_height_m = r.occupancy_obstacle_height_m;
  options.max_obstacle_height_m = r.occupancy_max_obstacle_height_m;
  options.min_points_per_cell = r.occupancy_min_points_per_cell;
  options.inflate_radius_m = r.occupancy_inflate_radius_m;
  const auto bounds = lidar_localization::occupancyBoundsFromPoints(core.map_points, 1.0);
  core.occupancy = lidar_localization::occupancyGridFromGenerated(
    lidar_localization::generateOccupancyMap(core.map_points, bounds, options));
}

// A Hessian's localizability metrics into a C result.
void fillLocalizability(
  const lidar_localization::RegistrationLocalizabilityMetrics & metrics, lidarloc_result & result)
{
  result.localizability_valid = metrics.valid ? 1 : 0;
  for (int k = 0; k < 6; ++k) {
    result.eigenvalues[k] = metrics.eigenvalues[k];
    result.weakest_eigenvector[k] = metrics.weakest_eigenvector[k];
  }
  result.weak_ratio = metrics.weak_ratio;
  result.correspondence_count = metrics.correspondence_count;
  result.score_hessian_valid = metrics.enabled ? 1 : 0;
  for (std::size_t k = 0; k < 36; ++k) {
    result.score_hessian[k] = metrics.score_hessian[k];
  }
}

}  // namespace

extern "C" {

lidarloc_config lidarloc_default_config(void)
{
  const lidar_localization::RegistrationEngineConfig d;
  lidarloc_config c{};
  c.registration_method = "NDT_OMP";
  c.transform_epsilon = d.transform_epsilon;
  c.ndt_resolution = d.ndt_resolution;
  c.ndt_step_size = d.ndt_step_size;
  c.ndt_max_iterations = d.ndt_max_iterations;
  c.ndt_num_threads = d.ndt_num_threads;
  c.gicp_corr_randomness = d.gicp_corr_randomness;
  c.gicp_max_correspondence_distance = d.gicp_max_correspondence_distance;
  c.vgicp_voxel_resolution = d.vgicp_voxel_resolution;
  c.voxel_leaf_size = d.voxel_leaf_size;
  c.filter_scan = 1;
  c.enable_local_map_crop = d.enable_local_map_crop ? 1 : 0;
  c.local_map_radius = d.local_map_radius;
  c.local_map_min_points = d.local_map_min_points;
  c.local_map_update_distance = d.local_map_update_distance;
  c.scan_max_range = d.scan_max_range;
  c.ndt_init_scans_required = d.ndt_init_scans_required;
  c.inlier_ratio_distance_m = 0.5;
  c.inlier_ratio_min_points = 200;
  c.localizability = 1;
  c.map_distance_filter_m = 0.0;
  c.map_distance_filter_min_kept_ratio = 0.3;
  return c;
}

lidarloc_core * lidarloc_create(const lidarloc_config * config, char * error, size_t error_size)
{
  try {
    const lidarloc_config c = config ? *config : lidarloc_default_config();
    auto core = std::make_unique<lidarloc_core>(c);
    lidar_localization::RegistrationEngineConfig e;
    e.registration_method = core->registration_method;
    e.transform_epsilon = c.transform_epsilon;
    e.ndt_resolution = c.ndt_resolution;
    e.ndt_step_size = c.ndt_step_size;
    e.ndt_max_iterations = c.ndt_max_iterations;
    e.ndt_num_threads = c.ndt_num_threads;
    e.gicp_corr_randomness = c.gicp_corr_randomness;
    e.gicp_max_correspondence_distance = c.gicp_max_correspondence_distance;
    e.vgicp_voxel_resolution = c.vgicp_voxel_resolution;
    e.voxel_leaf_size = c.voxel_leaf_size;
    e.enable_local_map_crop = c.enable_local_map_crop != 0;
    e.local_map_radius = c.local_map_radius;
    e.local_map_min_points = c.local_map_min_points;
    e.local_map_update_distance = c.local_map_update_distance;
    e.scan_max_range = c.scan_max_range;
    e.ndt_init_scans_required = c.ndt_init_scans_required;
    const lidarloc_tracking tracking = lidarloc_default_tracking();
    lidarloc_set_tracking(core.get(), &tracking);
    if (!core->engine.configure(e)) {
      writeError(error, error_size, core->last_log);
      return nullptr;
    }
    return core.release();
  } catch (const std::exception & e) {
    writeError(error, error_size, e.what());
    return nullptr;
  }
}

void lidarloc_destroy(lidarloc_core * core)
{
  delete core;
}

int lidarloc_set_map(lidarloc_core * core, const float * xyz, size_t count)
{
  if (!core || !xyz || count == 0) {
    return 1;
  }
  try {
    const auto map = toCloud(xyz, count);
    core->engine.setMap(map, lidar_localization::computeMapBounds(*map));
    core->map_set = true;
    core->occupancy.reset();
    core->map_points.assign(count, {});
    for (std::size_t i = 0; i < count; ++i) {
      core->map_points[i] = {xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]};
    }
    if (core->reacquisition.enable) {
      buildOccupancy(*core);
    }
    return 0;
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(core->log_mutex);
    core->last_log = e.what();
    return 2;
  }
}

int lidarloc_align(
  lidarloc_core * core, const float * xyz, size_t count, const double guess[16],
  lidarloc_result * result)
{
  if (!core || !xyz || count == 0 || !guess || !result || !core->map_set) {
    return 1;
  }
  try {
    *result = lidarloc_result{};
    const Eigen::Matrix4f seed = toMatrix(guess);
    fromMatrix(seed, result->pose);
    result->iterations = -1;
    result->fitness_score = std::numeric_limits<double>::max();
    result->inlier_ratio = std::numeric_limits<double>::quiet_NaN();
    Cloud::Ptr source = toCloud(xyz, count);
    if (core->config.filter_scan) {
      // As the node's enable_scan_voxel_filter (component_scan_preparation.cpp).
      Cloud::Ptr filtered(new Cloud());
      pcl::VoxelGrid<pcl::PointXYZI> grid;
      const auto leaf = static_cast<float>(core->config.voxel_leaf_size);
      grid.setLeafSize(leaf, leaf, leaf);
      grid.setInputCloud(source);
      grid.filter(*filtered);
      source = filtered;
    }
    result->source_points = source->size();
    if (source->empty()) {
      return 1;
    }
    core->engine.setSource(source);
    const Eigen::Matrix4f guess_refined = core->engine.refineSeedWithNdtInitializer(source, seed);
    if (!core->engine.setTargetForPose(guess_refined)) {
      return 0;
    }
    result->target_ready = 1;
    if (core->config.map_distance_filter_m > 0.0) {
      const auto filtered = core->engine.setSourceNearTarget(
        source, guess_refined, core->config.map_distance_filter_m,
        core->config.map_distance_filter_min_kept_ratio);
      result->map_distance_filter_kept_points = filtered.kept_points;
      result->map_distance_filter_applied =
        filtered.status == lidar_localization::MapDistanceFilterStatus::kApplied ? 1 : 0;
    }
    const auto aligned = core->engine.align(guess_refined);
    result->has_converged = aligned.has_converged ? 1 : 0;
    result->iterations = aligned.iterations;
    result->alignment_time_sec = aligned.alignment_time_sec;
    if (aligned.has_converged) {
      fromMatrix(aligned.final_transformation, result->pose);
    }
    const auto fitness = core->engine.fitness(
      core->config.inlier_ratio_distance_m, core->config.inlier_ratio_min_points);
    result->fitness_score = fitness.score;
    result->inlier_ratio = fitness.inlier_ratio;
    if (core->config.localizability) {
      if (const auto metrics = core->engine.localizability()) {
        fillLocalizability(*metrics, *result);
      }
    }
    return 0;
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(core->log_mutex);
    core->last_log = e.what();
    return 2;
  }
}

lidarloc_tracking lidarloc_default_tracking(void)
{
  const lidar_localization::MeasurementGateParams g;
  const lidar_localization::RecoveryRetryFromLastPoseParams r;
  lidarloc_tracking t{};
  t.score_threshold = g.score_threshold;
  t.min_inlier_ratio = 0.0;
  t.enable_seed_correction_guard = g.enable_seed_correction_guard ? 1 : 0;
  t.seed_correction_guard_translation_m = g.seed_correction_guard_translation_m;
  t.seed_correction_guard_yaw_deg = g.seed_correction_guard_yaw_deg;
  t.seed_correction_guard_release_rejections = g.seed_correction_guard_release_rejections;
  t.seed_correction_guard_warmup_accepts = g.seed_correction_guard_warmup_accepts;
  t.enable_seed_correction_window_guard = g.enable_seed_correction_window_guard ? 1 : 0;
  t.seed_correction_window_guard_sec = g.seed_correction_window_guard_sec;
  t.seed_correction_window_guard_translation_m = g.seed_correction_window_guard_translation_m;
  t.enable_recovery_retry_from_last_pose = r.enable ? 1 : 0;
  t.recovery_retry_from_last_pose_min_rejections = r.min_rejections;
  t.recovery_retry_from_last_pose_max_accepted_gap_sec = r.max_accepted_gap_sec;
  t.predict_pose_from_previous_delta = 1;
  t.previous_delta_max_extrapolation_sec = 1.0;
  const lidar_localization::MapDistanceFilterParams m;
  t.map_distance_filter_growth_m_per_s = m.growth_m_per_s;
  t.map_distance_filter_max_distance_m = m.max_distance_m;
  t.map_distance_filter_warmup_accepts = m.warmup_accepts;
  t.scan_period_sec = 0.1;
  return t;
}

int lidarloc_set_tracking(lidarloc_core * core, const lidarloc_tracking * tracking)
{
  if (!core || !tracking) {
    return 1;
  }
  const lidarloc_tracking & t = *tracking;
  core->tracking = t;
  core->gate = lidar_localization::MeasurementGateParamConfig{};
  core->gate.score_threshold = t.score_threshold;
  core->gate.enable_seed_correction_guard = t.enable_seed_correction_guard != 0;
  core->gate.seed_correction_guard_translation_m = t.seed_correction_guard_translation_m;
  core->gate.seed_correction_guard_yaw_deg = t.seed_correction_guard_yaw_deg;
  core->gate.seed_correction_guard_release_rejections =
    t.seed_correction_guard_release_rejections;
  core->gate.seed_correction_guard_warmup_accepts = t.seed_correction_guard_warmup_accepts;
  core->gate.enable_seed_correction_window_guard = t.enable_seed_correction_window_guard != 0;
  core->gate.seed_correction_window_guard_sec = t.seed_correction_window_guard_sec;
  core->gate.seed_correction_window_guard_translation_m =
    t.seed_correction_window_guard_translation_m;
  core->retry = lidar_localization::RecoveryRetryFromLastPoseParams{};
  core->retry.enable = t.enable_recovery_retry_from_last_pose != 0;
  core->retry.min_rejections = t.recovery_retry_from_last_pose_min_rejections;
  core->retry.max_accepted_gap_sec = t.recovery_retry_from_last_pose_max_accepted_gap_sec;
  return 0;
}

int lidarloc_set_initial_pose(lidarloc_core * core, const double pose[16], double stamp_sec)
{
  if (!core || !pose) {
    return 1;
  }
  core->current_pose = toMatrix(pose);
  core->tracker.reset(core->current_pose, stamp_sec);
  core->tracking_started = true;
  core->pending_reacquisition.reset();
  core->scans_since_reacquisition_attempt = 0;
  return 0;
}

lidarloc_reacquisition lidarloc_default_reacquisition(void)
{
  const lidar_localization::LocalReacquisitionParams p;
  const lidar_localization::OccupancyMapOptions o;
  lidarloc_reacquisition r{};
  r.enable = p.enable ? 1 : 0;
  r.min_rejections = p.min_rejections;
  r.attempt_interval_scans = p.attempt_interval_scans;
  r.search_radius_m = p.search_radius_m;
  r.yaw_window_deg = p.yaw_window_deg;
  r.max_candidates = p.max_candidates;
  r.candidate_separation_m = p.candidate_separation_m;
  r.min_fitness_ratio = p.min_fitness_ratio;
  r.scan_min_z_m = p.scan_min_z_m;
  r.scan_max_z_m = p.scan_max_z_m;
  r.scan_min_range_m = p.scan_min_range_m;
  r.scan_voxel_m = p.scan_voxel_m;
  r.scan_max_points = p.scan_max_points;
  r.angular_resolution_deg = p.angular_resolution_deg;
  r.pyramid_depth = p.pyramid_depth;
  r.occupancy_resolution_m = o.resolution_m;
  r.occupancy_obstacle_height_m = o.obstacle_height_m;
  r.occupancy_max_obstacle_height_m = o.max_obstacle_height_m;
  r.occupancy_min_points_per_cell = o.min_points_per_cell;
  r.occupancy_inflate_radius_m = o.inflate_radius_m;
  return r;
}

int lidarloc_set_reacquisition(lidarloc_core * core, const lidarloc_reacquisition * reacquisition)
{
  if (!core || !reacquisition) {
    return 1;
  }
  try {
    const lidarloc_reacquisition & r = *reacquisition;
    core->reacquisition_config = r;
    auto & p = core->reacquisition;
    p = lidar_localization::LocalReacquisitionParams{};
    p.enable = r.enable != 0;
    p.min_rejections = r.min_rejections;
    p.attempt_interval_scans = r.attempt_interval_scans;
    p.search_radius_m = r.search_radius_m;
    p.yaw_window_deg = r.yaw_window_deg;
    p.max_candidates = r.max_candidates;
    p.candidate_separation_m = r.candidate_separation_m;
    p.min_fitness_ratio = r.min_fitness_ratio;
    p.scan_min_z_m = r.scan_min_z_m;
    p.scan_max_z_m = r.scan_max_z_m;
    p.scan_min_range_m = r.scan_min_range_m;
    p.scan_voxel_m = r.scan_voxel_m;
    p.scan_max_points = r.scan_max_points;
    p.angular_resolution_deg = r.angular_resolution_deg;
    p.pyramid_depth = r.pyramid_depth;
    core->pending_reacquisition.reset();
    core->scans_since_reacquisition_attempt = 0;
    core->occupancy.reset();
    if (p.enable && core->map_set) {
      buildOccupancy(*core);
    }
    return 0;
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(core->log_mutex);
    core->last_log = e.what();
    return 2;
  }
}

// C++ helpers of lidarloc_localize, inside the extern "C" block.
extern "C++" {
namespace
{

// The attempt into the C result.
void fillResult(const lidar_localization::AlignmentAttempt & attempt, lidarloc_result & result)
{
  result.target_ready = attempt.target_ready ? 1 : 0;
  result.has_converged = attempt.has_converged ? 1 : 0;
  result.iterations = attempt.registration_iterations;
  fromMatrix(attempt.has_converged ? attempt.final_transformation : attempt.init_guess, result.pose);
  result.fitness_score = attempt.fitness_score;
  result.inlier_ratio = attempt.inlier_ratio;
  result.alignment_time_sec = attempt.alignment_time_sec;
  fillLocalizability(attempt.registration_localizability, result);
  result.map_distance_filter_kept_points = attempt.map_distance_filter_kept_points;
  result.map_distance_filter_applied = attempt.map_distance_filter_status == "applied" ? 1 : 0;
}

template<std::size_t N>
void writeString(const std::string & message, char (&out)[N])
{
  const std::size_t n = std::min(N - 1, message.size());
  std::memcpy(out, message.data(), n);
  out[n] = '\0';
}

// After a rejected scan with an odometry seed: local re-acquisition, as the
// node's maybeRunLocalReacquisition (component_local_reacquisition.cpp).
template<typename AttemptFrom>
void reacquire(
  lidarloc_core & core, const Cloud::Ptr & source, const Eigen::Matrix4f & bridged,
  double stamp_sec, AttemptFrom & attempt_from, lidarloc_update & update)
{
  namespace ll = lidar_localization;
  ++core.scans_since_reacquisition_attempt;
  const bool should_attempt = ll::shouldAttemptLocalReacquisition(
    core.reacquisition,
    ll::LocalReacquisitionTriggerInput{
      true,
      core.occupancy.has_value(),
      core.pending_reacquisition.has_value(),
      core.tracker.prediction.consecutive_rejected_updates,
      core.scans_since_reacquisition_attempt});
  if (!should_attempt || update.seeded_from_reacquisition || !core.occupancy) {
    return;
  }
  core.scans_since_reacquisition_attempt = 0;
  std::vector<std::array<double, 3>> points;
  points.reserve(source->size());
  for (const auto & point : source->points) {
    points.push_back({point.x, point.y, point.z});
  }
  // The candidates lie within search_radius_m of the bridged pose: the target
  // around it serves them all.
  const auto proposal = ll::proposeLocalReacquisition(
    *core.occupancy, points, bridged, core.reacquisition, core.gate.score_threshold,
    [&](const Eigen::Matrix4f & guess) -> std::optional<ll::LocalReacquisitionRefinement> {
      const auto attempt = attempt_from(guess, bridged, false);
      return ll::LocalReacquisitionRefinement{
        attempt.target_ready && attempt.has_converged, attempt.fitness_score,
        attempt.final_transformation};
    });
  update.reacquisition_attempted = 1;
  writeString(proposal.reason, update.reacquisition_reason);
  if (proposal.propose) {
    update.reacquisition_proposed = 1;
    fromMatrix(proposal.pose, update.reacquisition_pose);
    core.pending_reacquisition = proposal.pose * bridged.inverse();
  }
}

}  // namespace
}  // extern "C++"

int lidarloc_localize(
  lidarloc_core * core, const float * xyz, size_t count, double stamp_sec,
  const double seed[16], lidarloc_update * update)
{
  if (!core || !xyz || count == 0 || !update || !core->map_set || !core->tracking_started) {
    return 1;
  }
  try {
    namespace ll = lidar_localization;
    *update = lidarloc_update{};
    Cloud::Ptr source = toCloud(xyz, count);
    if (core->config.filter_scan) {
      // As the node's enable_scan_voxel_filter (component_scan_preparation.cpp).
      Cloud::Ptr filtered(new Cloud());
      pcl::VoxelGrid<pcl::PointXYZI> grid;
      const auto leaf = static_cast<float>(core->config.voxel_leaf_size);
      grid.setLeafSize(leaf, leaf, leaf);
      grid.setInputCloud(source);
      grid.filter(*filtered);
      source = filtered;
    }
    update->alignment.source_points = source->size();
    if (source->empty()) {
      return 1;
    }
    auto & tracker = core->tracker;
    const auto & tracking = core->tracking;
    // The seed, as the node's selectRegistrationSeed without the IMU: the
    // caller's (odometry), the previous delta, or the tracked pose.
    Eigen::Matrix4f init_guess = core->current_pose;
    if (seed) {
      init_guess = toMatrix(seed);
      if (core->pending_reacquisition) {
        // Seed this scan from the proposal (component_alignment.cpp): used
        // once; the gate on this scan decides whether it is kept.
        init_guess = *core->pending_reacquisition * init_guess;
        core->pending_reacquisition.reset();
        update->seeded_from_reacquisition = 1;
      }
    } else if (tracking.predict_pose_from_previous_delta && tracker.prediction.have_last_accepted_pose) {
      init_guess = tracker.prediction.predicted_pose_matrix *
        tracker.previousDeltaMotionAt(stamp_sec, tracking.previous_delta_max_extrapolation_sec);
    }
    const Eigen::Matrix4f bridged = init_guess;
    core->engine.setSource(source);
    init_guess = core->engine.refineSeedWithNdtInitializer(source, init_guess);
    fromMatrix(init_guess, update->seed);
    const auto map_distance_filter = ll::decideMapDistanceFilter(
      ll::MapDistanceFilterParams{
        core->config.map_distance_filter_m,
        tracking.map_distance_filter_growth_m_per_s,
        tracking.map_distance_filter_max_distance_m,
        tracking.map_distance_filter_warmup_accepts,
        core->config.map_distance_filter_min_kept_ratio},
      ll::MapDistanceFilterInput{
        tracker.prediction.have_last_accepted_pose,
        stamp_sec - tracker.prediction.last_accepted_pose_time_sec,
        tracking.scan_period_sec,
        static_cast<int>(std::min<std::size_t>(
          tracker.prediction.accepted_updates_since_reset,
          static_cast<std::size_t>(std::numeric_limits<int>::max())))});

    const ll::AlignmentScoring scoring{
      core->config.inlier_ratio_distance_m,
      core->config.inlier_ratio_min_points,
      tracking.min_inlier_ratio,
      core->gate.score_threshold,
      core->config.localizability != 0};
    // One attempt, as the node's runAlignmentAttempt: the target around
    // crop_center, and only the primary attempt filtered by map distance
    // (applyMapDistanceFilter).
    bool source_filtered = false;
    auto attempt_from = [&](
      const Eigen::Matrix4f & guess, const Eigen::Matrix4f & crop_center, bool filter) {
        ll::AlignmentAttempt attempt;
        attempt.init_guess = guess;
        if (!core->engine.setTargetForPose(crop_center)) {
          return attempt;
        }
        attempt.target_ready = true;
        if (filter && map_distance_filter.apply) {
          const auto filtered = core->engine.setSourceNearTarget(
            source, guess, map_distance_filter.distance_m,
            core->config.map_distance_filter_min_kept_ratio);
          source_filtered = filtered.status == ll::MapDistanceFilterStatus::kApplied;
          attempt.map_distance_filter_status = ll::mapDistanceFilterStatusName(filtered.status);
          attempt.map_distance_filter_kept_points = filtered.kept_points;
        } else if (source_filtered) {
          core->engine.setSource(source);
          source_filtered = false;
        }
        const auto aligned = core->engine.align(guess);
        ll::scoreAlignment(core->engine, aligned, scoring, attempt);
        tracker.fillSeedMetrics(attempt, stamp_sec);
        return attempt;
      };
    tracker.gate_scan_stamp_sec = stamp_sec;
    const auto primary = attempt_from(init_guess, init_guess, true);
    const auto pipeline = ll::runAlignmentPipeline(
      primary,
      ll::AlignmentPipelineInput{
        tracker.prediction.have_last_accepted_pose,
        tracker.prediction.consecutive_rejected_updates,
        stamp_sec,
        tracker.prediction.last_accepted_pose_time_sec,
        core->retry,
        false,
        "primary_retry_from_last_pose_rejected"},
      [&]() {
        const auto & last = tracker.prediction.last_accepted_pose_matrix;
        return attempt_from(last, last, false);
      },
      [&](const ll::AlignmentAttempt & attempt) {return tracker.evaluateGate(attempt, core->gate);});
    fillResult(pipeline.selected_attempt, update->alignment);
    update->retried_from_last_pose = pipeline.recovered_by_retry_from_last_pose ? 1 : 0;

    const auto advance_mode = ll::choosePredictionAdvanceMode(
      tracker.prediction.have_last_accepted_pose, false, false,
      tracking.predict_pose_from_previous_delta != 0);
    const auto handling = ll::decideAlignmentPipelineHandling(pipeline);
    std::string status = pipeline.status_message;
    int level = pipeline.status_level;
    if (!handling.continue_to_backend) {
      if (handling.advance_prediction_without_measurement) {
        tracker.advanceWithoutMeasurement(
          stamp_sec, advance_mode, Eigen::Matrix4f::Identity(),
          tracking.previous_delta_max_extrapolation_sec);
      }
    } else {
      // As the node's raw registration pose backend (component_pose_backend.cpp).
      tracker.recordReleaseHistory(pipeline.selected_attempt, pipeline.gate_result, core->gate);
      const auto result = ll::makePoseBackendResultFromLocalizationUpdate(
        pipeline.selected_attempt.final_transformation,
        pipeline.gate_result.status_level,
        pipeline.gate_result.status_message,
        ll::decideLocalizationUpdate(pipeline.gate_result));
      status = result.status_message;
      level = result.status_level;
      if (result.advance_prediction_without_measurement) {
        tracker.advanceWithoutMeasurement(
          stamp_sec, advance_mode, Eigen::Matrix4f::Identity(),
          tracking.previous_delta_max_extrapolation_sec);
      } else if (result.update_prediction_from_rejected_measurement) {
        tracker.rejectMeasurement(result.pose_matrix, stamp_sec);
      } else {
        if (result.update_current_pose) {
          core->current_pose = result.pose_matrix;
          update->accepted = 1;
        }
        if (result.update_prediction_state) {
          tracker.acceptMeasurement(result.pose_matrix, stamp_sec, core->gate);
        }
      }
    }
    if (!update->accepted && seed) {
      reacquire(*core, source, bridged, stamp_sec, attempt_from, *update);
    } else if (!update->accepted) {
      ++core->scans_since_reacquisition_attempt;
    } else if (update->seeded_from_reacquisition) {
      core->scans_since_reacquisition_attempt = 0;
    }
    update->status_level = level;
    writeString(status, update->status);
    fromMatrix(core->current_pose, update->pose);
    update->consecutive_rejected_updates = tracker.prediction.consecutive_rejected_updates;
    return 0;
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(core->log_mutex);
    core->last_log = e.what();
    return 2;
  }
}

const char * lidarloc_last_log(const lidarloc_core * core)
{
  if (!core) {
    return "";
  }
  std::lock_guard<std::mutex> lock(core->log_mutex);
  return core->last_log.c_str();
}

}  // extern "C"
