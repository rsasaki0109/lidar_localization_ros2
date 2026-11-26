#include "lidar_localization/core_c_api.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>

#include <pcl/filters/voxel_grid.h>

#include "lidar_localization/localization_update_policy.hpp"
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
        result->localizability_valid = metrics->valid ? 1 : 0;
        for (int k = 0; k < 6; ++k) {
          result->eigenvalues[k] = metrics->eigenvalues[k];
          result->weakest_eigenvector[k] = metrics->weakest_eigenvector[k];
        }
        result->weak_ratio = metrics->weak_ratio;
        result->correspondence_count = metrics->correspondence_count;
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
  return 0;
}

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
  const auto & metrics = attempt.registration_localizability;
  result.localizability_valid = metrics.valid ? 1 : 0;
  for (int k = 0; k < 6; ++k) {
    result.eigenvalues[k] = metrics.eigenvalues[k];
    result.weakest_eigenvector[k] = metrics.weakest_eigenvector[k];
  }
  result.weak_ratio = metrics.weak_ratio;
  result.correspondence_count = metrics.correspondence_count;
  result.map_distance_filter_kept_points = attempt.map_distance_filter_kept_points;
}

void writeStatus(const std::string & message, char (&out)[64])
{
  const std::size_t n = std::min(sizeof(out) - 1, message.size());
  std::memcpy(out, message.data(), n);
  out[n] = '\0';
}

}  // namespace

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
    } else if (tracking.predict_pose_from_previous_delta && tracker.prediction.have_last_accepted_pose) {
      init_guess = tracker.prediction.predicted_pose_matrix *
        tracker.previousDeltaMotionAt(stamp_sec, tracking.previous_delta_max_extrapolation_sec);
    }
    core->engine.setSource(source);
    init_guess = core->engine.refineSeedWithNdtInitializer(source, init_guess);
    fromMatrix(init_guess, update->seed);

    const ll::AlignmentScoring scoring{
      core->config.inlier_ratio_distance_m,
      core->config.inlier_ratio_min_points,
      tracking.min_inlier_ratio,
      core->gate.score_threshold,
      core->config.localizability != 0};
    // One attempt, as the node's runAlignmentAttempt.
    auto attempt_from = [&](const Eigen::Matrix4f & guess) {
        ll::AlignmentAttempt attempt;
        attempt.init_guess = guess;
        if (!core->engine.setTargetForPose(guess)) {
          return attempt;
        }
        attempt.target_ready = true;
        const auto aligned = core->engine.align(guess);
        ll::scoreAlignment(core->engine, aligned, scoring, attempt);
        tracker.fillSeedMetrics(attempt, stamp_sec);
        return attempt;
      };
    tracker.gate_scan_stamp_sec = stamp_sec;
    const auto primary = attempt_from(init_guess);
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
      [&]() {return attempt_from(tracker.prediction.last_accepted_pose_matrix);},
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
    update->status_level = level;
    writeStatus(status, update->status);
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
