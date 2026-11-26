#include "lidar_localization/core_c_api.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>

#include <pcl/filters/voxel_grid.h>

#include "lidar_localization/registration_engine.hpp"

struct lidarloc_core
{
  lidar_localization::RegistrationEngine engine;
  lidarloc_config config{};
  std::string registration_method;
  bool map_set{false};
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

const char * lidarloc_last_log(const lidarloc_core * core)
{
  if (!core) {
    return "";
  }
  std::lock_guard<std::mutex> lock(core->log_mutex);
  return core->last_log.c_str();
}

}  // extern "C"
