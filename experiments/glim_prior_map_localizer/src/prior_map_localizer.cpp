#include "glim_prior_map_localizer/prior_map_factor_policy.hpp"
#include "glim_prior_map_localizer/map_odom_policy.hpp"
#include "glim_prior_map_localizer/ply_loader.hpp"
#include "glim_prior_map_localizer/vgicp_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/logging.hpp>

#include <kiss_matcher/KISSMatcher.hpp>
#include <spdlog/spdlog.h>

namespace glim_prior_map_localizer
{
namespace
{

double environmentDouble(const char * name, double fallback)
{
  const char * raw = std::getenv(name);
  return raw == nullptr ? fallback : std::stod(raw);
}

std::size_t environmentSize(const char * name, std::size_t fallback)
{
  const char * raw = std::getenv(name);
  return raw == nullptr ? fallback : static_cast<std::size_t>(std::stoul(raw));
}

bool environmentBool(const char * name, bool fallback)
{
  const char * raw = std::getenv(name);
  if (raw == nullptr) {
    return fallback;
  }
  const std::string value(raw);
  if (value == "1" || value == "true" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "off") {
    return false;
  }
  throw std::invalid_argument(std::string(name) + " must be true or false");
}

double rotationDegrees(const Eigen::Isometry3d & transform)
{
  constexpr double radians_to_degrees = 57.295779513082320876;
  const double cosine = std::clamp(
    (transform.linear().trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine) * radians_to_degrees;
}

bool finiteTransform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() &&
         std::abs(transform.linear().determinant() - 1.0) < 1.0e-2;
}

std::vector<Eigen::Vector3f> loadPriorMap(const std::string & path)
{
  return loadPlyXyz(path);
}

std::vector<Eigen::Vector3f> submapPoints(const glim::SubMap & submap)
{
  std::vector<Eigen::Vector3f> points;
  points.reserve(submap.frame->size());
  for (std::size_t index = 0; index < submap.frame->size(); ++index) {
    const auto point = submap.frame->points[index].head<3>().cast<float>();
    if (point.allFinite()) {
      points.push_back(point);
    }
  }
  return points;
}

std::vector<Eigen::Vector3f> cropMap(
  const std::vector<Eigen::Vector3f> & map,
  const Eigen::Vector3d & center,
  double radius_m)
{
  std::vector<Eigen::Vector3f> cropped;
  const float squared_radius = static_cast<float>(radius_m * radius_m);
  const Eigen::Vector3f center_f = center.cast<float>();
  for (const auto & point : map) {
    if ((point - center_f).squaredNorm() <= squared_radius) {
      cropped.push_back(point);
    }
  }
  return cropped;
}

}  // namespace

class PriorMapLocalizer : public glim::ExtensionModule
{
public:
  PriorMapLocalizer()
  : logger_(glim::create_module_logger("prior_map_localizer")),
    prior_map_(loadPriorMap(requiredMapPath())),
    matcher_(matcherConfig())
  {
    gate_params_.min_final_inliers = environmentSize("GLIM_PRIOR_MAP_MIN_INLIERS", 10);
    gate_params_.max_local_correction_translation_m =
      environmentDouble("GLIM_PRIOR_MAP_MAX_LOCAL_TRANSLATION_M", 1.5);
    gate_params_.max_local_correction_rotation_deg =
      environmentDouble("GLIM_PRIOR_MAP_MAX_LOCAL_YAW_DEG", 10.0);
    gate_params_.allow_global_relocalization =
      environmentBool("GLIM_PRIOR_MAP_ALLOW_GLOBAL_RELOCALIZATION", true);
    gate_params_.max_global_anchor_translation_disagreement_m =
      environmentDouble("GLIM_PRIOR_MAP_MAX_GLOBAL_DISAGREEMENT_M", 2.0);
    gate_params_.max_global_anchor_rotation_disagreement_deg =
      environmentDouble("GLIM_PRIOR_MAP_MAX_GLOBAL_DISAGREEMENT_YAW_DEG", 10.0);
    vgicp_voxel_resolution_m_ =
      environmentDouble("GLIM_PRIOR_MAP_VGICP_VOXEL_RESOLUTION_M", 1.0);
    vgicp_max_iterations_ = static_cast<int>(
      environmentSize("GLIM_PRIOR_MAP_VGICP_MAX_ITERATIONS", 10));
    vgicp_min_inlier_fraction_ =
      environmentDouble("GLIM_PRIOR_MAP_VGICP_MIN_INLIER_FRACTION", 0.50);
    vgicp_max_normalized_error_ =
      environmentDouble("GLIM_PRIOR_MAP_VGICP_MAX_NORMALIZED_ERROR", 5.0);
    vgicp_max_coarse_delta_translation_m_ =
      environmentDouble("GLIM_PRIOR_MAP_VGICP_MAX_COARSE_DELTA_M", 2.0);
    vgicp_max_coarse_delta_rotation_deg_ =
      environmentDouble("GLIM_PRIOR_MAP_VGICP_MAX_COARSE_DELTA_DEG", 5.0);
    tracking_crop_radius_m_ =
      environmentDouble("GLIM_PRIOR_MAP_TRACKING_CROP_RADIUS_M", 35.0);
    bootstrap_crop_radius_m_ =
      environmentDouble("GLIM_PRIOR_MAP_BOOTSTRAP_CROP_RADIUS_M", 60.0);
    bootstrap_submap_stride_ = std::max<std::size_t>(
      1, environmentSize("GLIM_PRIOR_MAP_BOOTSTRAP_SUBMAP_STRIDE", 2));
    tracking_submap_stride_ = std::max<std::size_t>(
      1, environmentSize("GLIM_PRIOR_MAP_TRACKING_SUBMAP_STRIDE", 10));
    translation_gain_ = std::clamp(
      environmentDouble("GLIM_PRIOR_MAP_TRANSLATION_GAIN", 0.2), 0.0, 1.0);
    full_global_search_ =
      environmentBool("GLIM_PRIOR_MAP_FULL_GLOBAL_SEARCH", false);
    const char * bootstrap_x = std::getenv("GLIM_PRIOR_MAP_BOOTSTRAP_CENTER_X");
    const char * bootstrap_y = std::getenv("GLIM_PRIOR_MAP_BOOTSTRAP_CENTER_Y");
    const char * bootstrap_z = std::getenv("GLIM_PRIOR_MAP_BOOTSTRAP_CENTER_Z");
    const char * bootstrap_yaw = std::getenv("GLIM_PRIOR_MAP_BOOTSTRAP_YAW_DEG");
    const int bootstrap_values =
      (bootstrap_x != nullptr) + (bootstrap_y != nullptr) + (bootstrap_z != nullptr) +
      (bootstrap_yaw != nullptr);
    if (bootstrap_values != 0 && bootstrap_values != 4) {
      throw std::invalid_argument(
              "GLIM_PRIOR_MAP_BOOTSTRAP_CENTER_X/Y/Z and BOOTSTRAP_YAW_DEG are all required");
    }
    if (bootstrap_values == 4) {
      bootstrap_anchor_.translation() = Eigen::Vector3d(
        std::stod(bootstrap_x), std::stod(bootstrap_y), std::stod(bootstrap_z));
      constexpr double degrees_to_radians = 0.01745329251994329577;
      bootstrap_anchor_.linear() = Eigen::AngleAxisd(
        std::stod(bootstrap_yaw) * degrees_to_radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      bootstrap_anchor_available_ = true;
      tracking_anchor_ = bootstrap_anchor_;
      tracking_anchor_available_ = true;
    }

    using std::placeholders::_1;
    glim::OdometryEstimationCallbacks::on_new_frame.add(
      std::bind(&PriorMapLocalizer::onOdometryFrame, this, _1));
    glim::GlobalMappingCallbacks::on_insert_submap.add(
      std::bind(&PriorMapLocalizer::onSubmap, this, _1));
    logger_->info(
      "loaded prior map with {} points; mode={} crop={:.1f}/{:.1f}m stride={}/{} "
      "seeded={} translation_gain={:.3f}",
      prior_map_.size(), full_global_search_ ? "full-global" : "tracking-crop",
      bootstrap_crop_radius_m_, tracking_crop_radius_m_, bootstrap_submap_stride_,
      tracking_submap_stride_,
      bootstrap_anchor_available_, translation_gain_);
  }

private:
  static std::string requiredMapPath()
  {
    const char * path = std::getenv("GLIM_PRIOR_MAP_PATH");
    if (path == nullptr || std::string(path).empty()) {
      throw std::runtime_error("GLIM_PRIOR_MAP_PATH is required");
    }
    return path;
  }

  static kiss_matcher::KISSMatcherConfig matcherConfig()
  {
    kiss_matcher::KISSMatcherConfig config(
      static_cast<float>(environmentDouble("GLIM_PRIOR_MAP_VOXEL_SIZE_M", 0.75)));
    config.use_quatro_ = environmentBool("GLIM_PRIOR_MAP_USE_QUATRO", true);
    config.use_ratio_test_ = environmentBool("GLIM_PRIOR_MAP_USE_RATIO_TEST", true);
    config.num_max_corr_ = static_cast<int>(
      environmentSize("GLIM_PRIOR_MAP_MAX_CORRESPONDENCES", 5000));
    return config;
  }

  void onOdometryFrame(const glim::EstimationFrame::ConstPtr & frame)
  {
    Eigen::Isometry3d initial_anchor = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      if (!tracking_anchor_available_ || initial_anchor_sent_) {
        return;
      }
      initial_anchor = tracking_anchor_;
      initial_anchor_sent_ = true;
    }
    glim::GlobalMappingCallbacks::on_external_map_odom_update(
      frame->stamp, initial_anchor, true);
    logger_->info("published startup map-to-odom anchor at stamp={:.6f}", frame->stamp);
  }

  void onSubmap(const glim::SubMap::ConstPtr & submap)
  {
    const std::size_t active_stride = map_frame_initialized_ ?
      tracking_submap_stride_ : bootstrap_submap_stride_;
    if (static_cast<std::size_t>(submap->id) % active_stride != 0) {
      return;
    }
    const auto source = submapPoints(*submap);
    if (source.empty()) {
      logger_->warn("submap {} has no finite points", submap->id);
      return;
    }

    const auto origin_odom_frame = submap->origin_odom_frame();
    if (!origin_odom_frame) {
      logger_->warn("submap {} has no odometry origin frame", submap->id);
      return;
    }
    const Eigen::Isometry3d odom_from_submap = origin_odom_frame->T_world_sensor();
    Eigen::Isometry3d anchor_snapshot = Eigen::Isometry3d::Identity();
    bool anchor_available = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      anchor_snapshot = tracking_anchor_;
      anchor_available = tracking_anchor_available_;
    }
    const Eigen::Isometry3d predicted_in_map =
      anchor_available ? anchor_snapshot * odom_from_submap : odom_from_submap;
    const Eigen::Vector3d predicted_map_center = predicted_in_map.translation();
    const double crop_radius_m = map_frame_initialized_ ?
      tracking_crop_radius_m_ : bootstrap_crop_radius_m_;
    const auto cropped_map = full_global_search_ ?
      std::vector<Eigen::Vector3f>{} :
      cropMap(prior_map_, predicted_map_center, crop_radius_m);
    const auto & target = full_global_search_ ? prior_map_ : cropped_map;
    if (target.empty()) {
      logger_->warn("submap {} has no prior-map points in its tracking crop", submap->id);
      return;
    }

    // During normal tracking GLIM already provides the best VGICP initial
    // value. KISS is reserved for acquiring an unknown/global frame; running
    // it on every submap adds CPU and can move a good prediction into a
    // repeated-structure basin.
    bool use_coarse_global_candidate =
      !anchor_available || full_global_search_;
    Eigen::Isometry3d candidate = predicted_in_map;
    bool coarse_valid = true;
    std::size_t coarse_inliers = gate_params_.min_final_inliers;
    if (use_coarse_global_candidate) {
      matcher_.reset();
      const auto solution = matcher_.estimate(source, target);
      candidate.linear() = solution.rotation;
      candidate.translation() = solution.translation;
      coarse_valid = solution.valid && finiteTransform(candidate);
      coarse_inliers = matcher_.getNumFinalInliers();
      if (!coarse_valid || coarse_inliers < gate_params_.min_final_inliers) {
        logger_->info(
          "submap={} source={} target={} KISS_valid={} KISS_inliers={} "
          "decision=coarse_rejected",
          submap->id, source.size(), target.size(), solution.valid,
          coarse_inliers);
        return;
      }
    }

    auto refinement = refineWithVgicp(
      target, submap->frame, candidate, vgicp_voxel_resolution_m_,
      vgicp_max_iterations_);
    auto refinement_delta = refinement.pose * candidate.inverse();
    const auto refinementAccepted = [&]() {
        return refinement.valid &&
          refinement.inlier_fraction >= vgicp_min_inlier_fraction_ &&
          refinement.normalized_error <= vgicp_max_normalized_error_ &&
          (!use_coarse_global_candidate ||
          (refinement_delta.translation().norm() <=
          vgicp_max_coarse_delta_translation_m_ &&
          rotationDegrees(refinement_delta) <=
          vgicp_max_coarse_delta_rotation_deg_));
      };
    bool refinement_accepted = refinementAccepted();
    if (!refinement_accepted && !use_coarse_global_candidate) {
      logger_->info(
        "submap={} direct VGICP rejected; trying crop-local KISS fallback",
        submap->id);
      matcher_.reset();
      const auto solution = matcher_.estimate(source, target);
      coarse_valid = solution.valid;
      coarse_inliers = matcher_.getNumFinalInliers();
      if (solution.valid) {
        candidate.linear() = solution.rotation;
        candidate.translation() = solution.translation;
        coarse_valid = finiteTransform(candidate);
      }
      if (!coarse_valid || coarse_inliers < gate_params_.min_final_inliers) {
        logger_->info(
          "submap={} source={} target={} KISS_valid={} KISS_inliers={} "
          "decision=fallback_coarse_rejected",
          submap->id, source.size(), target.size(), solution.valid,
          coarse_inliers);
        return;
      }
      use_coarse_global_candidate = true;
      refinement = refineWithVgicp(
        target, submap->frame, candidate, vgicp_voxel_resolution_m_,
        vgicp_max_iterations_);
      refinement_delta = refinement.pose * candidate.inverse();
      refinement_accepted = refinementAccepted();
    }
    if (!refinement_accepted) {
      logger_->info(
        "submap={} KISS_inliers={} VGICP_valid={} VGICP_inliers={} "
        "VGICP_overlap={:.3f} VGICP_error={:.3f} delta={:.3f}m/{:.2f}deg "
        "decision=refinement_rejected",
        submap->id, coarse_inliers, refinement.valid,
        refinement.num_inliers, refinement.inlier_fraction,
        refinement.normalized_error, refinement_delta.translation().norm(),
        rotationDegrees(refinement_delta));
      return;
    }
    candidate = refinement.pose;

    // Compare in the external map frame. GLIM's odometry and global graph stay
    // in their native frame; only this map-to-odom anchor is updated.
    const Eigen::Isometry3d local_correction = candidate * predicted_in_map.inverse();
    const Eigen::Isometry3d registration_anchor = candidate * odom_from_submap.inverse();
    Eigen::Isometry3d map_odom_observation = registration_anchor;
    if (anchor_available) {
      map_odom_observation = observeMapOdomWithFixedRotation(
        candidate, odom_from_submap, anchor_snapshot.linear());
    }
    const Eigen::Isometry3d anchor_disagreement =
      previous_global_anchor_available_ ?
      previous_global_anchor_.inverse() * registration_anchor :
      Eigen::Isometry3d::Identity();

    auto active_gate_params = gate_params_;
    if (!map_frame_initialized_ && bootstrap_anchor_available_) {
      active_gate_params.allow_global_relocalization = false;
      active_gate_params.max_local_correction_translation_m =
        environmentDouble("GLIM_PRIOR_MAP_MAX_BOOTSTRAP_TRANSLATION_M", 10.0);
    } else {
      // In a seeded map frame, a large crop-local correction is accepted only
      // after two independent submaps agree on the same map-from-GLIM-world
      // anchor. Unseeded acquisition still requires explicit full-map mode.
      active_gate_params.allow_global_relocalization =
        gate_params_.allow_global_relocalization &&
        (anchor_available || full_global_search_);
    }
    const PriorMapFactorGateDecision decision = decidePriorMapFactor(
      active_gate_params,
      PriorMapFactorGateInput{
        coarse_valid,
        finiteTransform(candidate),
        coarse_inliers,
        local_correction.translation().norm(),
        rotationDegrees(local_correction),
        previous_global_anchor_available_,
        anchor_disagreement.translation().norm(),
        rotationDegrees(anchor_disagreement)});

    if (!decision.accepted &&
      decision.reason.find("global_") == 0)
    {
      previous_global_anchor_ = registration_anchor;
      previous_global_anchor_available_ = finiteTransform(registration_anchor);
    }

    logger_->info(
      "submap={} source={} target={} inliers={} candidate=({:.3f},{:.3f},{:.3f}) "
      "VGICP_inliers={} overlap={:.3f} error={:.3f} "
      "correction={:.3f}m/{:.2f}deg decision={}",
      submap->id, source.size(), target.size(), coarse_inliers,
      candidate.translation().x(), candidate.translation().y(), candidate.translation().z(),
      refinement.num_inliers, refinement.inlier_fraction,
      refinement.normalized_error,
      local_correction.translation().norm(), rotationDegrees(local_correction),
      decision.reason);
    if (!decision.accepted) {
      return;
    }

    Eigen::Isometry3d updated_anchor = Eigen::Isometry3d::Identity();
    bool reset = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      if (!tracking_anchor_available_) {
        tracking_anchor_ = map_odom_observation;
        tracking_anchor_available_ = true;
        reference_anchor_ = tracking_anchor_;
        reference_anchor_available_ = true;
        reset = true;
      } else if (!reference_anchor_available_) {
        reference_anchor_ = tracking_anchor_;
        reference_anchor_.translation() = map_odom_observation.translation();
        tracking_anchor_ = reference_anchor_;
        reference_anchor_available_ = true;
      } else {
        tracking_anchor_ = updateMapOdomTranslation(
          reference_anchor_, map_odom_observation, translation_gain_);
      }
      updated_anchor = tracking_anchor_;
      initial_anchor_sent_ = true;
    }
    glim::GlobalMappingCallbacks::on_external_map_odom_update(
      origin_odom_frame->stamp, updated_anchor, reset);
    logger_->info(
      "updated map-to-odom target translation=({:.3f},{:.3f},{:.3f}) gain={:.3f}",
      updated_anchor.translation().x(), updated_anchor.translation().y(),
      updated_anchor.translation().z(), translation_gain_);
    map_frame_initialized_ = true;
    previous_global_anchor_available_ = false;
  }

  std::shared_ptr<spdlog::logger> logger_;
  std::vector<Eigen::Vector3f> prior_map_;
  kiss_matcher::KISSMatcher matcher_;
  PriorMapFactorGateParams gate_params_;
  double vgicp_voxel_resolution_m_{1.0};
  int vgicp_max_iterations_{10};
  double vgicp_min_inlier_fraction_{0.50};
  double vgicp_max_normalized_error_{5.0};
  double vgicp_max_coarse_delta_translation_m_{2.0};
  double vgicp_max_coarse_delta_rotation_deg_{5.0};
  double tracking_crop_radius_m_{35.0};
  double bootstrap_crop_radius_m_{60.0};
  std::size_t bootstrap_submap_stride_{2};
  std::size_t tracking_submap_stride_{10};
  double translation_gain_{0.2};
  bool full_global_search_{false};
  bool map_frame_initialized_{false};
  bool bootstrap_anchor_available_{false};
  Eigen::Isometry3d bootstrap_anchor_{Eigen::Isometry3d::Identity()};
  std::mutex anchor_mutex_;
  bool tracking_anchor_available_{false};
  bool initial_anchor_sent_{false};
  Eigen::Isometry3d tracking_anchor_{Eigen::Isometry3d::Identity()};
  bool reference_anchor_available_{false};
  Eigen::Isometry3d reference_anchor_{Eigen::Isometry3d::Identity()};
  bool previous_global_anchor_available_{false};
  Eigen::Isometry3d previous_global_anchor_{Eigen::Isometry3d::Identity()};
};

}  // namespace glim_prior_map_localizer

extern "C" glim::ExtensionModule * create_extension_module()
{
  return new glim_prior_map_localizer::PriorMapLocalizer();
}
