#include "glim_prior_map_localizer/prior_map_factor_policy.hpp"
#include "glim_prior_map_localizer/ply_loader.hpp"
#include "glim_prior_map_localizer/vgicp_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/sub_map.hpp>
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
    }

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    glim::GlobalMappingCallbacks::on_insert_submap.add(
      std::bind(&PriorMapLocalizer::onSubmap, this, _1));
    glim::GlobalMappingCallbacks::on_smoother_update.add(
      std::bind(&PriorMapLocalizer::onSmootherUpdate, this, _1, _2, _3));
    logger_->info(
      "loaded prior map with {} points; mode={} crop={:.1f}/{:.1f}m stride={}/{} seeded={}",
      prior_map_.size(), full_global_search_ ? "full-global" : "tracking-crop",
      bootstrap_crop_radius_m_, tracking_crop_radius_m_, bootstrap_submap_stride_,
      tracking_submap_stride_,
      bootstrap_anchor_available_);
  }

private:
  struct PendingFactor
  {
    int submap_id;
    Eigen::Isometry3d map_from_submap;
    bool global_relocalization;
    gtsam_points::GaussianVoxelMapCPU::ConstPtr target_voxels;
    gtsam_points::PointCloud::ConstPtr source;
  };

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

    const Eigen::Isometry3d predicted = submap->T_world_origin;
    const Eigen::Isometry3d predicted_in_map =
      (!graph_frame_seeded_ && bootstrap_anchor_available_) ?
      bootstrap_anchor_ * predicted : predicted;
    Eigen::Vector3d predicted_map_center = predicted_in_map.translation();
    if (previous_global_anchor_available_) {
      predicted_map_center = previous_global_anchor_ * predicted.translation();
    }
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
      !graph_frame_seeded_ || full_global_search_;
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

    // Compare in the map frame. Before the graph is anchored, predicted_in_map
    // applies the approximate startup pose; afterwards GLIM's world is map.
    const Eigen::Isometry3d local_correction = candidate * predicted_in_map.inverse();
    const Eigen::Isometry3d global_anchor = candidate * predicted.inverse();
    const Eigen::Isometry3d anchor_disagreement =
      previous_global_anchor_available_ ?
      previous_global_anchor_.inverse() * global_anchor :
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
        (graph_frame_seeded_ || full_global_search_);
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
      previous_global_anchor_ = global_anchor;
      previous_global_anchor_available_ = finiteTransform(global_anchor);
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

    pending_factors_.push_back(
      PendingFactor{
        submap->id, candidate,
        decision.accepted_as_global_relocalization || !graph_frame_seeded_,
        refinement.target_voxels, submap->frame});
    map_frame_initialized_ = true;
    previous_global_anchor_available_ = false;
  }

  void onSmootherUpdate(
    gtsam_points::ISAM2Ext &,
    gtsam::NonlinearFactorGraph & new_factors,
    gtsam::Values & new_values)
  {
    using gtsam::symbol_shorthand::X;
    if (!graph_frame_seeded_ && bootstrap_anchor_available_ && new_values.exists(X(0))) {
      const auto glim_world_from_submap = new_values.at<gtsam::Pose3>(X(0));
      const Eigen::Isometry3d map_from_submap =
        bootstrap_anchor_ * Eigen::Isometry3d(glim_world_from_submap.matrix());
      new_values.update(X(0), gtsam::Pose3(map_from_submap.matrix()));
      graph_frame_seeded_ = true;
      logger_->info("seeded GLIM X(0) in the prior-map frame");
    }
    for (const auto & pending : pending_factors_) {
      if (pending.global_relocalization && new_values.exists(X(pending.submap_id))) {
        // The current submap has not entered iSAM2 yet. Seed its linearization
        // at the consensus-validated map pose so an 80+ metre frame change is
        // not linearized as one enormous increment from the GLIM-world pose.
        new_values.update(
          X(pending.submap_id), gtsam::Pose3(pending.map_from_submap.matrix()));
      }
      new_factors.emplace_shared<gtsam_points::IntegratedVGICPFactor>(
        gtsam::Pose3::Identity(), X(pending.submap_id),
        pending.target_voxels, pending.source);
      logger_->info(
        "injected {} prior-map VGICP factor on submap {}",
        pending.global_relocalization ? "global-relocalization" : "local-tracking",
        pending.submap_id);
    }
    pending_factors_.clear();
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
  bool full_global_search_{false};
  bool map_frame_initialized_{false};
  bool graph_frame_seeded_{false};
  bool bootstrap_anchor_available_{false};
  Eigen::Isometry3d bootstrap_anchor_{Eigen::Isometry3d::Identity()};
  bool previous_global_anchor_available_{false};
  Eigen::Isometry3d previous_global_anchor_{Eigen::Isometry3d::Identity()};
  std::vector<PendingFactor> pending_factors_;
};

}  // namespace glim_prior_map_localizer

extern "C" glim::ExtensionModule * create_extension_module()
{
  return new glim_prior_map_localizer::PriorMapLocalizer();
}
