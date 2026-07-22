#include "glim_prior_map_localizer/prior_map_factor_policy.hpp"
#include "glim_prior_map_localizer/map_odom_policy.hpp"
#include "glim_prior_map_localizer/map_odom_transition_policy.hpp"
#include "glim_prior_map_localizer/map_state_prior.hpp"
#include "glim_prior_map_localizer/ply_loader.hpp"
#include "glim_prior_map_localizer/vgicp_refiner.hpp"
#include "glim_prior_map_localizer/prior_map_overlap_policy.hpp"
#include "glim_prior_map_localizer/recovery_consensus_policy.hpp"
#include "glim_prior_map_localizer/recovery_pose_policy.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/nearest_neighbor_search.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/types/frame_traits.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/integrated_gicp_factor_coreset.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/logging.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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

gtsam_points::PointCloudCPU::Ptr makeTightlyCoupledPriorMap(
  const std::vector<Eigen::Vector3f> & map,
  double voxel_resolution_m,
  double covariance_scale,
  int covariance_neighbors,
  int num_threads)
{
  std::vector<Eigen::Vector4d> points;
  points.reserve(map.size());
  for (const auto & point : map) {
    if (point.allFinite()) {
      points.emplace_back(point.x(), point.y(), point.z(), 1.0);
    }
  }
  auto dense = std::make_shared<gtsam_points::PointCloudCPU>(points);
  auto sampled = gtsam_points::voxelgrid_sampling(
    dense, voxel_resolution_m, num_threads);
  auto covariances = gtsam_points::estimate_covariances(
    *sampled, covariance_neighbors, num_threads);
  for (auto & covariance : covariances) {
    covariance.topLeftCorner<3, 3>() *= covariance_scale;
  }
  sampled->add_covs(covariances);
  return sampled;
}

gtsam_points::PointCloudCPU::Ptr makeRecoveryLidarFrame(
  const glim::PreprocessedFrame & raw_frame,
  int covariance_neighbors,
  int num_threads,
  double voxel_resolution_m = 0.0)
{
  if (raw_frame.points.empty()) {
    return nullptr;
  }
  auto dense = std::make_shared<gtsam_points::PointCloudCPU>(
    raw_frame.points);
  auto source = voxel_resolution_m > 0.0 ?
    gtsam_points::voxelgrid_sampling(dense, voxel_resolution_m, num_threads) : dense;
  source->add_covs(gtsam_points::estimate_covariances(
    *source, covariance_neighbors, num_threads));
  return source;
}

gtsam_points::PointCloudCPU::Ptr makeRecoveryLidarFrame(
  const glim::EstimationFrame & frame,
  int covariance_neighbors,
  int num_threads)
{
  return frame.raw_frame ? makeRecoveryLidarFrame(
    *frame.raw_frame, covariance_neighbors, num_threads) : nullptr;
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

class PriorMapLocalizer : public glim::ExtensionModuleROS2
{
public:
  PriorMapLocalizer()
  : logger_(glim::create_module_logger("prior_map_localizer")),
    prior_map_(loadPriorMap(requiredMapPath())),
    matcher_(matcherConfig())
  {
    tightly_coupled_ =
      environmentBool("GLIM_PRIOR_MAP_TIGHTLY_COUPLED", false);
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
    recovery_max_normalized_error_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_MAX_NORMALIZED_ERROR", 12.0);
    recovery_min_inlier_fraction_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_MIN_INLIER_FRACTION", 0.75);
    recovery_rearm_cooldown_sec_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_REARM_COOLDOWN_SEC", 30.0);
    recovery_rerank_voxel_resolution_m_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_RERANK_VOXEL_RESOLUTION_M", 1.0);
    recovery_rerank_max_iterations_ = static_cast<int>(environmentSize(
      "GLIM_PRIOR_MAP_RECOVERY_RERANK_MAX_ITERATIONS", 5));
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
    // Sparse submap anchors stabilize map-degenerate directions but their XY
    // registration still contains noise. Keep the robust horizontal gain;
    // tightly coupled per-frame factors provide the undamped graph constraint
    // and Z is injected fully below.
    translation_gain_ = std::clamp(
      environmentDouble("GLIM_PRIOR_MAP_TRANSLATION_GAIN", 0.2), 0.0, 1.0);
    pending_vertical_update_enabled_ =
      std::getenv("GLIM_PRIOR_MAP_VERTICAL_GAIN") != nullptr;
    vertical_gain_ = std::clamp(
      environmentDouble("GLIM_PRIOR_MAP_VERTICAL_GAIN", translation_gain_), 0.0, 1.0);
    full_global_search_ =
      environmentBool("GLIM_PRIOR_MAP_FULL_GLOBAL_SEARCH", false);
    rerank_bbs_candidates_ =
      environmentBool("GLIM_PRIOR_MAP_RERANK_BBS_CANDIDATES", false);
    odom_bridge_recovery_enabled_ =
      environmentBool("GLIM_PRIOR_MAP_ODOM_BRIDGE_RECOVERY", false);
    odom_bridge_recovery_crop_radius_m_ = environmentDouble(
      "GLIM_PRIOR_MAP_ODOM_BRIDGE_RECOVERY_CROP_RADIUS_M", 70.0);
    odom_bridge_recovery_max_coarse_delta_m_ = environmentDouble(
      "GLIM_PRIOR_MAP_ODOM_BRIDGE_RECOVERY_MAX_COARSE_DELTA_M", 5.0);
    if (odom_bridge_recovery_crop_radius_m_ < tracking_crop_radius_m_) {
      throw std::invalid_argument(
              "GLIM odom-bridge recovery crop must cover the tracking crop");
    }
    map_factor_voxel_resolution_m_ =
      environmentDouble("GLIM_PRIOR_MAP_FACTOR_VOXEL_RESOLUTION_M", 0.5);
    map_factor_covariance_neighbors_ = static_cast<int>(
      environmentSize("GLIM_PRIOR_MAP_FACTOR_COVARIANCE_NEIGHBORS", 10));
    map_factor_covariance_scale_ = environmentDouble(
      "GLIM_PRIOR_MAP_FACTOR_COVARIANCE_SCALE", 0.25);
    if (map_factor_covariance_scale_ <= 0.0) {
      throw std::invalid_argument("GLIM prior-map covariance scale must be positive");
    }
    map_factor_max_correspondence_m_ =
      environmentDouble("GLIM_PRIOR_MAP_FACTOR_MAX_CORRESPONDENCE_M", 2.0);
    map_factor_coreset_size_ = static_cast<int>(
      environmentSize("GLIM_PRIOR_MAP_FACTOR_CORESET_SIZE", 32));
    map_factor_coreset_reuse_translation_m_ =
      environmentDouble("GLIM_PRIOR_MAP_FACTOR_CORESET_REUSE_TRANSLATION_M", 1.0);
    map_factor_coreset_reuse_rotation_rad_ =
      environmentDouble("GLIM_PRIOR_MAP_FACTOR_CORESET_REUSE_ROTATION_RAD", 0.035);
    map_factor_num_threads_ = static_cast<int>(
      environmentSize("GLIM_PRIOR_MAP_FACTOR_NUM_THREADS", 1));
    map_factor_overlap_stride_ = std::max<std::size_t>(
      1, environmentSize("GLIM_PRIOR_MAP_FACTOR_OVERLAP_STRIDE", 10));
    map_factor_frame_stride_ = std::max<std::size_t>(
      1, environmentSize("GLIM_PRIOR_MAP_FACTOR_FRAME_STRIDE", 2));
    map_factor_min_overlap_fraction_ = std::clamp(
      environmentDouble("GLIM_PRIOR_MAP_FACTOR_MIN_OVERLAP_FRACTION", 0.05),
      0.0, 1.0);
    map_factor_min_overlap_inliers_ = environmentSize(
      "GLIM_PRIOR_MAP_FACTOR_MIN_OVERLAP_INLIERS", 32);
    map_factor_min_tracking_crop_points_ = environmentSize(
      "GLIM_PRIOR_MAP_FACTOR_MIN_TRACKING_CROP_POINTS", 50000);
    map_state_prior_precision_ = environmentDouble(
      "GLIM_PRIOR_MAP_STATE_PRIOR_PRECISION", 1.0);
    if (map_state_prior_precision_ <= 0.0) {
      throw std::invalid_argument("GLIM prior-map state prior precision must be positive");
    }
    recovery_confirmation_frames_ = std::max<std::size_t>(
      2, environmentSize("GLIM_PRIOR_MAP_RECOVERY_CONFIRMATION_FRAMES", 3));
    recovery_rejection_frames_ = std::max<std::size_t>(
      1, environmentSize("GLIM_PRIOR_MAP_RECOVERY_REJECTION_FRAMES", 3));
    recovery_max_correction_translation_m_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_MAX_CORRECTION_M", 1.5);
    recovery_max_correction_rotation_deg_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_MAX_CORRECTION_DEG", 6.0);
    recovery_max_consensus_translation_m_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_MAX_CONSENSUS_M", 1.5);
    recovery_max_consensus_rotation_deg_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_MAX_CONSENSUS_DEG", 6.0);
    recovery_loss_overlap_fraction_ = std::clamp(environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_LOSS_OVERLAP_FRACTION", 0.75), 0.0, 1.0);
    recovery_loss_confirmation_frames_ = std::max<std::size_t>(
      1, environmentSize("GLIM_PRIOR_MAP_RECOVERY_LOSS_CONFIRMATION_FRAMES", 2));
    public_max_translation_step_m_ = environmentDouble(
      "GLIM_PRIOR_MAP_PUBLIC_MAX_TRANSLATION_STEP_M", 0.25);
    public_max_rotation_step_deg_ = environmentDouble(
      "GLIM_PRIOR_MAP_PUBLIC_MAX_ROTATION_STEP_DEG", 2.0);
    recovery_public_max_translation_step_m_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_PUBLIC_MAX_TRANSLATION_STEP_M", 1.0);
    recovery_public_max_rotation_step_deg_ = environmentDouble(
      "GLIM_PRIOR_MAP_RECOVERY_PUBLIC_MAX_ROTATION_STEP_DEG", 5.0);
    if (public_max_translation_step_m_ <= 0.0 || public_max_rotation_step_deg_ <= 0.0 ||
      recovery_public_max_translation_step_m_ <= 0.0 ||
      recovery_public_max_rotation_step_deg_ <= 0.0)
    {
      throw std::invalid_argument(
              "GLIM prior-map public transition bounds must be positive");
    }
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

    if (tightly_coupled_) {
      if (!bootstrap_anchor_available_) {
        throw std::invalid_argument(
                "tightly coupled prior-map mode currently requires a 4-DoF bootstrap seed");
      }
      prior_map_frame_ = makeTightlyCoupledPriorMap(
        prior_map_, map_factor_voxel_resolution_m_,
        map_factor_covariance_scale_,
        map_factor_covariance_neighbors_, map_factor_num_threads_);
      prior_map_tree_ =
        std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(
        prior_map_frame_);
      recovery_rerank_target_ = makeVgicpTarget(
        prior_map_frame_, vgicp_voxel_resolution_m_);
    }

    using std::placeholders::_1;
    glim::OdometryEstimationCallbacks::on_new_frame.add(
      std::bind(&PriorMapLocalizer::onOdometryFrame, this, _1));
    if (tightly_coupled_) {
      using std::placeholders::_2;
      using std::placeholders::_3;
      using std::placeholders::_4;
      glim::OdometryEstimationCallbacks::on_smoother_update.add(
        std::bind(
          &PriorMapLocalizer::onTightlyCoupledSmootherUpdate, this,
          _1, _2, _3, _4));
      glim::OdometryEstimationCallbacks::on_smoother_update_finish.add(
        std::bind(
          &PriorMapLocalizer::onTightlyCoupledSmootherUpdateFinish, this,
          _1));
    }
    glim::GlobalMappingCallbacks::on_insert_submap.add(
      std::bind(&PriorMapLocalizer::onSubmap, this, _1));
    logger_->info(
      "loaded prior map with {} points; architecture={} mode={} crop={:.1f}/{:.1f}m stride={}/{} "
      "seeded={} translation_gain={:.3f} vertical_gain={:.3f} pending_z={}",
      prior_map_.size(), tightly_coupled_ ? "tightly-coupled" : "external-map-odom",
      full_global_search_ ? "full-global" : "tracking-crop",
      bootstrap_crop_radius_m_, tracking_crop_radius_m_, bootstrap_submap_stride_,
      tracking_submap_stride_,
      bootstrap_anchor_available_, translation_gain_, vertical_gain_,
      pending_vertical_update_enabled_);
  }

  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions(
    rclcpp::Node & node) override
  {
    if (!tightly_coupled_) {
      return {};
    }
    recovery_requested_pub_ = node.create_publisher<std_msgs::msg::Bool>(
      "/reinitialization_requested",
      rclcpp::QoS(1).reliable().transient_local());
    publishRecoveryRequested(false);
    recovery_cloud_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/glil/recovery_points", rclcpp::SensorDataQoS());
    using Message = geometry_msgs::msg::PoseWithCovarianceStamped;
    return {
      std::make_shared<glim::TopicSubscription<Message>>(
        "/initialpose",
        std::bind(&PriorMapLocalizer::onRelocalizationCandidate, this,
          std::placeholders::_1)),
      std::make_shared<glim::TopicSubscription<geometry_msgs::msg::PoseArray>>(
        "/global_localization_node/candidates",
        std::bind(&PriorMapLocalizer::onGlobalCandidates, this,
          std::placeholders::_1))};
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
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_odometry_frame_ = frame;
      // Recovery queries are published from raw_frame in the LiDAR frame, so
      // timestamp alignment must store the matching LiDAR pose rather than
      // the graph's IMU pose.  This is a no-op for identity extrinsics and is
      // essential for general sensor rigs.
      odometry_history_.emplace_back(frame->stamp, frame->T_world_lidar);
      while (!odometry_history_.empty() &&
        frame->stamp - odometry_history_.front().first > 120.0)
      {
        odometry_history_.pop_front();
      }
      if (tightly_coupled_ && recovery_search_requested_.load() && frame->raw_frame &&
        (recovery_frame_history_.empty() ||
        frame->stamp - recovery_frame_history_.back().first >= 0.5))
      {
        recovery_frame_history_.emplace_back(frame->stamp, frame->raw_frame);
      }
      while (!recovery_frame_history_.empty() &&
        frame->stamp - recovery_frame_history_.front().first > 30.0)
      {
        recovery_frame_history_.pop_front();
      }
    }
    if (tightly_coupled_ && recovery_search_requested_.load()) {
      // Keep the recovery request latched on the wire while localization is
      // lost.  A single transition message can be missed when the supervisor
      // is starting or DDS endpoints are still matching, whereas recovery
      // clouds already provide a natural, bounded retry cadence.
      publishRecoveryRequested(true);
      publishRecoveryCloud(*frame);
    }
    if (tightly_coupled_) {
      return;
    }

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

  void onTightlyCoupledSmootherUpdate(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback & smoother,
    gtsam::NonlinearFactorGraph & new_factors,
    gtsam::Values & new_values,
    std::map<std::uint64_t, double> & new_stamps)
  {
    glim::EstimationFrame::ConstPtr frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      frame = latest_odometry_frame_;
    }
    if (!frame || !frame->frame || frame->frame->size() == 0) {
      return;
    }

    using gtsam::symbol_shorthand::X;
    const gtsam::Key pose_key = X(frame->id);
    if (!new_values.exists(pose_key)) {
      logger_->warn(
        "cannot insert tightly coupled map factor: X({}) is not a new value",
        frame->id);
      return;
    }

    if (!tightly_coupled_graph_seeded_) {
      const Eigen::Isometry3d map_from_imu =
        bootstrap_anchor_ * frame->T_world_imu;
      const Eigen::Isometry3d odom_from_map =
        frame->T_world_imu * map_from_imu.inverse();
      new_values.insert(map_state_key_, gtsam::Pose3(odom_from_map.matrix()));
      tightly_coupled_graph_seeded_ = true;
      logger_->info(
        "initialized tightly coupled odom-from-map state at X({})",
        frame->id);
    }

    const Eigen::Isometry3d odom_from_imu(
      new_values.at<gtsam::Pose3>(pose_key).matrix());
    activatePendingSparseAnchor(*frame, new_values, new_stamps);
    const Eigen::Isometry3d odom_from_lidar =
      odom_from_imu * frame->T_lidar_imu.inverse();
    evaluatePendingRecovery(
      *frame, odom_from_lidar, new_values, new_stamps);
    // A map state is refreshed in the fixed-lag timestamp map even while
    // overlap is lost.  Without a unary factor, all old scan-to-map factors
    // can age out and leave that value disconnected.  The smoother fallback
    // then removes mN, while this extension still references it on the next
    // frame.  Add one weak persistent prior whenever a new map-state epoch is
    // inserted; it keeps the map gauge connected without competing with the
    // much stronger scan-to-map Hessians.
    if (new_values.exists(map_state_key_)) {
      addMapStateKeepAlivePrior(
        new_factors, map_state_key_,
        new_values.at<gtsam::Pose3>(map_state_key_),
        map_state_prior_precision_);
    }
    new_stamps[map_state_key_] = frame->stamp;
    latest_tightly_coupled_stamp_ = frame->stamp;

    const gtsam::Pose3 odom_from_map_pose = new_values.exists(map_state_key_) ?
      new_values.at<gtsam::Pose3>(map_state_key_) :
      smoother.calculateEstimate<gtsam::Pose3>(map_state_key_);
    const Eigen::Isometry3d odom_from_map(odom_from_map_pose.matrix());
    const Eigen::Isometry3d map_from_imu =
      odom_from_map.inverse() * odom_from_imu;
    const auto [overlap_inliers, overlap_samples] = mapOverlap(
      *frame->frame, map_from_imu);
    const auto overlap = decidePriorMapOverlap(
      overlap_inliers, overlap_samples, map_factor_min_overlap_inliers_,
      map_factor_min_overlap_fraction_);
    const bool recovery_loss_evidence = isRecoveryLossEvidence(
      overlap, map_factor_min_overlap_inliers_, recovery_loss_overlap_fraction_);
    recovery_loss_evidence_frames_ = recovery_loss_evidence ?
      recovery_loss_evidence_frames_ + 1 : 0;
    if (recovery_loss_evidence_frames_ >= recovery_loss_confirmation_frames_ &&
      !recovery_search_requested_.load())
    {
      const bool recovery_rearm_allowed = recoveryRearmAllowed(
        last_verified_recovery_stamp_, frame->stamp, recovery_rearm_cooldown_sec_);
      if (recovery_rearm_allowed && !recovery_search_requested_.exchange(true)) {
        logger_->warn(
          "prior-map recovery search requested at X({}) after {} frames below "
          "{:.3f} overlap; public map-to-odom remains graph-driven until a "
          "strong global candidate confirms lock loss",
          frame->id, recovery_loss_evidence_frames_, recovery_loss_overlap_fraction_);
        publishRecoveryRequested(true);
      } else if (!recovery_rearm_allowed) {
        logger_->info(
          "suppressed recovery rearm {:.1f}s after verified recovery",
          frame->stamp - last_verified_recovery_stamp_);
      }
    }
    // The recovery threshold is deliberately stricter than the ordinary map
    // factor gate. Entering global search must freeze the public map frame, but
    // it must not discard still-valid local map correspondences. Those factors
    // remain in the same fixed-lag graph until the ordinary overlap gate fails;
    // a true kidnap naturally falls through that lower gate, while a sparse
    // normal road keeps its available absolute constraints.
    if (!overlap.sufficient) {
      if (map_factor_active_) {
        logger_->warn(
          "prior-map overlap lost at X({}): {}/{} ({:.3f}); "
          "continuing with scan-to-scan and IMU factors",
          frame->id, overlap.inliers, overlap.samples, overlap.fraction);
      }
      map_factor_active_ = false;
      return;
    }
    if (recovery_search_requested_.load() &&
      !map_factor_region_observable_.load())
    {
      if (map_factor_active_) {
        logger_->warn(
          "pausing prior-map factors in low-density tracking crop during "
          "global search");
      }
      map_factor_active_ = false;
      return;
    }
    if (!map_factor_active_) {
      logger_->info(
        "prior-map overlap acquired at X({}): {}/{} ({:.3f}); "
        "resuming unary map factors",
        frame->id, overlap.inliers, overlap.samples, overlap.fraction);
    }
    map_factor_active_ = true;

    // IMU and scan-to-scan factors remain at every LiDAR frame. Prior-map
    // factors are keyframe constraints; inserting the nearly identical map
    // Hessian at 10 Hz adds CPU pressure without useful independent geometry.
    if (frame->id % map_factor_frame_stride_ != 0) {
      return;
    }

    auto factor = gtsam::make_shared<glim::IntegratedGICPFactorCoreset>(
      map_state_key_, pose_key, prior_map_frame_, frame->frame,
      prior_map_tree_);
    factor->set_max_correspondence_distance(
      map_factor_max_correspondence_m_);
    factor->set_num_threads(map_factor_num_threads_);
    factor->set_coreset_size(map_factor_coreset_size_);
    factor->set_coreset_reuse_tolerance(
      map_factor_coreset_reuse_rotation_rad_,
      map_factor_coreset_reuse_translation_m_);
    new_factors.add(factor);
  }

  void onRelocalizationCandidate(
    const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> & message)
  {
    if (rerank_bbs_candidates_) {
      // PoseArray re-ranking is authoritative in this mode. The supervisor's
      // rank-1 compatibility topic can arrive after the batch has already
      // recovered and must not start a second graph epoch.
      logger_->info(
        "ignored rank-1 /initialpose because GLIL BBS batch reranking is enabled");
      return;
    }
    const auto & pose = message->pose.pose;
    Eigen::Quaterniond orientation(
      pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z);
    if (!orientation.coeffs().allFinite() || orientation.norm() < 1.0e-6) {
      logger_->warn("rejected non-finite /initialpose recovery candidate");
      return;
    }
    orientation.normalize();
    Eigen::Isometry3d map_from_sensor = Eigen::Isometry3d::Identity();
    map_from_sensor.linear() = orientation.toRotationMatrix();
    map_from_sensor.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
    if (!finiteTransform(map_from_sensor)) {
      logger_->warn("rejected invalid /initialpose recovery candidate");
      return;
    }
    if (recovery_search_requested_.load()) {
      map_lock_lost_.store(true);
    }

    const double candidate_stamp =
      static_cast<double>(message->header.stamp.sec) +
      static_cast<double>(message->header.stamp.nanosec) * 1.0e-9;
    if (candidate_stamp <= 0.0) {
      queueRecoveryCandidate(map_from_sensor, "/initialpose", true);
      return;
    }

    Eigen::Isometry3d odom_from_sensor_at_candidate = Eigen::Isometry3d::Identity();
    bool history_found = false;
    double best_delta = 0.25;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      for (const auto & sample : odometry_history_) {
        const double delta = std::abs(sample.first - candidate_stamp);
        if (delta <= best_delta) {
          best_delta = delta;
          odom_from_sensor_at_candidate = sample.second;
          history_found = true;
        }
      }
    }
    if (!history_found) {
      logger_->warn(
        "rejected stale /initialpose candidate: no odometry sample near stamp={:.6f}",
        candidate_stamp);
      return;
    }
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    pending_recovery_map_from_sensor_ = map_from_sensor;
    pending_recovery_odom_from_map_ =
      odom_from_sensor_at_candidate * map_from_sensor.inverse();
    pending_recovery_anchor_initialized_ = true;
    pending_recovery_confirmations_ = 0;
    pending_recovery_failures_ = 0;
    pending_recovery_ = true;
    ++pending_recovery_generation_;
    logger_->info(
      "received time-aligned /initialpose recovery candidate generation={} "
      "stamp={:.6f} odom_delta={:.3f}s",
      pending_recovery_generation_, candidate_stamp, best_delta);
  }

  void onGlobalCandidates(
    const std::shared_ptr<const geometry_msgs::msg::PoseArray> & message)
  {
    if (!rerank_bbs_candidates_ || !recovery_search_requested_.load() ||
      message->poses.empty())
    {
      return;
    }
    const double candidate_stamp =
      static_cast<double>(message->header.stamp.sec) +
      static_cast<double>(message->header.stamp.nanosec) * 1.0e-9;
    glim::PreprocessedFrame::ConstPtr raw_frame;
    Eigen::Isometry3d odom_from_sensor_at_candidate = Eigen::Isometry3d::Identity();
    bool odometry_found = false;
    double best_stamp_delta = 0.55;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      for (const auto & sample : recovery_frame_history_) {
        const double delta = std::abs(sample.first - candidate_stamp);
        if (delta <= best_stamp_delta) {
          best_stamp_delta = delta;
          raw_frame = sample.second;
        }
      }
      double best_odom_delta = 0.55;
      for (const auto & sample : odometry_history_) {
        const double delta = std::abs(sample.first - candidate_stamp);
        if (delta <= best_odom_delta) {
          best_odom_delta = delta;
          odom_from_sensor_at_candidate = sample.second;
          odometry_found = true;
        }
      }
    }
    if (!raw_frame || !odometry_found) {
      logger_->warn(
        "ignored BBS candidate batch: no time-aligned raw/odometry frame near stamp={:.6f}",
        candidate_stamp);
      return;
    }
    const auto source = makeRecoveryLidarFrame(
      *raw_frame, map_factor_covariance_neighbors_, map_factor_num_threads_,
      recovery_rerank_voxel_resolution_m_);
    if (!source) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      if (bbs_batch_candidate_active_ || pending_recovery_) {
        logger_->info("ignored duplicate BBS candidate batch while recovery is pending");
        return;
      }
      bbs_batch_candidate_active_ = true;
    }

    bool selected = false;
    double selected_metric = std::numeric_limits<double>::infinity();
    double selected_overlap = 0.0;
    Eigen::Isometry3d selected_pose = Eigen::Isometry3d::Identity();
    std::size_t selected_index = 0;
    for (std::size_t index = 0; index < message->poses.size(); ++index) {
      const auto & pose = message->poses[index];
      Eigen::Quaterniond orientation(
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
      if (!orientation.coeffs().allFinite() || orientation.norm() < 1.0e-6) {
        continue;
      }
      orientation.normalize();
      Eigen::Isometry3d seed = Eigen::Isometry3d::Identity();
      seed.linear() = orientation.toRotationMatrix();
      seed.translation() = Eigen::Vector3d(
        pose.position.x, pose.position.y, pose.position.z);
      if (!finiteTransform(seed)) {
        continue;
      }
      if (!recovery_rerank_target_) {
        continue;
      }
      const auto refinement = refineWithVgicp(
        recovery_rerank_target_, source, seed, recovery_rerank_max_iterations_);
      const Eigen::Isometry3d correction = recoveryPoseCorrection(seed, refinement.pose);
      const bool admissible = refinement.valid &&
        refinement.inlier_fraction >= 0.30 &&
        refinement.normalized_error <= recovery_max_normalized_error_ &&
        correction.translation().norm() <= 2.0 &&
        rotationDegrees(correction) <= 10.0;
      const double metric = refinement.normalized_error /
        std::max(0.10, refinement.inlier_fraction);
      logger_->info(
        "BBS/VGICP rank={} valid={} overlap={:.3f} error={:.3f} "
        "correction={:.3f}m/{:.2f}deg metric={:.3f} admissible={}",
        index + 1, refinement.valid, refinement.inlier_fraction,
        refinement.normalized_error, correction.translation().norm(),
        rotationDegrees(correction), metric, admissible);
      if (admissible && metric < selected_metric) {
        selected = true;
        selected_metric = metric;
        selected_overlap = refinement.inlier_fraction;
        selected_pose = refinement.pose;
        selected_index = index;
      }
    }
    if (!selected) {
      {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        bbs_batch_candidate_active_ = false;
      }
      logger_->warn("BBS/VGICP rejected all {} global candidates", message->poses.size());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      pending_recovery_map_from_sensor_ = selected_pose;
      pending_recovery_odom_from_map_ =
        odom_from_sensor_at_candidate * selected_pose.inverse();
      pending_recovery_anchor_initialized_ = true;
      pending_recovery_confirmations_ = 0;
      pending_recovery_failures_ = 0;
      pending_recovery_ = true;
      bbs_batch_candidate_active_ = true;
      ++pending_recovery_generation_;
      logger_->info(
        "received time-aligned GLIL-ranked BBS/VGICP recovery candidate "
        "generation={} stamp={:.6f}",
        pending_recovery_generation_, candidate_stamp);
    }
    logger_->warn(
      "selected BBS candidate rank={} of {} after GLIL 3D reranking "
      "(metric={:.3f}, sparse_overlap={:.3f}); public map-to-odom remains "
      "graph-driven until full-resolution verification",
      selected_index + 1, message->poses.size(), selected_metric, selected_overlap);
  }

  void publishRecoveryRequested(bool requested)
  {
    if (!recovery_requested_pub_) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = requested;
    recovery_requested_pub_->publish(message);
  }

  void publishRecoveryCloud(const glim::EstimationFrame & frame)
  {
    // EstimationFrame::frame is deskewed and expressed in frame.frame_id
    // (IMU for OdometryEstimationIMU).  G2/BBS consumes a sensor-frame scan,
    // just like the original PointCloud2 input, so publishing frame here rotates
    // and translates the query into the wrong coordinate system.  raw_frame is
    // explicitly retained by GLIM in the LiDAR frame and carries the source scan
    // timestamp needed by the supervisor's odom-history compensation.
    if (!recovery_cloud_pub_ || !frame.raw_frame || frame.raw_frame->size() == 0) {
      return;
    }
    const auto & raw_frame = *frame.raw_frame;
    sensor_msgs::msg::PointCloud2 message;
    const double seconds_floor = std::floor(raw_frame.stamp);
    message.header.stamp.sec = static_cast<std::int32_t>(seconds_floor);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(
      std::llround((raw_frame.stamp - seconds_floor) * 1.0e9));
    if (message.header.stamp.nanosec >= 1000000000U) {
      ++message.header.stamp.sec;
      message.header.stamp.nanosec -= 1000000000U;
    }
    message.header.frame_id = "glim_lidar";
    message.height = 1;
    message.width = static_cast<std::uint32_t>(raw_frame.size());
    message.is_bigendian = false;
    message.is_dense = false;
    message.point_step = 3U * sizeof(float);
    message.row_step = message.width * message.point_step;
    message.fields.resize(3);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      auto & field = message.fields[axis];
      field.name = std::string(1, "xyz"[axis]);
      field.offset = static_cast<std::uint32_t>(axis * sizeof(float));
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
    }
    message.data.resize(message.row_step);
    for (std::size_t index = 0; index < raw_frame.points.size(); ++index) {
      const Eigen::Vector4d & point = raw_frame.points[index];
      const float xyz[3] = {
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z())};
      std::memcpy(
        message.data.data() + index * message.point_step, xyz, sizeof(xyz));
    }
    recovery_cloud_pub_->publish(message);
  }

  bool queueRecoveryCandidate(
    const Eigen::Isometry3d & map_from_sensor,
    const char * source,
    bool replace_pending = false)
  {
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    if (pending_recovery_ && !replace_pending) {
      logger_->info(
        "ignored {} recovery candidate while generation={} is pending",
        source, pending_recovery_generation_);
      return false;
    }
    pending_recovery_map_from_sensor_ = map_from_sensor;
    pending_recovery_anchor_initialized_ = false;
    pending_recovery_confirmations_ = 0;
    pending_recovery_failures_ = 0;
    pending_recovery_ = true;
    ++pending_recovery_generation_;
    logger_->info(
      "received {} recovery candidate generation={} at ({:.3f},{:.3f},{:.3f})",
      source, pending_recovery_generation_, map_from_sensor.translation().x(),
      map_from_sensor.translation().y(), map_from_sensor.translation().z());
    return true;
  }

  void activatePendingSparseAnchor(
    const glim::EstimationFrame & frame,
    gtsam::Values & new_values,
    std::map<std::uint64_t, double> & new_stamps)
  {
    Eigen::Isometry3d odom_from_map = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(sparse_anchor_mutex_);
      if (!pending_sparse_anchor_) {
        return;
      }
      odom_from_map = pending_sparse_odom_from_map_;
      pending_sparse_anchor_ = false;
    }
    ++map_state_epoch_;
    map_state_key_ = gtsam::Symbol('m', map_state_epoch_);
    new_values.insert(map_state_key_, gtsam::Pose3(odom_from_map.matrix()));
    new_stamps[map_state_key_] = frame.stamp;
    map_factor_active_ = false;
    logger_->info(
      "activated verified sparse prior-map anchor as map state m{}",
      map_state_epoch_);
  }

  void evaluatePendingRecovery(
    const glim::EstimationFrame & frame,
    const Eigen::Isometry3d & odom_from_lidar,
    gtsam::Values & new_values,
    std::map<std::uint64_t, double> & new_stamps)
  {
    std::size_t generation = 0;
    Eigen::Isometry3d candidate_odom_from_map = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      if (!pending_recovery_) {
        return;
      }
      generation = pending_recovery_generation_;
      if (!pending_recovery_anchor_initialized_) {
        pending_recovery_odom_from_map_ =
          odom_from_lidar * pending_recovery_map_from_sensor_.inverse();
        pending_recovery_anchor_initialized_ = true;
      }
      candidate_odom_from_map = pending_recovery_odom_from_map_;
    }

    // The candidate belongs to the query scan timestamp. Ranking dozens of
    // hypotheses can take several seconds while LiDAR processing continues,
    // so advance it with post-query odometry before verifying it. Candidate
    // callbacks form this anchor from time-aligned odometry; un-stamped manual
    // candidates initialize it above at their first verification frame.
    const Eigen::Isometry3d predicted_map_from_lidar = propagateRecoveryPose(
      candidate_odom_from_map, odom_from_lidar);
    const auto target = cropMap(
      prior_map_, predicted_map_from_lidar.translation(), tracking_crop_radius_m_);
    const auto recovery_source = makeRecoveryLidarFrame(
      frame, map_factor_covariance_neighbors_, map_factor_num_threads_);
    VgicpRefinementResult refinement;
    if (!target.empty() && recovery_source) {
      refinement = refineWithVgicp(
        target, recovery_source, predicted_map_from_lidar,
        vgicp_voxel_resolution_m_, vgicp_max_iterations_);
    }
    // Measure the candidate correction in the sensor-local tangent frame.
    // The opposite product rotates the map-frame origin and turns a small
    // angular correction into a large apparent translation far from (0, 0).
    const Eigen::Isometry3d correction =
      recoveryPoseCorrection(predicted_map_from_lidar, refinement.pose);
    const Eigen::Isometry3d observed_odom_from_map =
      odom_from_lidar * refinement.pose.inverse();
    const Eigen::Isometry3d disagreement = correction;
    const bool accepted =
      refinement.valid &&
      refinement.inlier_fraction >= recovery_min_inlier_fraction_ &&
      refinement.normalized_error <= recovery_max_normalized_error_ &&
      correction.translation().norm() <= recovery_max_correction_translation_m_ &&
      rotationDegrees(correction) <= recovery_max_correction_rotation_deg_ &&
      disagreement.translation().norm() <= recovery_max_consensus_translation_m_ &&
      rotationDegrees(disagreement) <= recovery_max_consensus_rotation_deg_;
    logger_->info(
      "recovery generation={} frame={} valid={} inliers={} overlap={:.3f} error={:.3f} "
      "correction={:.3f}m/{:.2f}deg disagreement={:.3f}m/{:.2f}deg accepted={}",
      generation, frame.id, refinement.valid, refinement.num_inliers,
      refinement.inlier_fraction, refinement.normalized_error,
      correction.translation().norm(), rotationDegrees(correction),
      disagreement.translation().norm(), rotationDegrees(disagreement), accepted);
    if (accepted && !map_lock_lost_.exchange(true)) {
      logger_->warn(
        "confirmed map lock loss from full-resolution recovery overlap {:.3f}; "
        "holding public map-to-odom during remaining consensus frames",
        refinement.inlier_fraction);
    }

    RecoveryConsensusAction consensus_action = RecoveryConsensusAction::kPending;
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      if (!pending_recovery_ || generation != pending_recovery_generation_) {
        return;
      }
      if (accepted) {
        pending_recovery_odom_from_map_ = observed_odom_from_map;
        pending_recovery_map_from_sensor_ = refinement.pose;
      }
      RecoveryConsensusState state{
        pending_recovery_confirmations_, pending_recovery_failures_};
      consensus_action = updateRecoveryConsensus(
        state, accepted, recovery_confirmation_frames_, recovery_rejection_frames_);
      pending_recovery_confirmations_ = state.confirmations;
      pending_recovery_failures_ = state.failures;
      if (consensus_action == RecoveryConsensusAction::kReject) {
        pending_recovery_ = false;
        bbs_batch_candidate_active_ = false;
        map_lock_lost_.store(false);
        logger_->warn(
          "rejected recovery candidate generation={} after {} inconsistent frames",
          generation, pending_recovery_failures_);
      } else if (consensus_action == RecoveryConsensusAction::kActivate) {
        pending_recovery_ = false;
        bbs_batch_candidate_active_ = false;
      }
    }
    if (consensus_action != RecoveryConsensusAction::kActivate) {
      return;
    }

    ++map_state_epoch_;
    map_state_key_ = gtsam::Symbol('m', map_state_epoch_);
    new_values.insert(
      map_state_key_, gtsam::Pose3(observed_odom_from_map.matrix()));
    new_stamps[map_state_key_] = frame.stamp;
    map_factor_active_ = false;
    {
      std::lock_guard<std::mutex> lock(public_transform_mutex_);
      verified_recovery_transition_pending_ = true;
      verified_recovery_transition_active_ = true;
    }
    map_lock_lost_.store(false);
    recovery_search_requested_.store(false);
    recovery_loss_evidence_frames_ = 0;
    last_verified_recovery_stamp_ = frame.stamp;
    publishRecoveryRequested(false);
    logger_->warn(
      "activated independently verified recovery generation={} as map state m{}",
      generation, map_state_epoch_);
  }

  void onTightlyCoupledSmootherUpdateFinish(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback & smoother)
  {
    if (!tightly_coupled_graph_seeded_) {
      return;
    }
    const gtsam::Pose3 odom_from_map =
      smoother.calculateEstimate<gtsam::Pose3>(map_state_key_);
    const Eigen::Isometry3d target_map_from_odom(
      odom_from_map.inverse().matrix());
    if (!finiteTransform(target_map_from_odom)) {
      logger_->error("refusing non-finite tightly coupled map-to-odom estimate");
      return;
    }
    BoundedMapOdomStep public_step;
    bool verified_recovery_transition_started = false;
    Eigen::Isometry3d published_map_from_odom = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(public_transform_mutex_);
      if (map_lock_lost_.load() && public_map_odom_initialized_) {
        // Outside the prior map, preserve the last trusted map->odom anchor
        // and let continuous GLIL odometry carry the vehicle. Freezing the
        // composed sensor pose would incorrectly pin ordinary map exits.
        public_step.transform = public_map_from_odom_;
        public_map_odom_initialized_ = true;
      } else if (!public_map_odom_initialized_) {
        public_step.transform = target_map_from_odom;
        public_map_odom_initialized_ = true;
      } else {
        const double max_translation_step = verified_recovery_transition_active_ ?
          recovery_public_max_translation_step_m_ : public_max_translation_step_m_;
        const double max_rotation_step = verified_recovery_transition_active_ ?
          recovery_public_max_rotation_step_deg_ : public_max_rotation_step_deg_;
        public_step = boundMapOdomStep(
          public_map_from_odom_, target_map_from_odom,
          max_translation_step, max_rotation_step);
        if (verified_recovery_transition_active_) {
          verified_recovery_transition_started = verified_recovery_transition_pending_;
          verified_recovery_transition_pending_ = false;
          if (!public_step.translation_limited && !public_step.rotation_limited) {
            verified_recovery_transition_active_ = false;
          }
        }
      }
      public_map_from_odom_ = public_step.transform;
      published_map_from_odom = public_map_from_odom_;
    }
    glim::GlobalMappingCallbacks::on_external_map_odom_update(
      latest_tightly_coupled_stamp_, published_map_from_odom, true);
    if (verified_recovery_transition_started) {
      logger_->warn(
        "started bounded independently verified recovery transition");
    }
    if (public_step.translation_limited || public_step.rotation_limited) {
      logger_->warn(
        "bounded public map-to-odom transition (translation_limited={}, rotation_limited={})",
        public_step.translation_limited, public_step.rotation_limited);
    }
  }

  std::pair<std::size_t, std::size_t> mapOverlap(
    const gtsam_points::PointCloud & source,
    const Eigen::Isometry3d & map_from_imu) const
  {
    std::size_t inliers = 0;
    std::size_t samples = 0;
    const double max_squared_distance =
      map_factor_max_correspondence_m_ * map_factor_max_correspondence_m_;
    for (
      std::size_t index = 0; index < source.size();
      index += map_factor_overlap_stride_)
    {
      const Eigen::Vector4d point =
        map_from_imu * gtsam_points::frame::point(source, index);
      std::size_t neighbor = 0;
      double squared_distance = 0.0;
      const std::size_t found = prior_map_tree_->knn_search(
        point.data(), 1, &neighbor, &squared_distance, max_squared_distance);
      ++samples;
      inliers += found != 0 && squared_distance < max_squared_distance;
    }
    return {inliers, samples};
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
    const bool odom_bridge_recovery =
      tightly_coupled_ && odom_bridge_recovery_enabled_ &&
      recovery_search_requested_.load();
    const double crop_radius_m = odom_bridge_recovery ?
      odom_bridge_recovery_crop_radius_m_ :
      (map_frame_initialized_ ? tracking_crop_radius_m_ : bootstrap_crop_radius_m_);
    const auto cropped_map = full_global_search_ ?
      std::vector<Eigen::Vector3f>{} :
      cropMap(prior_map_, predicted_map_center, crop_radius_m);
    const auto & target = full_global_search_ ? prior_map_ : cropped_map;
    if (tightly_coupled_ && !full_global_search_) {
      map_factor_region_observable_.store(
        target.size() >= map_factor_min_tracking_crop_points_);
    }
    if (target.empty()) {
      logger_->warn("submap {} has no prior-map points in its tracking crop", submap->id);
      return;
    }

    // During normal tracking GLIM already provides the best VGICP initial
    // value. KISS is reserved for acquiring an unknown/global frame; running
    // it on every submap adds CPU and can move a good prediction into a
    // repeated-structure basin.
    bool use_coarse_global_candidate =
      !anchor_available || full_global_search_ || odom_bridge_recovery;
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
        const double max_coarse_delta_m = odom_bridge_recovery ?
          odom_bridge_recovery_max_coarse_delta_m_ :
          vgicp_max_coarse_delta_translation_m_;
        return refinement.valid &&
          refinement.inlier_fraction >= vgicp_min_inlier_fraction_ &&
          refinement.normalized_error <= vgicp_max_normalized_error_ &&
          (!use_coarse_global_candidate ||
          (refinement_delta.translation().norm() <=
          max_coarse_delta_m &&
          rotationDegrees(refinement_delta) <=
          vgicp_max_coarse_delta_rotation_deg_));
      };
    bool refinement_accepted = refinementAccepted();
    if (!refinement_accepted && !use_coarse_global_candidate) {
      logger_->info(
        "submap={} direct VGICP rejected valid={} inliers={} overlap={:.3f} "
        "error={:.3f} delta={:.3f}m/{:.2f}deg; trying crop-local KISS fallback",
        submap->id, refinement.valid, refinement.num_inliers,
        refinement.inlier_fraction, refinement.normalized_error,
        refinement_delta.translation().norm(), rotationDegrees(refinement_delta));
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
        "refined=({:.3f},{:.3f},{:.3f}) decision=refinement_rejected",
        submap->id, coarse_inliers, refinement.valid,
        refinement.num_inliers, refinement.inlier_fraction,
        refinement.normalized_error, refinement_delta.translation().norm(),
        rotationDegrees(refinement_delta), refinement.pose.translation().x(),
        refinement.pose.translation().y(), refinement.pose.translation().z());
      return;
    }
    candidate = refinement.pose;

    // Compare in the external map frame. GLIM's odometry and global graph stay
    // in their native frame; only this map-to-odom anchor is updated.
    const Eigen::Isometry3d local_correction = candidate * predicted_in_map.inverse();
    const Eigen::Isometry3d registration_anchor = candidate * odom_from_submap.inverse();
    if (odom_bridge_recovery) {
      glim::EstimationFrame::ConstPtr latest_frame;
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame = latest_odometry_frame_;
      }
      if (!latest_frame) {
        logger_->warn(
          "cannot compose odom-bridge recovery candidate for submap={}: no odometry frame",
          submap->id);
        return;
      }
      const Eigen::Isometry3d map_from_latest_lidar =
        registration_anchor * latest_frame->T_world_lidar;
      if (!finiteTransform(map_from_latest_lidar)) {
        logger_->warn(
          "rejected non-finite odom-bridge candidate for submap={}", submap->id);
        return;
      }
      queueRecoveryCandidate(map_from_latest_lidar, "odom-bridge");
      return;
    }
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
      if (
        decision.reason == "global_consensus_pending" &&
        pending_vertical_update_enabled_)
      {
        Eigen::Isometry3d vertical_anchor = Eigen::Isometry3d::Identity();
        bool vertical_anchor_available = false;
        {
          std::lock_guard<std::mutex> lock(anchor_mutex_);
          if (tracking_anchor_available_ && reference_anchor_available_) {
            tracking_anchor_ = updateMapOdomVertical(
              reference_anchor_, tracking_anchor_, map_odom_observation, vertical_gain_);
            vertical_anchor = tracking_anchor_;
            vertical_anchor_available = true;
          }
        }
        if (vertical_anchor_available) {
          glim::GlobalMappingCallbacks::on_external_map_odom_update(
            origin_odom_frame->stamp, vertical_anchor, false);
          logger_->info(
            "updated pending-consensus map-to-odom Z target={:.3f} gain={:.3f}",
            vertical_anchor.translation().z(), vertical_gain_);
        }
      }
      return;
    }

    Eigen::Isometry3d updated_anchor = Eigen::Isometry3d::Identity();
    bool reset = false;
    // Preserve the complete vertical component of a verified sparse
    // observation in tightly coupled mode.  Long sloped runs are weak along Z,
    // while full XY injection amplifies submap registration noise; XY retains
    // the configured robust gain.  The public map->odom callback additionally
    // applies its independent 0.25 m / 2 deg transition bound.
    const double anchor_horizontal_gain = translation_gain_;
    const double anchor_vertical_gain = tightly_coupled_ ? 1.0 : vertical_gain_;
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
          reference_anchor_, map_odom_observation,
          anchor_horizontal_gain, anchor_vertical_gain);
      }
      updated_anchor = tracking_anchor_;
      initial_anchor_sent_ = true;
    }
    if (tightly_coupled_) {
      std::lock_guard<std::mutex> lock(sparse_anchor_mutex_);
      pending_sparse_odom_from_map_ = updated_anchor.inverse();
      pending_sparse_anchor_ = true;
    } else {
      glim::GlobalMappingCallbacks::on_external_map_odom_update(
        origin_odom_frame->stamp, updated_anchor, reset);
    }
    logger_->info(
      "updated map-to-odom target translation=({:.3f},{:.3f},{:.3f}) "
      "gain={:.3f}/{:.3f}",
      updated_anchor.translation().x(), updated_anchor.translation().y(),
      updated_anchor.translation().z(), anchor_horizontal_gain, anchor_vertical_gain);
    map_frame_initialized_ = true;
    previous_global_anchor_available_ = false;
  }

  std::shared_ptr<spdlog::logger> logger_;
  std::vector<Eigen::Vector3f> prior_map_;
  gtsam_points::PointCloudCPU::Ptr prior_map_frame_;
  std::shared_ptr<const gtsam_points::NearestNeighborSearch> prior_map_tree_;
  std::shared_ptr<const gtsam_points::GaussianVoxelMapCPU> recovery_rerank_target_;
  kiss_matcher::KISSMatcher matcher_;
  PriorMapFactorGateParams gate_params_;
  double vgicp_voxel_resolution_m_{1.0};
  int vgicp_max_iterations_{10};
  double vgicp_min_inlier_fraction_{0.50};
  double vgicp_max_normalized_error_{5.0};
  // Recovery candidates are generated from an intentionally non-deskewed raw
  // LiDAR scan so that a discontinuous kidnap event cannot corrupt the query
  // through IMU deskewing.  Its verifier therefore needs a slightly wider
  // residual gate than normal tracking, while retaining the 50% overlap,
  // bounded correction, and three-consecutive-frame consensus gates.
  double recovery_max_normalized_error_{12.0};
  // Repetitive road geometry can produce locally consistent aliases around
  // 0.5--0.6 overlap. A global recovery must satisfy a stronger overlap gate
  // than ordinary tracking before it is allowed to move the public map frame.
  double recovery_min_inlier_fraction_{0.75};
  double recovery_rearm_cooldown_sec_{30.0};
  double recovery_rerank_voxel_resolution_m_{1.0};
  int recovery_rerank_max_iterations_{5};
  double last_verified_recovery_stamp_{-1.0};
  double vgicp_max_coarse_delta_translation_m_{2.0};
  double vgicp_max_coarse_delta_rotation_deg_{5.0};
  double tracking_crop_radius_m_{35.0};
  double bootstrap_crop_radius_m_{60.0};
  std::size_t bootstrap_submap_stride_{2};
  std::size_t tracking_submap_stride_{10};
  double translation_gain_{0.2};
  double vertical_gain_{0.2};
  bool pending_vertical_update_enabled_{false};
  bool full_global_search_{false};
  // The sparse submap KISS bridge is an optional fallback. On the Koide
  // kidnap replay its synchronous crop search held the callback for ~22 s,
  // starving the lower-cost BBS scan path and making the eventual fix stale.
  bool odom_bridge_recovery_enabled_{false};
  double odom_bridge_recovery_crop_radius_m_{70.0};
  double odom_bridge_recovery_max_coarse_delta_m_{5.0};
  bool tightly_coupled_{false};
  bool tightly_coupled_graph_seeded_{false};
  double map_factor_voxel_resolution_m_{0.5};
  int map_factor_covariance_neighbors_{10};
  double map_factor_covariance_scale_{0.25};
  double map_factor_max_correspondence_m_{2.0};
  int map_factor_coreset_size_{32};
  double map_factor_coreset_reuse_translation_m_{1.0};
  double map_factor_coreset_reuse_rotation_rad_{0.035};
  int map_factor_num_threads_{1};
  std::size_t map_factor_overlap_stride_{10};
  std::size_t map_factor_frame_stride_{2};
  double map_factor_min_overlap_fraction_{0.05};
  std::size_t map_factor_min_overlap_inliers_{32};
  std::size_t map_factor_min_tracking_crop_points_{50000};
  double map_state_prior_precision_{1.0};
  bool map_factor_active_{false};
  std::atomic_bool map_factor_region_observable_{true};
  std::atomic_bool map_lock_lost_{false};
  std::atomic_bool recovery_search_requested_{false};
  gtsam::Key map_state_key_{gtsam::Symbol('m', 0)};
  std::uint64_t map_state_epoch_{0};
  double latest_tightly_coupled_stamp_{0.0};

  std::mutex sparse_anchor_mutex_;
  bool pending_sparse_anchor_{false};
  Eigen::Isometry3d pending_sparse_odom_from_map_{Eigen::Isometry3d::Identity()};

  std::mutex recovery_mutex_;
  bool pending_recovery_{false};
  bool rerank_bbs_candidates_{false};
  bool bbs_batch_candidate_active_{false};
  bool pending_recovery_anchor_initialized_{false};
  std::size_t pending_recovery_generation_{0};
  std::size_t pending_recovery_confirmations_{0};
  std::size_t pending_recovery_failures_{0};
  Eigen::Isometry3d pending_recovery_map_from_sensor_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d pending_recovery_odom_from_map_{Eigen::Isometry3d::Identity()};
  std::size_t recovery_confirmation_frames_{3};
  std::size_t recovery_rejection_frames_{3};
  double recovery_max_correction_translation_m_{1.5};
  double recovery_max_correction_rotation_deg_{6.0};
  double recovery_max_consensus_translation_m_{1.5};
  double recovery_max_consensus_rotation_deg_{6.0};
  double recovery_loss_overlap_fraction_{0.75};
  std::size_t recovery_loss_confirmation_frames_{2};
  std::size_t recovery_loss_evidence_frames_{0};
  double public_max_translation_step_m_{0.25};
  double public_max_rotation_step_deg_{2.0};
  double recovery_public_max_translation_step_m_{1.0};
  double recovery_public_max_rotation_step_deg_{5.0};
  bool public_map_odom_initialized_{false};
  std::mutex public_transform_mutex_;
  Eigen::Isometry3d public_map_from_odom_{Eigen::Isometry3d::Identity()};
  bool verified_recovery_transition_pending_{false};
  bool verified_recovery_transition_active_{false};
  bool map_frame_initialized_{false};
  bool bootstrap_anchor_available_{false};
  Eigen::Isometry3d bootstrap_anchor_{Eigen::Isometry3d::Identity()};
  std::mutex anchor_mutex_;
  std::mutex frame_mutex_;
  glim::EstimationFrame::ConstPtr latest_odometry_frame_;
  std::deque<std::pair<double, Eigen::Isometry3d>> odometry_history_;
  std::deque<std::pair<double, glim::PreprocessedFrame::ConstPtr>>
  recovery_frame_history_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_requested_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr recovery_cloud_pub_;
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
