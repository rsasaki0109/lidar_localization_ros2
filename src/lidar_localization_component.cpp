#include <lidar_localization/lidar_localization_component.hpp>

#include "lidar_localization/alignment_attempt_policy.hpp"
#include "lidar_localization/alignment_diagnostic_ros_adapter.hpp"
#include "lidar_localization/alignment_pipeline_policy.hpp"
#include "lidar_localization/alignment_retry_policy.hpp"
#include "lidar_localization/alignment_diagnostics_policy.hpp"
#include "lidar_localization/alignment_status_policy.hpp"
#include "lidar_localization/imu_preintegration_guard_policy.hpp"
#include "lidar_localization/local_map_target_policy.hpp"
#include "lidar_localization/localization_update_policy.hpp"
#include "lidar_localization/map_initialization_policy.hpp"
#include "lidar_localization/measurement_gate_policy.hpp"
#include "lidar_localization/ndt_initializer_policy.hpp"
#include "lidar_localization/parameter_validation_policy.hpp"
#include "lidar_localization/point_cloud_conversion.hpp"
#include "lidar_localization/pose_covariance_policy.hpp"
#include "lidar_localization/pose_publish_policy.hpp"
#include "lidar_localization/pose_backend_selection_policy.hpp"
#include "lidar_localization/pose_backend_result_policy.hpp"
#include "lidar_localization/recovery_supervisor_state_policy.hpp"
#include "lidar_localization/reinitialization_latch_policy.hpp"
#include "lidar_localization/reinitialization_request_output_policy.hpp"
#include "lidar_localization/registration_backend_policy.hpp"
#include "lidar_localization/registration_observation_policy.hpp"
#include "lidar_localization/prediction_state_policy.hpp"
#include "lidar_localization/scan_admission_policy.hpp"

#include <chrono>

#include <pcl/common/common.h>

namespace
{
double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

constexpr std::size_t kRegistrationSourceCloudKeepAliveCount = 4096;
constexpr std::size_t kRegistrationTargetCloudKeepAliveCount = 4096;

void keep_cloud_alive(
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> * recent_clouds,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  std::size_t max_count)
{
  if (!recent_clouds || !cloud || max_count == 0) {
    return;
  }
  recent_clouds->push_back(cloud);
  while (recent_clouds->size() > max_count) {
    recent_clouds->pop_front();
  }
}

lidar_localization::PredictionStateSnapshot make_prediction_state_snapshot(
  bool have_last_accepted_pose,
  const Eigen::Matrix4f & last_accepted_pose_matrix,
  const Eigen::Matrix4f & predicted_pose_matrix,
  const Eigen::Matrix4f & last_relative_motion_matrix,
  std::size_t consecutive_rejected_updates,
  double last_accepted_pose_time_sec,
  double predicted_pose_time_sec)
{
  return {
    have_last_accepted_pose,
    last_accepted_pose_matrix,
    predicted_pose_matrix,
    last_relative_motion_matrix,
    consecutive_rejected_updates,
    last_accepted_pose_time_sec,
    predicted_pose_time_sec};
}

}  // namespace

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("enable_map_odom_tf", false);
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("gicp_corr_randomness", 20);
  declare_parameter("gicp_max_correspondence_distance", 2.0);
  declare_parameter("vgicp_voxel_resolution", 1.0);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("enable_scan_voxel_filter", true);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("cloud_queue_depth", 1);
  declare_parameter("min_scan_interval_sec", 0.0);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_twist_prediction", false);
  declare_parameter("twist_prediction_use_angular_velocity", true);
  declare_parameter("max_twist_prediction_dt", 0.5);
  declare_parameter("use_imu", false);
  declare_parameter("use_twist_ekf", false);
  declare_parameter("ekf_sigma_pos", 0.01);
  declare_parameter("ekf_sigma_vel", 0.5);
  declare_parameter("ekf_sigma_yaw", 0.01);
  declare_parameter("ekf_sigma_gyro_bias", 0.001);
  declare_parameter("ekf_sigma_speed_bias", 0.01);
  declare_parameter("ekf_sigma_ndt_pos", 0.1);
  declare_parameter("ekf_sigma_ndt_yaw", 0.02);
  declare_parameter("ekf_fitness_nominal", 1.0);
  declare_parameter("ekf_fitness_scale_factor", 2.0);
  declare_parameter("ekf_fitness_reject", 50.0);
  declare_parameter("use_gtsam_smoother", false);
  declare_parameter("gtsam_odom_sigma_x", 0.05);
  declare_parameter("gtsam_odom_sigma_y", 0.02);
  declare_parameter("gtsam_odom_sigma_yaw", 0.01);
  declare_parameter("gtsam_ndt_sigma_x", 0.1);
  declare_parameter("gtsam_ndt_sigma_y", 0.1);
  declare_parameter("gtsam_ndt_sigma_yaw", 0.02);
  declare_parameter("gtsam_fitness_nominal", 1.0);
  declare_parameter("gtsam_fitness_scale_factor", 2.0);
  declare_parameter("gtsam_fitness_reject", 50.0);
  declare_parameter("gtsam_huber_k", 1.345);
  // IMU preintegration parameters
  declare_parameter("use_imu_preintegration", true);
  declare_parameter("imu_preintegration_use_base_frame_transform", false);
  declare_parameter("imu_gyro_noise_density", 0.01);
  declare_parameter("imu_accel_noise_density", 0.1);
  declare_parameter("imu_gyro_random_walk", 0.0001);
  declare_parameter("imu_accel_random_walk", 0.001);
  declare_parameter("imu_ndt_sigma_z", 0.1);
  declare_parameter("imu_ndt_sigma_roll", 0.05);
  declare_parameter("imu_ndt_sigma_pitch", 0.05);
  declare_parameter("imu_bias_prior_sigma_gyro", 0.01);
  declare_parameter("imu_bias_prior_sigma_accel", 0.1);
  declare_parameter("imu_prediction_correction_guard_translation_m", 2.0);
  declare_parameter("imu_prediction_correction_guard_yaw_deg", 4.0);
  declare_parameter("enable_debug", false);
  declare_parameter("predict_pose_from_previous_delta", true);
  declare_parameter("enable_local_map_crop", false);
  declare_parameter("local_map_radius", 150.0);
  declare_parameter("local_map_min_points", 100);
  declare_parameter("reject_above_score_threshold", true);
  declare_parameter("enable_consistency_recovery_gate", false);
  declare_parameter("consistency_recovery_min_rejections", 10);
  declare_parameter("consistency_recovery_score_margin", 2.0);
  declare_parameter("consistency_recovery_max_translation_m", 0.05);
  declare_parameter("consistency_recovery_max_yaw_deg", 0.5);
  declare_parameter("enable_post_reject_strict_score_threshold", false);
  declare_parameter("post_reject_strict_min_rejections", 100);
  declare_parameter("post_reject_strict_score_threshold", 5.5);
  declare_parameter("enable_open_loop_strict_score_threshold", false);
  declare_parameter("open_loop_strict_min_accepted_gap_sec", 15.0);
  declare_parameter("open_loop_strict_min_seed_translation_m", 100.0);
  declare_parameter("open_loop_strict_score_threshold", 5.25);
  declare_parameter("enable_borderline_seed_rejection_gate", false);
  declare_parameter("borderline_seed_gate_score_threshold", 5.25);
  declare_parameter("borderline_seed_gate_min_seed_translation_m", 1.0);
  declare_parameter("enable_rejected_seed_update", false);
  declare_parameter("rejected_seed_update_min_rejections", 0);
  declare_parameter("rejected_seed_update_max_fitness", 10.0);
  declare_parameter("rejected_seed_update_max_correction_translation_m", 2.0);
  declare_parameter("rejected_seed_update_max_correction_yaw_deg", 2.0);
  declare_parameter("enable_recovery_retry_from_last_pose", false);
  declare_parameter("recovery_retry_from_last_pose_min_rejections", 1);
  declare_parameter("recovery_retry_from_last_pose_max_accepted_gap_sec", 1.0);
  declare_parameter("recovery_retry_from_last_pose_max_seed_translation_m", 1000000000.0);
  declare_parameter("enable_reinitialization_request_output", true);
  declare_parameter("enable_reinitialization_request_latch", true);
  declare_parameter("reinitialization_trigger_threshold", 0.95);
  declare_parameter("reinitialization_trigger_gap_scale_sec", 30.0);
  declare_parameter("reinitialization_trigger_seed_translation_scale_m", 100.0);
  declare_parameter("reinitialization_trigger_reject_streak_scale", 200.0);
  declare_parameter("reinitialization_trigger_fitness_explosion_threshold", 1000.0);
  declare_parameter("enable_timer_publishing", false);
  declare_parameter("pose_publish_frequency", 10.0);
  declare_parameter("viz_downsample", false);
  declare_parameter("viz_voxel_leaf_size", 0.5);
}

PCLLocalization::~PCLLocalization()
{
  releaseRuntimeResources(true);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  shutting_down_ = false;
  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  status_pub_->on_activate();
  reinitialization_request_pub_->on_activate();
  initial_map_pub_->on_activate();
  const builtin_interfaces::msg::Time activation_stamp = now();
  publishReinitializationRequest(activation_stamp, ReinitializationRequestDecision{});

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2 raw_map_cloud;
    bool map_has_intensity = true;
    const auto map_file_format = lidar_localization::detectMapFileFormat(map_path_);
    if (map_file_format == lidar_localization::MapFileFormat::kPcd) {
      RCLCPP_INFO(get_logger(), "Loading pcd map from: %s", map_path_.c_str());
      if (pcl::io::loadPCDFile(map_path_, raw_map_cloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load pcd file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else if (map_file_format == lidar_localization::MapFileFormat::kPly) {
      RCLCPP_INFO(get_logger(), "Loading ply map from: %s", map_path_.c_str());
      if (pcl::io::loadPLYFile(map_path_, raw_map_cloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load ply file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else {
      RCLCPP_ERROR(
        get_logger(), "Unsupported map file format. Please use .pcd or .ply: %s",
        map_path_.c_str());
      return CallbackReturn::FAILURE;
    }

    map_has_intensity =
      lidar_localization::convertPclCloudToXyzi(raw_map_cloud, *map_cloud_ptr);
    if (!map_has_intensity) {
      RCLCPP_WARN(
        get_logger(),
        "Map point cloud does not contain intensity. Falling back to xyz with zero intensity.");
    }

    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());
    const auto map_bounds = lidar_localization::computeMapBounds(*map_cloud_ptr);
    map_bounds_valid_ = map_bounds.valid;
    map_min_pt_ = map_bounds.min_point;
    map_max_pt_ = map_bounds.max_point;

    const auto map_msg = lidar_localization::makeInitialMapMessage(
      raw_map_cloud, global_frame_id_, viz_downsample_, viz_voxel_leaf_size_);
    initial_map_pub_->publish(map_msg);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    const auto target_setup =
      lidar_localization::planInitialMapTargetSetup(
        enable_local_map_crop_, registration_method_);
    use_local_map_crop_ = target_setup.use_local_map_crop;
    if (use_local_map_crop_) {
      // Keep the raw full map and only voxel-filter the cropped local target per scan.
      full_map_cloud_ptr_ = map_cloud_ptr;
      RCLCPP_INFO(
        get_logger(), "Local map cropping enabled. Full map: %ld pts, radius: %.0fm",
        full_map_cloud_ptr_->size(), local_map_radius_);
      // Avoid building a full-map NDT target here. It can overflow on city-scale maps.
      if (target_setup.create_ndt_initializer) {
        use_ndt_initializer_ = true;
        ndt_init_scan_count_ = 0;
        ndt_initializer_.reset(
          new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt_initializer_->setStepSize(ndt_step_size_);
        ndt_initializer_->setResolution(ndt_resolution_);
        ndt_initializer_->setTransformationEpsilon(transform_epsilon_);
        ndt_initializer_->setNumThreads(
          lidar_localization::resolveRegistrationThreadCount(
            ndt_num_threads_, omp_get_max_threads()));
        ndt_initializer_->setInputTarget(map_cloud_ptr);
        RCLCPP_INFO(get_logger(), "NDT initializer created (%d scans before GICP switch)",
                    ndt_init_scans_required_);
      } else {
        use_ndt_initializer_ = false;
        ndt_initializer_.reset();
      }
    } else if (target_setup.set_full_map_as_registration_target) {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  // Start Nav2 bond if available
#ifdef LIDAR_LOCALIZATION_HAVE_NAV2_BOND
  if (use_bond_) {
    bond_ = std::make_unique<bond::Bond>("bond", get_name(), shared_from_this());
    bond_->setHeartbeatPeriod(0.1);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
    RCLCPP_INFO(get_logger(), "Nav2 bond started");
  }
#endif

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

#ifdef LIDAR_LOCALIZATION_HAVE_NAV2_BOND
  if (bond_) {
    bond_.reset();
    RCLCPP_INFO(get_logger(), "Nav2 bond stopped");
  }
#endif

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  status_pub_->on_deactivate();
  reinitialization_request_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  releaseRuntimeResources(false);

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
  releaseRuntimeResources(false);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::releaseRuntimeResources(bool leak_target_clouds_for_shutdown)
{
  shutting_down_ = true;

#ifdef LIDAR_LOCALIZATION_HAVE_NAV2_BOND
  bond_.reset();
#endif

  pose_publish_timer_.reset();

  initial_pose_sub_.reset();
  map_sub_.reset();
  odom_sub_.reset();
  twist_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  latest_twist_msg_.reset();
  last_scan_ptr_.reset();
  path_ptr_.reset();
  corrent_pose_with_cov_stamped_ptr_.reset();

  initial_map_pub_.reset();
  path_pub_.reset();
  status_pub_.reset();
  reinitialization_request_pub_.reset();
  pose_pub_.reset();

  ndt_initializer_.reset();
  use_ndt_initializer_ = false;
  registration_ = nullptr;
  pcl_registration_.reset();
  ndt_omp_registration_.reset();
  gicp_omp_registration_.reset();
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  small_gicp_registration_.reset();
#endif

  imu_preintegration_fallback_mode_ = false;
  reinitialization_requested_ = false;
  reinitialization_request_reason_ = "not_requested";
  reinitialization_request_score_ = 0.0;
  reinitialization_request_latched_ = false;
  reinitialization_request_latch_reason_ = "not_requested";
  reinitialization_request_latch_score_ = 0.0;
  reinitialization_request_latch_stamp_sec_ = 0.0;
  recovery_supervisor_state_ = RecoverySupervisorState::kTracking;
  recovery_supervisor_action_ = "idle";
  recovery_supervisor_state_entered_stamp_sec_ = 0.0;
  recovery_supervisor_transition_count_ = 0;
  last_imu_stamp_ = 0.0;
  last_scan_stamp_for_imu_ = 0.0;
  recent_source_clouds_.clear();
  if (leak_target_clouds_for_shutdown) {
    // NDT_OMP can still hold shutdown-path ownership relationships to the last
    // target clouds. Leak them during process teardown instead of crashing.
    auto * leaked_target_clouds =
      new std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr>();
    leaked_target_clouds->swap(recent_target_clouds_);
    (void)leaked_target_clouds;
  } else {
    recent_target_clouds_.clear();
  }
  if (leak_target_clouds_for_shutdown) {
    auto * leaked_full_map_cloud =
      new pcl::PointCloud<pcl::PointXYZI>::Ptr(std::move(full_map_cloud_ptr_));
    (void)leaked_full_map_cloud;
  } else {
    full_map_cloud_ptr_.reset();
  }
  map_bounds_valid_ = false;
  consecutive_crop_failures_ = 0;
  crop_failure_guard_active_ = false;
  last_crop_out_of_bounds_log_time_ = std::chrono::steady_clock::time_point{};
  last_crop_failure_streak_log_time_ = std::chrono::steady_clock::time_point{};
  map_recieved_ = false;
  initialpose_recieved_ = false;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("enable_map_odom_tf", enable_map_odom_tf_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", measurement_gate_config_.score_threshold);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("gicp_corr_randomness", gicp_corr_randomness_);
  get_parameter("gicp_max_correspondence_distance", gicp_max_correspondence_distance_);
  get_parameter("vgicp_voxel_resolution", vgicp_voxel_resolution_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("enable_scan_voxel_filter", enable_scan_voxel_filter_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  int requested_cloud_queue_depth = cloud_queue_depth_;
  get_parameter("cloud_queue_depth", requested_cloud_queue_depth);
  const auto cloud_queue_depth =
    lidar_localization::normalizePositiveIntParameter(requested_cloud_queue_depth);
  cloud_queue_depth_ = cloud_queue_depth.value;
  if (cloud_queue_depth.was_adjusted) {
    RCLCPP_WARN(
      get_logger(), "cloud_queue_depth must be positive; using %d", cloud_queue_depth_);
  }
  get_parameter("min_scan_interval_sec", min_scan_interval_sec_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_twist_prediction", use_twist_prediction_);
  get_parameter("twist_prediction_use_angular_velocity", twist_prediction_use_angular_velocity_);
  get_parameter("max_twist_prediction_dt", max_twist_prediction_dt_);
  get_parameter("use_imu", use_imu_);
  get_parameter("use_twist_ekf", use_twist_ekf_);
  if (use_twist_ekf_) {
    TwistEkf::Params ekf_params;
    get_parameter("ekf_sigma_pos", ekf_params.sigma_pos);
    get_parameter("ekf_sigma_vel", ekf_params.sigma_vel);
    get_parameter("ekf_sigma_yaw", ekf_params.sigma_yaw);
    get_parameter("ekf_sigma_gyro_bias", ekf_params.sigma_gyro_bias);
    get_parameter("ekf_sigma_speed_bias", ekf_params.sigma_speed_bias);
    get_parameter("ekf_sigma_ndt_pos", ekf_params.sigma_ndt_pos);
    get_parameter("ekf_sigma_ndt_yaw", ekf_params.sigma_ndt_yaw);
    get_parameter("ekf_fitness_nominal", ekf_params.fitness_nominal);
    get_parameter("ekf_fitness_scale_factor", ekf_params.fitness_scale_factor);
    get_parameter("ekf_fitness_reject", ekf_params.fitness_reject);
    twist_ekf_.setParams(ekf_params);
    twist_ekf_.reset();
  }
  get_parameter("use_gtsam_smoother", use_gtsam_smoother_);
  if (use_gtsam_smoother_) {
    TwistGtsamSmoother::Params gtsam_params;
    get_parameter("gtsam_odom_sigma_x", gtsam_params.odom_sigma_x);
    get_parameter("gtsam_odom_sigma_y", gtsam_params.odom_sigma_y);
    get_parameter("gtsam_odom_sigma_yaw", gtsam_params.odom_sigma_yaw);
    get_parameter("gtsam_ndt_sigma_x", gtsam_params.ndt_sigma_x);
    get_parameter("gtsam_ndt_sigma_y", gtsam_params.ndt_sigma_y);
    get_parameter("gtsam_ndt_sigma_yaw", gtsam_params.ndt_sigma_yaw);
    get_parameter("gtsam_fitness_nominal", gtsam_params.fitness_nominal);
    get_parameter("gtsam_fitness_scale_factor", gtsam_params.fitness_scale_factor);
    get_parameter("gtsam_fitness_reject", gtsam_params.fitness_reject);
    get_parameter("gtsam_huber_k", gtsam_params.huber_k);
    gtsam_smoother_.setParams(gtsam_params);
    gtsam_smoother_.reset();
  }
  get_parameter("use_imu_preintegration", use_imu_preintegration_);
  get_parameter(
    "imu_preintegration_use_base_frame_transform",
    imu_preintegration_use_base_frame_transform_);
  if (use_imu_preintegration_) {
    ImuGtsamSmoother::Params imu_params;
    // Reuse GTSAM NDT sigmas for x, y, yaw
    get_parameter("gtsam_ndt_sigma_x", imu_params.ndt_sigma_x);
    get_parameter("gtsam_ndt_sigma_y", imu_params.ndt_sigma_y);
    get_parameter("gtsam_ndt_sigma_yaw", imu_params.ndt_sigma_yaw);
    get_parameter("imu_ndt_sigma_z", imu_params.ndt_sigma_z);
    get_parameter("imu_ndt_sigma_roll", imu_params.ndt_sigma_roll);
    get_parameter("imu_ndt_sigma_pitch", imu_params.ndt_sigma_pitch);
    get_parameter("gtsam_fitness_nominal", imu_params.fitness_nominal);
    get_parameter("gtsam_fitness_scale_factor", imu_params.fitness_scale_factor);
    get_parameter("gtsam_fitness_reject", imu_params.fitness_reject);
    get_parameter("gtsam_huber_k", imu_params.huber_k);
    get_parameter("imu_gyro_noise_density", imu_params.imu_params.gyro_noise_density);
    get_parameter("imu_accel_noise_density", imu_params.imu_params.accel_noise_density);
    get_parameter("imu_gyro_random_walk", imu_params.imu_params.gyro_random_walk);
    get_parameter("imu_accel_random_walk", imu_params.imu_params.accel_random_walk);
    get_parameter("imu_bias_prior_sigma_gyro", imu_params.bias_prior_sigma_gyro);
    get_parameter("imu_bias_prior_sigma_accel", imu_params.bias_prior_sigma_accel);
    get_parameter(
      "imu_prediction_correction_guard_translation_m",
      imu_prediction_correction_guard_translation_m_);
    get_parameter(
      "imu_prediction_correction_guard_yaw_deg",
      imu_prediction_correction_guard_yaw_deg_);
    imu_smoother_.params_ = imu_params;
    RCLCPP_INFO(get_logger(), "IMU preintegration smoother enabled");
  }
  get_parameter("enable_debug", enable_debug_);
  get_parameter("viz_downsample", viz_downsample_);
  get_parameter("viz_voxel_leaf_size", viz_voxel_leaf_size_);
  get_parameter("predict_pose_from_previous_delta", predict_pose_from_previous_delta_);
  get_parameter("enable_local_map_crop", enable_local_map_crop_);
  get_parameter("local_map_radius", local_map_radius_);
  int requested_local_map_min_points = static_cast<int>(local_map_min_points_);
  get_parameter("local_map_min_points", requested_local_map_min_points);
  const auto local_map_min_points =
    lidar_localization::normalizeLocalMapMinPoints(requested_local_map_min_points);
  local_map_min_points_ = local_map_min_points.value;
  if (local_map_min_points.was_adjusted) {
    RCLCPP_WARN(
      get_logger(), "local_map_min_points must be positive; using %zu",
      local_map_min_points_);
  }
  get_parameter("reject_above_score_threshold", measurement_gate_config_.reject_above_score_threshold);
  get_parameter("enable_consistency_recovery_gate", measurement_gate_config_.enable_consistency_recovery_gate);
  get_parameter("consistency_recovery_min_rejections", measurement_gate_config_.consistency_recovery_min_rejections);
  get_parameter("consistency_recovery_score_margin", measurement_gate_config_.consistency_recovery_score_margin);
  get_parameter(
    "consistency_recovery_max_translation_m",
    measurement_gate_config_.consistency_recovery_max_translation_m);
  get_parameter("consistency_recovery_max_yaw_deg", measurement_gate_config_.consistency_recovery_max_yaw_deg);
  get_parameter(
    "enable_post_reject_strict_score_threshold",
    measurement_gate_config_.enable_post_reject_strict_score_threshold);
  get_parameter("post_reject_strict_min_rejections", measurement_gate_config_.post_reject_strict_min_rejections);
  get_parameter("post_reject_strict_score_threshold", measurement_gate_config_.post_reject_strict_score_threshold);
  get_parameter(
    "enable_open_loop_strict_score_threshold",
    measurement_gate_config_.enable_open_loop_strict_score_threshold);
  get_parameter(
    "open_loop_strict_min_accepted_gap_sec",
    measurement_gate_config_.open_loop_strict_min_accepted_gap_sec);
  get_parameter(
    "open_loop_strict_min_seed_translation_m",
    measurement_gate_config_.open_loop_strict_min_seed_translation_m);
  get_parameter("open_loop_strict_score_threshold", measurement_gate_config_.open_loop_strict_score_threshold);
  get_parameter(
    "enable_borderline_seed_rejection_gate",
    measurement_gate_config_.enable_borderline_seed_rejection_gate);
  get_parameter("borderline_seed_gate_score_threshold", measurement_gate_config_.borderline_seed_gate_score_threshold);
  get_parameter(
    "borderline_seed_gate_min_seed_translation_m",
    measurement_gate_config_.borderline_seed_gate_min_seed_translation_m);
  get_parameter("enable_rejected_seed_update", measurement_gate_config_.enable_rejected_seed_update);
  get_parameter("rejected_seed_update_min_rejections", measurement_gate_config_.rejected_seed_update_min_rejections);
  get_parameter("rejected_seed_update_max_fitness", measurement_gate_config_.rejected_seed_update_max_fitness);
  get_parameter(
    "rejected_seed_update_max_correction_translation_m",
    measurement_gate_config_.rejected_seed_update_max_correction_translation_m);
  get_parameter(
    "rejected_seed_update_max_correction_yaw_deg",
    measurement_gate_config_.rejected_seed_update_max_correction_yaw_deg);
  get_parameter(
    "enable_recovery_retry_from_last_pose",
    recovery_retry_from_last_pose_config_.enable);
  get_parameter(
    "recovery_retry_from_last_pose_min_rejections",
    recovery_retry_from_last_pose_config_.min_rejections);
  get_parameter(
    "recovery_retry_from_last_pose_max_accepted_gap_sec",
    recovery_retry_from_last_pose_config_.max_accepted_gap_sec);
  get_parameter(
    "recovery_retry_from_last_pose_max_seed_translation_m",
    recovery_retry_from_last_pose_config_.max_seed_translation_m);
  get_parameter(
    "enable_reinitialization_request_output",
    enable_reinitialization_request_output_);
  get_parameter(
    "enable_reinitialization_request_latch",
    enable_reinitialization_request_latch_);
  get_parameter(
    "reinitialization_trigger_threshold",
    reinitialization_trigger_config_.threshold);
  get_parameter(
    "reinitialization_trigger_gap_scale_sec",
    reinitialization_trigger_config_.gap_scale_sec);
  get_parameter(
    "reinitialization_trigger_seed_translation_scale_m",
    reinitialization_trigger_config_.seed_translation_scale_m);
  get_parameter(
    "reinitialization_trigger_reject_streak_scale",
    reinitialization_trigger_config_.reject_streak_scale);
  get_parameter(
    "reinitialization_trigger_fitness_explosion_threshold",
    reinitialization_trigger_config_.fitness_explosion_threshold);
  get_parameter("enable_timer_publishing", enable_timer_publishing_);
  double requested_pose_publish_frequency = pose_publish_frequency_;
  get_parameter("pose_publish_frequency", requested_pose_publish_frequency);
  const auto pose_publish_frequency =
    lidar_localization::normalizePosePublishFrequencyHz(requested_pose_publish_frequency);
  pose_publish_frequency_ = pose_publish_frequency.value;
  if (pose_publish_frequency.was_adjusted) {
    RCLCPP_WARN(
      get_logger(), "pose_publish_frequency must be finite and positive; using %lf",
      pose_publish_frequency_);
  }

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"enable_map_odom_tf: %d", enable_map_odom_tf_);
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"gicp_corr_randomness: %d", gicp_corr_randomness_);
  RCLCPP_INFO(get_logger(),"gicp_max_correspondence_distance: %lf", gicp_max_correspondence_distance_);
  RCLCPP_INFO(get_logger(),"vgicp_voxel_resolution: %lf", vgicp_voxel_resolution_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"enable_scan_voxel_filter: %d", enable_scan_voxel_filter_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"cloud_queue_depth: %d", cloud_queue_depth_);
  RCLCPP_INFO(get_logger(),"min_scan_interval_sec: %lf", min_scan_interval_sec_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_twist_prediction: %d", use_twist_prediction_);
  RCLCPP_INFO(
    get_logger(), "twist_prediction_use_angular_velocity: %d",
    twist_prediction_use_angular_velocity_);
  RCLCPP_INFO(get_logger(),"max_twist_prediction_dt: %lf", max_twist_prediction_dt_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"use_imu_preintegration: %d", use_imu_preintegration_);
  RCLCPP_INFO(
    get_logger(), "imu_preintegration_use_base_frame_transform: %d",
    imu_preintegration_use_base_frame_transform_);
  RCLCPP_INFO(get_logger(),"use_twist_ekf: %d", use_twist_ekf_);
  RCLCPP_INFO(get_logger(),"use_gtsam_smoother: %d", use_gtsam_smoother_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(
    get_logger(), "predict_pose_from_previous_delta: %d", predict_pose_from_previous_delta_);
  RCLCPP_INFO(get_logger(), "enable_local_map_crop: %d", enable_local_map_crop_);
  RCLCPP_INFO(get_logger(), "local_map_radius: %lf", local_map_radius_);
  RCLCPP_INFO(
    get_logger(), "local_map_min_points: %zu", local_map_min_points_);
  RCLCPP_INFO(
    get_logger(), "reject_above_score_threshold: %d", measurement_gate_config_.reject_above_score_threshold);
  RCLCPP_INFO(
    get_logger(), "enable_consistency_recovery_gate: %d",
    measurement_gate_config_.enable_consistency_recovery_gate);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_min_rejections: %d",
    measurement_gate_config_.consistency_recovery_min_rejections);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_score_margin: %lf",
    measurement_gate_config_.consistency_recovery_score_margin);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_max_translation_m: %lf",
    measurement_gate_config_.consistency_recovery_max_translation_m);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_max_yaw_deg: %lf",
    measurement_gate_config_.consistency_recovery_max_yaw_deg);
  RCLCPP_INFO(
    get_logger(), "enable_post_reject_strict_score_threshold: %d",
    measurement_gate_config_.enable_post_reject_strict_score_threshold);
  RCLCPP_INFO(
    get_logger(), "post_reject_strict_min_rejections: %d",
    measurement_gate_config_.post_reject_strict_min_rejections);
  RCLCPP_INFO(
    get_logger(), "post_reject_strict_score_threshold: %lf",
    measurement_gate_config_.post_reject_strict_score_threshold);
  RCLCPP_INFO(
    get_logger(), "enable_open_loop_strict_score_threshold: %d",
    measurement_gate_config_.enable_open_loop_strict_score_threshold);
  RCLCPP_INFO(
    get_logger(), "open_loop_strict_min_accepted_gap_sec: %lf",
    measurement_gate_config_.open_loop_strict_min_accepted_gap_sec);
  RCLCPP_INFO(
    get_logger(), "open_loop_strict_min_seed_translation_m: %lf",
    measurement_gate_config_.open_loop_strict_min_seed_translation_m);
  RCLCPP_INFO(
    get_logger(), "open_loop_strict_score_threshold: %lf",
    measurement_gate_config_.open_loop_strict_score_threshold);
  RCLCPP_INFO(
    get_logger(), "enable_borderline_seed_rejection_gate: %d",
    measurement_gate_config_.enable_borderline_seed_rejection_gate);
  RCLCPP_INFO(
    get_logger(), "borderline_seed_gate_score_threshold: %lf",
    measurement_gate_config_.borderline_seed_gate_score_threshold);
  RCLCPP_INFO(
    get_logger(), "borderline_seed_gate_min_seed_translation_m: %lf",
    measurement_gate_config_.borderline_seed_gate_min_seed_translation_m);
  RCLCPP_INFO(get_logger(), "enable_rejected_seed_update: %d", measurement_gate_config_.enable_rejected_seed_update);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_min_rejections: %d",
    measurement_gate_config_.rejected_seed_update_min_rejections);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_max_fitness: %lf",
    measurement_gate_config_.rejected_seed_update_max_fitness);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_max_correction_translation_m: %lf",
    measurement_gate_config_.rejected_seed_update_max_correction_translation_m);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_max_correction_yaw_deg: %lf",
    measurement_gate_config_.rejected_seed_update_max_correction_yaw_deg);
  RCLCPP_INFO(
    get_logger(), "enable_recovery_retry_from_last_pose: %d",
    recovery_retry_from_last_pose_config_.enable);
  RCLCPP_INFO(
    get_logger(), "recovery_retry_from_last_pose_min_rejections: %d",
    recovery_retry_from_last_pose_config_.min_rejections);
  RCLCPP_INFO(
    get_logger(), "recovery_retry_from_last_pose_max_accepted_gap_sec: %lf",
    recovery_retry_from_last_pose_config_.max_accepted_gap_sec);
  RCLCPP_INFO(
    get_logger(), "recovery_retry_from_last_pose_max_seed_translation_m: %lf",
    recovery_retry_from_last_pose_config_.max_seed_translation_m);
  RCLCPP_INFO(
    get_logger(), "enable_reinitialization_request_output: %d",
    enable_reinitialization_request_output_);
  RCLCPP_INFO(
    get_logger(), "enable_reinitialization_request_latch: %d",
    enable_reinitialization_request_latch_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_threshold: %lf",
    reinitialization_trigger_config_.threshold);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_gap_scale_sec: %lf",
    reinitialization_trigger_config_.gap_scale_sec);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_seed_translation_scale_m: %lf",
    reinitialization_trigger_config_.seed_translation_scale_m);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_reject_streak_scale: %lf",
    reinitialization_trigger_config_.reject_streak_scale);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_fitness_explosion_threshold: %lf",
    reinitialization_trigger_config_.fitness_explosion_threshold);
  RCLCPP_INFO(
    get_logger(), "imu_prediction_correction_guard_translation_m: %lf",
    imu_prediction_correction_guard_translation_m_);
  RCLCPP_INFO(
    get_logger(), "imu_prediction_correction_guard_yaw_deg: %lf",
    imu_prediction_correction_guard_yaw_deg_);
  RCLCPP_INFO(get_logger(),"enable_timer_publishing: %d", enable_timer_publishing_);
  RCLCPP_INFO(get_logger(),"pose_publish_frequency: %lf", pose_publish_frequency_);
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "alignment_status",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  reinitialization_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "reinitialization_requested",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::twistReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::SensorDataQoS().keep_last(cloud_queue_depth_),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  if (enable_timer_publishing_) {
    pose_publish_timer_ = create_wall_timer(
      lidar_localization::posePublishPeriodFromFrequencyHz(pose_publish_frequency_),
      std::bind(&PCLLocalization::timerPublishPose, this));
  }

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  registration_ = nullptr;
  pcl_registration_.reset();
  ndt_omp_registration_.reset();
  gicp_omp_registration_.reset();
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  small_gicp_registration_.reset();
#endif
  recent_source_clouds_.clear();
  recent_target_clouds_.clear();

  const auto registration_backend =
    lidar_localization::parseRegistrationBackend(registration_method_);
  if (registration_backend == lidar_localization::RegistrationBackend::kPclGicp) {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    gicp->setCorrespondenceRandomness(gicp_corr_randomness_);
    gicp->setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    pcl_registration_ = gicp;
    registration_ = pcl_registration_.get();
  }
  else if (registration_backend == lidar_localization::RegistrationBackend::kPclNdt) {
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    pcl_registration_ = ndt;
    registration_ = pcl_registration_.get();
  }
  else if (registration_backend == lidar_localization::RegistrationBackend::kNdtOmp) {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    ndt_omp->setNumThreads(
      lidar_localization::resolveRegistrationThreadCount(
        ndt_num_threads_, omp_get_max_threads()));
    ndt_omp_registration_ = ndt_omp;
    registration_ = ndt_omp_registration_.get();
  }
  else if (registration_backend == lidar_localization::RegistrationBackend::kGicpOmp) {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    gicp_omp->setCorrespondenceRandomness(gicp_corr_randomness_);
    gicp_omp->setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    gicp_omp_registration_ = gicp_omp;
    registration_ = gicp_omp_registration_.get();
  }
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  else if (lidar_localization::isSmallGicpBackend(registration_backend)) {
    small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>::Ptr reg(
      new small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>());
    reg->setTransformationEpsilon(transform_epsilon_);
    reg->setCorrespondenceRandomness(gicp_corr_randomness_);
    reg->setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    reg->setVoxelResolution(vgicp_voxel_resolution_);
    reg->setRegistrationType(
      lidar_localization::smallGicpRegistrationType(registration_backend));
    reg->setNumThreads(
      lidar_localization::resolveRegistrationThreadCount(
        ndt_num_threads_, omp_get_max_threads()));
    small_gicp_registration_ = reg;
    registration_ = small_gicp_registration_.get();
  }
#else
  else if (lidar_localization::isSmallGicpBackend(registration_backend)) {
    RCLCPP_ERROR(
      get_logger(),
      "small_gicp backend requested but support is not available. Install small_gicp and rebuild.");
    exit(EXIT_FAILURE);
  }
#endif
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  if (registration_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "Registration backend setup failed.");
    exit(EXIT_FAILURE);
  }
  registration_->setMaximumIterations(ndt_max_iterations_);


  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (shutting_down_) {return;}
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  imu_preintegration_fallback_mode_ = false;
  reinitialization_request_latched_ = false;
  reinitialization_request_latch_reason_ = "not_requested";
  reinitialization_request_latch_score_ = 0.0;
  reinitialization_request_latch_stamp_sec_ = 0.0;
  recovery_supervisor_state_ = RecoverySupervisorState::kTracking;
  recovery_supervisor_action_ = "initial_pose_reset";
  recovery_supervisor_state_entered_stamp_sec_ = stamp_to_sec(msg->header.stamp);
  consecutive_crop_failures_ = 0;
  crop_failure_guard_active_ = false;
  last_crop_out_of_bounds_log_time_ = std::chrono::steady_clock::time_point{};
  last_crop_failure_streak_log_time_ = std::chrono::steady_clock::time_point{};
  resetPredictionState(currentPoseMatrix(), stamp_to_sec(msg->header.stamp));
  publishReinitializationRequest(msg->header.stamp, ReinitializationRequestDecision{});

  if (use_twist_ekf_) {
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    last_ndt_roll_ = static_cast<float>(roll);
    last_ndt_pitch_ = static_cast<float>(pitch);
    twist_ekf_.initialize(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      yaw,
      stamp_to_sec(msg->header.stamp));
    RCLCPP_INFO(get_logger(), "TwistEKF initialized from initial pose");
  }

  if (use_gtsam_smoother_) {
    tf2::Quaternion q_gtsam;
    tf2::fromMsg(msg->pose.pose.orientation, q_gtsam);
    double roll_g, pitch_g, yaw_g;
    tf2::Matrix3x3(q_gtsam).getRPY(roll_g, pitch_g, yaw_g);
    last_ndt_roll_ = static_cast<float>(roll_g);
    last_ndt_pitch_ = static_cast<float>(pitch_g);
    gtsam_smoother_.initialize(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      yaw_g,
      stamp_to_sec(msg->header.stamp));
    RCLCPP_INFO(get_logger(), "GTSAM smoother initialized from initial pose");
  }

  if (use_imu_preintegration_ && last_imu_stamp_ > 0.0) {
    tf2::Quaternion q_imu;
    tf2::fromMsg(msg->pose.pose.orientation, q_imu);
    double roll_i, pitch_i, yaw_i;
    tf2::Matrix3x3(q_imu).getRPY(roll_i, pitch_i, yaw_i);
    imu_smoother_.initialize(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      roll_i, pitch_i, yaw_i,
      0.0, 0.0, 0.0,  // initial velocity = 0
      stamp_to_sec(msg->header.stamp));
    last_scan_stamp_for_imu_ = stamp_to_sec(msg->header.stamp);
    RCLCPP_INFO(get_logger(), "IMU preintegration smoother initialized from initial pose");
  }
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  if(last_scan_ptr_) {
    cloudReceived(last_scan_ptr_);
  }

  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (shutting_down_) {return;}
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  const auto map_target_choice = lidar_localization::chooseMapSubscriptionTargetCloud(
    lidar_localization::usesFilteredTarget(registration_method_));
  if (map_target_choice == lidar_localization::LocalMapTargetCloud::kFilteredLocalMap) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
    keep_cloud_alive(
      &recent_target_clouds_, filtered_cloud_ptr, kRegistrationTargetCloudKeepAliveCount);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
    keep_cloud_alive(
      &recent_target_clouds_, map_cloud_ptr, kRegistrationTargetCloudKeepAliveCount);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (shutting_down_) {return;}
  if (!use_odom_) {return;}
  RCLCPP_DEBUG(get_logger(), "odomReceived");

  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation, previous_quat_tf);

  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;

  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += delta_position.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += delta_position.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z += delta_position.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
}

void PCLLocalization::twistReceived(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  if (shutting_down_) {return;}
  latest_twist_msg_ = msg;

  double stamp_sec = stamp_to_sec(msg->header.stamp);
  double vx = msg->twist.twist.linear.x;
  double wz = msg->twist.twist.angular.z;

  if (use_twist_ekf_ && twist_ekf_.isInitialized()) {
    twist_ekf_.predict(vx, wz, stamp_sec);
  }

  if (use_gtsam_smoother_ && gtsam_smoother_.isInitialized()) {
    gtsam_smoother_.predict(vx, wz, stamp_sec);
  }
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (shutting_down_) {return;}
  // IMU preintegration buffering (always runs if enabled, independent of use_imu_)
  Eigen::Vector3d preintegration_gyro(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3d preintegration_accel(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  bool preintegration_sample_ready = true;

  if (
    use_imu_preintegration_ &&
    !imu_preintegration_fallback_mode_ &&
    imu_preintegration_use_base_frame_transform_ &&
    msg->header.frame_id != base_frame_id_) {
    try {
      const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
        base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

      geometry_msgs::msg::Vector3Stamped angular_velocity;
      geometry_msgs::msg::Vector3Stamped linear_acceleration;
      geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
      geometry_msgs::msg::Vector3Stamped transformed_linear_acceleration;
      angular_velocity.header = msg->header;
      angular_velocity.vector = msg->angular_velocity;
      linear_acceleration.header = msg->header;
      linear_acceleration.vector = msg->linear_acceleration;

      tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
      tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

      preintegration_gyro = Eigen::Vector3d(
        transformed_angular_velocity.vector.x,
        transformed_angular_velocity.vector.y,
        transformed_angular_velocity.vector.z);
      preintegration_accel = Eigen::Vector3d(
        transformed_linear_acceleration.vector.x,
        transformed_linear_acceleration.vector.y,
        transformed_linear_acceleration.vector.z);
    } catch (tf2::TransformException & ex) {
      preintegration_sample_ready = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to transform IMU preintegration sample from %s to %s: %s",
        msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
    }
  }

  if (
    use_imu_preintegration_ &&
    !imu_preintegration_fallback_mode_ &&
    preintegration_sample_ready) {
    double stamp_sec = stamp_to_sec(msg->header.stamp);

    // Feed to smoother for dead-reckoning
    if (imu_smoother_.isInitialized() && last_imu_stamp_ > 0.0) {
      double dt = stamp_sec - last_imu_stamp_;
      if (dt > 0.0 && dt < 0.5) {
        imu_smoother_.integrateImu(preintegration_gyro, preintegration_accel, dt);
      }
    }
    last_imu_stamp_ = stamp_sec;
  }

  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    tf2::Quaternion imu_orientation;
    tf2::Quaternion base_from_imu;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);
    tf2::fromMsg(msg->orientation, imu_orientation);
    tf2::fromMsg(transform.transform.rotation, base_from_imu);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = tf2::toMsg(base_from_imu * imu_orientation);

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{
    static_cast<float>(tf_converted_imu.angular_velocity.x),
    static_cast<float>(tf_converted_imu.angular_velocity.y),
    static_cast<float>(tf_converted_imu.angular_velocity.z)};
  Eigen::Vector3f acc{
    static_cast<float>(tf_converted_imu.linear_acceleration.x),
    static_cast<float>(tf_converted_imu.linear_acceleration.y),
    static_cast<float>(tf_converted_imu.linear_acceleration.z)};
  Eigen::Quaternionf quat{
    static_cast<float>(tf_converted_imu.orientation.w),
    static_cast<float>(tf_converted_imu.orientation.x),
    static_cast<float>(tf_converted_imu.orientation.y),
    static_cast<float>(tf_converted_imu.orientation.z)};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  double scan_stamp_sec = 0.0;
  if (!admitScanMessage(msg, &scan_stamp_sec)) {
    return;
  }
  const PreparedScanCloud prepared_scan = prepareScanForRegistration(msg, scan_stamp_sec);
  if (!lidar_localization::isPreparedScanReady(prepared_scan.status)) {
    handleScanPreparationFailure(msg->header.stamp, prepared_scan, scan_stamp_sec);
    return;
  }
  const std::size_t filtered_point_count = prepared_scan.filtered_point_count;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr = prepared_scan.cloud;
  setRegistrationSourceCloud(tmp_ptr);

  const SelectedRegistrationSeed selected_seed = selectRegistrationSeed(scan_stamp_sec);
  const bool imu_prediction_ready = selected_seed.imu_prediction_ready;
  Eigen::Matrix4f init_guess =
    refineSeedWithNdtInitializer(tmp_ptr, selected_seed.init_guess);
  const auto pipeline_result = runAlignmentPipelineForScan(init_guess, scan_stamp_sec);

  logAlignmentPipelineRecovery(pipeline_result);
  if (handleTerminalAlignmentPipelineResult(
      msg->header.stamp,
      pipeline_result,
      filtered_point_count,
      scan_stamp_sec,
      imu_prediction_ready))
  {
    return;
  }
  if (!applyAcceptedAlignmentPipelineResult(
      msg->header.stamp,
      pipeline_result,
      filtered_point_count,
      scan_stamp_sec,
      imu_prediction_ready))
  {
    return;
  }

  printAlignmentDebugInfo(init_guess, pipeline_result.selected_attempt, filtered_point_count);
}

bool PCLLocalization::admitScanMessage(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  double * scan_stamp_sec)
{
  const bool timing_relevant =
    !shutting_down_ && static_cast<bool>(msg) && map_recieved_ && initialpose_recieved_;
  rclcpp::Time now;
  bool has_last_process_time = false;
  double elapsed_since_last_process_sec = 0.0;
  if (timing_relevant) {
    now = this->now();
    has_last_process_time = last_cloud_process_time_.nanoseconds() > 0;
    if (has_last_process_time) {
      elapsed_since_last_process_sec = (now - last_cloud_process_time_).seconds();
    }
  }

  const auto admission = lidar_localization::decideScanAdmission(
    lidar_localization::ScanAdmissionInput{
      shutting_down_,
      static_cast<bool>(msg),
      map_recieved_,
      initialpose_recieved_,
      min_scan_interval_sec_,
      has_last_process_time,
      elapsed_since_last_process_sec,
      consecutive_crop_failures_,
      crop_failure_guard_active_,
      have_last_accepted_pose_});

  if (admission.should_warn_null_scan) {
    RCLCPP_WARN(get_logger(), "Received null point cloud message");
  }
  if (admission.should_store_last_scan) {
    last_scan_ptr_ = msg;
  }
  if (admission.should_update_last_process_time) {
    last_cloud_process_time_ = now;
  }

  if (admission.status == lidar_localization::ScanAdmissionStatus::kCropFailureGuard) {
    const double current_scan_stamp_sec = stamp_to_sec(msg->header.stamp);
    if (scan_stamp_sec) {
      *scan_stamp_sec = current_scan_stamp_sec;
    }
    if (admission.should_activate_crop_failure_guard) {
      crop_failure_guard_active_ = true;
      if (admission.should_reset_prediction_to_last_accepted_pose) {
        predicted_pose_matrix_ = last_accepted_pose_matrix_;
        predicted_pose_time_sec_ = current_scan_stamp_sec;
      }
    }
    if (admission.should_log_crop_failure_guard_activation) {
      RCLCPP_ERROR(
        get_logger(),
        "Activating crop failure guard after %d consecutive crop failures; "
        "dropping subsequent scans until a new initial pose arrives.",
        consecutive_crop_failures_);
    }
  }

  if (!admission.accepted) {
    return false;
  }

  if (scan_stamp_sec) {
    *scan_stamp_sec = stamp_to_sec(msg->header.stamp);
  }
  RCLCPP_DEBUG(get_logger(), "cloudReceived");
  return true;
}

PCLLocalization::PreparedScanCloud PCLLocalization::prepareScanForRegistration(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  double scan_stamp_sec)
{
  PreparedScanCloud prepared_scan;

  const bool cloud_has_intensity = lidar_localization::hasPointField(msg->fields, "intensity");
  if (!cloud_has_intensity) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Input cloud does not contain intensity. Falling back to xyz with zero intensity.");
  }
  const bool can_direct_range_filter =
    lidar_localization::shouldUseDirectRangeFilter(
    lidar_localization::ScanPreprocessingPathInput{
      enable_scan_voxel_filter_,
      use_imu_,
      msg->header.frame_id,
      base_frame_id_});

  if (can_direct_range_filter) {
    const auto * x_field = lidar_localization::findPointField(msg->fields, "x");
    const auto * y_field = lidar_localization::findPointField(msg->fields, "y");
    const auto * z_field = lidar_localization::findPointField(msg->fields, "z");
    const auto * intensity_field = lidar_localization::findPointField(msg->fields, "intensity");
    if (!lidar_localization::hasRequiredXyzFields(
        lidar_localization::ScanXyzFieldAvailability{
          x_field != nullptr, y_field != nullptr, z_field != nullptr}))
    {
      prepared_scan.status =
        lidar_localization::classifyPreparedScan(false, true, false);
      return prepared_scan;
    }

    pcl::PointCloud<pcl::PointXYZI> tmp;
    const std::size_t point_count =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    tmp.reserve(point_count);
    for (std::size_t point_idx = 0; point_idx < point_count; ++point_idx) {
      const uint8_t * point_data = msg->data.data() + point_idx * msg->point_step;
      pcl::PointXYZI point;
      float intensity = 0.0f;
      if (!lidar_localization::readPointFieldAsFloat(point_data, *x_field, &point.x) ||
        !lidar_localization::readPointFieldAsFloat(point_data, *y_field, &point.y) ||
        !lidar_localization::readPointFieldAsFloat(point_data, *z_field, &point.z))
      {
        continue;
      }
      if (intensity_field) {
        lidar_localization::readPointFieldAsFloat(point_data, *intensity_field, &intensity);
      }
      point.intensity = intensity;
      ++prepared_scan.filtered_point_count;
      if (lidar_localization::isPointInScanRange(
          point.x, point.y, point.z, scan_min_range_, scan_max_range_))
      {
        tmp.push_back(point);
      }
    }
    prepared_scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    lidar_localization::convertSensorCloudToXyzi(*msg, *cloud_ptr);

    // If your cloud is not robot-centric, convert to base_frame.
    if (msg->header.frame_id != base_frame_id_) {
      RCLCPP_DEBUG(
        this->get_logger(), "Transforming point cloud from %s to %s",
        msg->header.frame_id.c_str(), base_frame_id_.c_str());
      geometry_msgs::msg::TransformStamped base_to_lidar_stamped;
      try {
        base_to_lidar_stamped = tfbuffer_.lookupTransform(
          base_frame_id_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
      } catch (const tf2::TransformException & ex) {
        prepared_scan.status =
          lidar_localization::classifyPreparedScan(true, false, false);
        RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
        return prepared_scan;
      }

      Eigen::Matrix4f initial_transformation =
        tf2::transformToEigen(base_to_lidar_stamped.transform).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*cloud_ptr, *transformed_cloud, initial_transformation);
      cloud_ptr = transformed_cloud;
    }

    if (use_imu_) {
      lidar_undistortion_.adjustDistortion(cloud_ptr, scan_stamp_sec);
    }

    pcl::PointCloud<pcl::PointXYZI> tmp;
    tmp.reserve(cloud_ptr->size());
    for (const auto & point : cloud_ptr->points) {
      if (lidar_localization::isPointInScanRange(
          point.x, point.y, point.z, scan_min_range_, scan_max_range_))
      {
        tmp.push_back(point);
      }
    }
    prepared_scan.filtered_point_count = tmp.size();
    prepared_scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(tmp));

    if (enable_scan_voxel_filter_ && !prepared_scan.cloud->empty()) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
      pcl::VoxelGrid<pcl::PointXYZI> scan_voxel_grid_filter;
      scan_voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      scan_voxel_grid_filter.setInputCloud(prepared_scan.cloud);
      scan_voxel_grid_filter.filter(*filtered_cloud_ptr);
      prepared_scan.filtered_point_count = filtered_cloud_ptr->size();
      prepared_scan.cloud = filtered_cloud_ptr;
    }
  }

  prepared_scan.status = lidar_localization::classifyPreparedScan(
    true,
    true,
    !prepared_scan.cloud || prepared_scan.cloud->empty());
  return prepared_scan;
}

void PCLLocalization::handleScanPreparationFailure(
  const builtin_interfaces::msg::Time & stamp,
  const PreparedScanCloud & prepared_scan,
  double scan_stamp_sec)
{
  if (prepared_scan.status == lidar_localization::ScanPreparationStatus::kTransformUnavailable) {
    return;
  }

  publishAlignmentStatus(
    stamp,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    lidar_localization::scanPreparationStatusMessage(prepared_scan.status),
    false,
    std::numeric_limits<double>::quiet_NaN(),
    0.0,
    prepared_scan.filtered_point_count,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    false);

  if (prepared_scan.status == lidar_localization::ScanPreparationStatus::kMissingXyzField) {
    RCLCPP_ERROR(get_logger(), "Input scan is missing x/y/z fields.");
  } else if (prepared_scan.status == lidar_localization::ScanPreparationStatus::kFilteredScanEmpty) {
    RCLCPP_WARN(get_logger(), "Filtered scan is empty after range filtering.");
  }

  if (lidar_localization::shouldAdvancePredictionAfterScanPreparationFailure(
      prepared_scan.status))
  {
    advancePredictionWithoutMeasurement(scan_stamp_sec);
  }
}

PCLLocalization::SelectedRegistrationSeed PCLLocalization::selectRegistrationSeed(
  double scan_stamp_sec)
{
  SelectedRegistrationSeed selected_seed;
  selected_seed.init_guess = currentPoseMatrix();

  const bool imu_has_new_samples =
    lidar_localization::hasNewImuSamples(last_imu_stamp_, last_scan_stamp_for_imu_);
  const bool imu_candidate_ready =
    use_imu_preintegration_ &&
    !imu_preintegration_fallback_mode_ &&
    imu_smoother_.isInitialized() &&
    imu_has_new_samples;
  Eigen::Matrix4f imu_init_guess = Eigen::Matrix4f::Identity();
  bool imu_prediction_finite = false;
  if (imu_candidate_ready) {
    imu_init_guess = imu_smoother_.predictedPoseMatrix();
    imu_prediction_finite = imu_init_guess.allFinite();
  }

  const lidar_localization::RegistrationSeedPolicyDecision seed_decision =
    lidar_localization::chooseRegistrationSeed(
    lidar_localization::RegistrationSeedPolicyInput{
      use_imu_preintegration_,
      imu_preintegration_fallback_mode_,
      imu_smoother_.isInitialized(),
      imu_has_new_samples,
      imu_prediction_finite,
      use_gtsam_smoother_,
      gtsam_smoother_.isInitialized(),
      use_twist_ekf_,
      twist_ekf_.isInitialized(),
      use_twist_prediction_,
      have_last_accepted_pose_,
      static_cast<bool>(latest_twist_msg_),
      predict_pose_from_previous_delta_});
  selected_seed.imu_prediction_ready = seed_decision.imu_prediction_ready;

  switch (seed_decision.source) {
    case lidar_localization::RegistrationSeedSource::kImuPreintegration:
      selected_seed.init_guess = imu_init_guess;
      break;
    case lidar_localization::RegistrationSeedSource::kGtsamSmoother:
      selected_seed.init_guess =
        gtsam_smoother_.predictedPoseMatrix(last_ndt_roll_, last_ndt_pitch_);
      break;
    case lidar_localization::RegistrationSeedSource::kTwistEkf:
      selected_seed.init_guess = twist_ekf_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
      break;
    case lidar_localization::RegistrationSeedSource::kTwistPrediction: {
      const double dt = lidar_localization::clampPredictionDt(
        scan_stamp_sec, predicted_pose_time_sec_, max_twist_prediction_dt_);
      selected_seed.init_guess = applyTwistPrediction(predicted_pose_matrix_, dt);
      break;
    }
    case lidar_localization::RegistrationSeedSource::kPreviousDelta:
      selected_seed.init_guess = predicted_pose_matrix_;
      break;
    case lidar_localization::RegistrationSeedSource::kCurrentPose:
      break;
  }

  if (seed_decision.ignored_non_finite_imu_prediction) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring non-finite IMU predicted pose and falling back to non-IMU seed.");
  }
  return selected_seed;
}

Eigen::Matrix4f PCLLocalization::refineSeedWithNdtInitializer(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud,
  const Eigen::Matrix4f & init_guess)
{
  const auto run_input = lidar_localization::NdtInitializerRunInput{
    use_ndt_initializer_,
    static_cast<bool>(ndt_initializer_) && static_cast<bool>(source_cloud),
    ndt_init_scan_count_,
    ndt_init_scans_required_};
  if (!lidar_localization::shouldRunNdtInitializer(run_input)) {
    return init_guess;
  }

  ndt_initializer_->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ndt_output(new pcl::PointCloud<pcl::PointXYZI>);
  ndt_initializer_->align(*ndt_output, init_guess);
  const auto progress_decision =
    lidar_localization::updateNdtInitializerProgress(
    lidar_localization::NdtInitializerProgressInput{
      run_input,
      ndt_initializer_->hasConverged()});
  if (!progress_decision.should_accept_refined_seed) {
    return init_guess;
  }

  Eigen::Matrix4f refined_seed = ndt_initializer_->getFinalTransformation();
  ndt_init_scan_count_ = progress_decision.next_scan_count;
  RCLCPP_INFO(
    get_logger(), "NDT init scan %d/%d fitness=%.3f",
    ndt_init_scan_count_, ndt_init_scans_required_, ndt_initializer_->getFitnessScore());
  if (progress_decision.should_reset_initializer) {
    RCLCPP_INFO(get_logger(), "NDT init complete, switching to %s", registration_method_.c_str());
    ndt_initializer_.reset();
  }
  return refined_seed;
}

bool PCLLocalization::setInputTargetForPose(const Eigen::Matrix4f & center_pose_matrix)
{
  if (!use_local_map_crop_ || !full_map_cloud_ptr_) {
    return true;
  }

  const float cx = center_pose_matrix(0, 3);
  const float cy = center_pose_matrix(1, 3);
  const lidar_localization::LocalMapCropRequest crop_request{
    cx,
    cy,
    local_map_radius_,
    local_map_min_points_,
    lidar_localization::LocalMapBounds2d{
      map_bounds_valid_,
      map_min_pt_.x,
      map_max_pt_.x,
      map_min_pt_.y,
      map_max_pt_.y}};
  const auto steady_now = std::chrono::steady_clock::now();

  auto handle_crop_failure = [&](lidar_localization::LocalMapTargetFailure failure) {
      const bool has_last_streak_log =
        last_crop_failure_streak_log_time_ != std::chrono::steady_clock::time_point{};
      const auto elapsed_since_streak_log =
        has_last_streak_log ?
        steady_now - last_crop_failure_streak_log_time_ :
        std::chrono::steady_clock::duration::zero();
      const bool has_last_bounds_log =
        last_crop_out_of_bounds_log_time_ != std::chrono::steady_clock::time_point{};
      const auto elapsed_since_bounds_log =
        has_last_bounds_log ?
        steady_now - last_crop_out_of_bounds_log_time_ :
        std::chrono::steady_clock::duration::zero();
      const auto decision = lidar_localization::handleLocalMapTargetFailure(
        lidar_localization::LocalMapTargetFailureHandlingInput{
          failure,
          consecutive_crop_failures_,
          has_last_streak_log,
          elapsed_since_streak_log,
          has_last_bounds_log,
          elapsed_since_bounds_log});
      consecutive_crop_failures_ = decision.consecutive_crop_failures;
      if (decision.should_log_failure_streak) {
        last_crop_failure_streak_log_time_ = steady_now;
        RCLCPP_WARN(
          get_logger(),
          "Crop failure streak reached %d while target setup keeps failing.",
          consecutive_crop_failures_);
      }
      if (decision.should_log_out_of_bounds) {
        last_crop_out_of_bounds_log_time_ = steady_now;
      }
      return decision;
    };

  const auto crop_validation =
    lidar_localization::validateLocalMapCropRequest(crop_request);
  if (!crop_validation.can_crop) {
    const auto failure_decision = handle_crop_failure(crop_validation.failure);
    if (failure_decision.should_log_out_of_bounds) {
      RCLCPP_WARN(
        get_logger(),
        "Crop center (%.1f, %.1f) is outside map bounds + radius, skipping alignment",
        static_cast<double>(cx), static_cast<double>(cy));
    }
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map =
    lidar_localization::cropLocalMapByRadius(
      *full_map_cloud_ptr_,
      crop_request.center_x,
      crop_request.center_y,
      crop_request.radius_m);
  const auto crop_size_validation =
    lidar_localization::validateLocalMapCropSize(
      local_map->size(), crop_request.min_points);
  if (!crop_size_validation.target_ready) {
    handle_crop_failure(crop_size_validation.failure);
    RCLCPP_WARN(
      get_logger(),
      "Local map crop too small (%zu points, min %zu) around (%.3f, %.3f) with radius %.1fm.",
      local_map->size(),
      local_map_min_points_,
      static_cast<double>(cx),
      static_cast<double>(cy),
      local_map_radius_);
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_local_map(
    new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(local_map);
  voxel_grid_filter_.filter(*filtered_local_map);
  const auto target_selection = lidar_localization::chooseTargetAfterLocalMapCrop(
    local_map->size(),
    filtered_local_map->size(),
    crop_request.min_points);
  if (target_selection.target_cloud ==
    lidar_localization::LocalMapTargetCloud::kFilteredLocalMap)
  {
    registration_->setInputTarget(filtered_local_map);
    keep_cloud_alive(
      &recent_target_clouds_, filtered_local_map, kRegistrationTargetCloudKeepAliveCount);
  } else {
    registration_->setInputTarget(local_map);
    keep_cloud_alive(
      &recent_target_clouds_, local_map, kRegistrationTargetCloudKeepAliveCount);
  }

  const auto success_decision = lidar_localization::handleLocalMapTargetSuccess();
  consecutive_crop_failures_ = success_decision.consecutive_crop_failures;
  crop_failure_guard_active_ = success_decision.crop_failure_guard_active;
  return true;
}

lidar_localization::AlignmentAttempt PCLLocalization::runAlignmentAttempt(
  const Eigen::Matrix4f & attempt_init_guess,
  const Eigen::Matrix4f & crop_center_pose_matrix,
  double scan_stamp_sec)
{
  lidar_localization::AlignmentAttempt attempt;
  attempt.init_guess = attempt_init_guess;
  if (!setInputTargetForPose(crop_center_pose_matrix)) {
    return attempt;
  }
  attempt.target_ready = true;

  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  rclcpp::Clock system_clock;
  const rclcpp::Time time_align_start = system_clock.now();
  registration_->align(output_cloud, attempt_init_guess);
  const rclcpp::Time time_align_end = system_clock.now();
  attempt.alignment_time_sec = time_align_end.seconds() - time_align_start.seconds();
  attempt.has_converged = registration_->hasConverged();
  attempt.fitness_score = registration_->getFitnessScore();

  const auto seed_metrics = lidar_localization::computeAlignmentSeedMetrics(
    have_last_accepted_pose_,
    last_accepted_pose_matrix_,
    attempt_init_guess,
    scan_stamp_sec,
    last_accepted_pose_time_sec_);
  attempt.seed_translation_since_accept_m = seed_metrics.translation_since_accept_m;
  attempt.seed_yaw_since_accept_deg = seed_metrics.yaw_since_accept_deg;
  attempt.accepted_gap_sec = seed_metrics.accepted_gap_sec;

  if (attempt.has_converged) {
    attempt.final_transformation = registration_->getFinalTransformation();
    const auto correction_metrics =
      lidar_localization::computeAlignmentCorrectionMetrics(
      attempt_init_guess,
      attempt.final_transformation);
    attempt.correction_translation_m = correction_metrics.translation_m;
    attempt.correction_yaw_deg = correction_metrics.yaw_deg;
  }

  return attempt;
}

lidar_localization::AlignmentPipelineResult PCLLocalization::runAlignmentPipelineForScan(
  const Eigen::Matrix4f & init_guess,
  double scan_stamp_sec)
{
  const lidar_localization::AlignmentAttempt primary_attempt =
    runAlignmentAttempt(init_guess, init_guess, scan_stamp_sec);
  return lidar_localization::runAlignmentPipeline(
    primary_attempt,
    lidar_localization::AlignmentPipelineInput{
      have_last_accepted_pose_,
      consecutive_rejected_updates_,
      scan_stamp_sec,
      last_accepted_pose_time_sec_,
      recoveryRetryFromLastPoseParams()},
    [&]() {
      return runAlignmentAttempt(
        last_accepted_pose_matrix_, last_accepted_pose_matrix_, scan_stamp_sec);
    },
    [this](const lidar_localization::AlignmentAttempt & attempt) {
      return evaluateMeasurementGateForAttempt(attempt);
    });
}

lidar_localization::MeasurementGateDecision PCLLocalization::evaluateMeasurementGateForAttempt(
  const lidar_localization::AlignmentAttempt & attempt)
{
  const auto gate_input = lidar_localization::makeMeasurementGateInput(
    attempt.fitness_score,
    attempt.accepted_gap_sec,
    attempt.seed_translation_since_accept_m,
    attempt.correction_translation_m,
    attempt.correction_yaw_deg,
    consecutive_rejected_updates_);
  auto gate =
    lidar_localization::evaluateMeasurementGate(measurementGateParams(), gate_input);
  if (gate.status_level == lidar_localization::kMeasurementGateWarn) {
    RCLCPP_WARN(
      get_logger(), "The fitness score is over %lf.", gate.effective_score_threshold);
  }

  return gate;
}

void PCLLocalization::logAlignmentPipelineRecovery(
  const lidar_localization::AlignmentPipelineResult & pipeline_result)
{
  const auto handling = lidar_localization::decideAlignmentPipelineHandling(pipeline_result);
  if (!handling.log_recovery_retry_success) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Recovery retry from last pose succeeded after %zu rejects: fitness=%.6f",
    consecutive_rejected_updates_,
    pipeline_result.selected_attempt.fitness_score);
}

bool PCLLocalization::handleTerminalAlignmentPipelineResult(
  const builtin_interfaces::msg::Time & stamp,
  const lidar_localization::AlignmentPipelineResult & pipeline_result,
  std::size_t filtered_point_count,
  double scan_stamp_sec,
  bool imu_prediction_ready)
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
    imu_prediction_ready);
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
  bool imu_prediction_ready)
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
      imu_prediction_ready))
  {
    return false;
  }

  if (!enable_timer_publishing_) {
    publishCurrentPose(stamp);
  }
  return true;
}

void PCLLocalization::setRegistrationSourceCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & source_cloud)
{
  registration_->setInputSource(source_cloud);
  keep_cloud_alive(&recent_source_clouds_, source_cloud, kRegistrationSourceCloudKeepAliveCount);
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

Eigen::Matrix4f PCLLocalization::currentPoseMatrix() const
{
  if (!corrent_pose_with_cov_stamped_ptr_) {
    return Eigen::Matrix4f::Identity();
  }

  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
  return affine.matrix().cast<float>();
}

Eigen::Matrix4f PCLLocalization::applyTwistPrediction(
  const Eigen::Matrix4f & pose_matrix,
  double dt_sec) const
{
  if (!latest_twist_msg_ || dt_sec <= 0.0) {
    return pose_matrix;
  }

  const auto & twist = latest_twist_msg_->twist.twist;
  Eigen::Affine3f affine(pose_matrix);
  Eigen::Vector3f linear_velocity(
    static_cast<float>(twist.linear.x),
    static_cast<float>(twist.linear.y),
    static_cast<float>(twist.linear.z));
  Eigen::Vector3f angular_velocity(
    static_cast<float>(twist.angular.x),
    static_cast<float>(twist.angular.y),
    static_cast<float>(twist.angular.z));

  const Eigen::Vector3f world_delta = affine.linear() * (linear_velocity * static_cast<float>(dt_sec));
  affine.translation() += world_delta;

  if (twist_prediction_use_angular_velocity_) {
    const float roll = angular_velocity.x() * static_cast<float>(dt_sec);
    const float pitch = angular_velocity.y() * static_cast<float>(dt_sec);
    const float yaw = angular_velocity.z() * static_cast<float>(dt_sec);
    const Eigen::Matrix3f delta_rotation =
      (Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    affine.linear() = affine.linear() * delta_rotation;
  }

  return affine.matrix();
}

void PCLLocalization::resetPredictionState(const Eigen::Matrix4f & pose_matrix, double stamp_sec)
{
  const auto state = lidar_localization::resetPredictionState(pose_matrix, stamp_sec);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
}

void PCLLocalization::updatePredictionState(
  const Eigen::Matrix4f & accepted_pose_matrix,
  double stamp_sec)
{
  const auto state = lidar_localization::updatePredictionStateFromAcceptedMeasurement(
    make_prediction_state_snapshot(
      have_last_accepted_pose_,
      last_accepted_pose_matrix_,
      predicted_pose_matrix_,
      last_relative_motion_matrix_,
      consecutive_rejected_updates_,
      last_accepted_pose_time_sec_,
      predicted_pose_time_sec_),
    accepted_pose_matrix,
    stamp_sec);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
}

void PCLLocalization::advancePredictionWithoutMeasurement(double stamp_sec)
{
  const auto advance_mode = lidar_localization::choosePredictionAdvanceMode(
    have_last_accepted_pose_,
    use_twist_prediction_,
    static_cast<bool>(latest_twist_msg_),
    predict_pose_from_previous_delta_);
  Eigen::Matrix4f twist_predicted_pose_matrix = Eigen::Matrix4f::Identity();
  if (advance_mode == lidar_localization::PredictionAdvanceMode::kTwistPrediction) {
    const double dt = std::clamp(stamp_sec - predicted_pose_time_sec_, 0.0, max_twist_prediction_dt_);
    twist_predicted_pose_matrix = applyTwistPrediction(predicted_pose_matrix_, dt);
  }
  const auto state = lidar_localization::advancePredictionWithoutMeasurement(
    make_prediction_state_snapshot(
      have_last_accepted_pose_,
      last_accepted_pose_matrix_,
      predicted_pose_matrix_,
      last_relative_motion_matrix_,
      consecutive_rejected_updates_,
      last_accepted_pose_time_sec_,
      predicted_pose_time_sec_),
    stamp_sec,
    advance_mode,
    twist_predicted_pose_matrix);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
}

void PCLLocalization::updatePredictionFromRejectedMeasurement(
  const Eigen::Matrix4f & rejected_pose_matrix,
  double stamp_sec)
{
  const auto state = lidar_localization::updatePredictionFromRejectedMeasurement(
    make_prediction_state_snapshot(
      have_last_accepted_pose_,
      last_accepted_pose_matrix_,
      predicted_pose_matrix_,
      last_relative_motion_matrix_,
      consecutive_rejected_updates_,
      last_accepted_pose_time_sec_,
      predicted_pose_time_sec_),
    rejected_pose_matrix,
    stamp_sec);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
}

bool PCLLocalization::applyRegistrationPoseBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  const lidar_localization::MeasurementGateDecision & gate_result,
  const builtin_interfaces::msg::Time & stamp,
  std::size_t filtered_point_count,
  double stamp_sec,
  bool imu_prediction_ready)
{
  const PoseBackendApplyContext context{
    observation,
    attempt,
    gate_result,
    stamp,
    filtered_point_count,
    stamp_sec,
    imu_prediction_ready};
  return dispatchRegistrationPoseBackend(selectRegistrationPoseBackend(), context);
}

lidar_localization::PoseBackendKind PCLLocalization::selectRegistrationPoseBackend() const
{
  return lidar_localization::selectPoseBackend(
    lidar_localization::PoseBackendSelectionInput{
      use_gtsam_smoother_,
      use_imu_preintegration_,
      last_imu_stamp_ > 0.0,
      use_twist_ekf_});
}

bool PCLLocalization::dispatchRegistrationPoseBackend(
  lidar_localization::PoseBackendKind backend,
  const PoseBackendApplyContext & context)
{
  switch (backend) {
    case lidar_localization::PoseBackendKind::kGtsamSmoother:
      return applyGtsamPoseBackend(context);

    case lidar_localization::PoseBackendKind::kImuPreintegration:
      return applyImuPreintegrationPoseBackend(context);

    case lidar_localization::PoseBackendKind::kTwistEkf:
      return applyTwistEkfPoseBackend(context);

    case lidar_localization::PoseBackendKind::kRawRegistration:
      return applyRawRegistrationPoseBackend(context);
  }

  return applyRawRegistrationPoseBackend(context);
}

bool PCLLocalization::applyGtsamPoseBackend(const PoseBackendApplyContext & context)
{
  updateBackendRollPitch(context.observation);
  return applyPlanarSmootherPoseBackendUpdate(
    updateGtsamPoseBackend(context.observation, context.attempt, context.stamp_sec),
    context);
}

void PCLLocalization::updateBackendRollPitch(
  const lidar_localization::RegistrationObservation & observation)
{
  last_ndt_roll_ = static_cast<float>(observation.roll);
  last_ndt_pitch_ = static_cast<float>(observation.pitch);
}

PCLLocalization::PlanarSmootherBackendUpdate
PCLLocalization::updateGtsamPoseBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  double stamp_sec)
{
  if (!gtsam_smoother_.isInitialized()) {
    gtsam_smoother_.initialize(
      observation.x, observation.y, observation.z, observation.yaw, stamp_sec);
  }

  PlanarSmootherBackendUpdate update;
  update.updated = gtsam_smoother_.update(
    observation.x, observation.y, observation.z, observation.yaw,
    attempt.fitness_score, stamp_sec);
  update.pose = gtsam_smoother_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
  update.rejected_status_message = "gtsam_update_rejected";
  return update;
}

PCLLocalization::PlanarSmootherBackendUpdate
PCLLocalization::updateTwistEkfPoseBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  double stamp_sec)
{
  if (!twist_ekf_.isInitialized()) {
    twist_ekf_.initialize(
      observation.x, observation.y, observation.z, observation.yaw, stamp_sec);
  }

  PlanarSmootherBackendUpdate update;
  update.updated = twist_ekf_.update(
    observation.x, observation.y, observation.z, observation.yaw, attempt.fitness_score);
  update.pose = twist_ekf_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
  update.rejected_status_message = "ekf_update_rejected";
  return update;
}

bool PCLLocalization::applyPlanarSmootherPoseBackendUpdate(
  const PlanarSmootherBackendUpdate & update,
  const PoseBackendApplyContext & context)
{
  const auto backend_result = lidar_localization::applyPoseBackendUpdateStatus(
    lidar_localization::makePoseBackendResult(
      update.pose, context.gate_result.status_level, context.gate_result.status_message),
    update.updated,
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    update.rejected_status_message);
  applyPoseBackendResult(
    backend_result, context.stamp, context.stamp_sec, context.attempt.fitness_score);
  publishAlignmentStatusForAttempt(
    context.stamp,
    backend_result.status_level,
    backend_result.status_message,
    context.attempt,
    context.filtered_point_count,
    context.imu_prediction_ready);
  return true;
}

bool PCLLocalization::applyImuPreintegrationPoseBackend(
  const PoseBackendApplyContext & context)
{
  const auto imu_update = updateImuPreintegrationBackend(
    context.observation, context.attempt, context.stamp_sec, context.imu_prediction_ready);
  logImuPreintegrationBackendWarnings(imu_update, context.attempt);

  const auto backend_result = makeImuPreintegrationPoseBackendResult(
    imu_update, context.gate_result.status_level, context.gate_result.status_message);
  applyPoseBackendResult(
    backend_result, context.stamp, context.stamp_sec, context.attempt.fitness_score);
  publishAlignmentStatusForAttempt(
    context.stamp,
    backend_result.status_level,
    backend_result.status_message,
    context.attempt,
    context.filtered_point_count,
    context.imu_prediction_ready);
  return true;
}

lidar_localization::ImuPreintegrationGuardParams
PCLLocalization::imuPreintegrationGuardParams() const
{
  return {
    imu_prediction_correction_guard_translation_m_,
    imu_prediction_correction_guard_yaw_deg_,
    lidar_localization::kDefaultImuSmootherMeasurementTranslationGuardM,
    lidar_localization::kDefaultImuSmootherMeasurementRotationGuardDeg};
}

void PCLLocalization::initializeImuPreintegrationSmootherIfNeeded(
  const lidar_localization::RegistrationObservation & observation,
  double stamp_sec)
{
  if (imu_smoother_.isInitialized()) {
    return;
  }

  imu_smoother_.initialize(
    observation.x, observation.y, observation.z,
    observation.roll, observation.pitch, observation.yaw,
    0.0, 0.0, 0.0, stamp_sec);
}

PCLLocalization::ImuPreintegrationBackendUpdate
PCLLocalization::updateImuPreintegrationBackend(
  const lidar_localization::RegistrationObservation & observation,
  const lidar_localization::AlignmentAttempt & attempt,
  double stamp_sec,
  bool imu_prediction_ready)
{
  initializeImuPreintegrationSmootherIfNeeded(observation, stamp_sec);

  ImuPreintegrationBackendUpdate update;
  update.pose = observation.pose_matrix;

  const auto guard_params = imuPreintegrationGuardParams();
  update.state = lidar_localization::beginImuPreintegrationBackendState(
    guard_params,
    lidar_localization::ImuPredictionCorrectionGuardInput{
      imu_preintegration_fallback_mode_,
      imu_prediction_ready,
      attempt.correction_translation_m,
      attempt.correction_yaw_deg});

  if (update.state.should_update_smoother) {
    update.updated = imu_smoother_.update(
      observation.x, observation.y, observation.z,
      observation.roll, observation.pitch, observation.yaw,
      attempt.fitness_score, stamp_sec);

    update.pose = imu_smoother_.poseMatrix();
    if (update.pose.allFinite()) {
      update.smoother_measurement_translation_delta_m = static_cast<double>(
        (update.pose.block<3, 1>(0, 3) -
        observation.pose_matrix.block<3, 1>(0, 3)).norm());
      update.smoother_measurement_rotation_delta_deg =
        lidar_localization::rotationDeltaDeg(observation.pose_matrix, update.pose);
    }

    update.state = lidar_localization::applyImuSmootherDivergenceDecision(
      update.state,
      guard_params,
      lidar_localization::ImuSmootherDivergenceInput{
        update.pose.allFinite(),
        update.smoother_measurement_translation_delta_m,
        update.smoother_measurement_rotation_delta_deg});
  }

  imu_preintegration_fallback_mode_ = update.state.fallback_mode;
  if (lidar_localization::shouldUseImuMeasurementPose(update.state)) {
    update.pose = observation.pose_matrix;
    update.updated = true;
  }

  last_scan_stamp_for_imu_ = stamp_sec;
  return update;
}

void PCLLocalization::logImuPreintegrationBackendWarnings(
  const ImuPreintegrationBackendUpdate & update,
  const lidar_localization::AlignmentAttempt & attempt)
{
  if (update.state.correction_guard_tripped) {
    RCLCPP_WARN(
      get_logger(),
      "IMU prediction required a large measurement correction (translation=%.3f m, yaw=%.3f deg). Disabling IMU preintegration for the remainder of this run.",
      attempt.correction_translation_m,
      attempt.correction_yaw_deg);
  }

  if (update.state.smoother_diverged) {
    RCLCPP_WARN(
      get_logger(),
      "IMU smoother diverged from measurement (translation=%.3f m, rotation=%.3f deg). Disabling IMU preintegration for the remainder of this run.",
      update.smoother_measurement_translation_delta_m,
      update.smoother_measurement_rotation_delta_deg);
  }
}

lidar_localization::PoseBackendResult
PCLLocalization::makeImuPreintegrationPoseBackendResult(
  const ImuPreintegrationBackendUpdate & update,
  uint8_t status_level,
  const std::string & status_message) const
{
  const auto imu_status = lidar_localization::decideImuPreintegrationStatus(
    update.state, update.updated);
  return lidar_localization::applyPoseBackendWarningStatus(
    lidar_localization::makePoseBackendResult(
      update.pose, status_level, status_message),
    imu_status.warning,
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    imu_status.status_message);
}

bool PCLLocalization::applyTwistEkfPoseBackend(
  const PoseBackendApplyContext & context)
{
  updateBackendRollPitch(context.observation);
  return applyPlanarSmootherPoseBackendUpdate(
    updateTwistEkfPoseBackend(context.observation, context.attempt, context.stamp_sec),
    context);
}

bool PCLLocalization::applyRawRegistrationPoseBackend(
  const PoseBackendApplyContext & context)
{
  const auto localization_update =
    lidar_localization::decideLocalizationUpdate(context.gate_result);
  const auto backend_result =
    lidar_localization::makePoseBackendResultFromLocalizationUpdate(
      context.attempt.final_transformation,
      context.gate_result.status_level,
      context.gate_result.status_message,
      localization_update);
  publishAlignmentStatusForAttempt(
    context.stamp,
    backend_result.status_level,
    backend_result.status_message,
    context.attempt,
    context.filtered_point_count,
    context.imu_prediction_ready);

  return applyPoseBackendResult(
    backend_result, context.stamp, context.stamp_sec, context.attempt.fitness_score);
}

bool PCLLocalization::applyPoseBackendResult(
  const lidar_localization::PoseBackendResult & result,
  const builtin_interfaces::msg::Time & stamp,
  double stamp_sec,
  double fitness_score)
{
  if (
    result.advance_prediction_without_measurement ||
    result.update_prediction_from_rejected_measurement)
  {
    return applyPoseBackendPredictionOnlyResult(result, stamp_sec);
  }

  applyAcceptedPoseBackendResult(result, stamp, stamp_sec, fitness_score);
  return result.continue_to_pose_publish;
}

bool PCLLocalization::applyPoseBackendPredictionOnlyResult(
  const lidar_localization::PoseBackendResult & result,
  double stamp_sec)
{
  if (result.advance_prediction_without_measurement) {
    advancePredictionWithoutMeasurement(stamp_sec);
    return result.continue_to_pose_publish;
  }
  if (result.update_prediction_from_rejected_measurement) {
    updatePredictionFromRejectedMeasurement(result.pose_matrix, stamp_sec);
    return result.continue_to_pose_publish;
  }
  return result.continue_to_pose_publish;
}

void PCLLocalization::applyAcceptedPoseBackendResult(
  const lidar_localization::PoseBackendResult & result,
  const builtin_interfaces::msg::Time & stamp,
  double stamp_sec,
  double fitness_score)
{
  if (result.update_current_pose) {
    setCurrentPoseFromMatrix(result.pose_matrix, stamp);
  }
  if (result.fill_pose_covariance) {
    fillPoseCovariance(fitness_score);
  }
  if (result.update_prediction_state) {
    updatePredictionState(result.pose_matrix, stamp_sec);
  }
}

void PCLLocalization::setCurrentPoseFromMatrix(
  const Eigen::Matrix4f & pose_matrix,
  const builtin_interfaces::msg::Time & stamp)
{
  corrent_pose_with_cov_stamped_ptr_->header.stamp = stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose =
    lidar_localization::poseFromMatrix(pose_matrix);
}

void PCLLocalization::publishCurrentPose(const builtin_interfaces::msg::Time & stamp)
{
  if (shutting_down_ || !pose_pub_ || !corrent_pose_with_cov_stamped_ptr_) {return;}
  publishPoseMessage(*corrent_pose_with_cov_stamped_ptr_);
  if (!publishPoseTransform(stamp, corrent_pose_with_cov_stamped_ptr_->pose.pose)) {
    return;
  }

  appendCurrentPoseToPath(stamp, corrent_pose_with_cov_stamped_ptr_->pose.pose);
  publishPathMessage();
}

void PCLLocalization::publishPoseMessage(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  pose_pub_->publish(pose);
}

void PCLLocalization::appendCurrentPoseToPath(
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & pose)
{
  lidar_localization::appendPoseToPath(
    *path_ptr_,
    lidar_localization::makePoseStamped(stamp, global_frame_id_, pose));
}

void PCLLocalization::publishPathMessage()
{
  path_pub_->publish(*path_ptr_);
}

bool PCLLocalization::publishPoseTransform(
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & pose)
{
  const geometry_msgs::msg::TransformStamped map_to_base_link_stamped =
    lidar_localization::makeMapToBaseTransform(
      stamp,
      global_frame_id_,
      base_frame_id_,
      pose);
  if (!enable_map_odom_tf_) {
    broadcaster_.sendTransform(map_to_base_link_stamped);
    return true;
  }

  return publishMapToOdomTransform(stamp, map_to_base_link_stamped);
}

bool PCLLocalization::publishMapToOdomTransform(
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::TransformStamped & map_to_base_link_stamped)
{
  geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
  try {
    odom_to_base_link_msg = tfbuffer_.lookupTransform(
      odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not get transform %s to %s: %s",
      base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
    return false;
  }
  broadcaster_.sendTransform(
    lidar_localization::composeMapToOdomTransform(
      stamp,
      global_frame_id_,
      odom_frame_id_,
      map_to_base_link_stamped,
      odom_to_base_link_msg));
  return true;
}

void PCLLocalization::fillPoseCovariance(double fitness_score)
{
  if (use_twist_ekf_ && twist_ekf_.isInitialized()) {
    corrent_pose_with_cov_stamped_ptr_->pose.covariance =
      lidar_localization::makeEkfPoseCovariance(twist_ekf_.covariance(), fitness_score);
    return;
  }

  corrent_pose_with_cov_stamped_ptr_->pose.covariance =
    lidar_localization::makeFitnessPoseCovariance(fitness_score);
}

PCLLocalization::ReinitializationRequestDecision
PCLLocalization::applyReinitializationRequestLatch(
  const builtin_interfaces::msg::Time & stamp,
  const ReinitializationRequestDecision & decision)
{
  const auto latch_result = lidar_localization::applyReinitializationRequestLatch(
    lidar_localization::ReinitializationRequestLatchInput{
      enable_reinitialization_request_latch_,
      lidar_localization::ReinitializationRequestLatchState{
        reinitialization_request_latched_,
        reinitialization_request_latch_reason_,
        reinitialization_request_latch_score_,
        reinitialization_request_latch_stamp_sec_},
      decision,
      stamp_to_sec(stamp)});

  reinitialization_request_latched_ = latch_result.state.latched;
  reinitialization_request_latch_reason_ = latch_result.state.reason;
  reinitialization_request_latch_score_ = latch_result.state.score;
  reinitialization_request_latch_stamp_sec_ = latch_result.state.stamp_sec;
  return latch_result.decision;
}

void PCLLocalization::updateRecoverySupervisorState(
  const builtin_interfaces::msg::Time & stamp,
  RecoverySupervisorState next_state,
  const std::string & action)
{
  const double stamp_sec = stamp_to_sec(stamp);
  const auto update = lidar_localization::updateRecoverySupervisorRuntimeState(
    lidar_localization::RecoverySupervisorStateUpdateInput{
      lidar_localization::RecoverySupervisorRuntimeState{
        recovery_supervisor_state_,
        recovery_supervisor_action_,
        recovery_supervisor_state_entered_stamp_sec_,
        recovery_supervisor_transition_count_},
      next_state,
      action,
      stamp_sec});

  if (update.transitioned) {
    RCLCPP_INFO(
      get_logger(),
      "Recovery supervisor state transition: %s -> %s (%s)",
      lidar_localization::recoverySupervisorStateName(update.previous_state),
      lidar_localization::recoverySupervisorStateName(update.state.state),
      action.c_str());
  }

  recovery_supervisor_state_ = update.state.state;
  recovery_supervisor_action_ = update.state.action;
  recovery_supervisor_state_entered_stamp_sec_ = update.state.entered_stamp_sec;
  recovery_supervisor_transition_count_ = update.state.transition_count;
}

void PCLLocalization::publishReinitializationRequest(
  const builtin_interfaces::msg::Time & stamp,
  const ReinitializationRequestDecision & decision)
{
  (void)stamp;
  const bool publisher_ready =
    reinitialization_request_pub_ && reinitialization_request_pub_->is_activated();
  const auto output = lidar_localization::prepareReinitializationRequestOutput(
    lidar_localization::ReinitializationRequestOutputInput{
      enable_reinitialization_request_output_,
      publisher_ready,
      decision});

  reinitialization_requested_ = output.state.requested;
  reinitialization_request_reason_ = output.state.reason;
  reinitialization_request_score_ = output.state.score;

  if (!output.should_publish) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = output.message_value;
  reinitialization_request_pub_->publish(msg);
}

lidar_localization::MeasurementGateParams PCLLocalization::measurementGateParams() const
{
  return lidar_localization::makeMeasurementGateParams(measurement_gate_config_);
}

lidar_localization::ReinitializationTriggerParams
PCLLocalization::reinitializationTriggerParams() const
{
  return reinitialization_trigger_config_;
}

lidar_localization::RecoveryRetryFromLastPoseParams
PCLLocalization::recoveryRetryFromLastPoseParams() const
{
  return recovery_retry_from_last_pose_config_;
}

void PCLLocalization::publishAlignmentStatusForAttempt(
  const builtin_interfaces::msg::Time & stamp,
  uint8_t level,
  const std::string & message,
  const lidar_localization::AlignmentAttempt & attempt,
  std::size_t filtered_point_count,
  bool imu_prediction_active)
{
  publishAlignmentStatus(
    stamp,
    level,
    message,
    attempt.has_converged,
    attempt.fitness_score,
    attempt.alignment_time_sec,
    filtered_point_count,
    attempt.correction_translation_m,
    attempt.correction_yaw_deg,
    attempt.seed_translation_since_accept_m,
    attempt.seed_yaw_since_accept_deg,
    attempt.accepted_gap_sec,
    imu_prediction_active);
}

void PCLLocalization::publishAlignmentStatus(
  const builtin_interfaces::msg::Time & stamp,
  uint8_t level,
  const std::string & message,
  bool has_converged,
  double fitness_score,
  double alignment_time_sec,
  std::size_t filtered_point_count,
  double correction_translation_m,
  double correction_yaw_deg,
  double seed_translation_since_accept_m,
  double seed_yaw_since_accept_deg,
  double accepted_gap_sec,
  bool imu_prediction_active)
{
  if (!status_pub_) {return;}

  const AlignmentStatusPublishInput publish_input{
    stamp,
    level,
    message,
    has_converged,
    fitness_score,
    alignment_time_sec,
    filtered_point_count,
    correction_translation_m,
    correction_yaw_deg,
    seed_translation_since_accept_m,
    seed_yaw_since_accept_deg,
    accepted_gap_sec,
    imu_prediction_active};

  const auto evaluation = evaluateAlignmentStatus(stamp, publish_input);
  auto status = makeAlignmentDiagnosticStatus(evaluation.status_input);

  appendAlignmentDiagnosticValues(
    status,
    prepareAlignmentDiagnosticValuesInput(
      evaluation.status_input,
      evaluation.status_preparation,
      evaluation.reinitialization_request));
  publishAlignmentDiagnosticStatus(stamp, status);
  publishReinitializationRequest(stamp, evaluation.reinitialization_request);
}

lidar_localization::AlignmentStatusInput PCLLocalization::makeAlignmentStatusInput(
  const AlignmentStatusPublishInput & input) const
{
  const double stamp_sec = stamp_to_sec(input.stamp);
  return lidar_localization::makeAlignmentStatusInput(
    lidar_localization::AlignmentStatusObservation{
      input.level,
      input.message,
      input.has_converged,
      input.fitness_score,
      input.alignment_time_sec,
      input.filtered_point_count,
      input.correction_translation_m,
      input.correction_yaw_deg,
      input.seed_translation_since_accept_m,
      input.seed_yaw_since_accept_deg,
      input.accepted_gap_sec,
      input.imu_prediction_active},
    makeAlignmentStatusRuntimeContext(stamp_sec));
}

lidar_localization::AlignmentStatusRuntimeContext
PCLLocalization::makeAlignmentStatusRuntimeContext(double stamp_sec) const
{
  const auto fallback_seed_metrics = lidar_localization::computeAlignmentSeedMetrics(
    have_last_accepted_pose_,
    last_accepted_pose_matrix_,
    predicted_pose_matrix_,
    stamp_sec,
    last_accepted_pose_time_sec_);

  return lidar_localization::AlignmentStatusRuntimeContext{
    registration_method_,
    consecutive_rejected_updates_,
    have_last_accepted_pose_,
    stamp_sec,
    last_accepted_pose_time_sec_,
    fallback_seed_metrics.translation_since_accept_m,
    map_recieved_,
    initialpose_recieved_,
    measurementGateParams(),
    reinitializationTriggerParams()};
}

diagnostic_msgs::msg::DiagnosticStatus PCLLocalization::makeAlignmentDiagnosticStatus(
  const lidar_localization::AlignmentStatusInput & status_input) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = status_input.level;
  status.name = "lidar_localization_ros2/alignment";
  status.message = status_input.message;
  status.hardware_id = status_input.registration_method;
  return status;
}

PCLLocalization::AlignmentStatusEvaluation PCLLocalization::evaluateAlignmentStatus(
  const builtin_interfaces::msg::Time & stamp,
  const AlignmentStatusPublishInput & publish_input)
{
  AlignmentStatusEvaluation evaluation;
  evaluation.status_input = makeAlignmentStatusInput(publish_input);
  evaluation.status_preparation =
    lidar_localization::prepareAlignmentStatus(evaluation.status_input);
  evaluation.reinitialization_request =
    applyReinitializationRequestLatch(
      stamp,
      evaluation.status_preparation.reinitialization_request);

  const auto recovery_evaluation =
    lidar_localization::evaluateAlignmentStatusRecovery(
      evaluation.status_input,
      evaluation.reinitialization_request);
  updateRecoverySupervisorState(
    stamp,
    recovery_evaluation.state,
    recovery_evaluation.action);
  return evaluation;
}

lidar_localization::AlignmentDiagnosticValuesInput
PCLLocalization::prepareAlignmentDiagnosticValuesInput(
  const lidar_localization::AlignmentStatusInput & status_input,
  const lidar_localization::AlignmentStatusPreparation & status_preparation,
  const ReinitializationRequestDecision & reinitialization_request) const
{
  return lidar_localization::makeAlignmentStatusDiagnosticValuesInput(
    lidar_localization::makeAlignmentStatusDiagnosticInput(
      status_input,
      status_preparation,
      reinitialization_request,
      lidar_localization::AlignmentStatusDiagnosticRuntimeContext{
        lidar_localization::recoverySupervisorStateName(recovery_supervisor_state_),
        recovery_supervisor_action_,
        recovery_supervisor_state_entered_stamp_sec_,
        recovery_supervisor_transition_count_,
        reinitialization_request_latched_,
        reinitialization_request_latch_stamp_sec_}));
}

void PCLLocalization::appendAlignmentDiagnosticValues(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const lidar_localization::AlignmentDiagnosticValuesInput & diagnostic_values_input) const
{
  for (const auto & value :
    lidar_localization::makeRosAlignmentDiagnosticKeyValues(diagnostic_values_input))
  {
    status.values.push_back(value);
  }
}

diagnostic_msgs::msg::DiagnosticArray PCLLocalization::makeAlignmentDiagnosticArray(
  const builtin_interfaces::msg::Time & stamp,
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  diagnostic_msgs::msg::DiagnosticArray status_array;
  status_array.header.stamp = stamp;
  status_array.header.frame_id = base_frame_id_;
  status_array.status.push_back(status);
  return status_array;
}

void PCLLocalization::publishAlignmentDiagnosticStatus(
  const builtin_interfaces::msg::Time & stamp,
  const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  status_pub_->publish(makeAlignmentDiagnosticArray(stamp, status));
}

void PCLLocalization::timerPublishPose()
{
  if (shutting_down_ || !pose_pub_ || !path_pub_ || !path_ptr_) {return;}
  if (!corrent_pose_with_cov_stamped_ptr_) {return;}
  geometry_msgs::msg::PoseWithCovarianceStamped pose_copy =
    lidar_localization::stampPoseWithCovariance(*corrent_pose_with_cov_stamped_ptr_, now());

  appendCurrentPoseToPath(pose_copy.header.stamp, pose_copy.pose.pose);

  nav_msgs::msg::Path path_copy = *path_ptr_;

  publishPoseMessage(pose_copy);
  path_pub_->publish(path_copy);

  publishPoseTransform(pose_copy.header.stamp, pose_copy.pose.pose);
}
