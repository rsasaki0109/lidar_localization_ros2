#include "component_internal.hpp"
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
  declare_parameter("use_odom_tf_prediction", false);
  declare_parameter("constrain_odom_tf_prediction_to_planar", false);
  declare_parameter("constrain_odom_tf_prediction_height_only", false);
  declare_parameter("publish_bridge_pose_when_lost", false);
  declare_parameter("enable_map_odom_anchor_fitness_gate", false);
  declare_parameter("map_odom_anchor_max_fitness", 1.5);
  declare_parameter("map_odom_anchor_max_correction_rotation_deg", 10.0);
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
  declare_parameter("scan_time_range_max_duration_ratio", 2.0);
  declare_parameter("cloud_queue_depth", 1);
  declare_parameter("imu_queue_depth", 2000);
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
  declareImuPreintegrationParameters();
  declare_parameter("enable_debug", false);
  declare_parameter("predict_pose_from_previous_delta", true);
  declare_parameter("enable_local_map_crop", false);
  declare_parameter("local_map_radius", 150.0);
  declare_parameter("local_map_min_points", 100);
  declare_parameter(
    "registration_source_cloud_keep_alive_count",
    static_cast<int>(lidar_localization::kDefaultRegistrationSourceCloudKeepAliveCount));
  declare_parameter(
    "registration_target_cloud_keep_alive_count",
    static_cast<int>(lidar_localization::kDefaultRegistrationTargetCloudKeepAliveCount));
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
  declare_parameter("enable_odom_tf_prediction_correction_guard", false);
  declare_parameter("odom_tf_prediction_correction_guard_translation_m", 2.0);
  declare_parameter("odom_tf_prediction_correction_guard_yaw_deg", 30.0);
  declare_parameter("enable_odom_tf_prediction_recovery_correction_guard", false);
  declare_parameter("odom_tf_prediction_recovery_min_rejections", 30);
  declare_parameter("odom_tf_prediction_recovery_max_fitness", 1.5);
  declare_parameter("odom_tf_prediction_recovery_guard_translation_m", 5.0);
  declare_parameter("odom_tf_prediction_recovery_guard_yaw_deg", 30.0);
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
  declare_parameter("reinitialization_request_clear_ok_samples", 5);
  declare_parameter("reinitialization_request_clear_max_fitness", 1.0);
  declare_parameter("reinitialization_request_clear_max_correction_translation_m", 0.5);
  declare_parameter("reinitialization_trigger_threshold", 0.95);
  declare_parameter("reinitialization_trigger_gap_scale_sec", 30.0);
  declare_parameter("reinitialization_trigger_seed_translation_scale_m", 100.0);
  declare_parameter("reinitialization_trigger_reject_streak_scale", 200.0);
  declare_parameter("reinitialization_trigger_fitness_explosion_threshold", 1000.0);
  declare_parameter("diagnostics_weak_overlap_min_filtered_points", 100);
  declare_parameter("diagnostics_stale_prediction_min_gap_sec", 2.0);
  declare_parameter("diagnostics_overload_alignment_time_sec", 0.3);
  declare_parameter("pose_covariance_mode", "error_floor");
  declare_parameter("covariance_xy_floor_std_m", 0.2);
  declare_parameter("covariance_xy_std_per_fitness_m", 0.1);
  declare_parameter("covariance_xy_max_std_m", 5.0);
  declare_parameter("covariance_z_floor_std_m", 0.3);
  declare_parameter("covariance_yaw_floor_std_deg", 2.0);
  declare_parameter("covariance_yaw_std_per_fitness_deg", 1.0);
  declare_parameter("covariance_yaw_max_std_deg", 30.0);
  declare_parameter("covariance_roll_pitch_floor_std_deg", 1.5);
  declare_parameter("enable_timer_publishing", false);
  declare_parameter("pose_publish_frequency", 10.0);
  declare_parameter(
    "path_max_poses", static_cast<int>(lidar_localization::kDefaultPathMaxPoses));
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
  auto state_lock = callback_state_coordinator_.lockState();

  shutting_down_.store(false, std::memory_order_release);
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
  auto state_lock = callback_state_coordinator_.lockState();

  pose_pub_->on_activate();
  odom_bridge_pose_pub_->on_activate();
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
  auto state_lock = callback_state_coordinator_.lockState();

#ifdef LIDAR_LOCALIZATION_HAVE_NAV2_BOND
  if (bond_) {
    bond_.reset();
    RCLCPP_INFO(get_logger(), "Nav2 bond stopped");
  }
#endif

  pose_pub_->on_deactivate();
  odom_bridge_pose_pub_->on_deactivate();
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
  shutting_down_.store(true, std::memory_order_release);
  auto state_lock = callback_state_coordinator_.lockState();

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
  initial_pose_callback_group_.reset();
  pose_publish_callback_group_.reset();
  imu_callback_group_.reset();

  latest_twist_msg_.reset();
  last_scan_ptr_.reset();
  path_ptr_.reset();
  corrent_pose_with_cov_stamped_ptr_.reset();

  initial_map_pub_.reset();
  path_pub_.reset();
  status_pub_.reset();
  reinitialization_request_pub_.reset();
  pose_pub_.reset();
  odom_bridge_pose_pub_.reset();
  last_good_map_to_odom_ = geometry_msgs::msg::TransformStamped{};
  has_last_good_map_to_odom_ = false;
  odom_tf_constraint_anchor_pose_matrix_ = Eigen::Matrix4f::Identity();
  has_odom_tf_constraint_anchor_pose_ = false;
  odom_bridge_transform_history_.clear();
  last_odom_bridge_source_advance_node_stamp_ = builtin_interfaces::msg::Time{};
  has_last_odom_bridge_source_advance_node_stamp_ = false;

  auto registration_execution_lock =
    callback_state_coordinator_.lockRegistrationExecution();
  ndt_initializer_.reset();
  use_ndt_initializer_ = false;
  registration_ = nullptr;
  pcl_registration_.reset();
  ndt_omp_registration_.reset();
  gicp_omp_registration_.reset();
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  small_gicp_registration_.reset();
#endif
  registration_execution_lock.unlock();

  {
    std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
    imu_preintegration_fallback_mode_ = false;
    last_imu_stamp_ = 0.0;
    last_scan_stamp_for_imu_ = 0.0;
    latest_imu_seed_has_new_samples_ = false;
    latest_imu_seed_prediction_finite_ = true;
    latest_imu_seed_received_sample_count_ = 0;
    latest_imu_seed_integrated_sample_count_ = 0;
    latest_imu_seed_skipped_sample_count_ = 0;
    latest_imu_seed_transform_failure_count_ = 0;
    latest_imu_seed_non_finite_sample_count_ = 0;
    latest_imu_seed_invalid_dt_count_ = 0;
    latest_imu_seed_last_dt_sec_ = std::numeric_limits<double>::quiet_NaN();
    latest_imu_seed_last_sample_age_sec_ = std::numeric_limits<double>::quiet_NaN();
    latest_imu_seed_integration_window_sec_ = 0.0;
    latest_imu_open_loop_prediction_ = Eigen::Matrix4f::Identity();
    latest_imu_open_loop_prediction_available_ = false;
    latest_imu_seed_consistency_translation_error_m_ =
      std::numeric_limits<double>::quiet_NaN();
    latest_imu_seed_consistency_rotation_error_deg_ =
      std::numeric_limits<double>::quiet_NaN();
    latest_imu_seed_consistency_sample_passed_ = false;
    imu_seed_consistency_state_ = {};
    imu_dual_queue_.clear();
    imu_optimization_anchor_sample_.reset();
    imu_prediction_anchor_sample_.reset();
    latest_dual_queue_integrated_stamp_ = 0.0;
    resetImuPreintegrationSampleCounters();
  }
  reinitialization_requested_ = false;
  reinitialization_request_reason_ = "not_requested";
  reinitialization_request_score_ = 0.0;
  reinitialization_request_latched_ = false;
  reinitialization_request_latch_reason_ = "not_requested";
  reinitialization_request_latch_score_ = 0.0;
  reinitialization_request_latch_stamp_sec_ = 0.0;
  reinitialization_request_latch_consecutive_ok_samples_ = 0;
  recovery_supervisor_state_ = RecoverySupervisorState::kTracking;
  recovery_supervisor_action_ = "idle";
  recovery_supervisor_state_entered_stamp_sec_ = 0.0;
  recovery_supervisor_transition_count_ = 0;
  recent_source_clouds_.clear();
  // NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks): the target and full-map
  // clouds are intentionally leaked when leak_target_clouds_for_shutdown is set.
  // NDT_OMP can still hold shutdown-path ownership relationships to them, so
  // freeing them during process teardown crashes.
  if (leak_target_clouds_for_shutdown) {
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
  // NOLINTEND(clang-analyzer-cplusplus.NewDeleteLeaks)
  consecutive_crop_failures_ = 0;
  crop_failure_guard_active_ = false;
  last_crop_out_of_bounds_log_time_ = std::chrono::steady_clock::time_point{};
  last_crop_failure_streak_log_time_ = std::chrono::steady_clock::time_point{};
  map_recieved_ = false;
  initialpose_recieved_ = false;
}

