#include "component_internal.hpp"
void PCLLocalization::declareImuPreintegrationParameters()
{
  // Backend and input units.
  declare_parameter("use_imu_preintegration", true);
  declare_parameter("imu_preintegration_use_base_frame_transform", false);
  declare_parameter("imu_accel_scale", 1.0);
  declare_parameter("imu_dual_queue_enabled", true);

  // Seed safety and correction guards.
  declare_parameter("imu_seed_consistency_gate_enabled", true);
  declare_parameter("imu_seed_consistency_max_translation_error_m", 0.5);
  declare_parameter("imu_seed_consistency_max_rotation_error_deg", 5.0);
  declare_parameter("imu_seed_consistency_required_consecutive_passes", 5);
  declare_parameter("imu_prediction_correction_guard_translation_m", 2.0);
  declare_parameter("imu_prediction_correction_guard_yaw_deg", 10.0);
  declare_parameter("imu_prediction_correction_guard_warmup_accepts", 5);

  // Preintegration noise and factor model.
  declare_parameter("imu_gyro_noise_density", 0.01);
  declare_parameter("imu_accel_noise_density", 0.1);
  declare_parameter("imu_gyro_random_walk", 0.0001);
  declare_parameter("imu_accel_random_walk", 0.001);
  declare_parameter("imu_ndt_sigma_z", 0.1);
  declare_parameter("imu_ndt_sigma_roll", 0.05);
  declare_parameter("imu_ndt_sigma_pitch", 0.05);
  declare_parameter("imu_bias_prior_sigma_gyro", 0.01);
  declare_parameter("imu_bias_prior_sigma_accel", 0.1);

  // Deskew is enabled by default and falls back to the unmodified scan until
  // per-point timing and a usable motion estimate are available.
  declare_parameter("use_continuous_time_deskew", true);
  declare_parameter("continuous_time_deskew_mode", "relative_motion");
  declare_parameter("continuous_time_cloud_stamp_reference", "start");
  declare_parameter("continuous_time_deskew_reference_time_sec", 0.0);
  declare_parameter("continuous_time_pose_history_duration_sec", 2.0);
  declare_parameter("enable_localizability_guard", false);
  declare_parameter("localizability_min_xy_eigen_ratio", 0.05);
  declare_parameter("enable_registration_localizability_diagnostics", false);
}

void PCLLocalization::loadImuPreintegrationParameters()
{
  // Backend and input units.
  get_parameter("use_imu_preintegration", use_imu_preintegration_);
  get_parameter(
    "imu_preintegration_use_base_frame_transform",
    imu_preintegration_use_base_frame_transform_);
  get_parameter("imu_accel_scale", imu_accel_scale_);
  get_parameter("imu_dual_queue_enabled", imu_dual_queue_enabled_);
  if (!std::isfinite(imu_accel_scale_) || imu_accel_scale_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "Invalid imu_accel_scale %.6f; using 1.0", imu_accel_scale_);
    imu_accel_scale_ = 1.0;
  }

  // Seed safety and correction guards.
  get_parameter("imu_seed_consistency_gate_enabled", imu_seed_consistency_gate_enabled_);
  get_parameter(
    "imu_seed_consistency_max_translation_error_m",
    imu_seed_consistency_params_.max_translation_error_m);
  get_parameter(
    "imu_seed_consistency_max_rotation_error_deg",
    imu_seed_consistency_params_.max_rotation_error_deg);
  int required_consecutive_passes = 5;
  get_parameter(
    "imu_seed_consistency_required_consecutive_passes", required_consecutive_passes);
  imu_seed_consistency_params_.required_consecutive_passes = static_cast<std::size_t>(
    std::max(1, required_consecutive_passes));
  // Experimental deskew and localizability diagnostics.
  get_parameter("use_continuous_time_deskew", use_continuous_time_deskew_);
  get_parameter("continuous_time_deskew_mode", continuous_time_deskew_mode_);
  get_parameter(
    "continuous_time_cloud_stamp_reference", continuous_time_cloud_stamp_reference_);
  get_parameter(
    "continuous_time_deskew_reference_time_sec",
    continuous_time_deskew_reference_time_sec_);
  get_parameter(
    "continuous_time_pose_history_duration_sec",
    continuous_time_pose_history_duration_sec_);
  get_parameter("enable_localizability_guard", enable_localizability_guard_);
  get_parameter(
    "enable_registration_localizability_diagnostics",
    enable_registration_localizability_diagnostics_);
  get_parameter(
    "localizability_min_xy_eigen_ratio", localizability_min_xy_eigen_ratio_);
  if (
    continuous_time_deskew_mode_ != "relative_motion" &&
    continuous_time_deskew_mode_ != "imu_pose_history" &&
    continuous_time_deskew_mode_ != "lidar_constant_velocity")
  {
    RCLCPP_WARN(
      get_logger(), "Unsupported continuous_time_deskew_mode '%s'; using relative_motion",
      continuous_time_deskew_mode_.c_str());
    continuous_time_deskew_mode_ = "relative_motion";
  }
  if (
    continuous_time_cloud_stamp_reference_ != "start" &&
    continuous_time_cloud_stamp_reference_ != "end")
  {
    RCLCPP_WARN(
      get_logger(), "Unsupported continuous_time_cloud_stamp_reference '%s'; using start",
      continuous_time_cloud_stamp_reference_.c_str());
    continuous_time_cloud_stamp_reference_ = "start";
  }
  if (
    !std::isfinite(continuous_time_pose_history_duration_sec_) ||
    continuous_time_pose_history_duration_sec_ <= 0.0)
  {
    RCLCPP_WARN(
      get_logger(), "continuous_time_pose_history_duration_sec must be positive; using 2.0");
    continuous_time_pose_history_duration_sec_ = 2.0;
  }

  if (!use_imu_preintegration_) {
    return;
  }

  get_parameter(
    "imu_prediction_correction_guard_translation_m",
    imu_prediction_correction_guard_translation_m_);
  get_parameter(
    "imu_prediction_correction_guard_yaw_deg",
    imu_prediction_correction_guard_yaw_deg_);
  get_parameter(
    "imu_prediction_correction_guard_warmup_accepts",
    imu_prediction_correction_guard_warmup_accepts_);

  // Preintegration noise and factor model.
  ImuGtsamSmoother::Params imu_params;
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
  imu_smoother_.params_ = imu_params;
  imu_prediction_smoother_.params_ = imu_params;
  RCLCPP_INFO(get_logger(), "IMU preintegration smoother enabled");
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("enable_map_odom_tf", enable_map_odom_tf_);
  get_parameter("use_odom_tf_prediction", use_odom_tf_prediction_);
  get_parameter(
    "constrain_odom_tf_prediction_to_planar",
    constrain_odom_tf_prediction_to_planar_);
  get_parameter(
    "constrain_odom_tf_prediction_height_only",
    constrain_odom_tf_prediction_height_only_);
  get_parameter("publish_bridge_pose_when_lost", publish_bridge_pose_when_lost_);
  get_parameter(
    "enable_map_odom_anchor_fitness_gate",
    enable_map_odom_anchor_fitness_gate_);
  get_parameter("map_odom_anchor_max_fitness", map_odom_anchor_max_fitness_);
  get_parameter(
    "map_odom_anchor_max_correction_rotation_deg",
    map_odom_anchor_max_correction_rotation_deg_);
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
  get_parameter(
    "scan_time_range_max_duration_ratio", scan_time_range_max_duration_ratio_);
  int requested_cloud_queue_depth = cloud_queue_depth_;
  get_parameter("cloud_queue_depth", requested_cloud_queue_depth);
  const auto cloud_queue_depth =
    lidar_localization::normalizePositiveIntParameter(requested_cloud_queue_depth);
  cloud_queue_depth_ = cloud_queue_depth.value;
  if (cloud_queue_depth.was_adjusted) {
    RCLCPP_WARN(
      get_logger(), "cloud_queue_depth must be positive; using %d", cloud_queue_depth_);
  }
  int requested_imu_queue_depth = imu_queue_depth_;
  get_parameter("imu_queue_depth", requested_imu_queue_depth);
  const auto imu_queue_depth =
    lidar_localization::normalizePositiveIntParameter(requested_imu_queue_depth);
  imu_queue_depth_ = imu_queue_depth.value;
  if (imu_queue_depth.was_adjusted) {
    RCLCPP_WARN(
      get_logger(), "imu_queue_depth must be positive; using %d", imu_queue_depth_);
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
  loadImuPreintegrationParameters();
  get_parameter("enable_debug", enable_debug_);
  get_parameter("viz_downsample", viz_downsample_);
  get_parameter("viz_voxel_leaf_size", viz_voxel_leaf_size_);
  get_parameter("predict_pose_from_previous_delta", predict_pose_from_previous_delta_);
  get_parameter("enable_local_map_crop", enable_local_map_crop_);
  get_parameter("local_map_radius", local_map_radius_);
  get_parameter("local_map_update_distance", local_map_update_distance_);
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
  int requested_registration_source_cloud_keep_alive_count =
    static_cast<int>(registration_source_cloud_keep_alive_count_);
  get_parameter(
    "registration_source_cloud_keep_alive_count",
    requested_registration_source_cloud_keep_alive_count);
  const auto registration_source_cloud_keep_alive_count =
    lidar_localization::normalizeRegistrationCloudKeepAliveCount(
    requested_registration_source_cloud_keep_alive_count,
    lidar_localization::kDefaultRegistrationSourceCloudKeepAliveCount);
  registration_source_cloud_keep_alive_count_ =
    registration_source_cloud_keep_alive_count.value;
  if (registration_source_cloud_keep_alive_count.was_adjusted) {
    RCLCPP_WARN(
      get_logger(),
      "registration_source_cloud_keep_alive_count must be >= 0; using %zu",
      registration_source_cloud_keep_alive_count_);
  }
  int requested_registration_target_cloud_keep_alive_count =
    static_cast<int>(registration_target_cloud_keep_alive_count_);
  get_parameter(
    "registration_target_cloud_keep_alive_count",
    requested_registration_target_cloud_keep_alive_count);
  const auto registration_target_cloud_keep_alive_count =
    lidar_localization::normalizeRegistrationCloudKeepAliveCount(
    requested_registration_target_cloud_keep_alive_count,
    lidar_localization::kDefaultRegistrationTargetCloudKeepAliveCount);
  registration_target_cloud_keep_alive_count_ =
    registration_target_cloud_keep_alive_count.value;
  if (registration_target_cloud_keep_alive_count.was_adjusted) {
    RCLCPP_WARN(
      get_logger(),
      "registration_target_cloud_keep_alive_count must be >= 0; using %zu",
      registration_target_cloud_keep_alive_count_);
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
  get_parameter(
    "enable_seed_correction_guard", measurement_gate_config_.enable_seed_correction_guard);
  get_parameter(
    "seed_correction_guard_translation_m",
    measurement_gate_config_.seed_correction_guard_translation_m);
  get_parameter(
    "seed_correction_guard_yaw_deg", measurement_gate_config_.seed_correction_guard_yaw_deg);
  get_parameter(
    "seed_correction_guard_release_rejections",
    measurement_gate_config_.seed_correction_guard_release_rejections);
  get_parameter(
    "seed_correction_guard_warmup_accepts",
    measurement_gate_config_.seed_correction_guard_warmup_accepts);
  get_parameter(
    "enable_odom_tf_prediction_correction_guard",
    measurement_gate_config_.enable_odom_tf_prediction_correction_guard);
  get_parameter(
    "odom_tf_prediction_correction_guard_translation_m",
    measurement_gate_config_.odom_tf_prediction_correction_guard_translation_m);
  get_parameter(
    "odom_tf_prediction_correction_guard_yaw_deg",
    measurement_gate_config_.odom_tf_prediction_correction_guard_yaw_deg);
  get_parameter(
    "enable_odom_tf_prediction_recovery_correction_guard",
    measurement_gate_config_.enable_odom_tf_prediction_recovery_correction_guard);
  get_parameter(
    "odom_tf_prediction_recovery_min_rejections",
    measurement_gate_config_.odom_tf_prediction_recovery_min_rejections);
  get_parameter(
    "odom_tf_prediction_recovery_max_fitness",
    measurement_gate_config_.odom_tf_prediction_recovery_max_fitness);
  get_parameter(
    "odom_tf_prediction_recovery_guard_translation_m",
    measurement_gate_config_.odom_tf_prediction_recovery_guard_translation_m);
  get_parameter(
    "odom_tf_prediction_recovery_guard_yaw_deg",
    measurement_gate_config_.odom_tf_prediction_recovery_guard_yaw_deg);
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
    "reinitialization_request_clear_ok_samples",
    reinitialization_request_clear_ok_samples_);
  reinitialization_request_clear_ok_samples_ =
    std::max(1, reinitialization_request_clear_ok_samples_);
  get_parameter(
    "reinitialization_request_clear_max_fitness",
    reinitialization_request_clear_max_fitness_);
  reinitialization_request_clear_max_fitness_ =
    std::max(0.0, reinitialization_request_clear_max_fitness_);
  get_parameter(
    "reinitialization_request_clear_max_correction_translation_m",
    reinitialization_request_clear_max_correction_translation_m_);
  reinitialization_request_clear_max_correction_translation_m_ =
    std::max(0.0, reinitialization_request_clear_max_correction_translation_m_);
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
  int requested_weak_overlap_min_filtered_points =
    static_cast<int>(failure_taxonomy_params_.weak_overlap_min_filtered_points);
  get_parameter(
    "diagnostics_weak_overlap_min_filtered_points",
    requested_weak_overlap_min_filtered_points);
  failure_taxonomy_params_.weak_overlap_min_filtered_points =
    static_cast<std::size_t>(std::max(0, requested_weak_overlap_min_filtered_points));
  get_parameter(
    "diagnostics_stale_prediction_min_gap_sec",
    failure_taxonomy_params_.stale_prediction_min_gap_sec);
  get_parameter(
    "diagnostics_overload_alignment_time_sec",
    failure_taxonomy_params_.overload_alignment_time_sec);
  std::string pose_covariance_mode{"error_floor"};
  get_parameter("pose_covariance_mode", pose_covariance_mode);
  use_error_floor_covariance_ = pose_covariance_mode != "fitness_scaled";
  if (use_error_floor_covariance_ && pose_covariance_mode != "error_floor") {
    RCLCPP_WARN(
      get_logger(), "Unknown pose_covariance_mode '%s'; using error_floor",
      pose_covariance_mode.c_str());
  }
  get_parameter(
    "covariance_xy_floor_std_m", error_floor_covariance_params_.xy_floor_std_m);
  get_parameter(
    "covariance_xy_std_per_fitness_m",
    error_floor_covariance_params_.xy_std_per_fitness_m);
  get_parameter(
    "covariance_xy_max_std_m", error_floor_covariance_params_.xy_max_std_m);
  get_parameter(
    "covariance_z_floor_std_m", error_floor_covariance_params_.z_floor_std_m);
  get_parameter(
    "covariance_yaw_floor_std_deg",
    error_floor_covariance_params_.yaw_floor_std_deg);
  get_parameter(
    "covariance_yaw_std_per_fitness_deg",
    error_floor_covariance_params_.yaw_std_per_fitness_deg);
  get_parameter(
    "covariance_yaw_max_std_deg", error_floor_covariance_params_.yaw_max_std_deg);
  get_parameter(
    "covariance_roll_pitch_floor_std_deg",
    error_floor_covariance_params_.roll_pitch_floor_std_deg);
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
  RCLCPP_INFO(get_logger(),"use_odom_tf_prediction: %d", use_odom_tf_prediction_);
  RCLCPP_INFO(
    get_logger(), "constrain_odom_tf_prediction_to_planar: %d",
    constrain_odom_tf_prediction_to_planar_);
  RCLCPP_INFO(
    get_logger(), "constrain_odom_tf_prediction_height_only: %d",
    constrain_odom_tf_prediction_height_only_);
  RCLCPP_INFO(
    get_logger(),"publish_bridge_pose_when_lost: %d", publish_bridge_pose_when_lost_);
  RCLCPP_INFO(
    get_logger(), "enable_map_odom_anchor_fitness_gate: %d",
    enable_map_odom_anchor_fitness_gate_);
  RCLCPP_INFO(
    get_logger(), "map_odom_anchor_max_fitness: %lf",
    map_odom_anchor_max_fitness_);
  RCLCPP_INFO(
    get_logger(), "map_odom_anchor_max_correction_rotation_deg: %lf",
    map_odom_anchor_max_correction_rotation_deg_);
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
  RCLCPP_INFO(
    get_logger(), "scan_time_range_max_duration_ratio: %lf",
    scan_time_range_max_duration_ratio_);
  RCLCPP_INFO(get_logger(),"cloud_queue_depth: %d", cloud_queue_depth_);
  RCLCPP_INFO(get_logger(),"imu_queue_depth: %d", imu_queue_depth_);
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
    get_logger(),
    "imu_seed_consistency_gate: enabled=%d translation=%.3f m rotation=%.3f deg passes=%zu",
    imu_seed_consistency_gate_enabled_,
    imu_seed_consistency_params_.max_translation_error_m,
    imu_seed_consistency_params_.max_rotation_error_deg,
    imu_seed_consistency_params_.required_consecutive_passes);
  RCLCPP_INFO(get_logger(), "imu_dual_queue_enabled: %d", imu_dual_queue_enabled_);
  RCLCPP_INFO(
    get_logger(), "imu_preintegration_use_base_frame_transform: %d",
    imu_preintegration_use_base_frame_transform_);
  RCLCPP_INFO(get_logger(), "imu_accel_scale: %.8f", imu_accel_scale_);
  RCLCPP_INFO(get_logger(), "use_continuous_time_deskew: %d", use_continuous_time_deskew_);
  RCLCPP_INFO(
    get_logger(), "continuous_time_deskew_mode: %s", continuous_time_deskew_mode_.c_str());
  RCLCPP_INFO(
    get_logger(), "continuous_time_cloud_stamp_reference: %s",
    continuous_time_cloud_stamp_reference_.c_str());
  RCLCPP_INFO(
    get_logger(), "continuous_time_deskew_reference_time_sec: %lf",
    continuous_time_deskew_reference_time_sec_);
  RCLCPP_INFO(
    get_logger(), "continuous_time_pose_history_duration_sec: %lf",
    continuous_time_pose_history_duration_sec_);
  RCLCPP_INFO(
    get_logger(), "enable_localizability_guard: %d", enable_localizability_guard_);
  RCLCPP_INFO(
    get_logger(), "localizability_min_xy_eigen_ratio: %lf",
    localizability_min_xy_eigen_ratio_);
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
    get_logger(), "local_map_update_distance: %lf", local_map_update_distance_);
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
    get_logger(), "seed_correction_guard: enabled=%d translation_m=%lf yaw_deg=%lf "
    "release_rejections=%d warmup_accepts=%d",
    measurement_gate_config_.enable_seed_correction_guard,
    measurement_gate_config_.seed_correction_guard_translation_m,
    measurement_gate_config_.seed_correction_guard_yaw_deg,
    measurement_gate_config_.seed_correction_guard_release_rejections,
    measurement_gate_config_.seed_correction_guard_warmup_accepts);
  RCLCPP_INFO(
    get_logger(), "enable_odom_tf_prediction_correction_guard: %d",
    measurement_gate_config_.enable_odom_tf_prediction_correction_guard);
  RCLCPP_INFO(
    get_logger(), "odom_tf_prediction_correction_guard_translation_m: %lf",
    measurement_gate_config_.odom_tf_prediction_correction_guard_translation_m);
  RCLCPP_INFO(
    get_logger(), "odom_tf_prediction_correction_guard_yaw_deg: %lf",
    measurement_gate_config_.odom_tf_prediction_correction_guard_yaw_deg);
  RCLCPP_INFO(
    get_logger(), "enable_odom_tf_prediction_recovery_correction_guard: %d",
    measurement_gate_config_.enable_odom_tf_prediction_recovery_correction_guard);
  RCLCPP_INFO(
    get_logger(), "odom_tf_prediction_recovery_min_rejections: %d",
    measurement_gate_config_.odom_tf_prediction_recovery_min_rejections);
  RCLCPP_INFO(
    get_logger(), "odom_tf_prediction_recovery_max_fitness: %lf",
    measurement_gate_config_.odom_tf_prediction_recovery_max_fitness);
  RCLCPP_INFO(
    get_logger(), "odom_tf_prediction_recovery_guard_translation_m: %lf",
    measurement_gate_config_.odom_tf_prediction_recovery_guard_translation_m);
  RCLCPP_INFO(
    get_logger(), "odom_tf_prediction_recovery_guard_yaw_deg: %lf",
    measurement_gate_config_.odom_tf_prediction_recovery_guard_yaw_deg);
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
    get_logger(), "reinitialization_request_clear_ok_samples: %d",
    reinitialization_request_clear_ok_samples_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_request_clear_max_fitness: %lf",
    reinitialization_request_clear_max_fitness_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_request_clear_max_correction_translation_m: %lf",
    reinitialization_request_clear_max_correction_translation_m_);
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
  RCLCPP_INFO(
    get_logger(), "imu_prediction_correction_guard_warmup_accepts: %d",
    imu_prediction_correction_guard_warmup_accepts_);
  RCLCPP_INFO(get_logger(),"enable_timer_publishing: %d", enable_timer_publishing_);
  RCLCPP_INFO(get_logger(),"pose_publish_frequency: %lf", pose_publish_frequency_);
}

