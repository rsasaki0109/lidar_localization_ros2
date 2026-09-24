#include "component_internal.hpp"
void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  odom_bridge_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "odom_bridge_pose",
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

  rclcpp::SubscriptionOptions initial_pose_subscription_options;
  initial_pose_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  initial_pose_subscription_options.callback_group = initial_pose_callback_group_;
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1),
    initial_pose_subscription_options);
  RCLCPP_INFO(
    get_logger(),
    "Initial pose subscription uses a dedicated callback group");

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

  rclcpp::SubscriptionOptions imu_subscription_options;
  if (use_imu_preintegration_ && !use_imu_) {
    imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_subscription_options.callback_group = imu_callback_group_;
    RCLCPP_INFO(
      get_logger(),
      "IMU preintegration subscription uses a dedicated callback group");
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS().keep_last(imu_queue_depth_),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1),
    imu_subscription_options);

  if (enable_timer_publishing_) {
    pose_publish_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pose_publish_timer_ = create_wall_timer(
      lidar_localization::posePublishPeriodFromFrequencyHz(pose_publish_frequency_),
      std::bind(&PCLLocalization::timerPublishPose, this),
      pose_publish_callback_group_);
    RCLCPP_INFO(
      get_logger(),
      "Pose publication timer uses a dedicated callback group");
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
    boost::shared_ptr<lidar_localization::DiagnosticNdtOmp<pcl::PointXYZI, pcl::PointXYZI>>
      ndt_omp(new lidar_localization::DiagnosticNdtOmp<pcl::PointXYZI, pcl::PointXYZI>());
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
  auto state_lock = callback_state_coordinator_.lockState();
  if (shutting_down_.load(std::memory_order_acquire)) {return;}
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  const auto admission = lidar_localization::decideInitialPoseAdmission(
    lidar_localization::InitialPoseAdmissionInput{
      global_frame_id_,
      msg->header.frame_id,
      msg->pose.pose,
      map_recieved_});
  if (!admission.accepted) {
    if (
      admission.status ==
      lidar_localization::InitialPoseAdmissionStatus::kRejectedFrameMismatch)
    {
      RCLCPP_WARN(
        get_logger(),
        "initial pose ignored: frame_id '%s' does not match global_frame_id '%s'. "
        "RViz 2D Pose Estimate must publish /initialpose in the map frame.",
        msg->header.frame_id.c_str(),
        global_frame_id_.c_str());
    } else if (
      admission.status ==
      lidar_localization::InitialPoseAdmissionStatus::kRejectedNonFinitePose)
    {
      RCLCPP_WARN(get_logger(), "initial pose ignored: pose contains non-finite values");
    }
    return;
  }
  if (admission.warn_map_not_ready) {
    RCLCPP_WARN(
      get_logger(),
      "initial pose accepted before map is ready; scans will wait until map load completes");
  }
  initialpose_recieved_ = true;
  last_initial_pose_stamp_sec_ = stamp_to_sec(msg->header.stamp);
  callback_state_coordinator_.advanceInitialPoseGeneration();
  corrent_pose_with_cov_stamped_ptr_ = msg;
  {
    std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
    imu_preintegration_fallback_mode_ = false;
    imu_guard_warmup_accepts_remaining_.store(
      imu_prediction_correction_guard_warmup_accepts_, std::memory_order_release);
    resetImuPreintegrationSampleCounters();
  }
  reinitialization_request_latched_ = false;
  reinitialization_request_latch_reason_ = "not_requested";
  reinitialization_request_latch_score_ = 0.0;
  reinitialization_request_latch_stamp_sec_ = 0.0;
  reinitialization_request_latch_consecutive_ok_samples_ = 0;
  recovery_supervisor_state_ = RecoverySupervisorState::kTracking;
  recovery_supervisor_action_ = "initial_pose_reset";
  recovery_supervisor_state_entered_stamp_sec_ = stamp_to_sec(msg->header.stamp);
  consecutive_crop_failures_ = 0;
  crop_failure_guard_active_ = false;
  last_crop_out_of_bounds_log_time_ = std::chrono::steady_clock::time_point{};
  last_crop_failure_streak_log_time_ = std::chrono::steady_clock::time_point{};
  // The initialpose is authoritative for every anchor: if the last-accepted
  // pose stayed at the pre-reset track, the crop-failure guard and the
  // retry-from-last-pose path would re-seed alignment there after a few
  // rejections and silently undo the reset.
  have_last_accepted_pose_ = true;
  last_accepted_pose_matrix_ = currentPoseMatrix();
  last_accepted_pose_time_sec_ = stamp_to_sec(msg->header.stamp);
  consecutive_rejected_updates_ = 0;
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

  {
    std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
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
      imu_seed_consistency_state_ = {};
      latest_imu_open_loop_prediction_available_ = false;
      imu_prediction_smoother_.initializeState(
        imu_smoother_.position(), imu_smoother_.rotation(), imu_smoother_.velocity(),
        imu_smoother_.gyroBias(), imu_smoother_.accelBias(),
        last_scan_stamp_for_imu_);
      imu_dual_queue_.clear();
      imu_optimization_anchor_sample_.reset();
      imu_prediction_anchor_sample_.reset();
      latest_dual_queue_integrated_stamp_ = last_scan_stamp_for_imu_;
      RCLCPP_INFO(get_logger(), "IMU preintegration smoother initialized from initial pose");
    }
  }
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
  if (publishPoseTransform(msg->header.stamp, corrent_pose_with_cov_stamped_ptr_->pose.pose)) {
    if (enable_map_odom_tf_) {
      RCLCPP_INFO(
        get_logger(),
        "published %s -> %s TF from initial pose",
        global_frame_id_.c_str(),
        odom_frame_id_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "published %s -> %s TF from initial pose",
        global_frame_id_.c_str(),
        base_frame_id_.c_str());
    }
  } else if (enable_map_odom_tf_) {
    RCLCPP_WARN(
      get_logger(),
      "initial pose accepted but %s -> %s TF was not published; ensure %s -> %s TF exists",
      global_frame_id_.c_str(),
      odom_frame_id_.c_str(),
      odom_frame_id_.c_str(),
      base_frame_id_.c_str());
  }

  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto state_lock = callback_state_coordinator_.lockState();
  if (shutting_down_.load(std::memory_order_acquire)) {return;}
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
    lidar_localization::keepRegistrationCloudAlive(
      recent_target_clouds_, filtered_cloud_ptr, registration_target_cloud_keep_alive_count_);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
    lidar_localization::keepRegistrationCloudAlive(
      recent_target_clouds_, map_cloud_ptr, registration_target_cloud_keep_alive_count_);
  }

  // The map topic replaces the whole registration target.
  local_map_target_cached_ = false;
  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto state_lock = callback_state_coordinator_.lockState();
  if (shutting_down_.load(std::memory_order_acquire)) {return;}

  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  const auto admission = lidar_localization::decideOdomAdmission(
    lidar_localization::OdomAdmissionInput{
      use_odom_,
      initialpose_recieved_,
      static_cast<bool>(corrent_pose_with_cov_stamped_ptr_),
      dt_odom,
      corrent_pose_with_cov_stamped_ptr_
        ? lidar_localization::isPoseFinite(corrent_pose_with_cov_stamped_ptr_->pose.pose)
        : false,
      1.0});
  if (!admission.accepted) {
    if (admission.status == lidar_localization::OdomAdmissionStatus::kIntervalTooLarge) {
      last_odom_received_time_ = current_odom_received_time;
      RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    } else if (admission.status == lidar_localization::OdomAdmissionStatus::kIntervalNegative) {
      RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    } else if (
      admission.status == lidar_localization::OdomAdmissionStatus::kWaitingForInitialPose)
    {
      RCLCPP_DEBUG(get_logger(), "odomReceived before initial pose; skipping");
    } else if (
      admission.status == lidar_localization::OdomAdmissionStatus::kMissingCurrentPose)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "odomReceived before current pose is initialized; skipping");
    } else if (
      admission.status == lidar_localization::OdomAdmissionStatus::kCurrentPoseNonFinite)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "current pose is non-finite; skipping odom integration");
    }
    return;
  }

  last_odom_received_time_ = current_odom_received_time;
  RCLCPP_DEBUG(get_logger(), "odomReceived");

  const geometry_msgs::msg::Pose previous_pose =
    corrent_pose_with_cov_stamped_ptr_->pose.pose;

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

  if (!lidar_localization::isPoseFinite(corrent_pose_with_cov_stamped_ptr_->pose.pose)) {
    corrent_pose_with_cov_stamped_ptr_->pose.pose = previous_pose;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "odom integration produced non-finite pose; keeping previous pose");
  }
}

void PCLLocalization::twistReceived(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  auto state_lock = callback_state_coordinator_.lockState();
  if (shutting_down_.load(std::memory_order_acquire)) {return;}
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
  if (shutting_down_.load(std::memory_order_acquire)) {return;}
  // IMU preintegration buffering (always runs if enabled, independent of use_imu_)
  Eigen::Vector3d preintegration_gyro(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3d preintegration_accel(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  bool preintegration_sample_ready = true;
  {
    std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
    const bool collect_preintegration_sample =
      use_imu_preintegration_ && !imu_preintegration_fallback_mode_;
    if (collect_preintegration_sample) {
      ++imu_preintegration_received_sample_count_since_scan_;
    }

    if (
      collect_preintegration_sample &&
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
        ++imu_preintegration_transform_failure_count_since_scan_;
        ++imu_preintegration_skipped_sample_count_since_scan_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Failed to transform IMU preintegration sample from %s to %s: %s",
          msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
      }
    }

    if (
      collect_preintegration_sample &&
      preintegration_sample_ready &&
      (!preintegration_gyro.allFinite() || !preintegration_accel.allFinite()))
    {
      preintegration_sample_ready = false;
      ++imu_preintegration_non_finite_sample_count_since_scan_;
      ++imu_preintegration_skipped_sample_count_since_scan_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping non-finite IMU preintegration sample");
    }

    if (collect_preintegration_sample && preintegration_sample_ready) {
      preintegration_accel = lidar_localization::scaleImuAcceleration(
        preintegration_accel, imu_accel_scale_);
    }

    if (collect_preintegration_sample && preintegration_sample_ready) {
      double stamp_sec = stamp_to_sec(msg->header.stamp);
      const double dt = last_imu_stamp_ > 0.0 ? stamp_sec - last_imu_stamp_ : 0.0;

      if (imu_dual_queue_enabled_) {
        if (!imu_dual_queue_.push(
            lidar_localization::TimestampedImuSample{
              stamp_sec, preintegration_gyro, preintegration_accel}))
        {
          ++imu_preintegration_invalid_dt_count_since_scan_;
          ++imu_preintegration_skipped_sample_count_since_scan_;
        } else {
          imu_preintegration_skipped_sample_count_since_scan_ +=
            imu_dual_queue_.trimTo(static_cast<std::size_t>(imu_queue_depth_));
        }
        // Legacy single-stream integration is retained as the A/B baseline.
      } else if (imu_smoother_.isInitialized() && last_imu_stamp_ > 0.0) {
        imu_preintegration_last_dt_sec_ = dt;
        if (lidar_localization::isValidImuPreintegrationDt(dt)) {
          imu_smoother_.integrateImu(preintegration_gyro, preintegration_accel, dt);
          ++imu_preintegration_integrated_sample_count_since_scan_;
        } else {
          ++imu_preintegration_invalid_dt_count_since_scan_;
          ++imu_preintegration_skipped_sample_count_since_scan_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
          "Skipping IMU preintegration sample with invalid dt %.6f sec",
          dt);
        }
      }

      // Keep an optimization-correction-independent, rotation-only pose history.
      // It is used by the experimental piecewise deskew mode. Translation remains
      // zero here intentionally: integrating accelerometer translation without a
      // timestamped bias, gravity, and velocity state is less safe than omitting it.
      if (
        !continuous_time_imu_pose_history_.empty() && last_imu_stamp_ > 0.0 &&
        lidar_localization::isValidImuPreintegrationDt(dt)) {
        const Eigen::Vector3d deskew_gyro =
          preintegration_gyro - imu_smoother_.gyro_bias_;
        const Eigen::Vector3f rotation_vector =
          deskew_gyro.cast<float>() * static_cast<float>(dt);
        const float angle = rotation_vector.norm();
        if (std::isfinite(angle) && angle > 1e-9f) {
          const Eigen::Quaternionf delta(
            Eigen::AngleAxisf(angle, rotation_vector / angle));
          continuous_time_imu_orientation_ =
            (continuous_time_imu_orientation_ * delta).normalized();
        }
      } else {
        continuous_time_imu_orientation_ = Eigen::Quaternionf::Identity();
        continuous_time_imu_pose_history_.clear();
      }
      lidar_localization::TimestampedPose history_sample;
      history_sample.stamp_sec = stamp_sec;
      history_sample.pose.block<3, 3>(0, 0) =
        continuous_time_imu_orientation_.toRotationMatrix();
      continuous_time_imu_pose_history_.push_back(history_sample);
      const double oldest_stamp_to_keep =
        stamp_sec - continuous_time_pose_history_duration_sec_;
      while (
        continuous_time_imu_pose_history_.size() > 2 &&
        continuous_time_imu_pose_history_[1].stamp_sec < oldest_stamp_to_keep)
      {
        continuous_time_imu_pose_history_.pop_front();
      }
      last_imu_stamp_ = stamp_sec;
    }
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
  auto state_lock = callback_state_coordinator_.lockState();
  if (shutting_down_.load(std::memory_order_acquire)) {return;}
  double scan_stamp_sec = 0.0;
  if (!admitScanMessage(msg, &scan_stamp_sec)) {
    return;
  }
  // Odom bridge: keep map -> odom alive (re-stamped from the last accepted
  // match) on every admitted scan callback, whether or not this particular
  // scan ends up accepted below. See republishFrozenMapToOdomTransform and the
  // freeze_as_last_good guard on publishMapToOdomTransform.
  republishFrozenMapToOdomTransform(msg->header.stamp);
  const PreparedScanCloud prepared_scan = prepareScanForRegistration(msg, scan_stamp_sec);
  if (!lidar_localization::isPreparedScanReady(prepared_scan.status)) {
    handleScanPreparationFailure(msg->header.stamp, prepared_scan, scan_stamp_sec);
    publishBridgePoseAsRejectedOutput(msg->header.stamp);
    return;
  }
  const std::size_t filtered_point_count = prepared_scan.filtered_point_count;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr = prepared_scan.cloud;
  setRegistrationSourceCloud(tmp_ptr);

  const std::uint64_t seed_generation =
    callback_state_coordinator_.initialPoseGeneration();
  const SelectedRegistrationSeed selected_seed =
    selectRegistrationSeed(msg->header.stamp, scan_stamp_sec);
  const bool imu_prediction_ready = selected_seed.imu_prediction_ready;
  const std::string registration_seed_source =
    lidar_localization::registrationSeedSourceName(selected_seed.source);
  Eigen::Matrix4f init_guess =
    refineSeedWithNdtInitializer(tmp_ptr, selected_seed.init_guess);
  const auto pipeline_result = runAlignmentPipelineForScan(
    init_guess,
    scan_stamp_sec,
    selected_seed.source,
    imu_prediction_ready,
    state_lock,
    seed_generation);

  if (
    shutting_down_.load(std::memory_order_acquire) ||
    !callback_state_coordinator_.initialPoseGenerationMatches(seed_generation))
  {
    // An /initialpose was accepted while this scan was being aligned; the seed
    // (and therefore the result) belongs to the pre-reset belief, so applying
    // it would silently undo the reset.
    RCLCPP_INFO(
      get_logger(),
      "discarding alignment result seeded before the latest /initialpose");
    return;
  }

  logAlignmentPipelineRecovery(pipeline_result);
  if (handleTerminalAlignmentPipelineResult(
      msg->header.stamp,
      pipeline_result,
      filtered_point_count,
      scan_stamp_sec,
      imu_prediction_ready,
      registration_seed_source))
  {
    publishBridgePoseAsRejectedOutput(msg->header.stamp);
    return;
  }
  if (!applyAcceptedAlignmentPipelineResult(
      msg->header.stamp,
      pipeline_result,
      filtered_point_count,
      scan_stamp_sec,
      imu_prediction_ready,
      registration_seed_source))
  {
    publishBridgePoseAsRejectedOutput(msg->header.stamp);
    return;
  }

  printAlignmentDebugInfo(init_guess, pipeline_result.selected_attempt, filtered_point_count);
}

