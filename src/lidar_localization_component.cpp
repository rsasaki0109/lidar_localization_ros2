#include <lidar_localization/lidar_localization_component.hpp>
#include <chrono>
#include <cstring>

#include <pcl/common/common.h>

namespace
{
double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool uses_filtered_target(const std::string & registration_method)
{
  return registration_method == "GICP" || registration_method == "GICP_OMP" ||
         registration_method == "SMALL_GICP" || registration_method == "SMALL_VGICP";
}

bool supports_ndt_initializer(const std::string & registration_method)
{
  return registration_method == "GICP" || registration_method == "GICP_OMP" ||
         registration_method == "SMALL_GICP" || registration_method == "SMALL_VGICP";
}

constexpr std::size_t kRegistrationSourceCloudKeepAliveCount = 4096;
constexpr std::size_t kRegistrationTargetCloudKeepAliveCount = 4096;
constexpr double kImuSmootherMeasurementTranslationGuardM = 2.0;
constexpr double kImuSmootherMeasurementRotationGuardDeg = 10.0;

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

double rotation_delta_deg(
  const Eigen::Matrix3f & reference_rotation,
  const Eigen::Matrix3f & candidate_rotation)
{
  const Eigen::Matrix3d relative_rotation =
    reference_rotation.cast<double>().transpose() * candidate_rotation.cast<double>();
  const double trace =
    std::clamp((relative_rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(trace) * 180.0 / M_PI;
}

template<typename FieldContainerT>
bool has_field(const FieldContainerT & fields, const std::string & field_name)
{
  return std::any_of(
    fields.begin(), fields.end(),
    [&field_name](const auto & field) {
      return field.name == field_name;
    });
}

const sensor_msgs::msg::PointField * find_field(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  const std::string & field_name)
{
  const auto it = std::find_if(
    fields.begin(), fields.end(),
    [&field_name](const auto & field) {
      return field.name == field_name;
    });
  return it == fields.end() ? nullptr : &(*it);
}

bool read_field_as_float(
  const uint8_t * point_data,
  const sensor_msgs::msg::PointField & field,
  float * value)
{
  const uint8_t * field_ptr = point_data + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      *value = static_cast<float>(*reinterpret_cast<const int8_t *>(field_ptr));
      return true;
    case sensor_msgs::msg::PointField::UINT8:
      *value = static_cast<float>(*reinterpret_cast<const uint8_t *>(field_ptr));
      return true;
    case sensor_msgs::msg::PointField::INT16: {
      int16_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    case sensor_msgs::msg::PointField::FLOAT32:
      std::memcpy(value, field_ptr, sizeof(float));
      return true;
    case sensor_msgs::msg::PointField::FLOAT64: {
      double raw;
      std::memcpy(&raw, field_ptr, sizeof(raw));
      *value = static_cast<float>(raw);
      return true;
    }
    default:
      return false;
  }
}

void copy_xyz_to_xyzi(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  output.clear();
  output.reserve(input.size());
  output.header = input.header;
  output.width = input.width;
  output.height = input.height;
  output.is_dense = input.is_dense;

  for (const auto & point : input.points) {
    pcl::PointXYZI converted;
    converted.x = point.x;
    converted.y = point.y;
    converted.z = point.z;
    converted.intensity = 0.0f;
    output.push_back(converted);
  }
}

void convert_sensor_cloud_to_xyzi(
  const sensor_msgs::msg::PointCloud2 & input,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  const auto * x_field = find_field(input.fields, "x");
  const auto * y_field = find_field(input.fields, "y");
  const auto * z_field = find_field(input.fields, "z");
  const auto * intensity_field = find_field(input.fields, "intensity");
  if (!x_field || !y_field || !z_field) {
    output.clear();
    output.width = 0;
    output.height = 1;
    output.is_dense = false;
    pcl_conversions::toPCL(input.header, output.header);
    return;
  }

  output.clear();
  output.reserve(static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height));
  pcl_conversions::toPCL(input.header, output.header);
  output.is_dense = input.is_dense;

  const std::size_t point_count =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
  for (std::size_t point_idx = 0; point_idx < point_count; ++point_idx) {
    const uint8_t * point_data = input.data.data() + point_idx * input.point_step;
    pcl::PointXYZI point;
    float intensity = 0.0f;
    if (!read_field_as_float(point_data, *x_field, &point.x) ||
      !read_field_as_float(point_data, *y_field, &point.y) ||
      !read_field_as_float(point_data, *z_field, &point.z))
    {
      continue;
    }
    if (intensity_field) {
      read_field_as_float(point_data, *intensity_field, &intensity);
    }
    point.intensity = intensity;
    output.push_back(point);
  }
}

bool convert_pcl_cloud_to_xyzi(
  const pcl::PCLPointCloud2 & input,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  if (has_field(input.fields, "intensity")) {
    pcl::fromPCLPointCloud2(input, output);
    return true;
  }

  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  pcl::fromPCLPointCloud2(input, xyz_cloud);
  copy_xyz_to_xyzi(xyz_cloud, output);
  return false;
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
  declare_parameter("reinitialization_trigger_threshold", 0.95);
  declare_parameter("reinitialization_trigger_gap_scale_sec", 30.0);
  declare_parameter("reinitialization_trigger_seed_translation_scale_m", 100.0);
  declare_parameter("reinitialization_trigger_reject_streak_scale", 200.0);
  declare_parameter("reinitialization_trigger_fitness_explosion_threshold", 1000.0);
  declare_parameter("enable_timer_publishing", false);
  declare_parameter("pose_publish_frequency", 10.0);
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
    // load a pcd or ply file
    if (map_path_.rfind(".pcd") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Loading pcd map from: %s", map_path_.c_str());
      if (pcl::io::loadPCDFile(map_path_, raw_map_cloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load pcd file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else if (map_path_.rfind(".ply") != std::string::npos) {
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

    map_has_intensity = convert_pcl_cloud_to_xyzi(raw_map_cloud, *map_cloud_ptr);
    if (!map_has_intensity) {
      RCLCPP_WARN(
        get_logger(),
        "Map point cloud does not contain intensity. Falling back to xyz with zero intensity.");
    }

    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());
    if (!map_cloud_ptr->empty()) {
      pcl::getMinMax3D(*map_cloud_ptr, map_min_pt_, map_max_pt_);
      map_bounds_valid_ = true;
    } else {
      map_bounds_valid_ = false;
    }
    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    // Store full map for local cropping (GICP methods)
    use_local_map_crop_ = enable_local_map_crop_ || uses_filtered_target(registration_method_);
    if (use_local_map_crop_) {
      // Keep the raw full map and only voxel-filter the cropped local target per scan.
      full_map_cloud_ptr_ = map_cloud_ptr;
      RCLCPP_INFO(get_logger(), "Local map cropping enabled. Full map: %ld pts, radius: %.0fm",
                  full_map_cloud_ptr_->size(), local_map_radius_);
      // Avoid building a full-map NDT target here. It can overflow on city-scale maps.
      if (supports_ndt_initializer(registration_method_)) {
        use_ndt_initializer_ = true;
        ndt_init_scan_count_ = 0;
        ndt_initializer_.reset(
          new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt_initializer_->setStepSize(ndt_step_size_);
        ndt_initializer_->setResolution(ndt_resolution_);
        ndt_initializer_->setTransformationEpsilon(transform_epsilon_);
        ndt_initializer_->setNumThreads(
          ndt_num_threads_ > 0 ? ndt_num_threads_ : omp_get_max_threads());
        ndt_initializer_->setInputTarget(map_cloud_ptr);
        RCLCPP_INFO(get_logger(), "NDT initializer created (%d scans before GICP switch)",
                    ndt_init_scans_required_);
      } else {
        use_ndt_initializer_ = false;
        ndt_initializer_.reset();
      }
    } else {
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
  get_parameter("score_threshold", score_threshold_);
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
  get_parameter("cloud_queue_depth", cloud_queue_depth_);
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
  get_parameter("predict_pose_from_previous_delta", predict_pose_from_previous_delta_);
  get_parameter("enable_local_map_crop", enable_local_map_crop_);
  get_parameter("local_map_radius", local_map_radius_);
  int local_map_min_points = static_cast<int>(local_map_min_points_);
  get_parameter("local_map_min_points", local_map_min_points);
  local_map_min_points_ = static_cast<std::size_t>(std::max(local_map_min_points, 1));
  get_parameter("reject_above_score_threshold", reject_above_score_threshold_);
  get_parameter("enable_consistency_recovery_gate", enable_consistency_recovery_gate_);
  get_parameter("consistency_recovery_min_rejections", consistency_recovery_min_rejections_);
  get_parameter("consistency_recovery_score_margin", consistency_recovery_score_margin_);
  get_parameter(
    "consistency_recovery_max_translation_m",
    consistency_recovery_max_translation_m_);
  get_parameter("consistency_recovery_max_yaw_deg", consistency_recovery_max_yaw_deg_);
  get_parameter(
    "enable_post_reject_strict_score_threshold",
    enable_post_reject_strict_score_threshold_);
  get_parameter("post_reject_strict_min_rejections", post_reject_strict_min_rejections_);
  get_parameter("post_reject_strict_score_threshold", post_reject_strict_score_threshold_);
  get_parameter(
    "enable_open_loop_strict_score_threshold",
    enable_open_loop_strict_score_threshold_);
  get_parameter(
    "open_loop_strict_min_accepted_gap_sec",
    open_loop_strict_min_accepted_gap_sec_);
  get_parameter(
    "open_loop_strict_min_seed_translation_m",
    open_loop_strict_min_seed_translation_m_);
  get_parameter("open_loop_strict_score_threshold", open_loop_strict_score_threshold_);
  get_parameter(
    "enable_borderline_seed_rejection_gate",
    enable_borderline_seed_rejection_gate_);
  get_parameter("borderline_seed_gate_score_threshold", borderline_seed_gate_score_threshold_);
  get_parameter(
    "borderline_seed_gate_min_seed_translation_m",
    borderline_seed_gate_min_seed_translation_m_);
  get_parameter("enable_rejected_seed_update", enable_rejected_seed_update_);
  get_parameter("rejected_seed_update_min_rejections", rejected_seed_update_min_rejections_);
  get_parameter("rejected_seed_update_max_fitness", rejected_seed_update_max_fitness_);
  get_parameter(
    "rejected_seed_update_max_correction_translation_m",
    rejected_seed_update_max_correction_translation_m_);
  get_parameter(
    "rejected_seed_update_max_correction_yaw_deg",
    rejected_seed_update_max_correction_yaw_deg_);
  get_parameter(
    "enable_recovery_retry_from_last_pose",
    enable_recovery_retry_from_last_pose_);
  get_parameter(
    "recovery_retry_from_last_pose_min_rejections",
    recovery_retry_from_last_pose_min_rejections_);
  get_parameter(
    "recovery_retry_from_last_pose_max_accepted_gap_sec",
    recovery_retry_from_last_pose_max_accepted_gap_sec_);
  get_parameter(
    "recovery_retry_from_last_pose_max_seed_translation_m",
    recovery_retry_from_last_pose_max_seed_translation_m_);
  get_parameter(
    "enable_reinitialization_request_output",
    enable_reinitialization_request_output_);
  get_parameter(
    "reinitialization_trigger_threshold",
    reinitialization_trigger_threshold_);
  get_parameter(
    "reinitialization_trigger_gap_scale_sec",
    reinitialization_trigger_gap_scale_sec_);
  get_parameter(
    "reinitialization_trigger_seed_translation_scale_m",
    reinitialization_trigger_seed_translation_scale_m_);
  get_parameter(
    "reinitialization_trigger_reject_streak_scale",
    reinitialization_trigger_reject_streak_scale_);
  get_parameter(
    "reinitialization_trigger_fitness_explosion_threshold",
    reinitialization_trigger_fitness_explosion_threshold_);
  get_parameter("enable_timer_publishing", enable_timer_publishing_);
  get_parameter("pose_publish_frequency", pose_publish_frequency_);

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
    get_logger(), "reject_above_score_threshold: %d", reject_above_score_threshold_);
  RCLCPP_INFO(
    get_logger(), "enable_consistency_recovery_gate: %d",
    enable_consistency_recovery_gate_);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_min_rejections: %d",
    consistency_recovery_min_rejections_);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_score_margin: %lf",
    consistency_recovery_score_margin_);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_max_translation_m: %lf",
    consistency_recovery_max_translation_m_);
  RCLCPP_INFO(
    get_logger(), "consistency_recovery_max_yaw_deg: %lf",
    consistency_recovery_max_yaw_deg_);
  RCLCPP_INFO(
    get_logger(), "enable_post_reject_strict_score_threshold: %d",
    enable_post_reject_strict_score_threshold_);
  RCLCPP_INFO(
    get_logger(), "post_reject_strict_min_rejections: %d",
    post_reject_strict_min_rejections_);
  RCLCPP_INFO(
    get_logger(), "post_reject_strict_score_threshold: %lf",
    post_reject_strict_score_threshold_);
  RCLCPP_INFO(
    get_logger(), "enable_open_loop_strict_score_threshold: %d",
    enable_open_loop_strict_score_threshold_);
  RCLCPP_INFO(
    get_logger(), "open_loop_strict_min_accepted_gap_sec: %lf",
    open_loop_strict_min_accepted_gap_sec_);
  RCLCPP_INFO(
    get_logger(), "open_loop_strict_min_seed_translation_m: %lf",
    open_loop_strict_min_seed_translation_m_);
  RCLCPP_INFO(
    get_logger(), "open_loop_strict_score_threshold: %lf",
    open_loop_strict_score_threshold_);
  RCLCPP_INFO(
    get_logger(), "enable_borderline_seed_rejection_gate: %d",
    enable_borderline_seed_rejection_gate_);
  RCLCPP_INFO(
    get_logger(), "borderline_seed_gate_score_threshold: %lf",
    borderline_seed_gate_score_threshold_);
  RCLCPP_INFO(
    get_logger(), "borderline_seed_gate_min_seed_translation_m: %lf",
    borderline_seed_gate_min_seed_translation_m_);
  RCLCPP_INFO(get_logger(), "enable_rejected_seed_update: %d", enable_rejected_seed_update_);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_min_rejections: %d",
    rejected_seed_update_min_rejections_);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_max_fitness: %lf",
    rejected_seed_update_max_fitness_);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_max_correction_translation_m: %lf",
    rejected_seed_update_max_correction_translation_m_);
  RCLCPP_INFO(
    get_logger(), "rejected_seed_update_max_correction_yaw_deg: %lf",
    rejected_seed_update_max_correction_yaw_deg_);
  RCLCPP_INFO(
    get_logger(), "enable_recovery_retry_from_last_pose: %d",
    enable_recovery_retry_from_last_pose_);
  RCLCPP_INFO(
    get_logger(), "recovery_retry_from_last_pose_min_rejections: %d",
    recovery_retry_from_last_pose_min_rejections_);
  RCLCPP_INFO(
    get_logger(), "recovery_retry_from_last_pose_max_accepted_gap_sec: %lf",
    recovery_retry_from_last_pose_max_accepted_gap_sec_);
  RCLCPP_INFO(
    get_logger(), "recovery_retry_from_last_pose_max_seed_translation_m: %lf",
    recovery_retry_from_last_pose_max_seed_translation_m_);
  RCLCPP_INFO(
    get_logger(), "enable_reinitialization_request_output: %d",
    enable_reinitialization_request_output_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_threshold: %lf",
    reinitialization_trigger_threshold_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_gap_scale_sec: %lf",
    reinitialization_trigger_gap_scale_sec_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_seed_translation_scale_m: %lf",
    reinitialization_trigger_seed_translation_scale_m_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_reject_streak_scale: %lf",
    reinitialization_trigger_reject_streak_scale_);
  RCLCPP_INFO(
    get_logger(), "reinitialization_trigger_fitness_explosion_threshold: %lf",
    reinitialization_trigger_fitness_explosion_threshold_);
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
    auto period = std::chrono::duration<double>(1.0 / pose_publish_frequency_);
    pose_publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
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

  if (registration_method_ == "GICP") {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    gicp->setCorrespondenceRandomness(gicp_corr_randomness_);
    gicp->setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    pcl_registration_ = gicp;
    registration_ = pcl_registration_.get();
  }
  else if (registration_method_ == "NDT") {
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    pcl_registration_ = ndt;
    registration_ = pcl_registration_.get();
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    ndt_omp_registration_ = ndt_omp;
    registration_ = ndt_omp_registration_.get();
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    gicp_omp->setCorrespondenceRandomness(gicp_corr_randomness_);
    gicp_omp->setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    gicp_omp_registration_ = gicp_omp;
    registration_ = gicp_omp_registration_.get();
  }
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  else if (registration_method_ == "SMALL_GICP" || registration_method_ == "SMALL_VGICP") {
    small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>::Ptr reg(
      new small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>());
    reg->setTransformationEpsilon(transform_epsilon_);
    reg->setCorrespondenceRandomness(gicp_corr_randomness_);
    reg->setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    reg->setVoxelResolution(vgicp_voxel_resolution_);
    reg->setRegistrationType(registration_method_ == "SMALL_VGICP" ? "VGICP" : "GICP");
    if (ndt_num_threads_ > 0) {
      reg->setNumThreads(ndt_num_threads_);
    } else {
      reg->setNumThreads(omp_get_max_threads());
    }
    small_gicp_registration_ = reg;
    registration_ = small_gicp_registration_.get();
  }
#else
  else if (registration_method_ == "SMALL_GICP" || registration_method_ == "SMALL_VGICP") {
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

  if (uses_filtered_target(registration_method_)) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
    keep_cloud_alive(&recent_target_clouds_, filtered_cloud_ptr, kRegistrationTargetCloudKeepAliveCount);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
    keep_cloud_alive(&recent_target_clouds_, map_cloud_ptr, kRegistrationTargetCloudKeepAliveCount);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (shutting_down_) {return;}
  if (!use_odom_) {return;}
  RCLCPP_INFO(get_logger(), "odomReceived");

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
  if (use_imu_preintegration_ && !imu_preintegration_fallback_mode_) {
    double stamp_sec = stamp_to_sec(msg->header.stamp);
    const Eigen::Vector3d gyro(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    const Eigen::Vector3d accel(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // Feed to smoother for dead-reckoning
    if (imu_smoother_.isInitialized() && last_imu_stamp_ > 0.0) {
      double dt = stamp_sec - last_imu_stamp_;
      if (dt > 0.0 && dt < 0.5) {
        imu_smoother_.integrateImu(gyro, accel, dt);
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
  if (shutting_down_) {return;}
  if (!msg) {
    RCLCPP_WARN(get_logger(), "Received null point cloud message");
    return;
  }

  last_scan_ptr_ = msg;

  if (!map_recieved_ || !initialpose_recieved_) {return;}

  // Skip scans that arrive too soon after the last processed scan.
  if (min_scan_interval_sec_ > 0.0 && last_cloud_process_time_.nanoseconds() > 0) {
    const auto now = this->now();
    const double dt = (now - last_cloud_process_time_).seconds();
    if (dt < min_scan_interval_sec_) {
      return;
    }
  }
  last_cloud_process_time_ = this->now();

  RCLCPP_INFO(get_logger(), "cloudReceived");
  const double scan_stamp_sec = stamp_to_sec(msg->header.stamp);
  if (consecutive_crop_failures_ > 100) {
    if (!crop_failure_guard_active_) {
      crop_failure_guard_active_ = true;
      if (have_last_accepted_pose_) {
        predicted_pose_matrix_ = last_accepted_pose_matrix_;
        predicted_pose_time_sec_ = scan_stamp_sec;
      }
      RCLCPP_ERROR(
        get_logger(),
        "Activating crop failure guard after %d consecutive crop failures; "
        "dropping subsequent scans until a new initial pose arrives.",
        consecutive_crop_failures_);
    }
    return;
  }
  const bool cloud_has_intensity = has_field(msg->fields, "intensity");
  if (!cloud_has_intensity) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Input cloud does not contain intensity. Falling back to xyz with zero intensity.");
  }
  const bool can_direct_range_filter =
    !enable_scan_voxel_filter_ &&
    !use_imu_ &&
    msg->header.frame_id == base_frame_id_;
  std::size_t filtered_point_count = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr;
  if (can_direct_range_filter) {
    const auto * x_field = find_field(msg->fields, "x");
    const auto * y_field = find_field(msg->fields, "y");
    const auto * z_field = find_field(msg->fields, "z");
    const auto * intensity_field = find_field(msg->fields, "intensity");
    if (!x_field || !y_field || !z_field) {
      publishAlignmentStatus(
        msg->header.stamp,
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "scan_missing_xyz_field",
        false,
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        false);
      RCLCPP_ERROR(get_logger(), "Input scan is missing x/y/z fields.");
      advancePredictionWithoutMeasurement(stamp_to_sec(msg->header.stamp));
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> tmp;
    const std::size_t point_count =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    tmp.reserve(point_count);
    for (std::size_t point_idx = 0; point_idx < point_count; ++point_idx) {
      const uint8_t * point_data = msg->data.data() + point_idx * msg->point_step;
      pcl::PointXYZI point;
      float intensity = 0.0f;
      if (!read_field_as_float(point_data, *x_field, &point.x) ||
        !read_field_as_float(point_data, *y_field, &point.y) ||
        !read_field_as_float(point_data, *z_field, &point.z))
      {
        continue;
      }
      if (intensity_field) {
        read_field_as_float(point_data, *intensity_field, &intensity);
      }
      point.intensity = intensity;
      ++filtered_point_count;
      const double range = std::hypot(point.x, point.y);
      if (scan_min_range_ < range && range < scan_max_range_) {
        tmp.push_back(point);
      }
    }
    tmp_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    convert_sensor_cloud_to_xyzi(*msg, *cloud_ptr);

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
        RCLCPP_ERROR(
            this->get_logger(), "Could not transform %s to %s: %s",
            msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
        return;
      }

      Eigen::Matrix4f initial_transformation =
        tf2::transformToEigen(base_to_lidar_stamped.transform).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*cloud_ptr, *transformed_cloud, initial_transformation);
      cloud_ptr = transformed_cloud;
    }

    if (use_imu_) {
      double received_time = msg->header.stamp.sec +
        msg->header.stamp.nanosec * 1e-9;
      lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
    }

    pcl::PointCloud<pcl::PointXYZI> tmp;
    tmp.reserve(cloud_ptr->size());
    for (const auto & point : cloud_ptr->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double range = std::hypot(point.x, point.y);
      if (scan_min_range_ < range && range < scan_max_range_) {
        tmp.push_back(point);
      }
    }
    filtered_point_count = tmp.size();
    tmp_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(tmp));

    if (enable_scan_voxel_filter_ && !tmp_ptr->empty()) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
      pcl::VoxelGrid<pcl::PointXYZI> scan_voxel_grid_filter;
      scan_voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      scan_voxel_grid_filter.setInputCloud(tmp_ptr);
      scan_voxel_grid_filter.filter(*filtered_cloud_ptr);
      filtered_point_count = filtered_cloud_ptr->size();
      tmp_ptr = filtered_cloud_ptr;
    }
    cloud_ptr.reset();
  }

  if (tmp_ptr->empty()) {
    publishAlignmentStatus(
      msg->header.stamp,
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "filtered_scan_empty",
      false,
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      filtered_point_count,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      false);
    RCLCPP_WARN(get_logger(), "Filtered scan is empty after range filtering.");
	    advancePredictionWithoutMeasurement(stamp_to_sec(msg->header.stamp));
	    return;
  }
  registration_->setInputSource(tmp_ptr);
  keep_cloud_alive(&recent_source_clouds_, tmp_ptr, kRegistrationSourceCloudKeepAliveCount);

  Eigen::Matrix4f init_guess = currentPoseMatrix();
  bool init_guess_uses_prediction = false;
  bool imu_prediction_ready =
    use_imu_preintegration_ &&
    !imu_preintegration_fallback_mode_ &&
    imu_smoother_.isInitialized() &&
    last_imu_stamp_ > last_scan_stamp_for_imu_;
  if (imu_prediction_ready) {
    const Eigen::Matrix4f imu_init_guess = imu_smoother_.predictedPoseMatrix();
    if (imu_init_guess.allFinite()) {
      init_guess = imu_init_guess;
    } else {
      imu_prediction_ready = false;
      RCLCPP_WARN(
        get_logger(),
        "Ignoring non-finite IMU predicted pose and falling back to non-IMU seed.");
    }
  } else if (use_gtsam_smoother_ && gtsam_smoother_.isInitialized()) {
    init_guess = gtsam_smoother_.predictedPoseMatrix(last_ndt_roll_, last_ndt_pitch_);
  } else if (use_twist_ekf_ && twist_ekf_.isInitialized()) {
    init_guess = twist_ekf_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
  } else if (use_twist_prediction_ && have_last_accepted_pose_ && latest_twist_msg_) {
    const double dt = std::clamp(
      scan_stamp_sec - predicted_pose_time_sec_, 0.0, max_twist_prediction_dt_);
    init_guess = applyTwistPrediction(predicted_pose_matrix_, dt);
    init_guess_uses_prediction = true;
  } else if (predict_pose_from_previous_delta_ && have_last_accepted_pose_) {
    init_guess = predicted_pose_matrix_;
    init_guess_uses_prediction = true;
  }

  struct AlignmentAttempt
  {
    bool target_ready{false};
    bool has_converged{false};
    double alignment_time_sec{0.0};
    double fitness_score{std::numeric_limits<double>::quiet_NaN()};
    Eigen::Matrix4f init_guess{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f final_transformation{Eigen::Matrix4f::Identity()};
    double correction_translation_m{std::numeric_limits<double>::quiet_NaN()};
    double correction_yaw_deg{std::numeric_limits<double>::quiet_NaN()};
    double seed_translation_since_accept_m{std::numeric_limits<double>::quiet_NaN()};
    double seed_yaw_since_accept_deg{std::numeric_limits<double>::quiet_NaN()};
    double accepted_gap_sec{std::numeric_limits<double>::quiet_NaN()};
  };

  struct MeasurementGateResult
  {
    uint8_t status_level{diagnostic_msgs::msg::DiagnosticStatus::OK};
    std::string status_message{"ok"};
    bool reject_measurement{false};
    bool post_reject_strict_active{false};
    bool open_loop_strict_active{false};
    bool borderline_seed_gate_active{false};
    bool rejected_seed_update_applied{false};
    double effective_score_threshold{0.0};
  };

  auto set_input_target_for_pose = [&](const Eigen::Matrix4f & center_pose_matrix) -> bool {
      if (!use_local_map_crop_ || !full_map_cloud_ptr_) {
        return true;
      }

      const float cx = center_pose_matrix(0, 3);
      const float cy = center_pose_matrix(1, 3);
      const float cz = center_pose_matrix(2, 3);
      const auto steady_now = std::chrono::steady_clock::now();
      auto maybe_log_crop_failure_streak = [&]() {
          if (consecutive_crop_failures_ <= 100) {
            return;
          }
          if (
            last_crop_failure_streak_log_time_ == std::chrono::steady_clock::time_point{} ||
            steady_now - last_crop_failure_streak_log_time_ >= std::chrono::seconds(5))
          {
            last_crop_failure_streak_log_time_ = steady_now;
            RCLCPP_WARN(
              get_logger(),
              "Crop failure streak reached %d while target setup keeps failing.",
              consecutive_crop_failures_);
          }
        };
      if (
        map_bounds_valid_ &&
        (cx < map_min_pt_.x - local_map_radius_ || cx > map_max_pt_.x + local_map_radius_ ||
        cy < map_min_pt_.y - local_map_radius_ || cy > map_max_pt_.y + local_map_radius_))
      {
        ++consecutive_crop_failures_;
        maybe_log_crop_failure_streak();
        if (
          last_crop_out_of_bounds_log_time_ == std::chrono::steady_clock::time_point{} ||
          steady_now - last_crop_out_of_bounds_log_time_ >= std::chrono::seconds(5))
        {
          last_crop_out_of_bounds_log_time_ = steady_now;
          RCLCPP_WARN(
            get_logger(),
            "Crop center (%.1f, %.1f) is outside map bounds + radius, skipping alignment",
            static_cast<double>(cx), static_cast<double>(cy));
        }
        return false;
      }
      const float r2 = static_cast<float>(local_map_radius_ * local_map_radius_);
      pcl::PointCloud<pcl::PointXYZI>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZI>());
      local_map->reserve(full_map_cloud_ptr_->size() / 10);
      for (const auto & p : full_map_cloud_ptr_->points) {
        const float dx = p.x - cx;
        const float dy = p.y - cy;
        if (dx * dx + dy * dy <= r2) {
          local_map->push_back(p);
        }
      }
      if (local_map->size() < local_map_min_points_) {
        ++consecutive_crop_failures_;
        maybe_log_crop_failure_streak();
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
      if (filtered_local_map->size() >= local_map_min_points_) {
        registration_->setInputTarget(filtered_local_map);
        keep_cloud_alive(
          &recent_target_clouds_, filtered_local_map, kRegistrationTargetCloudKeepAliveCount);
      } else {
        registration_->setInputTarget(local_map);
        keep_cloud_alive(
          &recent_target_clouds_, local_map, kRegistrationTargetCloudKeepAliveCount);
      }
      consecutive_crop_failures_ = 0;
      crop_failure_guard_active_ = false;
      return true;
    };

  auto run_alignment_attempt = [&](const Eigen::Matrix4f & attempt_init_guess,
                                   const Eigen::Matrix4f & crop_center_pose_matrix) {
      AlignmentAttempt attempt;
      attempt.init_guess = attempt_init_guess;
      if (!set_input_target_for_pose(crop_center_pose_matrix)) {
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

      if (have_last_accepted_pose_) {
        const Eigen::Matrix4f seed_delta_matrix =
          last_accepted_pose_matrix_.inverse() * attempt_init_guess;
        attempt.seed_translation_since_accept_m = static_cast<double>(
          seed_delta_matrix.block<3, 1>(0, 3).norm());
        attempt.seed_yaw_since_accept_deg = std::abs(std::atan2(
          static_cast<double>(seed_delta_matrix(1, 0)),
          static_cast<double>(seed_delta_matrix(0, 0)))) * 180.0 / M_PI;
        attempt.accepted_gap_sec = std::max(0.0, scan_stamp_sec - last_accepted_pose_time_sec_);
      }

      if (attempt.has_converged) {
        attempt.final_transformation = registration_->getFinalTransformation();
        const Eigen::Matrix4f correction_matrix =
          attempt_init_guess.inverse() * attempt.final_transformation;
        attempt.correction_translation_m = static_cast<double>(
          correction_matrix.block<3, 1>(0, 3).norm());
        attempt.correction_yaw_deg = std::abs(std::atan2(
          static_cast<double>(correction_matrix(1, 0)),
          static_cast<double>(correction_matrix(0, 0)))) * 180.0 / M_PI;
      }

      return attempt;
    };

  auto evaluate_measurement_gate = [&](const AlignmentAttempt & attempt) {
      MeasurementGateResult gate;
      gate.effective_score_threshold = computeEffectiveScoreThreshold(
        attempt.accepted_gap_sec,
        attempt.seed_translation_since_accept_m,
        &gate.post_reject_strict_active,
        &gate.open_loop_strict_active);
      gate.borderline_seed_gate_active = computeBorderlineSeedGateActive(
        attempt.fitness_score,
        attempt.seed_translation_since_accept_m,
        gate.effective_score_threshold);

      if (attempt.fitness_score > gate.effective_score_threshold || gate.borderline_seed_gate_active) {
        gate.status_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        if (gate.borderline_seed_gate_active) {
          gate.status_message = reject_above_score_threshold_ ?
            "fitness_score_over_borderline_seed_gate_rejected" :
            "fitness_score_over_borderline_seed_gate";
        } else if (
          gate.open_loop_strict_active &&
          gate.effective_score_threshold == open_loop_strict_score_threshold_)
        {
          gate.status_message = reject_above_score_threshold_ ?
            "fitness_score_over_open_loop_strict_threshold_rejected" :
            "fitness_score_over_open_loop_strict_threshold";
        } else if (gate.post_reject_strict_active) {
          gate.status_message = reject_above_score_threshold_ ?
            "fitness_score_over_post_reject_strict_threshold_rejected" :
            "fitness_score_over_post_reject_strict_threshold";
        } else {
          gate.status_message = reject_above_score_threshold_ ?
            "fitness_score_over_threshold_rejected" :
            "fitness_score_over_threshold";
        }
        gate.reject_measurement = reject_above_score_threshold_;

        const bool recovery_candidate =
          enable_consistency_recovery_gate_ &&
          gate.reject_measurement &&
          static_cast<int>(consecutive_rejected_updates_) >= consistency_recovery_min_rejections_ &&
          attempt.fitness_score <= score_threshold_ + consistency_recovery_score_margin_ &&
          attempt.correction_translation_m <= consistency_recovery_max_translation_m_ &&
          attempt.correction_yaw_deg <= consistency_recovery_max_yaw_deg_;
        if (recovery_candidate) {
          gate.reject_measurement = false;
          gate.status_message = "fitness_score_over_threshold_consistency_recovered";
        }

        const bool rejected_seed_update_candidate =
          enable_rejected_seed_update_ &&
          gate.reject_measurement &&
          static_cast<int>(consecutive_rejected_updates_) >= rejected_seed_update_min_rejections_ &&
          attempt.fitness_score <= rejected_seed_update_max_fitness_ &&
          attempt.correction_translation_m <=
            rejected_seed_update_max_correction_translation_m_ &&
          attempt.correction_yaw_deg <= rejected_seed_update_max_correction_yaw_deg_;
        if (rejected_seed_update_candidate) {
          gate.rejected_seed_update_applied = true;
          gate.status_message += "_seeded";
        }
        RCLCPP_WARN(get_logger(), "The fitness score is over %lf.", gate.effective_score_threshold);
      }

      return gate;
    };

  // NDT initializer phase: use NDT for first N scans, then switch to GICP
  if (use_ndt_initializer_ && ndt_init_scan_count_ < ndt_init_scans_required_) {
    ndt_initializer_->setInputSource(tmp_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ndt_output(new pcl::PointCloud<pcl::PointXYZI>);
    ndt_initializer_->align(*ndt_output, init_guess);
    if (ndt_initializer_->hasConverged()) {
      init_guess = ndt_initializer_->getFinalTransformation();
      ++ndt_init_scan_count_;
      RCLCPP_INFO(get_logger(), "NDT init scan %d/%d fitness=%.3f",
                  ndt_init_scan_count_, ndt_init_scans_required_,
                  ndt_initializer_->getFitnessScore());
      if (ndt_init_scan_count_ >= ndt_init_scans_required_) {
        RCLCPP_INFO(get_logger(), "NDT init complete, switching to %s", registration_method_.c_str());
        ndt_initializer_.reset();
      }
    }
  }

  AlignmentAttempt selected_attempt = run_alignment_attempt(init_guess, init_guess);
  MeasurementGateResult gate_result;
  bool recovered_by_retry_from_last_pose = false;
  const double recovery_accepted_gap_sec =
    have_last_accepted_pose_ ? std::max(0.0, scan_stamp_sec - last_accepted_pose_time_sec_) :
    std::numeric_limits<double>::quiet_NaN();
  const bool allow_recovery_retry_from_last_pose =
    enable_recovery_retry_from_last_pose_ &&
    have_last_accepted_pose_ &&
    static_cast<int>(consecutive_rejected_updates_) >=
      recovery_retry_from_last_pose_min_rejections_;

  auto try_recovery_retry_from_last_pose = [&]() -> bool {
      if (!allow_recovery_retry_from_last_pose) {
        return false;
      }
      const double accepted_gap_sec = std::isfinite(selected_attempt.accepted_gap_sec) ?
        selected_attempt.accepted_gap_sec : recovery_accepted_gap_sec;
      if (
        !std::isfinite(accepted_gap_sec) ||
        accepted_gap_sec >
        recovery_retry_from_last_pose_max_accepted_gap_sec_)
      {
        return false;
      }
      if (
        std::isfinite(selected_attempt.seed_translation_since_accept_m) &&
        selected_attempt.seed_translation_since_accept_m >
        recovery_retry_from_last_pose_max_seed_translation_m_)
      {
        return false;
      }

      const AlignmentAttempt retry_attempt = run_alignment_attempt(
        last_accepted_pose_matrix_,
        last_accepted_pose_matrix_);
      if (!retry_attempt.target_ready || !retry_attempt.has_converged) {
        return false;
      }

      const MeasurementGateResult retry_gate = evaluate_measurement_gate(retry_attempt);
      if (retry_gate.reject_measurement) {
        return false;
      }

      selected_attempt = retry_attempt;
      gate_result = retry_gate;
      gate_result.status_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      gate_result.status_message = "recovery_retry_from_last_pose_recovered";
      gate_result.reject_measurement = false;
      gate_result.rejected_seed_update_applied = false;
      recovered_by_retry_from_last_pose = true;
      RCLCPP_INFO(
        get_logger(),
        "Recovery retry from last pose succeeded after %zu rejects: fitness=%.6f",
        consecutive_rejected_updates_,
        retry_attempt.fitness_score);
      return true;
    };

  if (!selected_attempt.target_ready) {
    if (!try_recovery_retry_from_last_pose()) {
      publishAlignmentStatus(
        msg->header.stamp,
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "local_map_crop_too_small",
        false,
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        filtered_point_count,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        imu_prediction_ready);
      advancePredictionWithoutMeasurement(scan_stamp_sec);
      return;
    }
  }

  if (!selected_attempt.has_converged) {
    if (!try_recovery_retry_from_last_pose()) {
      publishAlignmentStatus(
        msg->header.stamp,
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "registration_not_converged",
        selected_attempt.has_converged,
        selected_attempt.fitness_score,
        selected_attempt.alignment_time_sec,
        filtered_point_count,
        selected_attempt.correction_translation_m,
        selected_attempt.correction_yaw_deg,
        selected_attempt.seed_translation_since_accept_m,
        selected_attempt.seed_yaw_since_accept_deg,
        selected_attempt.accepted_gap_sec,
        imu_prediction_ready);
      RCLCPP_WARN(get_logger(), "The registration didn't converge.");
      advancePredictionWithoutMeasurement(scan_stamp_sec);
      return;
    }
  } else {
    gate_result = evaluate_measurement_gate(selected_attempt);
    if (gate_result.reject_measurement) {
      (void)try_recovery_retry_from_last_pose();
    }
  }

  bool has_converged = selected_attempt.has_converged;
  double fitness_score = selected_attempt.fitness_score;
  double correction_translation_m = selected_attempt.correction_translation_m;
  double correction_yaw_deg = selected_attempt.correction_yaw_deg;
  double seed_translation_since_accept_m = selected_attempt.seed_translation_since_accept_m;
  double seed_yaw_since_accept_deg = selected_attempt.seed_yaw_since_accept_deg;
  double accepted_gap_sec = selected_attempt.accepted_gap_sec;
  uint8_t status_level = gate_result.status_level;
  std::string status_message = gate_result.status_message;
  bool reject_measurement = gate_result.reject_measurement;
  bool rejected_seed_update_applied = gate_result.rejected_seed_update_applied;
  Eigen::Matrix4f final_transformation = selected_attempt.final_transformation;

  if (!has_converged) {
    publishAlignmentStatus(
      msg->header.stamp,
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "registration_not_converged",
      has_converged,
      fitness_score,
      selected_attempt.alignment_time_sec,
      filtered_point_count,
      correction_translation_m,
      correction_yaw_deg,
      seed_translation_since_accept_m,
      seed_yaw_since_accept_deg,
      accepted_gap_sec,
      imu_prediction_ready);
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    advancePredictionWithoutMeasurement(scan_stamp_sec);
    return;
  }
  if (recovered_by_retry_from_last_pose && status_level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    status_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  if (use_gtsam_smoother_) {
    // Extract roll, pitch, yaw from NDT rotation matrix
    Eigen::Matrix3f ndt_rot = final_transformation.block<3, 3>(0, 0);
    last_ndt_pitch_ = std::asin(-ndt_rot(2, 0));
    last_ndt_roll_ = std::atan2(ndt_rot(2, 1), ndt_rot(2, 2));
    double ndt_yaw = static_cast<double>(std::atan2(ndt_rot(1, 0), ndt_rot(0, 0)));
    double ndt_px = static_cast<double>(final_transformation(0, 3));
    double ndt_py = static_cast<double>(final_transformation(1, 3));
    double ndt_pz = static_cast<double>(final_transformation(2, 3));

    if (!gtsam_smoother_.isInitialized()) {
      gtsam_smoother_.initialize(ndt_px, ndt_py, ndt_pz, ndt_yaw, scan_stamp_sec);
    }

    bool gtsam_updated = gtsam_smoother_.update(
      ndt_px, ndt_py, ndt_pz, ndt_yaw, fitness_score, scan_stamp_sec);

    // Use smoothed state for published pose
    Eigen::Matrix4f gtsam_pose = gtsam_smoother_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
    Eigen::Matrix3d gtsam_rot_mat = gtsam_pose.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond quat_eig(gtsam_rot_mat);
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

    corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
    corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = gtsam_smoother_.px();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = gtsam_smoother_.py();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = gtsam_smoother_.pz();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;

    fillPoseCovariance(fitness_score);
    updatePredictionState(gtsam_pose, scan_stamp_sec);

    if (!gtsam_updated) {
      status_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status_message = "gtsam_update_rejected";
    }
    publishAlignmentStatus(
      msg->header.stamp, status_level, status_message,
      has_converged, fitness_score, selected_attempt.alignment_time_sec,
      filtered_point_count,
      correction_translation_m,
      correction_yaw_deg,
      seed_translation_since_accept_m,
      seed_yaw_since_accept_deg,
      accepted_gap_sec,
      imu_prediction_ready);
  } else if (use_imu_preintegration_ && last_imu_stamp_ > 0.0) {
    // Extract full 6DOF from registration result
    Eigen::Matrix3f ndt_rot = final_transformation.block<3, 3>(0, 0);
    double ndt_roll = static_cast<double>(std::atan2(ndt_rot(2, 1), ndt_rot(2, 2)));
    double ndt_pitch = static_cast<double>(std::asin(-std::clamp(ndt_rot(2, 0), -1.0f, 1.0f)));
    double ndt_yaw = static_cast<double>(std::atan2(ndt_rot(1, 0), ndt_rot(0, 0)));
    double ndt_px = static_cast<double>(final_transformation(0, 3));
    double ndt_py = static_cast<double>(final_transformation(1, 3));
    double ndt_pz = static_cast<double>(final_transformation(2, 3));

    if (!imu_smoother_.isInitialized()) {
      imu_smoother_.initialize(
        ndt_px, ndt_py, ndt_pz, ndt_roll, ndt_pitch, ndt_yaw,
        0.0, 0.0, 0.0, scan_stamp_sec);
    }

    bool imu_updated = true;
    bool imu_state_reset = false;
    bool imu_correction_guard_tripped = false;
    Eigen::Matrix4f imu_pose = final_transformation;
    if (
      !imu_preintegration_fallback_mode_ &&
      imu_prediction_ready &&
      (
        (std::isfinite(correction_translation_m) &&
        correction_translation_m > imu_prediction_correction_guard_translation_m_) ||
        (std::isfinite(correction_yaw_deg) &&
        correction_yaw_deg > imu_prediction_correction_guard_yaw_deg_)))
    {
      RCLCPP_WARN(
        get_logger(),
        "IMU prediction required a large measurement correction (translation=%.3f m, yaw=%.3f deg). Disabling IMU preintegration for the remainder of this run.",
        correction_translation_m,
        correction_yaw_deg);
      imu_preintegration_fallback_mode_ = true;
      imu_state_reset = true;
      imu_correction_guard_tripped = true;
    }

    if (!imu_preintegration_fallback_mode_) {
      imu_updated = imu_smoother_.update(
        ndt_px, ndt_py, ndt_pz, ndt_roll, ndt_pitch, ndt_yaw,
        fitness_score, scan_stamp_sec);

      imu_pose = imu_smoother_.poseMatrix();
      double imu_measurement_translation_delta_m = std::numeric_limits<double>::quiet_NaN();
      double imu_measurement_rotation_delta_deg = std::numeric_limits<double>::quiet_NaN();
      if (imu_pose.allFinite()) {
        imu_measurement_translation_delta_m = static_cast<double>(
          (imu_pose.block<3, 1>(0, 3) - final_transformation.block<3, 1>(0, 3)).norm());
        imu_measurement_rotation_delta_deg = rotation_delta_deg(
          final_transformation.block<3, 3>(0, 0),
          imu_pose.block<3, 3>(0, 0));
      }

      const bool imu_pose_diverged =
        !imu_pose.allFinite() ||
        !std::isfinite(imu_measurement_translation_delta_m) ||
        !std::isfinite(imu_measurement_rotation_delta_deg) ||
        imu_measurement_translation_delta_m > kImuSmootherMeasurementTranslationGuardM ||
        imu_measurement_rotation_delta_deg > kImuSmootherMeasurementRotationGuardDeg;
      if (imu_pose_diverged) {
        RCLCPP_WARN(
          get_logger(),
          "IMU smoother diverged from measurement (translation=%.3f m, rotation=%.3f deg). Disabling IMU preintegration for the remainder of this run.",
          imu_measurement_translation_delta_m,
          imu_measurement_rotation_delta_deg);
        imu_preintegration_fallback_mode_ = true;
        imu_state_reset = true;
      }
    }

    if (imu_preintegration_fallback_mode_) {
      imu_pose = final_transformation;
      imu_updated = true;
    }

    Eigen::Matrix3d imu_rot_mat = imu_pose.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond quat_eig(imu_rot_mat);
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

    corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
    corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = imu_pose(0, 3);
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = imu_pose(1, 3);
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = imu_pose(2, 3);
    corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;

    fillPoseCovariance(fitness_score);
    updatePredictionState(imu_pose, scan_stamp_sec);
    last_scan_stamp_for_imu_ = scan_stamp_sec;

    if (imu_state_reset) {
      status_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status_message = imu_correction_guard_tripped ?
        "imu_prediction_correction_guard_imu_disabled" :
        "imu_smoother_diverged_imu_disabled";
    } else if (!imu_updated) {
      status_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status_message = "imu_smoother_update_rejected";
    }
    publishAlignmentStatus(
      msg->header.stamp, status_level, status_message,
      has_converged, fitness_score, selected_attempt.alignment_time_sec,
      filtered_point_count,
      correction_translation_m,
      correction_yaw_deg,
      seed_translation_since_accept_m,
      seed_yaw_since_accept_deg,
      accepted_gap_sec,
      imu_prediction_ready);
  } else if (use_twist_ekf_) {
    // Extract roll, pitch, yaw from NDT rotation matrix using atan2
    // (eulerAngles() has gimbal-lock ambiguity)
    Eigen::Matrix3f ndt_rot = final_transformation.block<3, 3>(0, 0);
    last_ndt_pitch_ = std::asin(-ndt_rot(2, 0));
    last_ndt_roll_ = std::atan2(ndt_rot(2, 1), ndt_rot(2, 2));
    double ndt_yaw = static_cast<double>(
      std::atan2(ndt_rot(1, 0), ndt_rot(0, 0)));
    double ndt_px = static_cast<double>(final_transformation(0, 3));
    double ndt_py = static_cast<double>(final_transformation(1, 3));
    double ndt_pz = static_cast<double>(final_transformation(2, 3));

    if (!twist_ekf_.isInitialized()) {
      twist_ekf_.initialize(ndt_px, ndt_py, ndt_pz, ndt_yaw, scan_stamp_sec);
    }

    // EKF update with NDT measurement (not gated by score_threshold)
    // The EKF adaptively scales measurement noise by fitness
    bool ekf_updated = twist_ekf_.update(ndt_px, ndt_py, ndt_pz, ndt_yaw, fitness_score);

    // Use EKF state for published pose
    Eigen::Matrix4f ekf_pose = twist_ekf_.poseMatrix(last_ndt_roll_, last_ndt_pitch_);
    Eigen::Matrix3d ekf_rot_mat = ekf_pose.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond quat_eig(ekf_rot_mat);
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

    corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
    corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = twist_ekf_.px();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = twist_ekf_.py();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = twist_ekf_.pz();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;

    // Also update prediction state for compatibility
    fillPoseCovariance(fitness_score);
    updatePredictionState(ekf_pose, scan_stamp_sec);

    // Publish diagnostics with EKF info
    if (!ekf_updated) {
      status_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status_message = "ekf_update_rejected";
    }
    publishAlignmentStatus(
      msg->header.stamp, status_level, status_message,
      has_converged, fitness_score, selected_attempt.alignment_time_sec,
      filtered_point_count,
      correction_translation_m,
      correction_yaw_deg,
      seed_translation_since_accept_m,
      seed_yaw_since_accept_deg,
      accepted_gap_sec,
      imu_prediction_ready);
  } else {
    // Original flow
    publishAlignmentStatus(
      msg->header.stamp, status_level, status_message,
      has_converged, fitness_score, selected_attempt.alignment_time_sec,
      filtered_point_count,
      correction_translation_m,
      correction_yaw_deg,
      seed_translation_since_accept_m,
      seed_yaw_since_accept_deg,
      accepted_gap_sec,
      imu_prediction_ready);

    if (reject_measurement) {
      if (rejected_seed_update_applied) {
        updatePredictionFromRejectedMeasurement(final_transformation, scan_stamp_sec);
      } else {
        advancePredictionWithoutMeasurement(scan_stamp_sec);
      }
      return;
    }

    setCurrentPoseFromMatrix(final_transformation, msg->header.stamp);
    fillPoseCovariance(fitness_score);
    updatePredictionState(final_transformation, scan_stamp_sec);
  }
    
  // publish here if timer is not enabled

  if (!enable_timer_publishing_){
    publishCurrentPose(msg->header.stamp);
  }

  if (enable_debug_) {
    std::cout << "number of filtered cloud points: " << filtered_point_count << std::endl;
    std::cout << "align time:" << selected_attempt.alignment_time_sec <<
      "[sec]" << std::endl;
    std::cout << "has converged: " << has_converged << std::endl;
    std::cout << "fitness score: " << fitness_score << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;
    /* delta_angle check
     * trace(RotationMatrix) = 2(cos(theta) + 1)
     */
    double init_cos_angle = 0.5 *
      (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
      (final_transformation.coeff(0,
      0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
  }

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
  have_last_accepted_pose_ = true;
  last_accepted_pose_matrix_ = pose_matrix;
  predicted_pose_matrix_ = pose_matrix;
  last_relative_motion_matrix_ = Eigen::Matrix4f::Identity();
  consecutive_rejected_updates_ = 0;
  last_accepted_pose_time_sec_ = stamp_sec;
  predicted_pose_time_sec_ = stamp_sec;
}

void PCLLocalization::updatePredictionState(
  const Eigen::Matrix4f & accepted_pose_matrix,
  double stamp_sec)
{
  if (!have_last_accepted_pose_) {
    resetPredictionState(accepted_pose_matrix, stamp_sec);
    return;
  }

  if (consecutive_rejected_updates_ == 0) {
    last_relative_motion_matrix_ = last_accepted_pose_matrix_.inverse() * accepted_pose_matrix;
  }

  last_accepted_pose_matrix_ = accepted_pose_matrix;
  predicted_pose_matrix_ = accepted_pose_matrix * last_relative_motion_matrix_;
  have_last_accepted_pose_ = true;
  consecutive_rejected_updates_ = 0;
  last_accepted_pose_time_sec_ = stamp_sec;
  predicted_pose_time_sec_ = stamp_sec;
}

void PCLLocalization::advancePredictionWithoutMeasurement(double stamp_sec)
{
  if (!have_last_accepted_pose_) {
    return;
  }

  if (use_twist_prediction_ && latest_twist_msg_) {
    const double dt = std::clamp(stamp_sec - predicted_pose_time_sec_, 0.0, max_twist_prediction_dt_);
    predicted_pose_matrix_ = applyTwistPrediction(predicted_pose_matrix_, dt);
    predicted_pose_time_sec_ = stamp_sec;
  } else if (predict_pose_from_previous_delta_) {
    predicted_pose_matrix_ = predicted_pose_matrix_ * last_relative_motion_matrix_;
    predicted_pose_time_sec_ = stamp_sec;
  } else {
    return;
  }
  ++consecutive_rejected_updates_;
}

void PCLLocalization::updatePredictionFromRejectedMeasurement(
  const Eigen::Matrix4f & rejected_pose_matrix,
  double stamp_sec)
{
  if (!have_last_accepted_pose_) {
    return;
  }

  predicted_pose_matrix_ = rejected_pose_matrix;
  predicted_pose_time_sec_ = stamp_sec;
  ++consecutive_rejected_updates_;
}

void PCLLocalization::setCurrentPoseFromMatrix(
  const Eigen::Matrix4f & pose_matrix,
  const builtin_interfaces::msg::Time & stamp)
{
  Eigen::Matrix3d rot_mat = pose_matrix.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  corrent_pose_with_cov_stamped_ptr_->header.stamp = stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = static_cast<double>(pose_matrix(0, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = static_cast<double>(pose_matrix(1, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = static_cast<double>(pose_matrix(2, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
}

void PCLLocalization::publishCurrentPose(const builtin_interfaces::msg::Time & stamp)
{
  if (shutting_down_ || !pose_pub_ || !corrent_pose_with_cov_stamped_ptr_) {return;}
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  geometry_msgs::msg::TransformStamped map_to_base_link_stamped;
  map_to_base_link_stamped.header.stamp = stamp;
  map_to_base_link_stamped.header.frame_id = global_frame_id_;
  map_to_base_link_stamped.child_frame_id = base_frame_id_;
  map_to_base_link_stamped.transform.translation.x = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x;
  map_to_base_link_stamped.transform.translation.y = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y;
  map_to_base_link_stamped.transform.translation.z = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z;
  map_to_base_link_stamped.transform.rotation = corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation;
  if (!enable_map_odom_tf_) {
    broadcaster_.sendTransform(map_to_base_link_stamped);
  } else {
    tf2::Transform map_to_base_link_tf;
    tf2::fromMsg(map_to_base_link_stamped.transform, map_to_base_link_tf);

    geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
    try {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not get transform %s to %s: %s",
        base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
      return;
    }
    tf2::Transform odom_to_base_link_tf;
    tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

    tf2::Transform map_to_odom_tf = map_to_base_link_tf * odom_to_base_link_tf.inverse();
    geometry_msgs::msg::TransformStamped map_to_odom_stamped;
    map_to_odom_stamped.header.stamp = stamp;
    map_to_odom_stamped.header.frame_id = global_frame_id_;
    map_to_odom_stamped.child_frame_id = odom_frame_id_;
    map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
    broadcaster_.sendTransform(map_to_odom_stamped);
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = stamp;
  pose_stamped.header.frame_id = global_frame_id_;
  pose_stamped.pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  path_ptr_->poses.push_back(pose_stamped);
  path_pub_->publish(*path_ptr_);
}

void PCLLocalization::fillPoseCovariance(double fitness_score)
{
  // Nav2 PoseWithCovarianceStamped.pose.covariance: 6x6 row-major [36]
  // Indices: [0]=xx, [7]=yy, [14]=zz, [21]=roll, [28]=pitch, [35]=yaw
  auto & cov = corrent_pose_with_cov_stamped_ptr_->pose.covariance;
  std::fill(cov.begin(), cov.end(), 0.0);

  // Use EKF covariance matrix if available (most accurate)
  if (use_twist_ekf_ && twist_ekf_.isInitialized()) {
    const auto & P = twist_ekf_.covariance();
    // EKF state: [px,py,pz,vx,vy,vz,yaw,gyro_bias,speed_bias]
    cov[0] = P(0, 0);    // xx
    cov[1] = P(0, 1);    // xy
    cov[6] = P(1, 0);    // yx
    cov[7] = P(1, 1);    // yy
    cov[14] = P(2, 2);   // zz
    cov[35] = P(6, 6);   // yaw-yaw
    // roll/pitch not estimated by EKF — use fitness-based estimate
    double scale = std::max(1.0, fitness_score);
    cov[21] = 0.001 * scale;
    cov[28] = 0.001 * scale;
    return;
  }

  // Fitness-based covariance for smoother / standard paths
  double scale = std::max(1.0, fitness_score);
  cov[0] = 0.01 * scale;    // x
  cov[7] = 0.01 * scale;    // y
  cov[14] = 0.05 * scale;   // z
  cov[21] = 0.001 * scale;  // roll
  cov[28] = 0.001 * scale;  // pitch
  cov[35] = 0.0005 * scale; // yaw
}

double PCLLocalization::computeEffectiveScoreThreshold(
  double accepted_gap_sec,
  double seed_translation_since_accept_m,
  bool * post_reject_strict_active,
  bool * open_loop_strict_active) const
{
  const bool post_reject_active =
    enable_post_reject_strict_score_threshold_ &&
    static_cast<int>(consecutive_rejected_updates_) >= post_reject_strict_min_rejections_ &&
    post_reject_strict_score_threshold_ < score_threshold_;
  const bool open_loop_active =
    enable_open_loop_strict_score_threshold_ &&
    accepted_gap_sec >= open_loop_strict_min_accepted_gap_sec_ &&
    seed_translation_since_accept_m >= open_loop_strict_min_seed_translation_m_ &&
    open_loop_strict_score_threshold_ < score_threshold_;

  if (post_reject_strict_active != nullptr) {
    *post_reject_strict_active = post_reject_active;
  }
  if (open_loop_strict_active != nullptr) {
    *open_loop_strict_active = open_loop_active;
  }

  double effective_score_threshold = score_threshold_;
  if (post_reject_active && post_reject_strict_score_threshold_ < effective_score_threshold) {
    effective_score_threshold = post_reject_strict_score_threshold_;
  }
  if (open_loop_active && open_loop_strict_score_threshold_ < effective_score_threshold) {
    effective_score_threshold = open_loop_strict_score_threshold_;
  }
  return effective_score_threshold;
}

bool PCLLocalization::computeBorderlineSeedGateActive(
  double fitness_score,
  double seed_translation_since_accept_m,
  double effective_score_threshold) const
{
  return enable_borderline_seed_rejection_gate_ &&
         borderline_seed_gate_score_threshold_ < effective_score_threshold &&
         fitness_score > borderline_seed_gate_score_threshold_ &&
         fitness_score <= effective_score_threshold &&
         seed_translation_since_accept_m >= borderline_seed_gate_min_seed_translation_m_;
}

PCLLocalization::ReinitializationRequestDecision
PCLLocalization::computeReinitializationRequest(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & status_message,
  double fitness_score,
  double seed_translation_since_accept_m,
  double accepted_gap_sec) const
{
  ReinitializationRequestDecision decision;

  double effective_gap_sec = accepted_gap_sec;
  if (!std::isfinite(effective_gap_sec) && have_last_accepted_pose_) {
    effective_gap_sec = std::max(0.0, stamp_to_sec(stamp) - last_accepted_pose_time_sec_);
  }

  double effective_seed_translation_m = seed_translation_since_accept_m;
  if (!std::isfinite(effective_seed_translation_m) && have_last_accepted_pose_) {
    const Eigen::Matrix4f seed_delta_matrix =
      last_accepted_pose_matrix_.inverse() * predicted_pose_matrix_;
    effective_seed_translation_m = static_cast<double>(
      seed_delta_matrix.block<3, 1>(0, 3).norm());
  }

  const bool target_unavailable = status_message == "local_map_crop_too_small";
  const bool not_converged = status_message == "registration_not_converged";
  const bool rejected_measurement = status_message.rfind("fitness_score_over_", 0) == 0;
  const bool failure = target_unavailable || not_converged || rejected_measurement;

  if (!failure) {
    decision.reason = "not_failure";
    return decision;
  }

  const double gap_component =
    std::isfinite(effective_gap_sec) && reinitialization_trigger_gap_scale_sec_ > 0.0 ?
    std::min(1.0, effective_gap_sec / reinitialization_trigger_gap_scale_sec_) : 0.0;
  const double seed_component =
    std::isfinite(effective_seed_translation_m) &&
    reinitialization_trigger_seed_translation_scale_m_ > 0.0 ?
    std::min(1.0, effective_seed_translation_m / reinitialization_trigger_seed_translation_scale_m_) : 0.0;
  const double streak_component =
    reinitialization_trigger_reject_streak_scale_ > 0.0 ?
    std::min(
      1.0,
      static_cast<double>(consecutive_rejected_updates_) /
      reinitialization_trigger_reject_streak_scale_) : 0.0;
  const bool fitness_exploded =
    std::isfinite(fitness_score) &&
    fitness_score >= reinitialization_trigger_fitness_explosion_threshold_;

  decision.score =
    0.45 * gap_component +
    0.30 * seed_component +
    0.20 * streak_component +
    (target_unavailable ? 0.15 : 0.0) +
    (fitness_exploded ? 0.15 : 0.0);

  if (decision.score < reinitialization_trigger_threshold_) {
    decision.reason = "reinit_not_requested";
    return decision;
  }

  decision.requested = true;
  if (target_unavailable) {
    decision.reason = "target_unavailable_reinit_requested";
  } else if (fitness_exploded) {
    decision.reason = "fitness_exploded_reinit_requested";
  } else if (gap_component >= 1.0) {
    decision.reason = "accepted_gap_reinit_requested";
  } else if (streak_component >= 1.0) {
    decision.reason = "reject_streak_reinit_requested";
  } else {
    decision.reason = "reinit_score_exceeded";
  }
  return decision;
}

void PCLLocalization::publishReinitializationRequest(
  const builtin_interfaces::msg::Time & stamp,
  const ReinitializationRequestDecision & decision)
{
  reinitialization_requested_ = decision.requested;
  reinitialization_request_reason_ = decision.reason;
  reinitialization_request_score_ = decision.score;

  if (
    !enable_reinitialization_request_output_ ||
    !reinitialization_request_pub_ ||
    !reinitialization_request_pub_->is_activated())
  {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = decision.requested;
  reinitialization_request_pub_->publish(msg);
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

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "lidar_localization_ros2/alignment";
  status.message = message;
  status.hardware_id = registration_method_;

  auto append_value =
    [&status](const std::string & key, const std::string & value)
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };

  append_value("registration_method", registration_method_);
  append_value("has_converged", has_converged ? "true" : "false");
  append_value("fitness_score", std::to_string(fitness_score));
  append_value("score_threshold", std::to_string(score_threshold_));
  bool post_reject_strict_active = false;
  bool open_loop_strict_active = false;
  const double effective_score_threshold = computeEffectiveScoreThreshold(
    accepted_gap_sec,
    seed_translation_since_accept_m,
    &post_reject_strict_active,
    &open_loop_strict_active);
  const bool borderline_seed_gate_active = computeBorderlineSeedGateActive(
    fitness_score,
    seed_translation_since_accept_m,
    effective_score_threshold);
  const ReinitializationRequestDecision reinitialization_request =
    computeReinitializationRequest(
    stamp,
    message,
    fitness_score,
    seed_translation_since_accept_m,
    accepted_gap_sec);
  append_value("effective_score_threshold", std::to_string(effective_score_threshold));
  append_value(
    "post_reject_strict_score_threshold_active",
    post_reject_strict_active ? "true" : "false");
  append_value(
    "open_loop_strict_score_threshold_active",
    open_loop_strict_active ? "true" : "false");
  append_value(
    "borderline_seed_rejection_gate_active",
    borderline_seed_gate_active ? "true" : "false");
  append_value("alignment_time_sec", std::to_string(alignment_time_sec));
  append_value("filtered_point_count", std::to_string(filtered_point_count));
  append_value("correction_translation_m", std::to_string(correction_translation_m));
  append_value("correction_yaw_deg", std::to_string(correction_yaw_deg));
  append_value(
    "seed_translation_since_accept_m",
    std::to_string(seed_translation_since_accept_m));
  append_value(
    "seed_yaw_since_accept_deg",
    std::to_string(seed_yaw_since_accept_deg));
  append_value("accepted_gap_sec", std::to_string(accepted_gap_sec));
  append_value(
    "consecutive_rejected_updates",
    std::to_string(consecutive_rejected_updates_));
  append_value(
    "imu_prediction_active",
    imu_prediction_active ? "true" : "false");
  append_value(
    "reinitialization_requested",
    reinitialization_request.requested ? "true" : "false");
  append_value(
    "reinitialization_request_reason",
    reinitialization_request.reason);
  append_value(
    "reinitialization_request_score",
    std::to_string(reinitialization_request.score));
  append_value("map_received", map_recieved_ ? "true" : "false");
  append_value("initialpose_received", initialpose_recieved_ ? "true" : "false");

  diagnostic_msgs::msg::DiagnosticArray status_array;
  status_array.header.stamp = stamp;
  status_array.header.frame_id = base_frame_id_;
  status_array.status.push_back(status);
  status_pub_->publish(status_array);
  publishReinitializationRequest(stamp, reinitialization_request);
}

void PCLLocalization::timerPublishPose()
{
  if (shutting_down_ || !pose_pub_ || !path_pub_ || !path_ptr_) {return;}
  if (!corrent_pose_with_cov_stamped_ptr_) {return;}
  geometry_msgs::msg::PoseWithCovarianceStamped pose_copy = *corrent_pose_with_cov_stamped_ptr_;
  pose_copy.header.stamp = now();

  geometry_msgs::msg::PoseStamped stamped;
  stamped.header = pose_copy.header;
  stamped.header.frame_id = global_frame_id_;
  stamped.pose = pose_copy.pose.pose;
  path_ptr_->poses.push_back(stamped);

  nav_msgs::msg::Path path_copy = *path_ptr_;

  pose_pub_->publish(pose_copy);
  path_pub_->publish(path_copy);

  geometry_msgs::msg::TransformStamped map_to_base_link_stamped;
  map_to_base_link_stamped.header.stamp = pose_copy.header.stamp;
  map_to_base_link_stamped.header.frame_id = global_frame_id_;
  map_to_base_link_stamped.child_frame_id = base_frame_id_;
  map_to_base_link_stamped.transform.translation.x = pose_copy.pose.pose.position.x;
  map_to_base_link_stamped.transform.translation.y = pose_copy.pose.pose.position.y;
  map_to_base_link_stamped.transform.translation.z = pose_copy.pose.pose.position.z;
  map_to_base_link_stamped.transform.rotation = pose_copy.pose.pose.orientation;

  if (!enable_map_odom_tf_) {
    broadcaster_.sendTransform(map_to_base_link_stamped);
  } else {
    tf2::Transform map_to_base_link_tf;
    tf2::fromMsg(map_to_base_link_stamped.transform, map_to_base_link_tf);

    geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
    try {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, pose_copy.header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not get transform %s to %s: %s",
        base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
      return;
    }
    tf2::Transform odom_to_base_link_tf;
    tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

    tf2::Transform map_to_odom_tf = map_to_base_link_tf * odom_to_base_link_tf.inverse();
    geometry_msgs::msg::TransformStamped map_to_odom_stamped;
    map_to_odom_stamped.header.stamp = pose_copy.header.stamp;
    map_to_odom_stamped.header.frame_id = global_frame_id_;
    map_to_odom_stamped.child_frame_id = odom_frame_id_;
    map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
    
    broadcaster_.sendTransform(map_to_odom_stamped);
  }
}
