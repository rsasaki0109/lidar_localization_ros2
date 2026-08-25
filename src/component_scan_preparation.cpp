#include "component_internal.hpp"
bool PCLLocalization::admitScanMessage(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  double * scan_stamp_sec)
{
  double current_scan_stamp_sec = 0.0;
  if (msg) {
    current_scan_stamp_sec = stamp_to_sec(msg->header.stamp);
  }

  const bool is_shutting_down = shutting_down_.load(std::memory_order_acquire);
  const bool timing_relevant =
    !is_shutting_down && static_cast<bool>(msg) && map_recieved_ && initialpose_recieved_;
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
      is_shutting_down,
      static_cast<bool>(msg),
      map_recieved_,
      initialpose_recieved_,
      current_scan_stamp_sec,
      last_initial_pose_stamp_sec_,
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

  if (admission.status == lidar_localization::ScanAdmissionStatus::kPredatesInitialPose) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "dropping scan stamped %.3f s before the latest initialpose",
      last_initial_pose_stamp_sec_ - current_scan_stamp_sec);
  }

  if (admission.status == lidar_localization::ScanAdmissionStatus::kCropFailureGuard) {
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
    *scan_stamp_sec = current_scan_stamp_sec;
  }
  RCLCPP_DEBUG(get_logger(), "cloudReceived");
  return true;
}

PCLLocalization::PreparedScanCloud PCLLocalization::prepareScanForRegistration(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  double scan_stamp_sec)
{
  PreparedScanCloud prepared_scan;
  latest_continuous_time_deskew_applied_ = false;
  latest_continuous_time_deskew_status_ = use_continuous_time_deskew_ ?
    "continuous_time_deskew_scan_not_prepared" :
    "continuous_time_deskew_disabled";
  latest_continuous_time_deskew_point_count_ = 0;
  latest_continuous_time_deskew_skipped_invalid_time_count_ = 0;
  latest_continuous_time_deskew_clamped_time_count_ = 0;
  latest_continuous_time_deskew_pose_history_coverage_ratio_ = 0.0;

  const bool cloud_has_intensity = lidar_localization::hasPointField(msg->fields, "intensity");
  if (!cloud_has_intensity) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Input cloud does not contain intensity. Falling back to xyz with zero intensity.");
  }
  const lidar_localization::PointRelativeTimes point_relative_times =
    lidar_localization::extractPointRelativeTimesSeconds(*msg);
  prepared_scan.point_time_reference_sec = point_relative_times.reference_time_sec;
  const lidar_localization::ScanTimeRangeStatus point_time_status =
    lidar_localization::classifyScanTimeRange(
    lidar_localization::ScanTimeRangeEvaluationInput{
      point_relative_times.has_time_field,
      point_relative_times.valid,
      point_relative_times.duration_sec,
      point_relative_times.valid_point_count,
      point_relative_times.invalid_point_count,
      scan_period_,
      scan_time_range_max_duration_ratio_});
  latest_scan_time_status_ = point_time_status;
  latest_scan_time_field_ = point_relative_times.has_time_field ?
    point_relative_times.field_name :
    "none";
  latest_scan_time_duration_sec_ = point_relative_times.duration_sec;
  latest_scan_time_valid_point_count_ = point_relative_times.valid_point_count;
  latest_scan_time_invalid_point_count_ = point_relative_times.invalid_point_count;
  switch (point_time_status) {
    case lidar_localization::ScanTimeRangeStatus::kNoTimeField:
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Input cloud has no per-point time field; continuous-time deskew diagnostics are unavailable.");
      break;
    case lidar_localization::ScanTimeRangeStatus::kInvalid:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud per-point time field '%s' is invalid (valid_points=%zu, invalid_points=%zu); "
        "continuous-time deskew diagnostics are unavailable for this scan.",
        point_relative_times.field_name.c_str(),
        point_relative_times.valid_point_count,
        point_relative_times.invalid_point_count);
      break;
    case lidar_localization::ScanTimeRangeStatus::kDurationTooLarge:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud per-point time field '%s' spans %.6f s, expected scan_period %.6f s; "
        "check driver time units before enabling continuous-time deskew.",
        point_relative_times.field_name.c_str(),
        point_relative_times.duration_sec,
        scan_period_);
      break;
    case lidar_localization::ScanTimeRangeStatus::kReady:
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Input cloud per-point time field '%s' spans %.6f s (%zu valid points).",
        point_relative_times.field_name.c_str(),
        point_relative_times.duration_sec,
        point_relative_times.valid_point_count);
      break;
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
        if (point_relative_times.has_time_field) {
          prepared_scan.pre_voxel_relative_times_sec.push_back(
            lidar_localization::relativeTimeOrNaN(point_relative_times, point_idx));
        }
      }
    }
    prepared_scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(tmp));
    if (point_relative_times.has_time_field) {
      prepared_scan.relative_times_sec = prepared_scan.pre_voxel_relative_times_sec;
      prepared_scan.relative_times_aligned_with_cloud =
        prepared_scan.relative_times_sec.size() == prepared_scan.cloud->size();
    }
    applyContinuousTimeDeskewIfEnabled(prepared_scan, scan_stamp_sec);
  } else {
    lidar_localization::TimedXyziCloud timed_cloud =
      lidar_localization::convertSensorCloudToTimedXyzi(*msg, point_relative_times);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>(timed_cloud.cloud));
    std::vector<double> cloud_relative_times = std::move(timed_cloud.relative_times_sec);

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
    for (std::size_t point_idx = 0; point_idx < cloud_ptr->points.size(); ++point_idx) {
      const auto & point = cloud_ptr->points[point_idx];
      if (lidar_localization::isPointInScanRange(
          point.x, point.y, point.z, scan_min_range_, scan_max_range_))
      {
        tmp.push_back(point);
        if (point_relative_times.has_time_field) {
          prepared_scan.pre_voxel_relative_times_sec.push_back(
            lidar_localization::relativeTimeOrNaN(cloud_relative_times, point_idx));
        }
      }
    }
    prepared_scan.filtered_point_count = tmp.size();
    prepared_scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(tmp));
    if (point_relative_times.has_time_field) {
      prepared_scan.relative_times_sec = prepared_scan.pre_voxel_relative_times_sec;
      prepared_scan.relative_times_aligned_with_cloud =
        prepared_scan.relative_times_sec.size() == prepared_scan.cloud->size();
    }
    applyContinuousTimeDeskewIfEnabled(prepared_scan, scan_stamp_sec);

    if (enable_scan_voxel_filter_ && !prepared_scan.cloud->empty()) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
      pcl::VoxelGrid<pcl::PointXYZI> scan_voxel_grid_filter;
      scan_voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      scan_voxel_grid_filter.setInputCloud(prepared_scan.cloud);
      scan_voxel_grid_filter.filter(*filtered_cloud_ptr);
      prepared_scan.filtered_point_count = filtered_cloud_ptr->size();
      prepared_scan.cloud = filtered_cloud_ptr;
      prepared_scan.relative_times_sec.clear();
      prepared_scan.relative_times_aligned_with_cloud = false;
    }
  }

  prepared_scan.status = lidar_localization::classifyPreparedScan(
    true,
    true,
    !prepared_scan.cloud || prepared_scan.cloud->empty());
  latest_horizontal_localizability_ = prepared_scan.cloud ?
    lidar_localization::evaluateHorizontalLocalizability(*prepared_scan.cloud) :
    lidar_localization::HorizontalLocalizability{};
  latest_localizability_guard_active_ =
    lidar_localization::shouldSuppressPreviousDeltaSeed(
    enable_localizability_guard_, latest_horizontal_localizability_,
    localizability_min_xy_eigen_ratio_);
  return prepared_scan;
}

bool PCLLocalization::applyContinuousTimeDeskewIfEnabled(
  PreparedScanCloud & prepared_scan,
  double scan_stamp_sec)
{
  latest_continuous_time_deskew_applied_ = false;
  latest_continuous_time_deskew_point_count_ = 0;
  latest_continuous_time_deskew_skipped_invalid_time_count_ = 0;
  latest_continuous_time_deskew_clamped_time_count_ = 0;
  latest_continuous_time_deskew_pose_history_coverage_ratio_ = 0.0;

  if (!use_continuous_time_deskew_) {
    latest_continuous_time_deskew_status_ =
      lidar_localization::continuousTimeDeskewStatusMessage(
      lidar_localization::ContinuousTimeDeskewStatus::kDisabled);
    return false;
  }

  Eigen::Matrix4f scan_start_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f scan_end_pose = Eigen::Matrix4f::Identity();
  bool prediction_finite = false;
  bool imu_samples_ready_for_deskew = false;
  bool imu_smoother_initialized = false;
  bool imu_preintegration_fallback_mode = false;
  const bool use_pose_history = continuous_time_deskew_mode_ == "imu_pose_history";
  const bool use_lidar_motion =
    continuous_time_deskew_mode_ == "lidar_constant_velocity";
  std::vector<lidar_localization::TimestampedPose> pose_history;
  {
    std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
    imu_smoother_initialized = imu_smoother_.isInitialized();
    imu_preintegration_fallback_mode = imu_preintegration_fallback_mode_;
    const bool has_new_imu_samples =
      lidar_localization::hasNewImuSamples(last_imu_stamp_, last_scan_stamp_for_imu_);
    const double imu_integration_window_sec =
      lidar_localization::imuIntegrationWindowSec(last_imu_stamp_, last_scan_stamp_for_imu_);
    const bool imu_window_too_large =
      lidar_localization::isImuIntegrationWindowTooLarge(
        imu_integration_window_sec,
        lidar_localization::imuMaximumIntegrationWindowSec(scan_period_));
    imu_samples_ready_for_deskew = use_pose_history ?
      !continuous_time_imu_pose_history_.empty() :
      (use_lidar_motion ?
      last_relative_motion_duration_sec_ > 0.0 :
      has_new_imu_samples && !imu_window_too_large);
    if (use_pose_history) {
      pose_history.assign(
        continuous_time_imu_pose_history_.begin(), continuous_time_imu_pose_history_.end());
      prediction_finite = std::all_of(
        pose_history.begin(), pose_history.end(),
        [](const lidar_localization::TimestampedPose & sample) {
          return std::isfinite(sample.stamp_sec) && sample.pose.allFinite();
        });
    } else if (use_lidar_motion) {
      prediction_finite = last_relative_motion_matrix_.allFinite() &&
        std::isfinite(last_relative_motion_duration_sec_) &&
        last_relative_motion_duration_sec_ > 0.0;
    } else if (
      use_imu_preintegration_ &&
      !imu_preintegration_fallback_mode &&
      imu_smoother_initialized &&
      imu_samples_ready_for_deskew)
    {
      scan_start_pose = imu_smoother_.poseMatrix();
      scan_end_pose = imu_smoother_.predictedPoseMatrix();
      prediction_finite = scan_start_pose.allFinite() && scan_end_pose.allFinite();
    }
  }

  const std::size_t cloud_size = prepared_scan.cloud ? prepared_scan.cloud->size() : 0;
  const auto decision = lidar_localization::decideContinuousTimeDeskew(
    lidar_localization::ContinuousTimeDeskewDecisionInput{
      use_continuous_time_deskew_,
      !prepared_scan.cloud || prepared_scan.cloud->empty(),
      latest_scan_time_status_,
      prepared_scan.relative_times_aligned_with_cloud,
      prepared_scan.relative_times_sec.size(),
      cloud_size,
      use_imu_preintegration_,
      imu_preintegration_fallback_mode,
      imu_smoother_initialized,
      imu_samples_ready_for_deskew,
      prediction_finite,
      !use_pose_history && !use_lidar_motion,
      !use_lidar_motion});
  if (!decision.should_apply) {
    latest_continuous_time_deskew_status_ =
      lidar_localization::continuousTimeDeskewStatusMessage(decision.status);
    return false;
  }

  if (use_lidar_motion) {
    const double motion_scale =
      latest_scan_time_duration_sec_ / last_relative_motion_duration_sec_;
    if (!std::isfinite(motion_scale) || motion_scale <= 0.0 || motion_scale > 1.5) {
      latest_continuous_time_deskew_status_ =
        "continuous_time_deskew_lidar_motion_interval_mismatch";
      return false;
    }
    const Eigen::Matrix4f scan_motion = lidar_localization::scaleRelativeMotion(
      last_relative_motion_matrix_, motion_scale);
    const auto deskew_result = lidar_localization::deskewPointCloudWithRelativeMotion(
      *prepared_scan.cloud,
      prepared_scan.relative_times_sec,
      latest_scan_time_duration_sec_,
      scan_motion,
      continuous_time_deskew_reference_time_sec_);
    if (!deskew_result.applied) {
      latest_continuous_time_deskew_status_ =
        "continuous_time_deskew_lidar_motion_not_applied";
      return false;
    }
    prepared_scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(deskew_result.cloud));
    latest_continuous_time_deskew_applied_ = true;
    latest_continuous_time_deskew_status_ =
      "continuous_time_deskew_lidar_motion_applied";
    latest_continuous_time_deskew_point_count_ = deskew_result.deskewed_point_count;
    latest_continuous_time_deskew_skipped_invalid_time_count_ =
      deskew_result.skipped_invalid_time_count;
    latest_continuous_time_deskew_clamped_time_count_ = deskew_result.clamped_time_count;
    return true;
  }

  if (use_pose_history) {
    const double scan_start_sec = continuous_time_cloud_stamp_reference_ == "end" ?
      scan_stamp_sec - latest_scan_time_duration_sec_ :
      scan_stamp_sec + prepared_scan.point_time_reference_sec;
    const auto deskew_result = lidar_localization::deskewPointCloudWithPoseHistory(
      *prepared_scan.cloud,
      prepared_scan.relative_times_sec,
      scan_start_sec,
      latest_scan_time_duration_sec_,
      pose_history,
      continuous_time_deskew_reference_time_sec_);
    latest_continuous_time_deskew_pose_history_coverage_ratio_ =
      deskew_result.coverage_ratio;
    latest_continuous_time_deskew_status_ =
      lidar_localization::poseHistoryDeskewStatusMessage(deskew_result.status);
    if (!deskew_result.applied) {
      return false;
    }
    prepared_scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(deskew_result.cloud));
    prepared_scan.relative_times_aligned_with_cloud =
      prepared_scan.relative_times_sec.size() == prepared_scan.cloud->size();
    latest_continuous_time_deskew_applied_ = true;
    latest_continuous_time_deskew_point_count_ = deskew_result.deskewed_point_count;
    latest_continuous_time_deskew_skipped_invalid_time_count_ =
      deskew_result.skipped_invalid_time_count;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Piecewise IMU pose-history deskew applied to %zu points (coverage=%.3f).",
      latest_continuous_time_deskew_point_count_,
      latest_continuous_time_deskew_pose_history_coverage_ratio_);
    return true;
  }

  const Eigen::Matrix4f start_to_end_motion = scan_start_pose.inverse() * scan_end_pose;
  const auto deskew_result = lidar_localization::deskewPointCloudWithRelativeMotion(
    *prepared_scan.cloud,
    prepared_scan.relative_times_sec,
    latest_scan_time_duration_sec_,
    start_to_end_motion,
    continuous_time_deskew_reference_time_sec_);
  if (!deskew_result.applied) {
    latest_continuous_time_deskew_status_ =
      lidar_localization::continuousTimeDeskewStatusMessage(
      lidar_localization::ContinuousTimeDeskewStatus::kNotApplied);
    return false;
  }

  prepared_scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(deskew_result.cloud));
  prepared_scan.relative_times_aligned_with_cloud =
    prepared_scan.relative_times_sec.size() == prepared_scan.cloud->size();
  latest_continuous_time_deskew_applied_ = true;
  latest_continuous_time_deskew_status_ =
    lidar_localization::continuousTimeDeskewStatusMessage(
    lidar_localization::ContinuousTimeDeskewStatus::kApplied);
  latest_continuous_time_deskew_point_count_ = deskew_result.deskewed_point_count;
  latest_continuous_time_deskew_skipped_invalid_time_count_ =
    deskew_result.skipped_invalid_time_count;
  latest_continuous_time_deskew_clamped_time_count_ = deskew_result.clamped_time_count;
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 10000,
    "Continuous-time deskew applied to %zu points (skipped_invalid_time=%zu, clamped_time=%zu).",
    latest_continuous_time_deskew_point_count_,
    latest_continuous_time_deskew_skipped_invalid_time_count_,
    latest_continuous_time_deskew_clamped_time_count_);
  return true;
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

void PCLLocalization::resetImuPreintegrationSampleCounters()
{
  imu_preintegration_received_sample_count_since_scan_ = 0;
  imu_preintegration_integrated_sample_count_since_scan_ = 0;
  imu_preintegration_skipped_sample_count_since_scan_ = 0;
  imu_preintegration_transform_failure_count_since_scan_ = 0;
  imu_preintegration_non_finite_sample_count_since_scan_ = 0;
  imu_preintegration_invalid_dt_count_since_scan_ = 0;
  imu_preintegration_last_dt_sec_ = std::numeric_limits<double>::quiet_NaN();
}

void PCLLocalization::snapshotImuPreintegrationSampleCountersForScan()
{
  latest_imu_seed_received_sample_count_ =
    imu_preintegration_received_sample_count_since_scan_;
  latest_imu_seed_integrated_sample_count_ =
    imu_preintegration_integrated_sample_count_since_scan_;
  latest_imu_seed_skipped_sample_count_ =
    imu_preintegration_skipped_sample_count_since_scan_;
  latest_imu_seed_transform_failure_count_ =
    imu_preintegration_transform_failure_count_since_scan_;
  latest_imu_seed_non_finite_sample_count_ =
    imu_preintegration_non_finite_sample_count_since_scan_;
  latest_imu_seed_invalid_dt_count_ =
    imu_preintegration_invalid_dt_count_since_scan_;
  latest_imu_seed_last_dt_sec_ = imu_preintegration_last_dt_sec_;
  resetImuPreintegrationSampleCounters();
}

