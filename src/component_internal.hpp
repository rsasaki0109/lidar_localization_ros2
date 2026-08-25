#ifndef LIDAR_LOCALIZATION__COMPONENT_INTERNAL_HPP_
#define LIDAR_LOCALIZATION__COMPONENT_INTERNAL_HPP_

// Shared include set and file-local helpers for the PCLLocalization
// implementation translation units (component_*.cpp).  This header is an
// implementation detail of src/ and is not installed or part of the public
// interface.

#include <lidar_localization/lidar_localization_component.hpp>

#include "lidar_localization/alignment_attempt_policy.hpp"
#include "lidar_localization/alignment_diagnostic_ros_adapter.hpp"
#include "lidar_localization/alignment_pipeline_policy.hpp"
#include "lidar_localization/alignment_retry_policy.hpp"
#include "lidar_localization/alignment_diagnostics_policy.hpp"
#include "lidar_localization/alignment_status_policy.hpp"
#include "lidar_localization/continuous_time_deskew_policy.hpp"
#include "lidar_localization/imu_pose_history_deskew.hpp"
#include "lidar_localization/deskew_readiness_policy.hpp"
#include "lidar_localization/imu_preintegration_diagnostics_policy.hpp"
#include "lidar_localization/imu_preintegration_guard_policy.hpp"
#include "lidar_localization/local_map_target_policy.hpp"
#include "lidar_localization/localizability_policy.hpp"
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
#include "lidar_localization/registration_cloud_keep_alive_policy.hpp"
#include "lidar_localization/registration_backend_policy.hpp"
#include "lidar_localization/registration_observation_policy.hpp"
#include "lidar_localization/prediction_state_policy.hpp"
#include "lidar_localization/initial_pose_admission_policy.hpp"
#include "lidar_localization/odom_integration_policy.hpp"
#include "lidar_localization/scan_admission_policy.hpp"

#include <chrono>

#include <pcl/common/common.h>

namespace lidar_localization_component_internal
{

inline double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline lidar_localization::PredictionStateSnapshot make_prediction_state_snapshot(
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

}  // namespace lidar_localization_component_internal

// Preserve the original unqualified call sites in the implementation files.
using lidar_localization_component_internal::stamp_to_sec;
using lidar_localization_component_internal::make_prediction_state_snapshot;

#endif  // LIDAR_LOCALIZATION__COMPONENT_INTERNAL_HPP_
