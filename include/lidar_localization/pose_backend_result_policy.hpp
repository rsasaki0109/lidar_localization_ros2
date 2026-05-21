#ifndef LIDAR_LOCALIZATION_POSE_BACKEND_RESULT_POLICY_HPP_
#define LIDAR_LOCALIZATION_POSE_BACKEND_RESULT_POLICY_HPP_

#include <cstdint>
#include <string>

#include <Eigen/Dense>

#include "lidar_localization/localization_update_policy.hpp"

namespace lidar_localization
{

struct PoseBackendResult
{
  Eigen::Matrix4f pose_matrix{Eigen::Matrix4f::Identity()};
  std::uint8_t status_level{0};
  std::string status_message{"ok"};
  bool update_current_pose{true};
  bool update_prediction_state{true};
  bool advance_prediction_without_measurement{false};
  bool update_prediction_from_rejected_measurement{false};
  bool fill_pose_covariance{true};
  bool continue_to_pose_publish{true};
};

inline PoseBackendResult makePoseBackendResult(
  const Eigen::Matrix4f & pose_matrix,
  std::uint8_t status_level,
  const std::string & status_message)
{
  PoseBackendResult result;
  result.pose_matrix = pose_matrix;
  result.status_level = status_level;
  result.status_message = status_message;
  return result;
}

inline PoseBackendResult makePoseBackendResultFromLocalizationUpdate(
  const Eigen::Matrix4f & pose_matrix,
  std::uint8_t status_level,
  const std::string & status_message,
  const LocalizationUpdateDecision & localization_update)
{
  PoseBackendResult result =
    makePoseBackendResult(pose_matrix, status_level, status_message);
  result.update_current_pose = localization_update.update_current_pose;
  result.update_prediction_state =
    localization_update.update_prediction_from_accepted_measurement;
  result.advance_prediction_without_measurement =
    localization_update.advance_prediction_without_measurement;
  result.update_prediction_from_rejected_measurement =
    localization_update.update_prediction_from_rejected_measurement;
  result.fill_pose_covariance =
    localization_update.update_current_pose ||
    localization_update.update_prediction_from_accepted_measurement;
  result.continue_to_pose_publish = localization_update.continue_to_pose_publish;
  return result;
}

inline PoseBackendResult applyPoseBackendWarningStatus(
  const PoseBackendResult & current,
  bool warning,
  std::uint8_t warning_level,
  const std::string & warning_message)
{
  if (!warning) {
    return current;
  }

  PoseBackendResult result = current;
  result.status_level = warning_level;
  result.status_message = warning_message;
  return result;
}

inline PoseBackendResult applyPoseBackendUpdateStatus(
  const PoseBackendResult & current,
  bool backend_updated,
  std::uint8_t warning_level,
  const std::string & rejected_status_message)
{
  return applyPoseBackendWarningStatus(
    current, !backend_updated, warning_level, rejected_status_message);
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POSE_BACKEND_RESULT_POLICY_HPP_
