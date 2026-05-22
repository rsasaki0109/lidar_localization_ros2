#ifndef LIDAR_LOCALIZATION_POSE_BACKEND_SELECTION_POLICY_HPP_
#define LIDAR_LOCALIZATION_POSE_BACKEND_SELECTION_POLICY_HPP_

namespace lidar_localization
{

enum class PoseBackendKind
{
  kGtsamSmoother = 0,
  kImuPreintegration,
  kTwistEkf,
  kRawRegistration,
};

struct PoseBackendSelectionInput
{
  bool use_gtsam_smoother{false};
  bool use_imu_preintegration{false};
  bool has_imu_stamp{false};
  bool use_twist_ekf{false};
};

inline PoseBackendKind selectPoseBackend(const PoseBackendSelectionInput & input)
{
  if (input.use_gtsam_smoother) {
    return PoseBackendKind::kGtsamSmoother;
  }
  if (input.use_imu_preintegration && input.has_imu_stamp) {
    return PoseBackendKind::kImuPreintegration;
  }
  if (input.use_twist_ekf) {
    return PoseBackendKind::kTwistEkf;
  }
  return PoseBackendKind::kRawRegistration;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_POSE_BACKEND_SELECTION_POLICY_HPP_
