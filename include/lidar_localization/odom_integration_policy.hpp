#ifndef LIDAR_LOCALIZATION_ODOM_INTEGRATION_POLICY_HPP_
#define LIDAR_LOCALIZATION_ODOM_INTEGRATION_POLICY_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "lidar_localization/pose_finite.hpp"

namespace lidar_localization
{

enum class OdomAdmissionStatus
{
  kAccepted = 0,
  kDisabled,
  kWaitingForInitialPose,
  kMissingCurrentPose,
  kIntervalTooLarge,
  kIntervalNegative,
  kCurrentPoseNonFinite,
};

struct OdomAdmissionInput
{
  bool use_odom{false};
  bool initial_pose_received{false};
  bool has_current_pose{false};
  double dt_odom_sec{0.0};
  bool current_pose_finite{true};
  double max_odom_interval_sec{1.0};
};

struct OdomAdmissionDecision
{
  OdomAdmissionStatus status{OdomAdmissionStatus::kAccepted};
  bool accepted{true};
};

inline OdomAdmissionDecision decideOdomAdmission(const OdomAdmissionInput & input)
{
  OdomAdmissionDecision decision;
  if (!input.use_odom) {
    decision.status = OdomAdmissionStatus::kDisabled;
    decision.accepted = false;
    return decision;
  }
  if (!input.initial_pose_received) {
    decision.status = OdomAdmissionStatus::kWaitingForInitialPose;
    decision.accepted = false;
    return decision;
  }
  if (!input.has_current_pose) {
    decision.status = OdomAdmissionStatus::kMissingCurrentPose;
    decision.accepted = false;
    return decision;
  }
  if (!input.current_pose_finite) {
    decision.status = OdomAdmissionStatus::kCurrentPoseNonFinite;
    decision.accepted = false;
    return decision;
  }
  if (input.dt_odom_sec > input.max_odom_interval_sec) {
    decision.status = OdomAdmissionStatus::kIntervalTooLarge;
    decision.accepted = false;
    return decision;
  }
  if (input.dt_odom_sec < 0.0) {
    decision.status = OdomAdmissionStatus::kIntervalNegative;
    decision.accepted = false;
    return decision;
  }
  return decision;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ODOM_INTEGRATION_POLICY_HPP_
