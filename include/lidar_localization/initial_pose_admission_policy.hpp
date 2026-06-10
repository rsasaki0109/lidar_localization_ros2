#ifndef LIDAR_LOCALIZATION_INITIAL_POSE_ADMISSION_POLICY_HPP_
#define LIDAR_LOCALIZATION_INITIAL_POSE_ADMISSION_POLICY_HPP_

#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "lidar_localization/pose_finite.hpp"

namespace lidar_localization
{

enum class InitialPoseAdmissionStatus
{
  kAccepted = 0,
  kRejectedFrameMismatch,
  kRejectedNonFinitePose,
};

struct InitialPoseAdmissionInput
{
  std::string expected_frame_id{"map"};
  std::string received_frame_id;
  geometry_msgs::msg::Pose pose;
  bool map_received{false};
};

struct InitialPoseAdmissionDecision
{
  InitialPoseAdmissionStatus status{InitialPoseAdmissionStatus::kAccepted};
  bool accepted{true};
  bool warn_map_not_ready{false};
};

inline InitialPoseAdmissionDecision decideInitialPoseAdmission(
  const InitialPoseAdmissionInput & input)
{
  InitialPoseAdmissionDecision decision;
  if (input.received_frame_id != input.expected_frame_id) {
    decision.status = InitialPoseAdmissionStatus::kRejectedFrameMismatch;
    decision.accepted = false;
    return decision;
  }
  if (!isPoseFinite(input.pose)) {
    decision.status = InitialPoseAdmissionStatus::kRejectedNonFinitePose;
    decision.accepted = false;
    return decision;
  }
  decision.warn_map_not_ready = !input.map_received;
  return decision;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_INITIAL_POSE_ADMISSION_POLICY_HPP_
