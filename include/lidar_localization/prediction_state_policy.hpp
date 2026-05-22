#ifndef LIDAR_LOCALIZATION_PREDICTION_STATE_POLICY_HPP_
#define LIDAR_LOCALIZATION_PREDICTION_STATE_POLICY_HPP_

#include <cstddef>

#include <Eigen/Geometry>

namespace lidar_localization
{

enum class PredictionAdvanceMode
{
  kNone = 0,
  kTwistPrediction,
  kPreviousDelta,
};

struct PredictionStateSnapshot
{
  bool have_last_accepted_pose{false};
  Eigen::Matrix4f last_accepted_pose_matrix{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f predicted_pose_matrix{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_relative_motion_matrix{Eigen::Matrix4f::Identity()};
  std::size_t consecutive_rejected_updates{0};
  double last_accepted_pose_time_sec{0.0};
  double predicted_pose_time_sec{0.0};
};

inline PredictionStateSnapshot resetPredictionState(
  const Eigen::Matrix4f & pose_matrix,
  double stamp_sec)
{
  PredictionStateSnapshot state;
  state.have_last_accepted_pose = true;
  state.last_accepted_pose_matrix = pose_matrix;
  state.predicted_pose_matrix = pose_matrix;
  state.last_relative_motion_matrix = Eigen::Matrix4f::Identity();
  state.consecutive_rejected_updates = 0;
  state.last_accepted_pose_time_sec = stamp_sec;
  state.predicted_pose_time_sec = stamp_sec;
  return state;
}

inline PredictionStateSnapshot updatePredictionStateFromAcceptedMeasurement(
  const PredictionStateSnapshot & current,
  const Eigen::Matrix4f & accepted_pose_matrix,
  double stamp_sec)
{
  if (!current.have_last_accepted_pose) {
    return resetPredictionState(accepted_pose_matrix, stamp_sec);
  }

  PredictionStateSnapshot next = current;
  if (current.consecutive_rejected_updates == 0) {
    next.last_relative_motion_matrix =
      current.last_accepted_pose_matrix.inverse() * accepted_pose_matrix;
  }

  next.last_accepted_pose_matrix = accepted_pose_matrix;
  next.predicted_pose_matrix = accepted_pose_matrix * next.last_relative_motion_matrix;
  next.have_last_accepted_pose = true;
  next.consecutive_rejected_updates = 0;
  next.last_accepted_pose_time_sec = stamp_sec;
  next.predicted_pose_time_sec = stamp_sec;
  return next;
}

inline PredictionAdvanceMode choosePredictionAdvanceMode(
  bool have_last_accepted_pose,
  bool use_twist_prediction,
  bool has_latest_twist,
  bool predict_pose_from_previous_delta)
{
  if (!have_last_accepted_pose) {
    return PredictionAdvanceMode::kNone;
  }
  if (use_twist_prediction && has_latest_twist) {
    return PredictionAdvanceMode::kTwistPrediction;
  }
  if (predict_pose_from_previous_delta) {
    return PredictionAdvanceMode::kPreviousDelta;
  }
  return PredictionAdvanceMode::kNone;
}

inline PredictionStateSnapshot advancePredictionWithoutMeasurement(
  const PredictionStateSnapshot & current,
  double stamp_sec,
  PredictionAdvanceMode mode,
  const Eigen::Matrix4f & twist_predicted_pose_matrix = Eigen::Matrix4f::Identity())
{
  if (!current.have_last_accepted_pose || mode == PredictionAdvanceMode::kNone) {
    return current;
  }

  PredictionStateSnapshot next = current;
  if (mode == PredictionAdvanceMode::kTwistPrediction) {
    next.predicted_pose_matrix = twist_predicted_pose_matrix;
  } else if (mode == PredictionAdvanceMode::kPreviousDelta) {
    next.predicted_pose_matrix =
      current.predicted_pose_matrix * current.last_relative_motion_matrix;
  }
  next.predicted_pose_time_sec = stamp_sec;
  ++next.consecutive_rejected_updates;
  return next;
}

inline PredictionStateSnapshot updatePredictionFromRejectedMeasurement(
  const PredictionStateSnapshot & current,
  const Eigen::Matrix4f & rejected_pose_matrix,
  double stamp_sec)
{
  if (!current.have_last_accepted_pose) {
    return current;
  }

  PredictionStateSnapshot next = current;
  next.predicted_pose_matrix = rejected_pose_matrix;
  next.predicted_pose_time_sec = stamp_sec;
  ++next.consecutive_rejected_updates;
  return next;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_PREDICTION_STATE_POLICY_HPP_
