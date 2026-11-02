#ifndef LIDAR_LOCALIZATION_PREDICTION_STATE_POLICY_HPP_
#define LIDAR_LOCALIZATION_PREDICTION_STATE_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

#include <Eigen/Geometry>

namespace lidar_localization
{

enum class PredictionAdvanceMode
{
  kNone = 0,
  kTwistPrediction,
  kPreviousDelta,
};

// The pose the next scan is predicted from, and the motion it is advanced by.
struct PredictionState
{
  bool have_last_accepted_pose{false};
  Eigen::Matrix4f last_accepted_pose_matrix{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f predicted_pose_matrix{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_relative_motion_matrix{Eigen::Matrix4f::Identity()};
  std::size_t consecutive_rejected_updates{0};
  double last_accepted_pose_time_sec{0.0};
  double predicted_pose_time_sec{0.0};
  // The time last_relative_motion_matrix took. It stays paired with the motion
  // across rejected updates.
  double last_relative_motion_duration_sec{0.0};
  // Whether last_relative_motion_matrix runs from one scan pose to the next. After a
  // reset it runs from the reset pose (an initial pose, say) to the first scan: a
  // correction over an arbitrary interval, which must not be rescaled.
  bool last_relative_motion_spans_scans{false};
  std::size_t accepted_updates_since_reset{0};
};

inline PredictionState resetPredictionState(
  const Eigen::Matrix4f & pose_matrix,
  double stamp_sec)
{
  PredictionState state;
  state.have_last_accepted_pose = true;
  state.last_accepted_pose_matrix = pose_matrix;
  state.predicted_pose_matrix = pose_matrix;
  state.last_relative_motion_matrix = Eigen::Matrix4f::Identity();
  state.consecutive_rejected_updates = 0;
  state.last_accepted_pose_time_sec = stamp_sec;
  state.predicted_pose_time_sec = stamp_sec;
  return state;
}

// extrapolate_previous_delta: the stored prediction is the accepted pose
// advanced by one previous delta, which is the seed for the next scan in
// previous-delta mode.  Twist prediction integrates its own motion from the
// stored prediction, so it must start from the accepted pose itself or the
// inter-scan motion is counted twice.
inline PredictionState updatePredictionStateFromAcceptedMeasurement(
  const PredictionState & current,
  const Eigen::Matrix4f & accepted_pose_matrix,
  double stamp_sec,
  bool extrapolate_previous_delta = true)
{
  if (!current.have_last_accepted_pose) {
    PredictionState next = resetPredictionState(accepted_pose_matrix, stamp_sec);
    next.accepted_updates_since_reset = 1;
    return next;
  }

  PredictionState next = current;
  if (current.consecutive_rejected_updates == 0) {
    next.last_relative_motion_matrix =
      current.last_accepted_pose_matrix.inverse() * accepted_pose_matrix;
    // Without a positive interval the motion and a duration would not pair.
    const double interval_sec = stamp_sec - current.last_accepted_pose_time_sec;
    const bool timed = std::isfinite(interval_sec) && interval_sec > 0.0;
    if (timed) {
      next.last_relative_motion_duration_sec = interval_sec;
    }
    next.last_relative_motion_spans_scans = current.accepted_updates_since_reset > 0 && timed;
  }

  next.last_accepted_pose_matrix = accepted_pose_matrix;
  next.predicted_pose_matrix = extrapolate_previous_delta ?
    accepted_pose_matrix * next.last_relative_motion_matrix :
    accepted_pose_matrix;
  next.have_last_accepted_pose = true;
  next.consecutive_rejected_updates = 0;
  next.last_accepted_pose_time_sec = stamp_sec;
  next.predicted_pose_time_sec = stamp_sec;
  if (next.accepted_updates_since_reset < std::numeric_limits<std::size_t>::max()) {
    ++next.accepted_updates_since_reset;
  }
  return next;
}

// The previous-delta motion for a scan at stamp_sec. A motion between two scan poses
// is rescaled from the time it took to the time since the predicted pose, at most
// max_extrapolation_sec. Applying one delta per scan whatever the time lags after
// dropped scans: on Boreas a 0.6 s gap at 9.5 m/s left the seed 5 m behind, and NDT
// locked 4 m back along the road. It also overshoots after a gap, when the delta spans
// the gap and the next scan follows 0.1 s later. Without a usable duration, time anchor
// or limit (max_extrapolation_sec <= 0) it is the stored motion, one delta per scan;
// so is the first motion after a reset (last_relative_motion_spans_scans).
inline Eigen::Matrix4f previousDeltaMotionAt(
  const PredictionState & state,
  double stamp_sec,
  double max_extrapolation_sec)
{
  const Eigen::Matrix4f & motion = state.last_relative_motion_matrix;
  const double motion_duration_sec = state.last_relative_motion_duration_sec;
  const double elapsed_sec = stamp_sec - state.predicted_pose_time_sec;
  if (!state.last_relative_motion_spans_scans ||
    !std::isfinite(motion_duration_sec) || motion_duration_sec <= 0.0 ||
    !std::isfinite(max_extrapolation_sec) || max_extrapolation_sec <= 0.0 ||
    state.predicted_pose_time_sec <= 0.0 || !std::isfinite(elapsed_sec) ||
    elapsed_sec <= 0.0 || !motion.allFinite())
  {
    return motion;
  }
  const double scale = std::min(elapsed_sec, max_extrapolation_sec) / motion_duration_sec;
  Eigen::Matrix4f scaled = Eigen::Matrix4f::Identity();
  scaled.block<3, 1>(0, 3) = motion.block<3, 1>(0, 3) * static_cast<float>(scale);
  // Rotation by scale times the angle of the motion, about its axis.
  Eigen::AngleAxisf rotation(Eigen::Matrix3f(motion.block<3, 3>(0, 0)));
  rotation.angle() *= static_cast<float>(scale);
  scaled.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  return scaled;
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

// previous_delta_motion: the motion to apply in kPreviousDelta mode (previousDeltaMotionAt);
// without it, the stored motion.
inline PredictionState advancePredictionWithoutMeasurement(
  const PredictionState & current,
  double stamp_sec,
  PredictionAdvanceMode mode,
  const Eigen::Matrix4f & twist_predicted_pose_matrix = Eigen::Matrix4f::Identity(),
  const std::optional<Eigen::Matrix4f> & previous_delta_motion = std::nullopt)
{
  if (!current.have_last_accepted_pose) {
    return current;
  }

  PredictionState next = current;
  if (mode == PredictionAdvanceMode::kTwistPrediction) {
    next.predicted_pose_matrix = twist_predicted_pose_matrix;
  } else if (mode == PredictionAdvanceMode::kPreviousDelta) {
    next.predicted_pose_matrix = current.predicted_pose_matrix *
      previous_delta_motion.value_or(current.last_relative_motion_matrix);
  }
  // A held pose keeps its time anchor, but this scan still counts as rejected.
  if (mode != PredictionAdvanceMode::kNone) {
    next.predicted_pose_time_sec = stamp_sec;
  }
  ++next.consecutive_rejected_updates;
  return next;
}

inline PredictionState updatePredictionFromRejectedMeasurement(
  const PredictionState & current,
  const Eigen::Matrix4f & rejected_pose_matrix,
  double stamp_sec)
{
  if (!current.have_last_accepted_pose) {
    return current;
  }

  PredictionState next = current;
  next.predicted_pose_matrix = rejected_pose_matrix;
  next.predicted_pose_time_sec = stamp_sec;
  ++next.consecutive_rejected_updates;
  return next;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_PREDICTION_STATE_POLICY_HPP_
