#include "component_internal.hpp"
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
  const auto state = lidar_localization::resetPredictionState(pose_matrix, stamp_sec);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
  last_relative_motion_duration_sec_ = 0.0;
}

void PCLLocalization::updatePredictionState(
  const Eigen::Matrix4f & accepted_pose_matrix,
  double stamp_sec)
{
  const double accepted_interval_sec = have_last_accepted_pose_ ?
    stamp_sec - last_accepted_pose_time_sec_ : 0.0;
  const auto state = lidar_localization::updatePredictionStateFromAcceptedMeasurement(
    make_prediction_state_snapshot(
      have_last_accepted_pose_,
      last_accepted_pose_matrix_,
      predicted_pose_matrix_,
      last_relative_motion_matrix_,
      consecutive_rejected_updates_,
      last_accepted_pose_time_sec_,
      predicted_pose_time_sec_),
    accepted_pose_matrix,
    stamp_sec);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
  if (std::isfinite(accepted_interval_sec) && accepted_interval_sec > 0.0) {
    last_relative_motion_duration_sec_ = accepted_interval_sec;
  }
}

void PCLLocalization::advancePredictionWithoutMeasurement(double stamp_sec)
{
  const auto advance_mode = lidar_localization::choosePredictionAdvanceMode(
    have_last_accepted_pose_,
    use_twist_prediction_,
    static_cast<bool>(latest_twist_msg_),
    predict_pose_from_previous_delta_);
  Eigen::Matrix4f twist_predicted_pose_matrix = Eigen::Matrix4f::Identity();
  if (advance_mode == lidar_localization::PredictionAdvanceMode::kTwistPrediction) {
    const double dt = std::clamp(stamp_sec - predicted_pose_time_sec_, 0.0, max_twist_prediction_dt_);
    twist_predicted_pose_matrix = applyTwistPrediction(predicted_pose_matrix_, dt);
  }
  const auto state = lidar_localization::advancePredictionWithoutMeasurement(
    make_prediction_state_snapshot(
      have_last_accepted_pose_,
      last_accepted_pose_matrix_,
      predicted_pose_matrix_,
      last_relative_motion_matrix_,
      consecutive_rejected_updates_,
      last_accepted_pose_time_sec_,
      predicted_pose_time_sec_),
    stamp_sec,
    advance_mode,
    twist_predicted_pose_matrix);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
}

void PCLLocalization::updatePredictionFromRejectedMeasurement(
  const Eigen::Matrix4f & rejected_pose_matrix,
  double stamp_sec)
{
  const auto state = lidar_localization::updatePredictionFromRejectedMeasurement(
    make_prediction_state_snapshot(
      have_last_accepted_pose_,
      last_accepted_pose_matrix_,
      predicted_pose_matrix_,
      last_relative_motion_matrix_,
      consecutive_rejected_updates_,
      last_accepted_pose_time_sec_,
      predicted_pose_time_sec_),
    rejected_pose_matrix,
    stamp_sec);
  have_last_accepted_pose_ = state.have_last_accepted_pose;
  last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  predicted_pose_matrix_ = state.predicted_pose_matrix;
  last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  predicted_pose_time_sec_ = state.predicted_pose_time_sec;
}

