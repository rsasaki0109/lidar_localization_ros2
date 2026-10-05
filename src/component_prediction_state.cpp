#include "component_internal.hpp"

namespace
{

lidar_localization::PredictionStateSnapshot make_prediction_state_snapshot(
  const PCLLocalization & component)
{
  return {
    component.have_last_accepted_pose_,
    component.last_accepted_pose_matrix_,
    component.predicted_pose_matrix_,
    component.last_relative_motion_matrix_,
    component.consecutive_rejected_updates_,
    component.last_accepted_pose_time_sec_,
    component.predicted_pose_time_sec_};
}

void apply_prediction_state(
  PCLLocalization & component,
  const lidar_localization::PredictionStateSnapshot & state)
{
  component.have_last_accepted_pose_ = state.have_last_accepted_pose;
  component.last_accepted_pose_matrix_ = state.last_accepted_pose_matrix;
  component.predicted_pose_matrix_ = state.predicted_pose_matrix;
  component.last_relative_motion_matrix_ = state.last_relative_motion_matrix;
  component.consecutive_rejected_updates_ = state.consecutive_rejected_updates;
  component.last_accepted_pose_time_sec_ = state.last_accepted_pose_time_sec;
  component.predicted_pose_time_sec_ = state.predicted_pose_time_sec;
}

}  // namespace

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
  if (!scan_twist_ || dt_sec <= 0.0) {
    return pose_matrix;
  }

  const auto & twist = *scan_twist_;
  Eigen::Affine3f affine(pose_matrix);
  Eigen::Vector3f linear_velocity(
    static_cast<float>(twist.linear.x()),
    static_cast<float>(twist.linear.y()),
    static_cast<float>(twist.linear.z()));
  Eigen::Vector3f angular_velocity(
    static_cast<float>(twist.angular.x()),
    static_cast<float>(twist.angular.y()),
    static_cast<float>(twist.angular.z()));

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
  apply_prediction_state(*this, state);
  last_relative_motion_duration_sec_ = 0.0;
  accepted_updates_since_reset_ = 0;
  settled_accepts_since_reset_ = 0;
}

void PCLLocalization::updatePredictionState(
  const Eigen::Matrix4f & accepted_pose_matrix,
  double stamp_sec)
{
  // Keep the duration paired with the held motion across rejected updates.
  const double accepted_interval_sec =
    have_last_accepted_pose_ && consecutive_rejected_updates_ == 0 ?
    stamp_sec - last_accepted_pose_time_sec_ : 0.0;
  // Store the pose at its timestamp in every prediction mode. Extrapolating
  // here makes a later switch to twist integrate the same interval twice.
  const auto state = lidar_localization::updatePredictionStateFromAcceptedMeasurement(
    make_prediction_state_snapshot(*this),
    accepted_pose_matrix,
    stamp_sec,
    false);
  apply_prediction_state(*this, state);
  if (std::isfinite(accepted_interval_sec) && accepted_interval_sec > 0.0) {
    last_relative_motion_duration_sec_ = accepted_interval_sec;
  }
  if (accepted_updates_since_reset_ < std::numeric_limits<std::size_t>::max()) {
    ++accepted_updates_since_reset_;
  }
  settled_accepts_since_reset_ = lidar_localization::nextSettledAccepts(
    settled_accepts_since_reset_, measurementGateParams(),
    last_gated_correction_translation_m_, last_gated_correction_yaw_deg_);
}

void PCLLocalization::advancePredictionWithoutMeasurement(double stamp_sec)
{
  const auto advance_mode = lidar_localization::choosePredictionAdvanceMode(
    have_last_accepted_pose_,
    use_twist_prediction_,
    static_cast<bool>(scan_twist_),
    predict_pose_from_previous_delta_);
  Eigen::Matrix4f twist_predicted_pose_matrix = Eigen::Matrix4f::Identity();
  if (advance_mode == lidar_localization::PredictionAdvanceMode::kTwistPrediction) {
    const double dt = lidar_localization::clampPredictionDt(
      stamp_sec, predicted_pose_time_sec_, max_twist_prediction_dt_);
    twist_predicted_pose_matrix = applyTwistPrediction(predicted_pose_matrix_, dt);
  }
  const auto state = lidar_localization::advancePredictionWithoutMeasurement(
    make_prediction_state_snapshot(*this),
    stamp_sec,
    advance_mode,
    twist_predicted_pose_matrix);
  apply_prediction_state(*this, state);
}

void PCLLocalization::updatePredictionFromRejectedMeasurement(
  const Eigen::Matrix4f & rejected_pose_matrix,
  double stamp_sec)
{
  const auto state = lidar_localization::updatePredictionFromRejectedMeasurement(
    make_prediction_state_snapshot(*this),
    rejected_pose_matrix,
    stamp_sec);
  apply_prediction_state(*this, state);
}
