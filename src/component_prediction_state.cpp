#include "component_internal.hpp"

Eigen::Matrix4f PCLLocalization::currentPoseMatrix() const
{
  if (!current_pose_with_cov_stamped_ptr_) {
    return Eigen::Matrix4f::Identity();
  }

  Eigen::Affine3d affine;
  tf2::fromMsg(current_pose_with_cov_stamped_ptr_->pose.pose, affine);
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
  scan_tracker_.reset(pose_matrix, stamp_sec);
}

void PCLLocalization::updatePredictionState(
  const Eigen::Matrix4f & accepted_pose_matrix,
  double stamp_sec)
{
  scan_tracker_.acceptMeasurement(accepted_pose_matrix, stamp_sec, measurement_gate_config_);
}

void PCLLocalization::advancePredictionWithoutMeasurement(double stamp_sec)
{
  const auto advance_mode = lidar_localization::choosePredictionAdvanceMode(
    prediction_state_.have_last_accepted_pose,
    use_twist_prediction_,
    static_cast<bool>(scan_twist_),
    predict_pose_from_previous_delta_);
  Eigen::Matrix4f twist_predicted_pose_matrix = Eigen::Matrix4f::Identity();
  if (advance_mode == lidar_localization::PredictionAdvanceMode::kTwistPrediction) {
    const double dt = lidar_localization::clampPredictionDt(
      stamp_sec, prediction_state_.predicted_pose_time_sec, max_twist_prediction_dt_);
    twist_predicted_pose_matrix =
      applyTwistPrediction(prediction_state_.predicted_pose_matrix, dt);
  }
  scan_tracker_.advanceWithoutMeasurement(
    stamp_sec, advance_mode, twist_predicted_pose_matrix, previous_delta_max_extrapolation_sec_);
}

Eigen::Matrix4f PCLLocalization::previousDeltaMotionAt(double stamp_sec) const
{
  return scan_tracker_.previousDeltaMotionAt(stamp_sec, previous_delta_max_extrapolation_sec_);
}

void PCLLocalization::updatePredictionFromRejectedMeasurement(
  const Eigen::Matrix4f & rejected_pose_matrix,
  double stamp_sec)
{
  scan_tracker_.rejectMeasurement(rejected_pose_matrix, stamp_sec);
}
