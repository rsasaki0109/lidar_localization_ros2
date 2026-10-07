#ifndef LIDAR_LOCALIZATION_EXPERIMENTS_SOFT_ODOM_CORRECTION_GATE_HPP_
#define LIDAR_LOCALIZATION_EXPERIMENTS_SOFT_ODOM_CORRECTION_GATE_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace lidar_localization_experiments
{

// Huber-style IRLS weight: 1.0 inside the threshold, threshold/|value| beyond
// it. Mirrors gtsam::noiseModel::mEstimator::Huber, which r3dl/GLIM use for
// their wheel-odometry BetweenFactor instead of a hard accept/reject gate
// (r3dl_core/src/r3dl/localization/localization.cpp, wheel_odom_use_robust).
inline double huberWeight(double value, double threshold)
{
  const double abs_value = std::fabs(value);
  if (threshold <= 0.0 || abs_value <= threshold) {
    return 1.0;
  }
  return threshold / abs_value;
}

struct SoftOdomCorrectionGateParams
{
  bool enable{false};
  double translation_threshold_m{0.3};
  double yaw_threshold_deg{5.0};
  // Below this weight the blended pose is already close enough to the pure
  // odom prediction that it is floored to 0 — keeps the original guard's
  // intent of fully discarding an implausible, aliased NDT jump instead of
  // blending a tiny sliver of it in forever.
  double floor_weight{0.05};
};

struct SoftOdomCorrectionGateInput
{
  double correction_translation_m{0.0};
  double correction_yaw_deg{0.0};
  // Same meaning as MeasurementGateInput::odom_tf_prediction_guard_applicable
  // in measurement_gate_policy.hpp: true only while the odom motion-model
  // mode has an established map->odom anchor.
  bool odom_tf_prediction_guard_applicable{false};
};

struct SoftOdomCorrectionGateDecision
{
  // 1.0 = trust the NDT measurement fully (today's hard-gate "accept").
  // 0.0 = trust the odom-predicted pose fully (today's hard-gate "reject").
  // Values strictly between 0 and 1 are the behavior the hard gate cannot
  // express.
  double ndt_weight{1.0};
  bool floored{false};
};

// Soft counterpart of evaluateMeasurementGate()'s
// odom_tf_prediction_correction_guard branch in measurement_gate_policy.hpp.
// The hard gate rejects the whole NDT measurement the instant either axis
// crosses its threshold, which makes the exact threshold value a sharp
// behavior cliff. This version gives each axis a Huber weight and lets the
// more suspicious axis (the smaller of the two weights) decide how much of
// the NDT measurement to blend in — full trust well inside the threshold,
// a graceful taper just past it, and a hard floor to 0 only for a genuinely
// implausible jump (the "aliased solution" case the original guard comment
// warns about).
inline SoftOdomCorrectionGateDecision evaluateSoftOdomCorrectionGate(
  const SoftOdomCorrectionGateParams & params,
  const SoftOdomCorrectionGateInput & input)
{
  SoftOdomCorrectionGateDecision decision;
  if (!params.enable || !input.odom_tf_prediction_guard_applicable) {
    return decision;
  }
  const double w_translation = huberWeight(
    input.correction_translation_m, params.translation_threshold_m);
  const double w_yaw = huberWeight(input.correction_yaw_deg, params.yaw_threshold_deg);
  decision.ndt_weight = std::min(w_translation, w_yaw);
  if (decision.ndt_weight < params.floor_weight) {
    decision.ndt_weight = 0.0;
    decision.floored = true;
  }
  return decision;
}

// Blends the odom-predicted pose and the NDT-measured pose at ndt_weight
// (1.0 = pure NDT, 0.0 = pure odom prediction): linear interpolation on
// translation, SLERP on rotation. This is the step that would replace
// today's binary "publish NDT pose" / "keep republishing the odom-bridge
// pose" branch if this candidate were ever promoted.
inline Eigen::Isometry3d blendOdomAndNdtPose(
  const Eigen::Isometry3d & odom_predicted_pose,
  const Eigen::Isometry3d & ndt_measured_pose,
  double ndt_weight)
{
  const double w = std::clamp(ndt_weight, 0.0, 1.0);
  const Eigen::Vector3d blended_translation =
    (1.0 - w) * odom_predicted_pose.translation() + w * ndt_measured_pose.translation();
  const Eigen::Quaterniond q_odom(odom_predicted_pose.rotation());
  const Eigen::Quaterniond q_ndt(ndt_measured_pose.rotation());
  const Eigen::Quaterniond blended_rotation = q_odom.slerp(w, q_ndt);

  Eigen::Isometry3d blended = Eigen::Isometry3d::Identity();
  blended.translation() = blended_translation;
  blended.linear() = blended_rotation.normalized().toRotationMatrix();
  return blended;
}

}  // namespace lidar_localization_experiments

#endif  // LIDAR_LOCALIZATION_EXPERIMENTS_SOFT_ODOM_CORRECTION_GATE_HPP_
