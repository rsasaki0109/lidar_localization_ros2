#include "soft_odom_correction_gate.hpp"

#include "lidar_localization/measurement_gate_policy.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace sg = lidar_localization_experiments;
namespace ll = lidar_localization;

namespace
{

bool nearlyEqual(double a, double b, double tol = 1e-9)
{
  return std::fabs(a - b) <= tol;
}

sg::SoftOdomCorrectionGateParams default_soft_params()
{
  sg::SoftOdomCorrectionGateParams params;
  params.enable = true;
  params.translation_threshold_m = 0.3;
  params.yaw_threshold_deg = 5.0;
  return params;
}

// Mirrors today's production default in loc_t16_odomseed.yaml.
ll::MeasurementGateParams default_hard_params()
{
  ll::MeasurementGateParams params;
  params.score_threshold = 6.0;
  params.reject_above_score_threshold = true;
  params.enable_odom_tf_prediction_correction_guard = true;
  params.odom_tf_prediction_correction_guard_translation_m = 0.3;
  params.odom_tf_prediction_correction_guard_yaw_deg = 5.0;
  return params;
}

}  // namespace

void test_within_threshold_both_gates_fully_trust_ndt()
{
  const auto hard = evaluateMeasurementGate(
    default_hard_params(),
    ll::makeMeasurementGateInput(0.5, 0.0, 0.0, 0.05, 1.0, 0, true));
  assert(!hard.reject_measurement);

  const auto soft = sg::evaluateSoftOdomCorrectionGate(
    default_soft_params(), {0.05, 1.0, true});
  assert(nearlyEqual(soft.ndt_weight, 1.0));
}

void test_just_past_threshold_hard_gate_fully_rejects_soft_gate_tapers()
{
  // 0.31 m vs a 0.30 m threshold: 3% past it.
  const auto hard = evaluateMeasurementGate(
    default_hard_params(),
    ll::makeMeasurementGateInput(0.5, 0.0, 0.0, 0.31, 1.0, 0, true));
  assert(hard.reject_measurement);  // binary cliff: NDT fully discarded

  const auto soft = sg::evaluateSoftOdomCorrectionGate(
    default_soft_params(), {0.31, 1.0, true});
  // Still mostly trusts NDT -- no behavior cliff at the exact threshold.
  assert(soft.ndt_weight > 0.9 && soft.ndt_weight < 1.0);
}

void test_moderate_overshoot_soft_gate_splits_trust()
{
  // 0.6 m is 2x the 0.3 m threshold -> Huber weight = 0.5.
  const auto soft = sg::evaluateSoftOdomCorrectionGate(
    default_soft_params(), {0.6, 1.0, true});
  assert(nearlyEqual(soft.ndt_weight, 0.5));
  assert(!soft.floored);
}

void test_more_suspicious_axis_dominates_the_combined_weight()
{
  // Translation is 10x over (weight 0.1); yaw is well inside its own
  // threshold (weight 1.0). The combined weight must follow translation's,
  // matching the hard gate's OR-style "either axis can reject" semantics.
  const auto soft = sg::evaluateSoftOdomCorrectionGate(
    default_soft_params(), {3.0, 1.0, true});
  assert(nearlyEqual(soft.ndt_weight, 0.1));
}

void test_extreme_spike_floors_to_zero_like_the_hard_gate()
{
  // 15 m vs 0.3 m: Huber weight would be 0.02, under the 0.05 floor.
  const auto soft = sg::evaluateSoftOdomCorrectionGate(
    default_soft_params(), {15.0, 1.0, true});
  assert(nearlyEqual(soft.ndt_weight, 0.0));
  assert(soft.floored);

  const auto hard = evaluateMeasurementGate(
    default_hard_params(),
    ll::makeMeasurementGateInput(0.5, 0.0, 0.0, 15.0, 1.0, 0, true));
  assert(hard.reject_measurement);  // both agree: discard this one
}

void test_disabled_or_not_applicable_always_fully_trusts_ndt()
{
  sg::SoftOdomCorrectionGateParams disabled = default_soft_params();
  disabled.enable = false;
  assert(nearlyEqual(
    sg::evaluateSoftOdomCorrectionGate(disabled, {5.0, 10.0, true}).ndt_weight, 1.0));

  assert(nearlyEqual(
    sg::evaluateSoftOdomCorrectionGate(default_soft_params(), {5.0, 10.0, false}).ndt_weight,
    1.0));
}

void test_blend_pose_endpoints_match_pure_odom_and_pure_ndt()
{
  Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();
  odom_pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

  Eigen::Isometry3d ndt_pose = Eigen::Isometry3d::Identity();
  ndt_pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  ndt_pose.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto at_zero = sg::blendOdomAndNdtPose(odom_pose, ndt_pose, 0.0);
  assert(at_zero.translation().isApprox(odom_pose.translation(), 1e-9));
  assert(Eigen::Quaterniond(at_zero.rotation()).isApprox(Eigen::Quaterniond(odom_pose.rotation()), 1e-9));

  const auto at_one = sg::blendOdomAndNdtPose(odom_pose, ndt_pose, 1.0);
  assert(at_one.translation().isApprox(ndt_pose.translation(), 1e-9));
  assert(Eigen::Quaterniond(at_one.rotation()).isApprox(Eigen::Quaterniond(ndt_pose.rotation()), 1e-9));
}

void test_blend_pose_at_half_weight_interpolates_translation()
{
  Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d ndt_pose = Eigen::Isometry3d::Identity();
  ndt_pose.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);

  const auto blended = sg::blendOdomAndNdtPose(odom_pose, ndt_pose, 0.5);
  assert(blended.translation().isApprox(Eigen::Vector3d(0.5, 1.0, 0.0), 1e-9));
}

int main()
{
  test_within_threshold_both_gates_fully_trust_ndt();
  test_just_past_threshold_hard_gate_fully_rejects_soft_gate_tapers();
  test_moderate_overshoot_soft_gate_splits_trust();
  test_more_suspicious_axis_dominates_the_combined_weight();
  test_extreme_spike_floors_to_zero_like_the_hard_gate();
  test_disabled_or_not_applicable_always_fully_trusts_ndt();
  test_blend_pose_endpoints_match_pure_odom_and_pure_ndt();
  test_blend_pose_at_half_weight_interpolates_translation();
  std::printf("all soft_odom_correction_gate tests passed\n");
  return 0;
}
