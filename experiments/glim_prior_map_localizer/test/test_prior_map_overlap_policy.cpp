#include "glim_prior_map_localizer/prior_map_overlap_policy.hpp"
#include "glim_prior_map_localizer/recovery_consensus_policy.hpp"
#include "glim_prior_map_localizer/map_odom_transition_policy.hpp"
#include "glim_prior_map_localizer/map_state_prior.hpp"
#include "glim_prior_map_localizer/recovery_pose_policy.hpp"

#include <cassert>
#include <cmath>

#include <Eigen/Geometry>
#include <gtsam/inference/Symbol.h>

namespace localizer = glim_prior_map_localizer;

int main()
{
  const auto empty = localizer::decidePriorMapOverlap(0, 0, 10, 0.1);
  assert(!empty.sufficient);
  assert(empty.fraction == 0.0);

  const auto too_few = localizer::decidePriorMapOverlap(9, 100, 10, 0.05);
  assert(!too_few.sufficient);

  const auto too_sparse = localizer::decidePriorMapOverlap(10, 100, 10, 0.11);
  assert(!too_sparse.sufficient);

  const auto acquired = localizer::decidePriorMapOverlap(10, 100, 10, 0.10);
  assert(acquired.sufficient);
  assert(acquired.fraction == 0.10);

  const auto reentered = localizer::decidePriorMapOverlap(32, 320, 32, 0.05);
  assert(reentered.sufficient);
  const auto sparse_but_well_aligned = localizer::decidePriorMapOverlap(20, 20, 32, 0.05);
  assert(!localizer::isRecoveryLossEvidence(sparse_but_well_aligned, 32, 0.75));
  const auto populated_normal_overlap = localizer::decidePriorMapOverlap(216, 255, 32, 0.05);
  assert(!localizer::isRecoveryLossEvidence(populated_normal_overlap, 32, 0.75));
  const auto populated_kidnap_overlap = localizer::decidePriorMapOverlap(170, 255, 32, 0.05);
  assert(localizer::isRecoveryLossEvidence(populated_kidnap_overlap, 32, 0.75));

  localizer::RecoveryConsensusState recovery;
  assert(localizer::updateRecoveryConsensus(recovery, true, 3, 2) ==
    localizer::RecoveryConsensusAction::kPending);
  assert(localizer::updateRecoveryConsensus(recovery, false, 3, 2) ==
    localizer::RecoveryConsensusAction::kPending);
  assert(recovery.confirmations == 0);
  assert(localizer::updateRecoveryConsensus(recovery, true, 3, 2) ==
    localizer::RecoveryConsensusAction::kPending);
  assert(localizer::updateRecoveryConsensus(recovery, true, 3, 2) ==
    localizer::RecoveryConsensusAction::kPending);
  assert(localizer::updateRecoveryConsensus(recovery, true, 3, 2) ==
    localizer::RecoveryConsensusAction::kActivate);

  localizer::RecoveryConsensusState rejected;
  assert(localizer::updateRecoveryConsensus(rejected, false, 3, 2) ==
    localizer::RecoveryConsensusAction::kPending);
  assert(localizer::updateRecoveryConsensus(rejected, false, 3, 2) ==
    localizer::RecoveryConsensusAction::kReject);

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(3.0, 4.0, 0.0);
  constexpr double pi = 3.14159265358979323846;
  target.linear() = Eigen::AngleAxisd(
    30.0 * pi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto bounded = localizer::boundMapOdomStep(
    Eigen::Isometry3d::Identity(), target, 1.0, 5.0);
  assert(bounded.translation_limited);
  assert(bounded.rotation_limited);
  assert(std::abs(bounded.transform.translation().norm() - 1.0) < 1.0e-12);
  const Eigen::AngleAxisd bounded_rotation(bounded.transform.linear());
  assert(std::abs(bounded_rotation.angle() - 5.0 * pi / 180.0) < 1.0e-12);

  const auto unchanged = localizer::boundMapOdomStep(target, target, 1.0, 5.0);
  assert(!unchanged.translation_limited);
  assert(!unchanged.rotation_limited);
  assert(unchanged.transform.matrix().isApprox(target.matrix(), 1.0e-12));

  Eigen::Isometry3d far_from_origin = Eigen::Isometry3d::Identity();
  far_from_origin.translation() = Eigen::Vector3d(-87.0, -9.0, -11.0);
  Eigen::Isometry3d nearby_observation = far_from_origin;
  nearby_observation.translation().x() += 0.3;
  nearby_observation.linear() = far_from_origin.linear() * Eigen::AngleAxisd(
    1.0 * pi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto local_correction = localizer::recoveryPoseCorrection(
    far_from_origin, nearby_observation);
  assert(std::abs(local_correction.translation().norm() - 0.3) < 1.0e-12);
  assert(std::abs(Eigen::AngleAxisd(local_correction.linear()).angle() -
    1.0 * pi / 180.0) < 1.0e-12);

  Eigen::Isometry3d candidate_map_from_sensor = Eigen::Isometry3d::Identity();
  candidate_map_from_sensor.translation() = Eigen::Vector3d(100.0, 40.0, -11.0);
  Eigen::Isometry3d candidate_odom_from_sensor = Eigen::Isometry3d::Identity();
  candidate_odom_from_sensor.translation() = Eigen::Vector3d(20.0, 5.0, 0.0);
  const Eigen::Isometry3d candidate_odom_from_map =
    candidate_odom_from_sensor * candidate_map_from_sensor.inverse();
  Eigen::Isometry3d current_odom_from_sensor = candidate_odom_from_sensor;
  current_odom_from_sensor.translation() += Eigen::Vector3d(2.0, -3.0, 0.0);
  const Eigen::Isometry3d propagated = localizer::propagateRecoveryPose(
    candidate_odom_from_map, current_odom_from_sensor);
  assert((propagated.translation() - Eigen::Vector3d(102.0, 37.0, -11.0)).norm() < 1.0e-9);
  assert(localizer::recoveryRearmAllowed(-1.0, 100.0, 30.0));
  assert(!localizer::recoveryRearmAllowed(100.0, 129.9, 30.0));
  assert(localizer::recoveryRearmAllowed(100.0, 130.0, 30.0));

  gtsam::NonlinearFactorGraph keep_alive_factors;
  const gtsam::Key map_key = gtsam::Symbol('m', 2);
  localizer::addMapStateKeepAlivePrior(
    keep_alive_factors, map_key, gtsam::Pose3(), 1.0);
  assert(keep_alive_factors.size() == 1);
  assert(keep_alive_factors.at(0)->keys().size() == 1);
  assert(keep_alive_factors.at(0)->keys().front() == map_key);
  return 0;
}
