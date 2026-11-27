#include "lidar_localization/scan_tracker.hpp"

#include <cmath>

namespace lidar_localization
{

void ScanTracker::reset(const Eigen::Matrix4f & pose_matrix, double stamp_sec)
{
  prediction = resetPredictionState(pose_matrix, stamp_sec);
  settled_accepts_since_reset = 0;
  seed_correction_window.clear();
  seed_correction_release_history.clear();
  seed_correction_agreement_history.clear();
  accepted_correction_history.clear();
  last_gate_agreement_released = false;
}

void ScanTracker::acceptMeasurement(
  const Eigen::Matrix4f & accepted_pose_matrix, double stamp_sec,
  const MeasurementGateParamConfig & gate_config)
{
  // Store the pose at its timestamp in every prediction mode. Extrapolating
  // here makes a later switch to twist integrate the same interval twice.
  prediction = updatePredictionStateFromAcceptedMeasurement(
    prediction, accepted_pose_matrix, stamp_sec, false);
  if (last_gate_agreement_released) {
    // The agreed correction is the new reference: the guards stay armed (their
    // warmup is not restarted) and the window guard measures the corrections
    // after it.
    seed_correction_window.clear();
  } else {
    settled_accepts_since_reset = nextSettledAccepts(
      settled_accepts_since_reset, makeMeasurementGateParams(gate_config),
      last_gated_correction_translation_m, last_gated_correction_yaw_deg,
      prediction.accepted_updates_since_reset == 1);
    if (gate_config.enable_seed_correction_window_guard) {
      seed_correction_window.add(
        stamp_sec, last_gated_correction_map_dx, last_gated_correction_map_dy,
        gate_config.seed_correction_window_guard_sec);
    }
  }
  if (gate_config.seed_correction_guard_agreement_release_scans > 0) {
    accepted_correction_history.add(
      stamp_sec, last_gated_correction_map_dx, last_gated_correction_map_dy,
      gate_config.seed_correction_guard_agreement_release_drift_sec);
  }
  last_gate_agreement_released = false;
}

void ScanTracker::advanceWithoutMeasurement(
  double stamp_sec, PredictionAdvanceMode mode,
  const Eigen::Matrix4f & twist_predicted_pose_matrix, double previous_delta_max_extrapolation_sec)
{
  prediction = advancePredictionWithoutMeasurement(
    prediction, stamp_sec, mode, twist_predicted_pose_matrix,
    previousDeltaMotionAt(stamp_sec, previous_delta_max_extrapolation_sec));
}

void ScanTracker::rejectMeasurement(const Eigen::Matrix4f & rejected_pose_matrix, double stamp_sec)
{
  prediction = updatePredictionFromRejectedMeasurement(prediction, rejected_pose_matrix, stamp_sec);
}

Eigen::Matrix4f ScanTracker::previousDeltaMotionAt(
  double stamp_sec, double previous_delta_max_extrapolation_sec) const
{
  return lidar_localization::previousDeltaMotionAt(
    prediction, stamp_sec, previous_delta_max_extrapolation_sec);
}

void ScanTracker::fillSeedMetrics(AlignmentAttempt & attempt, double scan_stamp_sec) const
{
  const auto seed_metrics = computeAlignmentSeedMetrics(
    prediction.have_last_accepted_pose,
    prediction.last_accepted_pose_matrix,
    attempt.init_guess,
    scan_stamp_sec,
    prediction.last_accepted_pose_time_sec);
  attempt.seed_translation_since_accept_m = seed_metrics.translation_since_accept_m;
  attempt.seed_yaw_since_accept_deg = seed_metrics.yaw_since_accept_deg;
  attempt.accepted_gap_sec = seed_metrics.accepted_gap_sec;
}

double ScanTracker::signedCorrectionYawDeg(const AlignmentAttempt & attempt)
{
  const auto yaw_of = [](const Eigen::Matrix4f & pose) {
      return std::atan2(static_cast<double>(pose(1, 0)), static_cast<double>(pose(0, 0)));
    };
  return std::remainder(
    yaw_of(attempt.final_transformation) - yaw_of(attempt.init_guess), 2.0 * M_PI) *
         180.0 / M_PI;
}

void ScanTracker::recordReleaseHistory(
  const AlignmentAttempt & attempt, const MeasurementGateDecision & gate,
  const MeasurementGateParamConfig & gate_config)
{
  const int release_scans = gate_config.seed_correction_guard_agreement_release_scans;
  if (release_scans > 0) {
    const double dx = attempt.final_transformation(0, 3) - attempt.init_guess(0, 3);
    const double dy = attempt.final_transformation(1, 3) - attempt.init_guess(1, 3);
    if (gate.status_message == kSeedCorrectionGuardAgreementReleased) {
      seed_correction_agreement_history.clear();
    } else if (!gate.reject_measurement) {
      if (attempt.has_converged) {
        seed_correction_agreement_history.shift(dx, dy, signedCorrectionYawDeg(attempt));
      }
    } else if (
      attempt.has_converged && isSeedCorrectionGuardRejection(gate.status_message) &&
      attempt.fitness_score <= gate.effective_score_threshold)
    {
      seed_correction_agreement_history.add(
        gate_scan_stamp_sec, dx, dy, signedCorrectionYawDeg(attempt),
        static_cast<std::size_t>(release_scans));
    }
  }
  const int agreement_scans = gate_config.seed_correction_guard_release_agreement_scans;
  if (agreement_scans <= 0) {
    return;
  }
  if (!gate.reject_measurement) {
    seed_correction_release_history.clear();
    return;
  }
  if (attempt.has_converged && isSeedCorrectionGuardRejection(gate.status_message)) {
    seed_correction_release_history.add(
      attempt.final_transformation(0, 3) - attempt.init_guess(0, 3),
      attempt.final_transformation(1, 3) - attempt.init_guess(1, 3),
      signedCorrectionYawDeg(attempt),
      static_cast<std::size_t>(agreement_scans));
  }
}

MeasurementGateDecision ScanTracker::gateDecision(
  const AlignmentAttempt & attempt, const MeasurementGateParamConfig & gate_config) const
{
  auto input = makeMeasurementGateInput(
    attempt.fitness_score,
    attempt.accepted_gap_sec,
    attempt.seed_translation_since_accept_m,
    attempt.correction_translation_m,
    attempt.correction_yaw_deg,
    prediction.consecutive_rejected_updates,
    settled_accepts_since_reset);
  if (gate_config.enable_seed_correction_window_guard && attempt.has_converged) {
    input.window_correction_translation_m = seed_correction_window.sumNormWith(
      gate_scan_stamp_sec,
      attempt.final_transformation(0, 3) - attempt.init_guess(0, 3),
      attempt.final_transformation(1, 3) - attempt.init_guess(1, 3),
      gate_config.seed_correction_window_guard_sec);
  }
  const int agreement_scans = gate_config.seed_correction_guard_release_agreement_scans;
  if (
    agreement_scans > 0 &&
    static_cast<int>(prediction.consecutive_rejected_updates) >=
    gate_config.seed_correction_guard_release_rejections)
  {
    input.release_agreed = attempt.has_converged &&
      seed_correction_release_history.agrees(
      attempt.final_transformation(0, 3) - attempt.init_guess(0, 3),
      attempt.final_transformation(1, 3) - attempt.init_guess(1, 3),
      signedCorrectionYawDeg(attempt),
      static_cast<std::size_t>(agreement_scans),
      gate_config.seed_correction_guard_release_agreement_translation_m,
      gate_config.seed_correction_guard_release_agreement_yaw_deg);
  }
  const int release_scans = gate_config.seed_correction_guard_agreement_release_scans;
  if (release_scans > 0 && attempt.has_converged) {
    const double dx = attempt.final_transformation(0, 3) - attempt.init_guess(0, 3);
    const double dy = attempt.final_transformation(1, 3) - attempt.init_guess(1, 3);
    // Only a horizontal correction beyond the per-scan guard: one that trips it
    // by its height or yaw alone has nothing to agree on here.
    input.agreement_released =
      std::hypot(dx, dy) > gate_config.seed_correction_guard_translation_m &&
      accepted_correction_history.sumNormWith(
      gate_scan_stamp_sec, dx, dy,
      gate_config.seed_correction_guard_agreement_release_drift_sec) <=
      gate_config.seed_correction_guard_agreement_release_drift_tolerance_m &&
      seed_correction_agreement_history.agrees(
      gate_scan_stamp_sec, dx, dy, signedCorrectionYawDeg(attempt),
      static_cast<std::size_t>(release_scans - 1),
      gate_config.seed_correction_guard_agreement_release_translation_m,
      gate_config.seed_correction_guard_agreement_release_yaw_deg,
      gate_config.seed_correction_guard_agreement_release_window_sec);
  }
  return evaluateMeasurementGate(makeMeasurementGateParams(gate_config), input);
}

MeasurementGateDecision ScanTracker::evaluateGate(
  const AlignmentAttempt & attempt, const MeasurementGateParamConfig & gate_config)
{
  last_gated_correction_translation_m = attempt.correction_translation_m;
  last_gated_correction_yaw_deg = attempt.correction_yaw_deg;
  last_gated_correction_map_dx = attempt.has_converged ?
    attempt.final_transformation(0, 3) - attempt.init_guess(0, 3) : 0.0;
  last_gated_correction_map_dy = attempt.has_converged ?
    attempt.final_transformation(1, 3) - attempt.init_guess(1, 3) : 0.0;
  auto decision = gateDecision(attempt, gate_config);
  last_gate_agreement_released =
    decision.status_message == kSeedCorrectionGuardAgreementReleased;
  return decision;
}

void scoreAlignment(
  const RegistrationEngine & engine, const RegistrationAlignResult & aligned,
  const AlignmentScoring & scoring, AlignmentAttempt & attempt)
{
  attempt.alignment_time_sec = aligned.alignment_time_sec;
  attempt.has_converged = aligned.has_converged;
  attempt.registration_iterations = aligned.iterations;
  // The inlier ratio comes from the same nearest-neighbour searches as the fitness, so
  // it is reported whether or not the gate reads it.
  const auto fitness =
    engine.fitness(scoring.inlier_ratio_distance_m, scoring.inlier_ratio_min_points);
  attempt.registration_fitness_score = fitness.score;
  attempt.inlier_ratio = fitness.inlier_ratio;
  attempt.fitness_score = scoring.min_inlier_ratio > 0.0 ?
    inlierRatioGateScore(fitness.inlier_ratio, scoring.min_inlier_ratio, scoring.score_threshold) :
    fitness.score;
  if (scoring.localizability) {
    if (const auto localizability = engine.localizability()) {
      attempt.registration_localizability = *localizability;
    }
  }
  if (attempt.has_converged) {
    attempt.final_transformation = aligned.final_transformation;
    const auto correction_metrics =
      computeAlignmentCorrectionMetrics(attempt.init_guess, attempt.final_transformation);
    attempt.correction_translation_m = correction_metrics.translation_m;
    attempt.correction_yaw_deg = correction_metrics.yaw_deg;
  }
}

}  // namespace lidar_localization
