// What lidar_localization remembers from scan to scan, without ROS: the
// prediction state (the last accepted pose, the predicted pose, the motion
// between them) and the measurement gate's own state (settled accepts since a
// reset, the windowed seed correction guard, the release agreement), with the
// operations the node runs on them for every scan. Also the scoring of one
// alignment (scoreAlignment).
//
// PCLLocalization (the ROS 2 node) keeps one ScanTracker; so does
// lidar_localization_core's C API (lidarloc_localize), which runs the node's
// per-scan flow without the IMU paths.
#pragma once

#include <cstddef>
#include <limits>

#include <Eigen/Core>

#include "lidar_localization/alignment_attempt_policy.hpp"
#include "lidar_localization/alignment_pipeline_policy.hpp"
#include "lidar_localization/measurement_gate_policy.hpp"
#include "lidar_localization/prediction_state_policy.hpp"
#include "lidar_localization/registration_engine.hpp"

namespace lidar_localization
{

class ScanTracker
{
public:
  PredictionState prediction;
  // Seed correction guard warmup: consecutive small-correction accepts since
  // the last reset or odometry gap, and the correction of the last gated scan.
  std::size_t settled_accepts_since_reset{0};
  double last_gated_correction_translation_m{std::numeric_limits<double>::infinity()};
  double last_gated_correction_yaw_deg{std::numeric_limits<double>::infinity()};
  // Windowed seed correction guard: map-frame horizontal correction of the last
  // gated scan, the stamp of the scan being gated, and the accepted corrections.
  double last_gated_correction_map_dx{0.0};
  double last_gated_correction_map_dy{0.0};
  double gate_scan_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  SeedCorrectionWindow seed_correction_window;
  // Release agreement: corrections of the latest guard-rejected scans.
  SeedCorrectionReleaseHistory seed_correction_release_history;
  // Agreement release: stamped corrections of the latest guard-rejected scans,
  // the accepted corrections within its drift horizon, and whether the last
  // gated scan was released by it.
  SeedCorrectionAgreementHistory seed_correction_agreement_history;
  SeedCorrectionWindow accepted_correction_history;
  bool last_gate_agreement_released{false};

  // Starts over at a pose (an initial pose, a reinitialization).
  void reset(const Eigen::Matrix4f & pose_matrix, double stamp_sec);
  // An accepted measurement: the pose at its stamp, the settled-accept count
  // and the windowed guard's history.
  void acceptMeasurement(
    const Eigen::Matrix4f & accepted_pose_matrix, double stamp_sec,
    const MeasurementGateParamConfig & gate_config);
  // No measurement: the prediction advances as `mode` says, with the
  // twist-predicted pose for kTwistPrediction.
  void advanceWithoutMeasurement(
    double stamp_sec, PredictionAdvanceMode mode,
    const Eigen::Matrix4f & twist_predicted_pose_matrix, double previous_delta_max_extrapolation_sec);
  // A rejected measurement the backend still moves the prediction with.
  void rejectMeasurement(const Eigen::Matrix4f & rejected_pose_matrix, double stamp_sec);
  // The motion of the last accepted interval, extrapolated to the stamp.
  Eigen::Matrix4f previousDeltaMotionAt(
    double stamp_sec, double previous_delta_max_extrapolation_sec) const;

  // How far the attempt's seed is from the last accepted pose, in distance,
  // yaw and time.
  void fillSeedMetrics(AlignmentAttempt & attempt, double scan_stamp_sec) const;
  // The gate's decision on an attempt (no state changes).
  MeasurementGateDecision gateDecision(
    const AlignmentAttempt & attempt, const MeasurementGateParamConfig & gate_config) const;
  // The gate's decision, remembering the attempt's correction for the next
  // accept (acceptMeasurement).
  MeasurementGateDecision evaluateGate(
    const AlignmentAttempt & attempt, const MeasurementGateParamConfig & gate_config);
  // Keeps the corrections of guard-rejected scans for the release agreement
  // and the agreement release.
  void recordReleaseHistory(
    const AlignmentAttempt & attempt, const MeasurementGateDecision & gate,
    const MeasurementGateParamConfig & gate_config);
  // The attempt's yaw correction, signed, degrees.
  static double signedCorrectionYawDeg(const AlignmentAttempt & attempt);
};

// How one alignment is scored (the node's parameters of the same names).
struct AlignmentScoring
{
  double inlier_ratio_distance_m{0.5};
  std::size_t inlier_ratio_min_points{200};
  // Above 0: the gate reads the inlier ratio, on the fitness scale.
  double min_inlier_ratio{0.0};
  double score_threshold{0.0};
  bool localizability{false};
};

// Fills an attempt from the backend's result: convergence, iterations, the
// fitness and inlier ratio (and the score the gate reads), the localizability,
// and with convergence the pose and its correction from the seed.
void scoreAlignment(
  const RegistrationEngine & engine, const RegistrationAlignResult & aligned,
  const AlignmentScoring & scoring, AlignmentAttempt & attempt);

}  // namespace lidar_localization
