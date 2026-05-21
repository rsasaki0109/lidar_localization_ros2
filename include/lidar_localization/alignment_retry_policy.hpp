#ifndef LIDAR_LOCALIZATION_ALIGNMENT_RETRY_POLICY_HPP_
#define LIDAR_LOCALIZATION_ALIGNMENT_RETRY_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "lidar_localization/measurement_gate_policy.hpp"

namespace lidar_localization
{

struct RecoveryRetryFromLastPoseParams
{
  bool enable{false};
  int min_rejections{1};
  double max_accepted_gap_sec{1.0};
  double max_seed_translation_m{1000000000.0};
};

struct RecoveryRetryFromLastPoseInput
{
  bool have_last_accepted_pose{false};
  std::size_t consecutive_rejected_updates{0};
  double selected_accepted_gap_sec{std::numeric_limits<double>::quiet_NaN()};
  double fallback_accepted_gap_sec{std::numeric_limits<double>::quiet_NaN()};
  double selected_seed_translation_since_accept_m{
    std::numeric_limits<double>::quiet_NaN()};
};

struct RecoveryRetryFromLastPoseDecision
{
  bool should_retry{false};
  std::string reason{"disabled"};
  double accepted_gap_sec{std::numeric_limits<double>::quiet_NaN()};
};

inline RecoveryRetryFromLastPoseParams makeRecoveryRetryFromLastPoseParams(
  bool enable,
  int min_rejections,
  double max_accepted_gap_sec,
  double max_seed_translation_m)
{
  return {
    enable,
    min_rejections,
    max_accepted_gap_sec,
    max_seed_translation_m};
}

inline double computeRecoveryRetryFallbackAcceptedGapSec(
  bool have_last_accepted_pose,
  double scan_stamp_sec,
  double last_accepted_pose_time_sec)
{
  if (!have_last_accepted_pose) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(0.0, scan_stamp_sec - last_accepted_pose_time_sec);
}

inline RecoveryRetryFromLastPoseInput makeRecoveryRetryFromLastPoseInput(
  bool have_last_accepted_pose,
  std::size_t consecutive_rejected_updates,
  double selected_accepted_gap_sec,
  double fallback_accepted_gap_sec,
  double selected_seed_translation_since_accept_m)
{
  return {
    have_last_accepted_pose,
    consecutive_rejected_updates,
    selected_accepted_gap_sec,
    fallback_accepted_gap_sec,
    selected_seed_translation_since_accept_m};
}

inline RecoveryRetryFromLastPoseDecision decideRecoveryRetryFromLastPose(
  const RecoveryRetryFromLastPoseParams & params,
  const RecoveryRetryFromLastPoseInput & input)
{
  RecoveryRetryFromLastPoseDecision decision;
  if (!params.enable) {
    decision.reason = "disabled";
    return decision;
  }
  if (!input.have_last_accepted_pose) {
    decision.reason = "no_last_accepted_pose";
    return decision;
  }
  if (static_cast<int>(input.consecutive_rejected_updates) < params.min_rejections) {
    decision.reason = "not_enough_rejections";
    return decision;
  }

  decision.accepted_gap_sec = std::isfinite(input.selected_accepted_gap_sec) ?
    input.selected_accepted_gap_sec : input.fallback_accepted_gap_sec;
  if (!std::isfinite(decision.accepted_gap_sec)) {
    decision.reason = "accepted_gap_unavailable";
    return decision;
  }
  if (decision.accepted_gap_sec > params.max_accepted_gap_sec) {
    decision.reason = "accepted_gap_too_large";
    return decision;
  }
  if (
    std::isfinite(input.selected_seed_translation_since_accept_m) &&
    input.selected_seed_translation_since_accept_m > params.max_seed_translation_m)
  {
    decision.reason = "seed_translation_too_large";
    return decision;
  }

  decision.should_retry = true;
  decision.reason = "retry_allowed";
  return decision;
}

inline bool isRecoveryRetryAlignmentUsable(bool target_ready, bool has_converged)
{
  return target_ready && has_converged;
}

inline bool shouldUseRecoveryRetryResult(
  bool target_ready,
  bool has_converged,
  bool retry_gate_reject_measurement)
{
  return isRecoveryRetryAlignmentUsable(target_ready, has_converged) &&
         !retry_gate_reject_measurement;
}

inline MeasurementGateDecision makeRecoveryRetryRecoveredGateDecision(
  const MeasurementGateDecision & gate)
{
  MeasurementGateDecision recovered_gate = gate;
  recovered_gate.status_level = kMeasurementGateWarn;
  recovered_gate.status_message = "recovery_retry_from_last_pose_recovered";
  recovered_gate.reject_measurement = false;
  recovered_gate.rejected_seed_update_applied = false;
  return recovered_gate;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_ALIGNMENT_RETRY_POLICY_HPP_
