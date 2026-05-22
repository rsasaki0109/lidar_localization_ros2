#ifndef LIDAR_LOCALIZATION_SCAN_ADMISSION_POLICY_HPP_
#define LIDAR_LOCALIZATION_SCAN_ADMISSION_POLICY_HPP_

namespace lidar_localization
{

enum class ScanAdmissionStatus
{
  kAccepted = 0,
  kShuttingDown,
  kNullScan,
  kWaitingForMapOrInitialPose,
  kThrottledByMinScanInterval,
  kCropFailureGuard,
};

struct ScanAdmissionInput
{
  bool shutting_down{false};
  bool has_scan_message{false};
  bool map_received{false};
  bool initial_pose_received{false};
  double min_scan_interval_sec{0.0};
  bool has_last_process_time{false};
  double elapsed_since_last_process_sec{0.0};
  int consecutive_crop_failures{0};
  bool crop_failure_guard_active{false};
  bool have_last_accepted_pose{false};
  int crop_failure_threshold{100};
};

struct ScanAdmissionDecision
{
  ScanAdmissionStatus status{ScanAdmissionStatus::kAccepted};
  bool accepted{true};
  bool should_store_last_scan{true};
  bool should_warn_null_scan{false};
  bool should_update_last_process_time{true};
  bool should_activate_crop_failure_guard{false};
  bool should_reset_prediction_to_last_accepted_pose{false};
  bool should_log_crop_failure_guard_activation{false};
};

inline ScanAdmissionDecision rejectScanAdmission(
  ScanAdmissionStatus status,
  bool should_store_last_scan,
  bool should_warn_null_scan = false,
  bool should_update_last_process_time = false)
{
  ScanAdmissionDecision decision;
  decision.status = status;
  decision.accepted = false;
  decision.should_store_last_scan = should_store_last_scan;
  decision.should_warn_null_scan = should_warn_null_scan;
  decision.should_update_last_process_time = should_update_last_process_time;
  return decision;
}

inline ScanAdmissionDecision decideScanAdmission(const ScanAdmissionInput & input)
{
  if (input.shutting_down) {
    return rejectScanAdmission(ScanAdmissionStatus::kShuttingDown, false);
  }
  if (!input.has_scan_message) {
    return rejectScanAdmission(ScanAdmissionStatus::kNullScan, false, true);
  }
  if (!input.map_received || !input.initial_pose_received) {
    return rejectScanAdmission(ScanAdmissionStatus::kWaitingForMapOrInitialPose, true);
  }
  if (
    input.min_scan_interval_sec > 0.0 &&
    input.has_last_process_time &&
    input.elapsed_since_last_process_sec < input.min_scan_interval_sec)
  {
    return rejectScanAdmission(ScanAdmissionStatus::kThrottledByMinScanInterval, true);
  }
  if (input.consecutive_crop_failures > input.crop_failure_threshold) {
    ScanAdmissionDecision decision =
      rejectScanAdmission(ScanAdmissionStatus::kCropFailureGuard, true, false, true);
    decision.should_activate_crop_failure_guard = !input.crop_failure_guard_active;
    decision.should_log_crop_failure_guard_activation = decision.should_activate_crop_failure_guard;
    decision.should_reset_prediction_to_last_accepted_pose =
      decision.should_activate_crop_failure_guard && input.have_last_accepted_pose;
    return decision;
  }
  return {};
}

inline bool isScanAdmissionAccepted(ScanAdmissionStatus status)
{
  return status == ScanAdmissionStatus::kAccepted;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_SCAN_ADMISSION_POLICY_HPP_
