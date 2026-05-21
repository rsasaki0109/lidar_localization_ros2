#include "lidar_localization/scan_admission_policy.hpp"

#include <cassert>

namespace ll = lidar_localization;

ll::ScanAdmissionInput ready_input()
{
  ll::ScanAdmissionInput input;
  input.has_scan_message = true;
  input.map_received = true;
  input.initial_pose_received = true;
  return input;
}

void test_shutdown_and_null_scan_do_not_store_or_update_time()
{
  auto input = ready_input();
  input.shutting_down = true;
  const auto shutting_down = ll::decideScanAdmission(input);
  assert(!shutting_down.accepted);
  assert(shutting_down.status == ll::ScanAdmissionStatus::kShuttingDown);
  assert(!shutting_down.should_store_last_scan);
  assert(!shutting_down.should_update_last_process_time);

  input = ready_input();
  input.has_scan_message = false;
  const auto null_scan = ll::decideScanAdmission(input);
  assert(!null_scan.accepted);
  assert(null_scan.status == ll::ScanAdmissionStatus::kNullScan);
  assert(null_scan.should_warn_null_scan);
  assert(!null_scan.should_store_last_scan);
}

void test_waiting_for_map_or_pose_stores_last_scan_without_processing()
{
  auto input = ready_input();
  input.map_received = false;
  const auto waiting_for_map = ll::decideScanAdmission(input);
  assert(!waiting_for_map.accepted);
  assert(waiting_for_map.status == ll::ScanAdmissionStatus::kWaitingForMapOrInitialPose);
  assert(waiting_for_map.should_store_last_scan);
  assert(!waiting_for_map.should_update_last_process_time);

  input = ready_input();
  input.initial_pose_received = false;
  const auto waiting_for_pose = ll::decideScanAdmission(input);
  assert(!waiting_for_pose.accepted);
  assert(waiting_for_pose.status == ll::ScanAdmissionStatus::kWaitingForMapOrInitialPose);
  assert(waiting_for_pose.should_store_last_scan);
}

void test_min_scan_interval_throttles_without_time_update()
{
  auto input = ready_input();
  input.min_scan_interval_sec = 0.1;
  input.has_last_process_time = true;
  input.elapsed_since_last_process_sec = 0.05;

  const auto decision = ll::decideScanAdmission(input);
  assert(!decision.accepted);
  assert(decision.status == ll::ScanAdmissionStatus::kThrottledByMinScanInterval);
  assert(decision.should_store_last_scan);
  assert(!decision.should_update_last_process_time);

  input.elapsed_since_last_process_sec = 0.1;
  assert(ll::decideScanAdmission(input).accepted);
}

void test_crop_failure_guard_updates_time_and_only_activates_once()
{
  auto input = ready_input();
  input.consecutive_crop_failures = 101;
  input.crop_failure_guard_active = false;
  input.have_last_accepted_pose = true;

  const auto first_guard = ll::decideScanAdmission(input);
  assert(!first_guard.accepted);
  assert(first_guard.status == ll::ScanAdmissionStatus::kCropFailureGuard);
  assert(first_guard.should_store_last_scan);
  assert(first_guard.should_update_last_process_time);
  assert(first_guard.should_activate_crop_failure_guard);
  assert(first_guard.should_reset_prediction_to_last_accepted_pose);
  assert(first_guard.should_log_crop_failure_guard_activation);

  input.crop_failure_guard_active = true;
  const auto active_guard = ll::decideScanAdmission(input);
  assert(!active_guard.accepted);
  assert(active_guard.should_update_last_process_time);
  assert(!active_guard.should_activate_crop_failure_guard);
  assert(!active_guard.should_reset_prediction_to_last_accepted_pose);
  assert(!active_guard.should_log_crop_failure_guard_activation);
}

void test_ready_scan_is_accepted_and_updates_time()
{
  const auto decision = ll::decideScanAdmission(ready_input());
  assert(decision.accepted);
  assert(ll::isScanAdmissionAccepted(decision.status));
  assert(decision.should_store_last_scan);
  assert(decision.should_update_last_process_time);
}

int main()
{
  test_shutdown_and_null_scan_do_not_store_or_update_time();
  test_waiting_for_map_or_pose_stores_last_scan_without_processing();
  test_min_scan_interval_throttles_without_time_update();
  test_crop_failure_guard_updates_time_and_only_activates_once();
  test_ready_scan_is_accepted_and_updates_time();
  return 0;
}
