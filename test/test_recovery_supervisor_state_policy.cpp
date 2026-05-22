#include "lidar_localization/recovery_supervisor_state_policy.hpp"

#include <cassert>
#include <cmath>

namespace ll = lidar_localization;

bool near(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1.0e-12;
}

void test_initializes_entered_stamp_without_transition()
{
  const auto result = ll::updateRecoverySupervisorRuntimeState(
    ll::RecoverySupervisorStateUpdateInput{
      ll::RecoverySupervisorRuntimeState{},
      ll::RecoverySupervisorState::kTracking,
      "accept_measurement",
      12.5});

  assert(!result.transitioned);
  assert(result.previous_state == ll::RecoverySupervisorState::kTracking);
  assert(result.state.state == ll::RecoverySupervisorState::kTracking);
  assert(result.state.action == "accept_measurement");
  assert(near(result.state.entered_stamp_sec, 12.5));
  assert(result.state.transition_count == 0);
}

void test_transition_updates_state_stamp_and_count()
{
  const auto result = ll::updateRecoverySupervisorRuntimeState(
    ll::RecoverySupervisorStateUpdateInput{
      ll::RecoverySupervisorRuntimeState{
        ll::RecoverySupervisorState::kTracking,
        "accept_measurement",
        4.0,
        2},
      ll::RecoverySupervisorState::kRecovering,
      "advance_prediction_without_measurement",
      20.0});

  assert(result.transitioned);
  assert(result.previous_state == ll::RecoverySupervisorState::kTracking);
  assert(result.state.state == ll::RecoverySupervisorState::kRecovering);
  assert(result.state.action == "advance_prediction_without_measurement");
  assert(near(result.state.entered_stamp_sec, 20.0));
  assert(result.state.transition_count == 3);
}

void test_same_state_preserves_existing_stamp_and_count()
{
  const auto result = ll::updateRecoverySupervisorRuntimeState(
    ll::RecoverySupervisorStateUpdateInput{
      ll::RecoverySupervisorRuntimeState{
        ll::RecoverySupervisorState::kDegraded,
        "accept_measurement_with_warning",
        8.0,
        5},
      ll::RecoverySupervisorState::kDegraded,
      "reject_measurement",
      30.0});

  assert(!result.transitioned);
  assert(result.previous_state == ll::RecoverySupervisorState::kDegraded);
  assert(result.state.state == ll::RecoverySupervisorState::kDegraded);
  assert(result.state.action == "reject_measurement");
  assert(near(result.state.entered_stamp_sec, 8.0));
  assert(result.state.transition_count == 5);
}

int main()
{
  test_initializes_entered_stamp_without_transition();
  test_transition_updates_state_stamp_and_count();
  test_same_state_preserves_existing_stamp_and_count();
  return 0;
}
