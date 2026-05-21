#ifndef LIDAR_LOCALIZATION_RECOVERY_SUPERVISOR_STATE_POLICY_HPP_
#define LIDAR_LOCALIZATION_RECOVERY_SUPERVISOR_STATE_POLICY_HPP_

#include "lidar_localization/recovery_supervisor.hpp"

#include <cstddef>
#include <string>

namespace lidar_localization
{

struct RecoverySupervisorRuntimeState
{
  RecoverySupervisorState state{RecoverySupervisorState::kTracking};
  std::string action{"accept_measurement"};
  double entered_stamp_sec{0.0};
  std::size_t transition_count{0};
};

struct RecoverySupervisorStateUpdateInput
{
  RecoverySupervisorRuntimeState current;
  RecoverySupervisorState next_state{RecoverySupervisorState::kTracking};
  std::string next_action{"accept_measurement"};
  double stamp_sec{0.0};
};

struct RecoverySupervisorStateUpdate
{
  RecoverySupervisorRuntimeState state;
  RecoverySupervisorState previous_state{RecoverySupervisorState::kTracking};
  bool transitioned{false};
};

inline RecoverySupervisorStateUpdate updateRecoverySupervisorRuntimeState(
  const RecoverySupervisorStateUpdateInput & input)
{
  RecoverySupervisorRuntimeState state = input.current;
  if (state.entered_stamp_sec <= 0.0) {
    state.entered_stamp_sec = input.stamp_sec;
  }

  const RecoverySupervisorState previous_state = state.state;
  const bool transitioned = input.next_state != state.state;
  if (transitioned) {
    state.state = input.next_state;
    state.entered_stamp_sec = input.stamp_sec;
    ++state.transition_count;
  }
  state.action = input.next_action;

  return {state, previous_state, transitioned};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_RECOVERY_SUPERVISOR_STATE_POLICY_HPP_
