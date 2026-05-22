#ifndef LIDAR_LOCALIZATION_REINITIALIZATION_LATCH_POLICY_HPP_
#define LIDAR_LOCALIZATION_REINITIALIZATION_LATCH_POLICY_HPP_

#include "lidar_localization/recovery_supervisor.hpp"

#include <algorithm>
#include <string>

namespace lidar_localization
{

struct ReinitializationRequestLatchState
{
  bool latched{false};
  std::string reason{"not_requested"};
  double score{0.0};
  double stamp_sec{0.0};
};

struct ReinitializationRequestLatchInput
{
  bool enabled{false};
  ReinitializationRequestLatchState state;
  ReinitializationRequestDecision decision;
  double stamp_sec{0.0};
};

struct ReinitializationRequestLatchResult
{
  ReinitializationRequestLatchState state;
  ReinitializationRequestDecision decision;
};

inline ReinitializationRequestLatchResult applyReinitializationRequestLatch(
  const ReinitializationRequestLatchInput & input)
{
  if (!input.enabled) {
    return {input.state, input.decision};
  }

  ReinitializationRequestLatchState state = input.state;
  if (input.decision.requested) {
    if (!state.latched) {
      state.stamp_sec = input.stamp_sec;
      state.reason = input.decision.reason;
    }
    state.latched = true;
    state.score = std::max(state.score, input.decision.score);
  }

  if (!state.latched) {
    return {state, input.decision};
  }

  ReinitializationRequestDecision decision = input.decision;
  decision.requested = true;
  decision.reason = state.reason;
  decision.score = std::max(state.score, input.decision.score);
  return {state, decision};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REINITIALIZATION_LATCH_POLICY_HPP_
