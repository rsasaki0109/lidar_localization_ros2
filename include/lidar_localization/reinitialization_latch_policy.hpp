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
  int consecutive_clear_samples{0};
};

struct ReinitializationRequestLatchInput
{
  bool enabled{false};
  ReinitializationRequestLatchState state;
  ReinitializationRequestDecision decision;
  double stamp_sec{0.0};
  bool clear_sample{false};
  int clear_samples_required{5};
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
    state.consecutive_clear_samples = 0;
  } else if (state.latched && input.clear_sample) {
    state.consecutive_clear_samples++;
    if (state.consecutive_clear_samples >= std::max(1, input.clear_samples_required)) {
      return {ReinitializationRequestLatchState{}, input.decision};
    }
  } else if (state.latched) {
    state.consecutive_clear_samples = 0;
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
