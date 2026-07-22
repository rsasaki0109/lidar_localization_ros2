#pragma once

#include <cstddef>

namespace glim_prior_map_localizer
{

enum class RecoveryConsensusAction
{
  kPending,
  kActivate,
  kReject,
};

struct RecoveryConsensusState
{
  std::size_t confirmations{0};
  std::size_t failures{0};
};

inline RecoveryConsensusAction updateRecoveryConsensus(
  RecoveryConsensusState & state,
  bool observation_consistent,
  std::size_t required_confirmations,
  std::size_t allowed_failures)
{
  if (observation_consistent) {
    ++state.confirmations;
    state.failures = 0;
    return state.confirmations >= required_confirmations ?
      RecoveryConsensusAction::kActivate : RecoveryConsensusAction::kPending;
  }
  state.confirmations = 0;
  ++state.failures;
  return state.failures >= allowed_failures ?
    RecoveryConsensusAction::kReject : RecoveryConsensusAction::kPending;
}

}  // namespace glim_prior_map_localizer
