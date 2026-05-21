#ifndef LIDAR_LOCALIZATION_REINITIALIZATION_REQUEST_OUTPUT_POLICY_HPP_
#define LIDAR_LOCALIZATION_REINITIALIZATION_REQUEST_OUTPUT_POLICY_HPP_

#include "lidar_localization/recovery_supervisor.hpp"

#include <string>

namespace lidar_localization
{

struct ReinitializationRequestOutputInput
{
  bool output_enabled{false};
  bool publisher_ready{false};
  ReinitializationRequestDecision decision;
};

struct ReinitializationRequestOutputState
{
  bool requested{false};
  std::string reason{"not_requested"};
  double score{0.0};
};

struct ReinitializationRequestOutput
{
  ReinitializationRequestOutputState state;
  bool should_publish{false};
  bool message_value{false};
};

inline ReinitializationRequestOutput prepareReinitializationRequestOutput(
  const ReinitializationRequestOutputInput & input)
{
  return {
    ReinitializationRequestOutputState{
      input.decision.requested,
      input.decision.reason,
      input.decision.score},
    input.output_enabled && input.publisher_ready,
    input.decision.requested};
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REINITIALIZATION_REQUEST_OUTPUT_POLICY_HPP_
