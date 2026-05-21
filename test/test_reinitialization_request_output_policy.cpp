#include "lidar_localization/reinitialization_request_output_policy.hpp"

#include <cassert>
#include <cmath>

namespace ll = lidar_localization;

bool near(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1.0e-12;
}

void test_state_mirrors_decision_even_when_output_disabled()
{
  const auto result = ll::prepareReinitializationRequestOutput(
    ll::ReinitializationRequestOutputInput{
      false,
      true,
      ll::ReinitializationRequestDecision{true, "accepted_gap_reinit_requested", 0.96}});

  assert(result.state.requested);
  assert(result.state.reason == "accepted_gap_reinit_requested");
  assert(near(result.state.score, 0.96));
  assert(!result.should_publish);
  assert(result.message_value);
}

void test_publish_requires_output_enabled_and_ready_publisher()
{
  const auto ready_result = ll::prepareReinitializationRequestOutput(
    ll::ReinitializationRequestOutputInput{
      true,
      true,
      ll::ReinitializationRequestDecision{false, "not_failure", 0.2}});
  const auto missing_publisher_result = ll::prepareReinitializationRequestOutput(
    ll::ReinitializationRequestOutputInput{
      true,
      false,
      ll::ReinitializationRequestDecision{true, "requested", 0.9}});

  assert(ready_result.should_publish);
  assert(!ready_result.message_value);
  assert(!missing_publisher_result.should_publish);
  assert(missing_publisher_result.message_value);
}

int main()
{
  test_state_mirrors_decision_even_when_output_disabled();
  test_publish_requires_output_enabled_and_ready_publisher();
  return 0;
}
