#include "lidar_localization/reinitialization_latch_policy.hpp"

#include <cassert>
#include <cmath>
#include <string>

namespace ll = lidar_localization;

bool near(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1.0e-12;
}

ll::ReinitializationRequestDecision request(
  bool requested,
  const std::string & reason,
  double score)
{
  return {requested, reason, score};
}

void test_disabled_returns_decision_and_state_unchanged()
{
  const ll::ReinitializationRequestLatchState state{true, "prior_reason", 0.8, 12.0};
  const auto decision = request(false, "current_reason", 0.2);

  const auto result = ll::applyReinitializationRequestLatch(
    ll::ReinitializationRequestLatchInput{false, state, decision, 30.0});

  assert(result.state.latched == state.latched);
  assert(result.state.reason == state.reason);
  assert(near(result.state.score, state.score));
  assert(near(result.state.stamp_sec, state.stamp_sec));
  assert(result.decision.requested == decision.requested);
  assert(result.decision.reason == decision.reason);
  assert(near(result.decision.score, decision.score));
}

void test_no_request_and_not_latched_returns_unchanged()
{
  const ll::ReinitializationRequestLatchState state;
  const auto decision = request(false, "not_failure", 0.1);

  const auto result = ll::applyReinitializationRequestLatch(
    ll::ReinitializationRequestLatchInput{true, state, decision, 30.0});

  assert(!result.state.latched);
  assert(result.state.reason == "not_requested");
  assert(near(result.state.score, 0.0));
  assert(near(result.state.stamp_sec, 0.0));
  assert(result.decision.requested == decision.requested);
  assert(result.decision.reason == decision.reason);
  assert(near(result.decision.score, decision.score));
}

void test_first_request_latches_stamp_reason_and_score()
{
  const auto decision = request(true, "accepted_gap_reinit_requested", 0.97);

  const auto result = ll::applyReinitializationRequestLatch(
    ll::ReinitializationRequestLatchInput{
      true,
      ll::ReinitializationRequestLatchState{},
      decision,
      42.5});

  assert(result.state.latched);
  assert(result.state.reason == decision.reason);
  assert(near(result.state.score, decision.score));
  assert(near(result.state.stamp_sec, 42.5));
  assert(result.decision.requested);
  assert(result.decision.reason == decision.reason);
  assert(near(result.decision.score, decision.score));
}

void test_latched_state_keeps_prior_reason_stamp_and_score()
{
  const ll::ReinitializationRequestLatchState state{
    true, "first_reinit_requested", 0.92, 10.0};
  const auto decision = request(false, "not_failure", 0.4);

  const auto result = ll::applyReinitializationRequestLatch(
    ll::ReinitializationRequestLatchInput{true, state, decision, 50.0});

  assert(result.state.latched);
  assert(result.state.reason == state.reason);
  assert(near(result.state.score, state.score));
  assert(near(result.state.stamp_sec, state.stamp_sec));
  assert(result.decision.requested);
  assert(result.decision.reason == state.reason);
  assert(near(result.decision.score, state.score));
}

void test_latched_decision_score_uses_current_score_when_higher()
{
  const ll::ReinitializationRequestLatchState state{
    true, "first_reinit_requested", 0.92, 10.0};
  const auto decision = request(false, "fitness_exploded_reinit_requested", 1.4);

  const auto result = ll::applyReinitializationRequestLatch(
    ll::ReinitializationRequestLatchInput{true, state, decision, 50.0});

  assert(result.state.latched);
  assert(result.state.reason == state.reason);
  assert(near(result.state.score, state.score));
  assert(near(result.state.stamp_sec, state.stamp_sec));
  assert(result.decision.requested);
  assert(result.decision.reason == state.reason);
  assert(near(result.decision.score, decision.score));
}

int main()
{
  test_disabled_returns_decision_and_state_unchanged();
  test_no_request_and_not_latched_returns_unchanged();
  test_first_request_latches_stamp_reason_and_score();
  test_latched_state_keeps_prior_reason_stamp_and_score();
  test_latched_decision_score_uses_current_score_when_higher();
  return 0;
}
