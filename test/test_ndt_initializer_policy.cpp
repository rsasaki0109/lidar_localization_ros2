#include "lidar_localization/ndt_initializer_policy.hpp"

#include <cassert>

namespace ll = lidar_localization;

void test_initializer_runs_only_while_enabled_available_and_incomplete()
{
  assert(ll::shouldRunNdtInitializer({true, true, 0, 5}));
  assert(!ll::shouldRunNdtInitializer({false, true, 0, 5}));
  assert(!ll::shouldRunNdtInitializer({true, false, 0, 5}));
  assert(!ll::shouldRunNdtInitializer({true, true, 5, 5}));
  assert(!ll::shouldRunNdtInitializer({true, true, 0, 0}));
}

void test_non_converged_run_keeps_count_and_seed()
{
  const auto decision = ll::updateNdtInitializerProgress(
    ll::NdtInitializerProgressInput{
      ll::NdtInitializerRunInput{true, true, 2, 5},
      false});

  assert(decision.should_run);
  assert(!decision.should_accept_refined_seed);
  assert(decision.next_scan_count == 2);
  assert(!decision.completed);
  assert(!decision.should_reset_initializer);
}

void test_converged_run_accepts_seed_and_advances_count()
{
  const auto decision = ll::updateNdtInitializerProgress(
    ll::NdtInitializerProgressInput{
      ll::NdtInitializerRunInput{true, true, 2, 5},
      true});

  assert(decision.should_run);
  assert(decision.should_accept_refined_seed);
  assert(decision.next_scan_count == 3);
  assert(!decision.completed);
  assert(!decision.should_reset_initializer);
}

void test_last_required_scan_completes_initializer()
{
  const auto decision = ll::updateNdtInitializerProgress(
    ll::NdtInitializerProgressInput{
      ll::NdtInitializerRunInput{true, true, 4, 5},
      true});

  assert(decision.should_run);
  assert(decision.should_accept_refined_seed);
  assert(decision.next_scan_count == 5);
  assert(decision.completed);
  assert(decision.should_reset_initializer);
}

int main()
{
  test_initializer_runs_only_while_enabled_available_and_incomplete();
  test_non_converged_run_keeps_count_and_seed();
  test_converged_run_accepts_seed_and_advances_count();
  test_last_required_scan_completes_initializer();
  return 0;
}
