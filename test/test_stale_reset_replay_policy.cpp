#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "lidar_localization/stale_reset_replay_policy.hpp"

namespace ll = lidar_localization;

void testPlan()
{
  ll::StaleResetReplayParams params;
  // Disabled by default.
  auto plan = ll::planStaleResetReplay(params, 100.0, 103.0, 100.0);
  assert(!plan.replay && plan.reason == "disabled");
  params.enable = true;
  plan = ll::planStaleResetReplay(params, 100.0, 103.0, 100.0);
  assert(plan.replay && plan.reason == "replay");
  // A reset about as old as the scan, or newer, is applied as it is.
  plan = ll::planStaleResetReplay(params, 100.0, 100.4, 100.0);
  assert(!plan.replay && plan.reason == "fresh");
  plan = ll::planStaleResetReplay(params, 100.0, 99.0, 100.0);
  assert(!plan.replay && plan.reason == "fresh");
  plan = ll::planStaleResetReplay(params, std::nan(""), 103.0, 100.0);
  assert(!plan.replay && plan.reason == "fresh");
  // Older than the kept scans: applied as it is.
  plan = ll::planStaleResetReplay(params, 100.0, 120.5, 100.0);
  assert(!plan.replay && plan.reason == "too_old");
  // The reset's own scan, or one right after it, must be kept.
  plan = ll::planStaleResetReplay(params, 100.0, 103.0, 100.2);
  assert(plan.replay);
  plan = ll::planStaleResetReplay(params, 100.0, 103.0, 100.3);
  assert(!plan.replay && plan.reason == "reset_scan_not_kept");
  plan = ll::planStaleResetReplay(params, 100.0, 103.0, std::nan(""));
  assert(!plan.replay && plan.reason == "reset_scan_not_kept");
  plan = ll::planStaleResetReplay(params, 100.0, 103.0, 99.9);
  assert(!plan.replay && plan.reason == "reset_scan_not_kept");
}

void testScanSelection()
{
  ll::StaleResetReplayParams params;
  const double start = -std::numeric_limits<double>::infinity();
  assert(ll::nextStaleResetReplayScan(params, {}, start, 0) == -1);
  std::vector<double> stamps;
  for (int i = 0; i <= 20; ++i) {
    stamps.push_back(100.0 + 0.1 * i);
  }
  // Three warm-up scans, then one per 0.5 s, and the newest.
  params.step_sec = 0.5;
  std::vector<int> replayed;
  double last = start;
  for (int next = ll::nextStaleResetReplayScan(params, stamps, last, 0); next >= 0;
    next = ll::nextStaleResetReplayScan(
      params, stamps, last, static_cast<int>(replayed.size())))
  {
    replayed.push_back(next);
    last = stamps[next];
  }
  assert((replayed == std::vector<int>{0, 1, 2, 7, 12, 17, 20}));
  // Caught up: nothing newer.
  assert(ll::nextStaleResetReplayScan(params, stamps, 102.0, 7) == -1);
  // A scan kept after a gap is the next step.
  const std::vector<double> gapped{100.0, 100.1, 100.2, 100.3, 101.5, 101.6};
  assert(ll::nextStaleResetReplayScan(params, gapped, 100.2, 3) == 4);
  // Scans kept while the replay ran: by default every second one at 10 Hz.
  params.step_sec = ll::StaleResetReplayParams{}.step_sec;
  assert(ll::nextStaleResetReplayScan(params, stamps, 100.2, 3) == 4);
  // Fewer than a step left: the newest.
  assert(ll::nextStaleResetReplayScan(params, stamps, 101.9, 9) == 20);

  assert(!ll::staleResetReplayFallenBehind(params, 100.0, 120.0));
  assert(ll::staleResetReplayFallenBehind(params, 100.0, 120.1));
}

void testStepAcceptance()
{
  assert(ll::staleResetReplayStepAccepted(true, true, 1.2, 6.0));
  assert(!ll::staleResetReplayStepAccepted(true, true, 6.0, 6.0));
  assert(!ll::staleResetReplayStepAccepted(false, true, 1.2, 6.0));
  assert(!ll::staleResetReplayStepAccepted(true, false, 1.2, 6.0));
  assert(!ll::staleResetReplayStepAccepted(true, true, std::nan(""), 6.0));
}

void testProgress()
{
  ll::StaleResetReplayParams params;
  ll::StaleResetReplayProgress progress;
  assert(ll::staleResetReplayFailure(params, progress) == "no_scans");
  for (int i = 0; i < 9; ++i) {
    progress = ll::recordStaleResetReplayStep(progress, true);
  }
  assert(ll::staleResetReplayFailure(params, progress).empty());
  // One rejection in ten is tolerated, not at the end.
  progress = ll::recordStaleResetReplayStep(progress, false);
  assert(!ll::staleResetReplayLost(params, progress));
  assert(ll::staleResetReplayFailure(params, progress) == "last_scan_rejected");
  progress = ll::recordStaleResetReplayStep(progress, true);
  assert(progress.steps == 11 && progress.accepted == 10);
  assert(ll::staleResetReplayFailure(params, progress).empty());
  // Too few accepted overall.
  auto sparse = progress;
  sparse = ll::recordStaleResetReplayStep(sparse, false);
  sparse = ll::recordStaleResetReplayStep(sparse, true);
  assert(ll::staleResetReplayFailure(params, sparse) == "too_few_accepted");
  // Rejections in a row lose the track for good.
  for (int i = 0; i < params.max_consecutive_rejections; ++i) {
    progress = ll::recordStaleResetReplayStep(progress, false);
    assert(!ll::staleResetReplayLost(params, progress));
  }
  progress = ll::recordStaleResetReplayStep(progress, false);
  assert(ll::staleResetReplayLost(params, progress));
  assert(ll::staleResetReplayFailure(params, progress) == "lost_track");
}

int main()
{
  testPlan();
  testScanSelection();
  testStepAcceptance();
  testProgress();
  std::puts("test_stale_reset_replay_policy: ok");
  return 0;
}
