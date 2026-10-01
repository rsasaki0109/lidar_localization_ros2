#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "lidar_localization/local_reacquisition_policy.hpp"

namespace ll = lidar_localization;

void testTrigger()
{
  ll::LocalReacquisitionParams params;
  ll::LocalReacquisitionTriggerInput input{true, true, false, 10, 10};
  assert(!ll::shouldAttemptLocalReacquisition(params, input));  // disabled by default
  params.enable = true;
  assert(ll::shouldAttemptLocalReacquisition(params, input));
  auto blocked = input;
  blocked.odom_bridge_seed = false;
  assert(!ll::shouldAttemptLocalReacquisition(params, blocked));
  blocked = input;
  blocked.occupancy_ready = false;
  assert(!ll::shouldAttemptLocalReacquisition(params, blocked));
  blocked = input;
  blocked.proposal_pending = true;
  assert(!ll::shouldAttemptLocalReacquisition(params, blocked));
  blocked = input;
  blocked.consecutive_rejected_updates = 9;
  assert(!ll::shouldAttemptLocalReacquisition(params, blocked));
  blocked = input;
  blocked.scans_since_last_attempt = 9;
  assert(!ll::shouldAttemptLocalReacquisition(params, blocked));
}

std::string writeMap(const std::string & stem, int negate, const std::string & origin)
{
  const std::string pgm = "/tmp/" + stem + ".pgm";
  const std::string yaml = "/tmp/" + stem + ".yaml";
  // 3 columns x 2 rows. Top row (image row 0): black, white, gray(128).
  std::ofstream image(pgm, std::ios::binary);
  image << "P5\n# comment\n3 2\n255\n";
  const unsigned char pixels[6] = {0, 255, 128, 255, 255, 0};
  image.write(reinterpret_cast<const char *>(pixels), 6);
  image.close();
  std::ofstream meta(yaml);
  meta << "image: " << stem << ".pgm  # relative\nresolution: 0.5\norigin: " << origin
       << "\nnegate: " << negate << "\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  return yaml;
}

void testOccupancyLoader()
{
  const auto map = ll::loadOccupancyGridMap(writeMap("ll_reacq_map", 0, "[-1.0, 2.0, 0.0]"));
  assert(map.grid.width == 3 && map.grid.height == 2);
  assert(map.resolution_m == 0.5 && map.origin_x_m == -1.0 && map.origin_y_m == 2.0);
  // Image row 0 becomes grid row 1 (flipped so row 0 is the lowest y).
  assert(map.grid.at(1, 0) == 1 && map.grid.at(1, 1) == 0 && map.grid.at(1, 2) == 0);
  assert(map.grid.at(0, 0) == 0 && map.grid.at(0, 1) == 0 && map.grid.at(0, 2) == 1);
  const auto negated = ll::loadOccupancyGridMap(writeMap("ll_reacq_neg", 1, "[0, 0, 0]"));
  assert(negated.grid.at(1, 0) == 0 && negated.grid.at(1, 1) == 1);
  bool threw = false;
  try {
    ll::loadOccupancyGridMap(writeMap("ll_reacq_rot", 0, "[0, 0, 0.3]"));
  } catch (const std::runtime_error &) {
    threw = true;
  }
  assert(threw);
}

void testWindowAndHeadingFilter()
{
  ll::OccupancyGridMap map;
  map.grid = ll::bbs::Grid(100, 200);
  map.resolution_m = 0.5;
  map.origin_x_m = -10.0;
  map.origin_y_m = 5.0;
  const auto window = ll::occupancyWindowAround(map, 0.0, 10.0, 5.0);
  assert(window.x0 == 10 && window.y0 == 0 && window.width == 20 && window.height == 20);
  const auto clipped = ll::occupancyWindowAround(map, 88.0, 54.0, 5.0);
  assert(clipped.x0 + clipped.width == 200 && clipped.y0 + clipped.height == 100);
  map.grid.set(3, 12, 1);
  const auto crop = ll::cropGrid(map.grid, window);
  assert(crop.at(3, 2) == 1);

  ll::LocalReacquisitionParams params;
  params.max_candidates = 2;
  std::vector<ll::bbs::BbsGridCandidate> candidates(3);
  candidates[0].tx_cell = 4; candidates[0].ty_cell = 6; candidates[0].yaw_rad = M_PI;
  candidates[1].tx_cell = 2; candidates[1].ty_cell = 2; candidates[1].yaw_rad = 0.2;
  candidates[2].tx_cell = 8; candidates[2].ty_cell = 1; candidates[2].yaw_rad = -0.1;
  const auto kept = ll::selectHeadingConsistentCandidates(candidates, map, window, 0.0, params);
  assert(kept.size() == 2);  // the reversed heading is dropped
  assert(std::abs(kept[0].x_m - (-10.0 + (10 + 2 + 0.5) * 0.5)) < 1e-9);
  assert(std::abs(kept[0].y_m - (5.0 + (0 + 2 + 0.5) * 0.5)) < 1e-9);
  assert(std::abs(kept[1].yaw_rad + 0.1) < 1e-12);
  // Heading wrap-around is handled.
  const auto wrapped = ll::selectHeadingConsistentCandidates(
    candidates, map, window, M_PI - 0.1, params);
  assert(wrapped.size() == 1 && wrapped[0].yaw_rad == M_PI);
}

void testScanPreparation()
{
  ll::LocalReacquisitionParams params;
  params.scan_max_points = 3;
  const std::vector<std::array<double, 3>> points{
    {0.2, 0.0, 1.0},     // inside min range
    {5.0, 0.0, 0.2},     // below z band
    {5.0, 0.0, 6.0},     // above z band
    {5.0, 0.0, 1.0},
    {5.05, 0.05, 2.0},   // same 0.2 m cell as the previous point
    {6.0, 0.0, 1.0},
    {7.0, 0.0, 1.0},
    {8.0, 0.0, 1.0},
    {std::nan(""), 0.0, 1.0}};
  const auto xy = ll::prepareReacquisitionScanXy(points, params);
  assert(xy.size() == 3);
  assert(xy[0][0] == 5.0 && xy[2][0] == 8.0);
}

void testDecision()
{
  ll::LocalReacquisitionParams params;
  using C = ll::RefinedReacquisitionCandidate;
  // Clustered winners agree, a distinct wrong place fits clearly worse.
  auto decision = ll::decideLocalReacquisition(
    {C{true, 1.6, 0.0, 0.0}, C{true, 1.65, 0.5, 0.0}, C{true, 2.7, 4.0, 0.0}}, 6.0, params);
  assert(decision.propose && decision.index == 0);
  // A distinct place that fits almost as well is ambiguous.
  decision = ll::decideLocalReacquisition(
    {C{true, 1.6, 0.0, 0.0}, C{true, 1.7, 10.0, 0.0}}, 6.0, params);
  assert(!decision.propose && decision.reason == "ambiguous");
  // Best candidate must still pass the normal score threshold.
  decision = ll::decideLocalReacquisition({C{true, 7.2, 0.0, 0.0}}, 6.0, params);
  assert(!decision.propose && decision.reason == "best_fitness_above_threshold");
  // Non-converged candidates are ignored, including as ambiguity evidence.
  decision = ll::decideLocalReacquisition(
    {C{false, 0.1, 9.0, 0.0}, C{true, 2.0, 0.0, 0.0}}, 6.0, params);
  assert(decision.propose && decision.index == 1);
  decision = ll::decideLocalReacquisition({C{false, 0.1, 0.0, 0.0}}, 6.0, params);
  assert(!decision.propose && decision.reason == "no_converged_candidate");
  decision = ll::decideLocalReacquisition({}, 6.0, params);
  assert(!decision.propose && decision.reason == "no_candidates");
}

int main()
{
  testTrigger();
  testOccupancyLoader();
  testWindowAndHeadingFilter();
  testScanPreparation();
  testDecision();
  std::puts("test_local_reacquisition_policy: ok");
  return 0;
}
