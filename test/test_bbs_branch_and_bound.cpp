// Bit-exact verification of the C++ BBS port (bbs_branch_and_bound.hpp) against
// the Python reference (scripts/make_bbs_relocalization_attempts.py).
//
// The golden fixtures in bbs_golden_fixtures.inc are the frozen output of the
// Python search (regenerate with scripts/generate_bbs_golden_fixtures.py). This
// test runs the C++ port on the same inputs and asserts every candidate tuple
// (tx, ty, yaw_index, hit_count) matches -- proving the port is bit-exact without
// needing an idle machine (docs/g2_bbs_speedup.md). Pure C++/STL, so:
//
//   g++ -std=c++17 -I include test/test_bbs_branch_and_bound.cpp -o /tmp/t && /tmp/t

#include "lidar_localization/bbs_branch_and_bound.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "bbs_golden_fixtures.inc"

namespace bbs = lidar_localization::bbs;

namespace {

bbs::Grid grid_from_string(int height, int width, const std::string & flat)
{
  bbs::Grid g(height, width);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      g.set(y, x, flat[static_cast<size_t>(y) * width + x] == '1' ? 1 : 0);
    }
  }
  return g;
}

void check_fixture(const GoldenFixture & gf)
{
  const bbs::Grid occ = grid_from_string(gf.height, gf.width, gf.occ);
  const auto got = bbs::branch_and_bound_candidates(
    occ, gf.scan, gf.resolution_m, gf.ares_rad, gf.depth, gf.max_c, gf.nms);

  if (got.size() != gf.expected.size()) {
    std::fprintf(stderr, "[%s] candidate count %zu != expected %zu\n",
                 gf.name, got.size(), gf.expected.size());
    assert(false);
  }
  for (size_t i = 0; i < got.size(); ++i) {
    const auto & g = got[i];
    const auto & e = gf.expected[i];
    const bool ok = g.tx_cell == e.tx && g.ty_cell == e.ty &&
                    g.yaw_index == e.yaw_index && g.hit_count == e.hit;
    if (!ok) {
      std::fprintf(stderr,
        "[%s] candidate %zu mismatch: got (tx=%d ty=%d yaw=%d hit=%d) "
        "expected (tx=%d ty=%d yaw=%d hit=%d)\n",
        gf.name, i, g.tx_cell, g.ty_cell, g.yaw_index, g.hit_count,
        e.tx, e.ty, e.yaw_index, e.hit);
      assert(false);
    }
    // Score must be the exact integer ratio the Python candidate carried.
    const double expect_score =
      static_cast<double>(e.hit) / static_cast<double>(gf.scan.size());
    assert(g.score == expect_score);
    assert(g.point_count == static_cast<int>(gf.scan.size()));
  }
  std::printf("  [%s] %zu candidates match\n", gf.name, got.size());
}

// A couple of direct invariants on the helpers, independent of the golden.
void test_yaw_samples_count()
{
  // 5 degrees -> ceil(2pi / (5 deg)) = 72 yaw samples, first is 0.
  const auto yaws = bbs::yaw_samples(M_PI / 36.0);
  assert(yaws.size() == 72);
  assert(std::abs(yaws[0]) < 1e-12);
}

void test_upper_bound_level0_is_exact_occupancy()
{
  bbs::Grid occ(4, 4);
  occ.set(1, 1, 1);
  const auto pyr = bbs::build_occupancy_pyramid(occ, 2);
  const auto ub = bbs::build_upper_bound_pyramid(pyr);
  // Level 0 of the upper-bound pyramid must equal the occupancy (no dilation),
  // which is what lets the search score the exact map at level 0.
  assert(ub[0].data == occ.data);
}

}  // namespace

int main()
{
  test_yaw_samples_count();
  test_upper_bound_level0_is_exact_occupancy();
  const auto fixtures = golden_fixtures();
  assert(!fixtures.empty());
  for (const auto & gf : fixtures) {
    check_fixture(gf);
  }
  std::printf("all %zu golden fixtures bit-exact\n", fixtures.size());
  return 0;
}
