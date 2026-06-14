#ifndef BBS_BRANCH_AND_BOUND_HPP_
#define BBS_BRANCH_AND_BOUND_HPP_

// Pure C++ branch-and-bound (BBS_2D) global-localization search, a bit-exact
// port of branch_and_bound_candidates in scripts/make_bbs_relocalization_attempts.py.
//
// Why a port (docs/g2_bbs_speedup.md): profiling the Python search on the real
// Koide map showed ~61% of the time is the cold-path per-node gather -- 250k
// numpy calls each paying interpreter/per-call overhead on a tiny offset array --
// not arithmetic. The Python code works around that with an adaptive FFT hit map;
// in C++ the direct gather is a tight integer loop with no per-call overhead, so
// the FFT machinery is unnecessary and is dropped. The result is identical
// candidates (the FFT path was proven bit-exact to the direct gather) at a
// fraction of the time.
//
// This header is pure C++/STL (no ROS, no PCL, no FFT), so it compiles and is
// bit-exactly verifiable against the Python fixtures standalone:
//
//   g++ -std=c++17 -I include test/test_bbs_branch_and_bound.cpp -o /tmp/t && /tmp/t

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_localization {
namespace bbs {

// Row-major boolean grid (height rows, width columns). cell(y, x) is occupied.
struct Grid
{
  int height = 0;
  int width = 0;
  std::vector<uint8_t> data;  // height*width, 0/1

  Grid() = default;
  Grid(int h, int w) : height(h), width(w), data(static_cast<size_t>(h) * w, 0) {}

  inline uint8_t at(int y, int x) const { return data[static_cast<size_t>(y) * width + x]; }
  inline void set(int y, int x, uint8_t v) { data[static_cast<size_t>(y) * width + x] = v; }
};

struct BbsGridCandidate
{
  int tx_cell = 0;
  int ty_cell = 0;
  int yaw_index = 0;
  double yaw_rad = 0.0;
  double score = 0.0;
  int hit_count = 0;
  int point_count = 0;
};

// --- helpers, each mirroring the Python implementation exactly -----------------

// Python uses ((angle + pi) % (2*pi)) - pi with Python's floored modulo, which is
// always non-negative. std::fmod can return a negative remainder, so fix it up.
inline double py_normalize_angle_rad(double angle)
{
  const double two_pi = 2.0 * M_PI;
  double m = std::fmod(angle + M_PI, two_pi);
  if (m < 0.0) m += two_pi;
  return m - M_PI;
}

// _max_pool_2x2: pad to even dims with false, then max over 2x2 blocks.
inline Grid max_pool_2x2(const Grid & grid)
{
  const int ph = (grid.height + 1) / 2;
  const int pw = (grid.width + 1) / 2;
  Grid out(ph, pw);
  for (int by = 0; by < ph; ++by) {
    for (int bx = 0; bx < pw; ++bx) {
      uint8_t v = 0;
      for (int dy = 0; dy < 2; ++dy) {
        const int y = by * 2 + dy;
        if (y >= grid.height) continue;
        for (int dx = 0; dx < 2; ++dx) {
          const int x = bx * 2 + dx;
          if (x >= grid.width) continue;
          v |= grid.at(y, x);
        }
      }
      out.set(by, bx, v);
    }
  }
  return out;
}

// _dilate_one_cell: 3x3 dilation (OR of all 9 shifts, zero-padded border).
inline Grid dilate_one_cell(const Grid & grid)
{
  Grid out(grid.height, grid.width);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      uint8_t v = 0;
      for (int dy = -1; dy <= 1 && !v; ++dy) {
        const int ny = y + dy;
        if (ny < 0 || ny >= grid.height) continue;
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = x + dx;
          if (nx < 0 || nx >= grid.width) continue;
          if (grid.at(ny, nx)) { v = 1; break; }
        }
      }
      out.set(y, x, v);
    }
  }
  return out;
}

inline std::vector<Grid> build_occupancy_pyramid(const Grid & occupancy, int depth)
{
  std::vector<Grid> levels;
  levels.push_back(occupancy);
  for (int i = 0; i < depth; ++i) {
    const Grid & prev = levels.back();
    if (prev.height == 1 && prev.width == 1) break;
    levels.push_back(max_pool_2x2(prev));
  }
  return levels;
}

inline std::vector<Grid> build_upper_bound_pyramid(const std::vector<Grid> & pyramid)
{
  std::vector<Grid> levels;
  levels.push_back(pyramid[0]);  // level 0 is the exact occupancy (no dilation)
  for (size_t i = 1; i < pyramid.size(); ++i) {
    levels.push_back(dilate_one_cell(pyramid[i]));
  }
  return levels;
}

// _yaw_samples: count = max(1, ceil(2pi/ares)); step = 2pi/count; normalized.
inline std::vector<double> yaw_samples(double angular_resolution_rad)
{
  const double two_pi = 2.0 * M_PI;
  int count = static_cast<int>(std::ceil(two_pi / angular_resolution_rad));
  if (count < 1) count = 1;
  const double step = two_pi / static_cast<double>(count);
  std::vector<double> yaws;
  yaws.reserve(count);
  for (int i = 0; i < count; ++i) {
    yaws.push_back(py_normalize_angle_rad(i * step));
  }
  return yaws;
}

// Deduplicated rotated-scan cell offsets for one (yaw, level): qy/qx are the
// floored cell offsets, count is how many scan points landed in that cell.
struct OffsetCell
{
  int qy;
  int qx;
  int count;
};

// --- the search ----------------------------------------------------------------

namespace detail {

struct HeapNode
{
  double neg_bound;
  long seq;
  int level;
  int tx_cell;
  int ty_cell;
  int yaw_index;
};

// Pop order must match Python's heapq on (neg_bound, seq): smallest first.
struct HeapCmp
{
  bool operator()(const HeapNode & a, const HeapNode & b) const
  {
    if (a.neg_bound != b.neg_bound) return a.neg_bound > b.neg_bound;
    return a.seq > b.seq;
  }
};

inline bool candidate_less(const BbsGridCandidate & a, const BbsGridCandidate & b)
{
  // _candidate_sort_key = (-score, yaw_index, ty_cell, tx_cell), ascending.
  if (a.score != b.score) return a.score > b.score;
  if (a.yaw_index != b.yaw_index) return a.yaw_index < b.yaw_index;
  if (a.ty_cell != b.ty_cell) return a.ty_cell < b.ty_cell;
  return a.tx_cell < b.tx_cell;
}

// Direct integer gather: sum of counts over offset cells that are in-bounds and
// occupied in `grid`, at node (level, tx, ty). Identical to the Python gather.
inline int gather_hits(
  const Grid & grid, const std::vector<OffsetCell> & offsets, int level,
  int tx_cell, int ty_cell)
{
  const int base_y = ty_cell >> level;
  const int base_x = tx_cell >> level;
  int hit = 0;
  for (const auto & o : offsets) {
    const int iy = o.qy + base_y;
    const int ix = o.qx + base_x;
    if (iy < 0 || ix < 0 || iy >= grid.height || ix >= grid.width) continue;
    if (grid.at(iy, ix)) hit += o.count;
  }
  return hit;
}

}  // namespace detail

// Bit-exact port of branch_and_bound_candidates (the FFT path omitted, see file
// header). occupancy is the boolean map; scan_xy_m is the sensor-frame scan in
// metres; resolution_m maps metres to cells; the rest mirror the Python args.
inline std::vector<BbsGridCandidate> branch_and_bound_candidates(
  const Grid & occupancy,
  const std::vector<std::array<double, 2>> & scan_xy_m,
  double resolution_m,
  double angular_resolution_rad,
  int pyramid_depth,
  int max_candidates,
  int nms_radius_cells = 0)
{
  if (max_candidates <= 0) return {};
  if (resolution_m <= 0.0) throw std::invalid_argument("resolution_m must be positive");
  if (angular_resolution_rad <= 0.0)
    throw std::invalid_argument("angular_resolution_rad must be positive");
  if (scan_xy_m.empty()) return {};

  const int height = occupancy.height;
  const int width = occupancy.width;
  const int n_points = static_cast<int>(scan_xy_m.size());

  // scan in grid units.
  std::vector<std::array<double, 2>> scan_grid(scan_xy_m.size());
  for (size_t i = 0; i < scan_xy_m.size(); ++i) {
    scan_grid[i] = {scan_xy_m[i][0] / resolution_m, scan_xy_m[i][1] / resolution_m};
  }

  const std::vector<Grid> pyramid = build_occupancy_pyramid(occupancy, pyramid_depth);
  const std::vector<Grid> ub = build_upper_bound_pyramid(pyramid);
  const int start_level = static_cast<int>(ub.size()) - 1;
  const std::vector<double> yaws = yaw_samples(angular_resolution_rad);
  const int n_yaws = static_cast<int>(yaws.size());
  const int n_levels = static_cast<int>(ub.size());

  // offset_cache[yaw][level] -> deduplicated (qy, qx, count). Dedup order does
  // not affect the gathered sum, so any stable container is fine.
  std::vector<std::vector<std::vector<OffsetCell>>> offset_cache(n_yaws);
  for (int yi = 0; yi < n_yaws; ++yi) {
    const double c = std::cos(yaws[yi]);
    const double s = std::sin(yaws[yi]);
    offset_cache[yi].resize(n_levels);
    std::vector<double> rx(n_points), ry(n_points);
    for (int p = 0; p < n_points; ++p) {
      rx[p] = c * scan_grid[p][0] - s * scan_grid[p][1];
      ry[p] = s * scan_grid[p][0] + c * scan_grid[p][1];
    }
    for (int level = 0; level < n_levels; ++level) {
      const double factor = static_cast<double>(1 << level);
      std::map<std::pair<int, int>, int> dedup;  // (qy, qx) -> count
      for (int p = 0; p < n_points; ++p) {
        const int qx = static_cast<int>(std::floor((0.5 + rx[p]) / factor));
        const int qy = static_cast<int>(std::floor((0.5 + ry[p]) / factor));
        ++dedup[{qy, qx}];
      }
      auto & cells = offset_cache[yi][level];
      cells.reserve(dedup.size());
      for (const auto & kv : dedup) {
        cells.push_back({kv.first.first, kv.first.second, kv.second});
      }
    }
  }

  const int initial_step = 1 << start_level;

  // Seed the heap: every top-level block of every yaw, sequence assigned in
  // yaw-major, then ty, then tx order (to match the Python pop order exactly).
  std::priority_queue<detail::HeapNode, std::vector<detail::HeapNode>, detail::HeapCmp> heap;
  long sequence = 0;
  for (int yi = 0; yi < n_yaws; ++yi) {
    const auto & offs = offset_cache[yi][start_level];
    for (int ty = 0; ty < height; ty += initial_step) {
      for (int tx = 0; tx < width; tx += initial_step) {
        const int hit = detail::gather_hits(ub[start_level], offs, start_level, tx, ty);
        const double bound = static_cast<double>(hit) / static_cast<double>(n_points);
        heap.push({-bound, sequence, start_level, tx, ty, yi});
        ++sequence;
      }
    }
  }

  std::vector<BbsGridCandidate> best;
  double kth_score = -std::numeric_limits<double>::infinity();
  const double eps = 1.0e-12;

  while (!heap.empty()) {
    const detail::HeapNode node = heap.top();
    heap.pop();
    const double bound = -node.neg_bound;
    if (static_cast<int>(best.size()) >= max_candidates && bound <= kth_score + eps) break;

    const int level = node.level;
    const int yi = node.yaw_index;

    if (level == 0) {
      const int hit = detail::gather_hits(ub[0], offset_cache[yi][0], 0, node.tx_cell, node.ty_cell);
      BbsGridCandidate cand;
      cand.tx_cell = node.tx_cell;
      cand.ty_cell = node.ty_cell;
      cand.yaw_index = yi;
      cand.yaw_rad = yaws[yi];
      cand.score = static_cast<double>(hit) / static_cast<double>(n_points);
      cand.hit_count = hit;
      cand.point_count = n_points;

      if (nms_radius_cells > 0) {
        bool suppressed = false;
        for (size_t i = 0; i < best.size(); ++i) {
          if (std::abs(best[i].tx_cell - cand.tx_cell) <= nms_radius_cells &&
              std::abs(best[i].ty_cell - cand.ty_cell) <= nms_radius_cells) {
            if (cand.score > best[i].score) best[i] = cand;
            suppressed = true;
            break;
          }
        }
        if (!suppressed) best.push_back(cand);
      } else {
        best.push_back(cand);
      }
      std::stable_sort(best.begin(), best.end(), detail::candidate_less);
      if (static_cast<int>(best.size()) > max_candidates) best.resize(max_candidates);
      if (static_cast<int>(best.size()) >= max_candidates) kth_score = best.back().score;
      continue;
    }

    const int child_level = level - 1;
    const int child_step = 1 << child_level;  // >= 1, so dy/dx iterate {0, child_step}
    for (int dy = 0; dy <= child_step; dy += child_step) {
      const int child_ty = node.ty_cell + dy;
      if (child_ty >= height) continue;
      for (int dx = 0; dx <= child_step; dx += child_step) {
        const int child_tx = node.tx_cell + dx;
        if (child_tx >= width) continue;
        const int hit = detail::gather_hits(
          ub[child_level], offset_cache[yi][child_level], child_level, child_tx, child_ty);
        const double child_bound = static_cast<double>(hit) / static_cast<double>(n_points);
        if (static_cast<int>(best.size()) >= max_candidates && child_bound < kth_score - eps)
          continue;
        heap.push({-child_bound, sequence, child_level, child_tx, child_ty, yi});
        ++sequence;
      }
    }
  }

  std::stable_sort(best.begin(), best.end(), detail::candidate_less);
  if (static_cast<int>(best.size()) > max_candidates) best.resize(max_candidates);
  return best;
}

}  // namespace bbs
}  // namespace lidar_localization

#endif  // BBS_BRANCH_AND_BOUND_HPP_
