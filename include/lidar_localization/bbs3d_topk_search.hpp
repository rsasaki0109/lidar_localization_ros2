// Top-k 3D branch-and-bound global search for the optional G2 engine "bbs3d".
//
// The search runs over the hierarchical voxel maps of 3D-BBS
// (https://github.com/KOKIAOKI/3d_bbs, vendored in third_party/bbs3d). The angular
// split per level, the initial transformation set and the voxel scoring below follow
// 3D-BBS cpu_bbs3d/bbs3d.cpp (private there) and are adapted from it under its MIT
// licence:
//
//   MIT License, Copyright (c) 2023 KokiAoki
//   (full text in third_party/bbs3d/LICENSE)
//
// The library returns the single best pose. This search keeps the same best-first
// branch-and-bound running until it has k leaves that are at least
// min_separation_m (3D) away from every better accepted leaf, so the startup checks
// that look for nearby alternatives get candidates to compare. It stops on a time
// limit and on a cap of queued nodes, and then returns the leaves accepted so far.
// ROS-free, so it is unit tested on its own (test/test_bbs3d_topk_search.cpp).

#ifndef LIDAR_LOCALIZATION_BBS3D_TOPK_SEARCH_HPP_
#define LIDAR_LOCALIZATION_BBS3D_TOPK_SEARCH_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cpu_bbs3d/voxelmaps.hpp>
#include <discrete_transformation/discrete_transformation.hpp>

namespace lidar_localization
{
namespace bbs3d
{

using Points = std::vector<Eigen::Vector3d>;

struct SearchParams
{
  // Answers wanted; each at least min_separation_m from every better one.
  int k{8};
  double min_separation_m{3.0};
  // Absolute floor: a leaf needs this share of the scan points in occupied voxels.
  double min_score_ratio{0.0};
  // Once the best leaf is found, nodes below this share of its score are pruned.
  double relative_score_floor{0.9};
  // Hard wall-clock limit of one search; <= 0 means none.
  int timeout_ms{10000};
  // Stop when the queue of open nodes grows past this many (32 bytes each);
  // 0 means no cap.
  std::size_t max_queue_nodes{4000000};
  int num_threads{4};
  // Roll and pitch searched within +-max_tilt_rad (the scan is levelled).
  double max_tilt_rad{0.02};
};

struct Candidate
{
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  int score{0};
  double score_ratio{0.0};
};

struct SearchResult
{
  std::vector<Candidate> candidates;
  bool timed_out{false};
  bool queue_capped{false};
  std::size_t expanded{0};
  std::size_t max_queue{0};
  std::size_t initial{0};
  std::size_t scan_points{0};
  double elapsed_ms{0.0};
};

namespace detail
{

struct VoxelKeyHash
{
  std::size_t operator()(const std::array<std::int64_t, 3> & key) const
  {
    std::size_t seed = 0;
    for (const auto value : key) {
      seed ^= std::hash<std::int64_t>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6U) +
        (seed >> 2U);
    }
    return seed;
  }
};

struct AngularLevel
{
  Eigen::Vector3i num_division{Eigen::Vector3i::Ones()};
  Eigen::Vector3d rpy_res{Eigen::Vector3d::Zero()};
  Eigen::Vector3d min_rpy{Eigen::Vector3d::Zero()};
};

}  // namespace detail

// Points within [min_range_m, max_range_m] of the origin (a max <= 0 keeps all).
inline Points range_filter(const Points & points, double min_range_m, double max_range_m)
{
  Points kept;
  kept.reserve(points.size());
  for (const auto & point : points) {
    const double range = point.norm();
    if (!std::isfinite(range) || range < min_range_m) {
      continue;
    }
    if (max_range_m > 0.0 && range > max_range_m) {
      continue;
    }
    kept.push_back(point);
  }
  return kept;
}

// Centroid of the points in each leaf_m voxel, ordered by voxel index (x, then y,
// then z), the order numpy.unique(axis=0) gives. A leaf <= 0 keeps the points.
inline Points voxel_centroid_filter(const Points & points, double leaf_m)
{
  if (!(leaf_m > 0.0) || points.empty()) {
    return points;
  }
  using Key = std::array<std::int64_t, 3>;
  std::unordered_map<Key, std::size_t, detail::VoxelKeyHash> index;
  index.reserve(points.size());
  std::vector<Key> keys;
  std::vector<Eigen::Vector3d> sums;
  std::vector<std::size_t> counts;
  for (const auto & point : points) {
    if (!point.allFinite()) {
      continue;
    }
    const Key key{
      static_cast<std::int64_t>(std::floor(point.x() / leaf_m)),
      static_cast<std::int64_t>(std::floor(point.y() / leaf_m)),
      static_cast<std::int64_t>(std::floor(point.z() / leaf_m))};
    const auto inserted = index.emplace(key, keys.size());
    if (inserted.second) {
      keys.push_back(key);
      sums.push_back(point);
      counts.push_back(1);
    } else {
      sums[inserted.first->second] += point;
      ++counts[inserted.first->second];
    }
  }
  std::vector<std::size_t> order(keys.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  std::sort(order.begin(), order.end(), [&keys](std::size_t a, std::size_t b) {
    return keys[a] < keys[b];
  });
  Points filtered;
  filtered.reserve(order.size());
  for (const auto i : order) {
    filtered.push_back(sums[i] / static_cast<double>(counts[i]));
  }
  return filtered;
}

class TopkSearch
{
public:
  using Trans = DiscreteTransformation<double>;

  // Builds the voxel hierarchy once (finest voxels min_level_res_m, max_level
  // coarser levels, each twice the size of the one below). x, y and z are
  // searched over the bounding box of map_points.
  TopkSearch(const Points & map_points, double min_level_res_m, int max_level)
  : min_level_res_(min_level_res_m), max_level_(max_level)
  {
    if (map_points.empty()) {
      throw std::invalid_argument("bbs3d: the map has no points");
    }
    if (!(min_level_res_m > 0.0) || max_level < 0 || max_level > 16) {
      throw std::invalid_argument("bbs3d: min_level_res must be > 0 and max_level 0-16");
    }
    const auto start = std::chrono::steady_clock::now();
    voxelmaps_ = std::make_unique<cpu::VoxelMaps>();
    voxelmaps_->set_min_res(min_level_res_m);
    voxelmaps_->set_max_level(max_level);
    voxelmaps_->create_voxelmaps(map_points, kVRate);
    min_xyz_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    max_xyz_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    for (const auto & point : map_points) {
      min_xyz_ = min_xyz_.cwiseMin(point);
      max_xyz_ = max_xyz_.cwiseMax(point);
    }
    build_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  }

  void set_trans_range(const Eigen::Vector3d & min_xyz, const Eigen::Vector3d & max_xyz)
  {
    min_xyz_ = min_xyz;
    max_xyz_ = max_xyz;
  }

  const Eigen::Vector3d & min_xyz() const {return min_xyz_;}
  const Eigen::Vector3d & max_xyz() const {return max_xyz_;}
  double build_ms() const {return build_ms_;}

  std::size_t bucket_count() const
  {
    std::size_t total = 0;
    for (const auto & buckets : voxelmaps_->multi_buckets_) {
      total += buckets.size();
    }
    return total;
  }

  // Bytes held by the voxel hierarchy (hash buckets of 4 ints each).
  std::size_t memory_bytes() const {return bucket_count() * sizeof(Eigen::Vector4i);}

  SearchResult search(const Points & scan, const SearchParams & params) const
  {
    SearchResult result;
    result.scan_points = scan.size();
    const auto start = std::chrono::steady_clock::now();
    if (scan.empty() || params.k <= 0) {
      return result;
    }
    const auto limit = start + std::chrono::milliseconds(
      params.timeout_ms > 0 ? params.timeout_ms : std::numeric_limits<int>::max());
    std::vector<detail::AngularLevel> levels(static_cast<std::size_t>(max_level_) + 1);
    if (!angular_levels(scan, params.max_tilt_rad, levels)) {
      return result;
    }
    const int num_threads = std::max(1, params.num_threads);

    int threshold = static_cast<int>(
      std::floor(static_cast<double>(scan.size()) * params.min_score_ratio));
    const double top_res = voxelmaps_->voxelmaps_res_[max_level_];
    const auto & top = levels[max_level_];
    std::vector<Trans> init;
    const int tx_min = static_cast<int>(std::floor(min_xyz_.x() / top_res));
    const int tx_max = static_cast<int>(std::ceil(max_xyz_.x() / top_res));
    const int ty_min = static_cast<int>(std::floor(min_xyz_.y() / top_res));
    const int ty_max = static_cast<int>(std::ceil(max_xyz_.y() / top_res));
    const int tz_min = static_cast<int>(std::floor(min_xyz_.z() / top_res));
    const int tz_max = static_cast<int>(std::ceil(max_xyz_.z() / top_res));
    for (int tx = tx_min; tx <= tx_max; ++tx) {
      for (int ty = ty_min; ty <= ty_max; ++ty) {
        for (int tz = tz_min; tz <= tz_max; ++tz) {
          for (int r = 0; r < top.num_division.x(); ++r) {
            for (int p = 0; p < top.num_division.y(); ++p) {
              for (int y = 0; y < top.num_division.z(); ++y) {
                init.emplace_back(0, max_level_, tx, ty, tz, r, p, y);
              }
            }
          }
        }
      }
    }
    result.initial = init.size();
    score_all(init, max_level_, levels, scan, num_threads);

    std::priority_queue<Trans> queue;
    for (const auto & trans : init) {
      if (trans.score >= threshold) {
        queue.push(trans);
      }
    }
    init.clear();
    init.shrink_to_fit();
    result.max_queue = queue.size();
    std::vector<Eigen::Vector3d> accepted_positions;
    const double min_separation_sq = params.min_separation_m * params.min_separation_m;
    while (!queue.empty() && static_cast<int>(result.candidates.size()) < params.k) {
      if (std::chrono::steady_clock::now() > limit) {
        result.timed_out = true;
        break;
      }
      if (params.max_queue_nodes > 0 && queue.size() > params.max_queue_nodes) {
        result.queue_capped = true;
        break;
      }
      Trans trans = queue.top();
      queue.pop();
      if (trans.score < threshold) {
        continue;
      }
      if (trans.is_leaf()) {
        const Eigen::Matrix4d pose =
          trans.create_matrix(min_level_res_, levels[0].rpy_res, levels[0].min_rpy);
        const Eigen::Vector3d position = pose.block<3, 1>(0, 3);
        const bool suppressed = std::any_of(
          accepted_positions.begin(), accepted_positions.end(),
          [&](const Eigen::Vector3d & other) {
            return (position - other).squaredNorm() < min_separation_sq;
          });
        if (!suppressed) {
          if (result.candidates.empty()) {
            threshold = std::max(
              threshold,
              static_cast<int>(std::ceil(params.relative_score_floor * trans.score)));
          }
          Candidate candidate;
          candidate.pose = pose;
          candidate.score = trans.score;
          candidate.score_ratio =
            static_cast<double>(trans.score) / static_cast<double>(scan.size());
          result.candidates.push_back(candidate);
          accepted_positions.push_back(position);
        }
        continue;
      }
      ++result.expanded;
      const int child_level = trans.level - 1;
      auto children = trans.branch(child_level, kVRate, levels[child_level].num_division);
      score_all(children, child_level, levels, scan, num_threads);
      for (const auto & child : children) {
        if (child.score >= threshold) {
          queue.push(child);
        }
      }
      result.max_queue = std::max(result.max_queue, queue.size());
    }
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    return result;
  }

private:
  static constexpr int kVRate = 2;

  // Angular resolution per level from the farthest scan point (bbs3d.cpp
  // calc_angular_info). False when the scan has no extent.
  bool angular_levels(
    const Points & scan, double max_tilt_rad, std::vector<detail::AngularLevel> & levels) const
  {
    double max_norm = 0.0;
    for (const auto & point : scan) {
      max_norm = std::max(max_norm, point.norm());
    }
    if (!(max_norm > 0.0)) {
      return false;
    }
    const double tilt = std::max(0.0, max_tilt_rad);
    const Eigen::Vector3d min_rpy(-tilt, -tilt, 0.0);
    const Eigen::Vector3d max_rpy(tilt, tilt, 2.0 * M_PI);
    const Eigen::Vector3d span = max_rpy - min_rpy;
    for (int i = max_level_; i >= 0; --i) {
      const double res = voxelmaps_->voxelmaps_res_[i];
      const double cosine = 1.0 - (res * res / (max_norm * max_norm)) * 0.5;
      double ori_res = std::acos(std::max(cosine, -1.0));
      ori_res = std::floor(ori_res * 10000) / 10000;
      auto & level = levels[i];
      for (int a = 0; a < 3; ++a) {
        const double step = ori_res <= std::abs(span[a]) ? ori_res : 0.0;
        double piece = span[a];
        if (i != max_level_ && levels[i + 1].rpy_res[a] != 0.0) {
          piece = levels[i + 1].rpy_res[a];
        }
        level.num_division[a] = step == 0.0 ? 1 : static_cast<int>(std::ceil(piece / step));
        level.rpy_res[a] = level.num_division[a] == 1 ? 0.0 : piece / level.num_division[a];
        level.min_rpy[a] = level.rpy_res[a] == 0.0 ? 0.0 : min_rpy[a];
      }
    }
    return true;
  }

  // Score = scan points that fall into an occupied voxel (or its neighbour mark)
  // of the given level (bbs3d.cpp calc_score).
  void score_all(
    std::vector<Trans> & set, int level, const std::vector<detail::AngularLevel> & levels,
    const Points & points, int num_threads) const
  {
    const auto & buckets = voxelmaps_->multi_buckets_[level];
    const auto num_buckets = static_cast<std::uint32_t>(buckets.size());
    if (num_buckets == 0) {
      return;
    }
    const int scan_count = voxelmaps_->get_max_bucket_scan_count();
    const double res = voxelmaps_->voxelmaps_res_[level];
    const double inv_res = 1.0 / res;
    const auto & rpy_res = levels[level].rpy_res;
    const auto & min_rpy = levels[level].min_rpy;
    const auto size = static_cast<std::int64_t>(set.size());
#pragma omp parallel for num_threads(num_threads)
    for (std::int64_t s = 0; s < size; ++s) {
      Trans & trans = set[static_cast<std::size_t>(s)];
      const Eigen::Transform<double, 3, Eigen::Affine> transform(
        trans.create_matrix(res, rpy_res, min_rpy));
      int score = 0;
      for (const auto & point : points) {
        const Eigen::Vector3i coord = ((transform * point).array() * inv_res).floor().cast<int>();
        const std::uint32_t hash =
          (static_cast<std::uint32_t>(coord[0]) * 73856093U) ^
          (static_cast<std::uint32_t>(coord[1]) * 19349669U) ^
          (static_cast<std::uint32_t>(coord[2]) * 83492791U);
        for (int j = 0; j < scan_count; ++j) {
          const auto & bucket = buckets[(hash + static_cast<std::uint32_t>(j)) % num_buckets];
          if (bucket.w() == 0) {
            break;
          }
          if (bucket.x() == coord.x() && bucket.y() == coord.y() && bucket.z() == coord.z()) {
            ++score;
            break;
          }
        }
      }
      trans.score = score;
    }
  }

  std::unique_ptr<cpu::VoxelMaps> voxelmaps_;
  double min_level_res_;
  int max_level_;
  double build_ms_{0.0};
  Eigen::Vector3d min_xyz_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_xyz_{Eigen::Vector3d::Zero()};
};

}  // namespace bbs3d
}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_BBS3D_TOPK_SEARCH_HPP_
