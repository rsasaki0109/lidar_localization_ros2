// The map-distance prefilter: before a scan is aligned, drop its points that lie
// farther than a distance from every map point at the predicted pose. People and
// vehicles that are not in the map are then left out of the registration and of the
// fitness and inlier ratio the gate reads, instead of dragging the alignment and
// failing min_inlier_ratio.
//
// The distance has to cover the error of the prediction: a point of a wall the
// prediction puts 2 m off is 2 m from the map too. So it grows with the time since
// the last accepted pose, and the filter is off where the prediction is not a
// tracked pose: before the first accepted pose, for the first accepted scans after an
// initial pose (a global fix or /initialpose), and once the gap has grown the
// distance past its maximum. A scan that would keep too few points (a pose far off,
// or the sensor almost blind) is aligned whole.
//
// On Boreas with trucks alongside (tools/benchmark/dynamic_obstacles.py) 28% of the
// 1.5 m voxels of each scan on the obstacles made the preset reject every scan by its
// inlier ratio, and without that gate NDT drifted to 0.35 m RMSE; see
// docs/benchmark.md#dynamic-obstacles.
#ifndef LIDAR_LOCALIZATION_MAP_DISTANCE_FILTER_HPP_
#define LIDAR_LOCALIZATION_MAP_DISTANCE_FILTER_HPP_

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace lidar_localization
{

struct MapDistanceFilterParams
{
  // The distance at the first scan after an accepted one; 0 turns the filter off.
  double distance_m{0.0};
  // How fast the distance grows with the time since the last accepted pose, beyond
  // one scan period.
  double growth_m_per_s{1.0};
  // Past this distance the prediction is not trusted: the scan is aligned whole.
  double max_distance_m{3.0};
  // Accepted scans after an initial pose before the filter starts. 0 trusts the initial
  // pose itself (an exact start), from the first scan.
  int warmup_accepts{5};
  // A scan that would keep less than this share of its points is aligned whole.
  double min_kept_ratio{0.3};
};

enum class MapDistanceFilterStatus
{
  kApplied,
  kDisabled,
  kNoAcceptedPose,
  kWarmingUp,
  kGapTooLong,
  kTooFewKept,
  kNoTargetTree,
  kRetrySeed,
};

inline const char * mapDistanceFilterStatusName(MapDistanceFilterStatus status)
{
  switch (status) {
    case MapDistanceFilterStatus::kApplied:
      return "applied";
    case MapDistanceFilterStatus::kDisabled:
      return "disabled";
    case MapDistanceFilterStatus::kNoAcceptedPose:
      return "no_accepted_pose";
    case MapDistanceFilterStatus::kWarmingUp:
      return "warming_up";
    case MapDistanceFilterStatus::kGapTooLong:
      return "gap_too_long";
    case MapDistanceFilterStatus::kTooFewKept:
      return "too_few_kept";
    case MapDistanceFilterStatus::kNoTargetTree:
      return "no_target_tree";
    case MapDistanceFilterStatus::kRetrySeed:
      return "retry_seed";
  }
  return "unknown";
}

struct MapDistanceFilterInput
{
  bool have_last_accepted_pose{false};
  // Seconds from the last accepted pose to this scan.
  double since_accept_sec{std::numeric_limits<double>::quiet_NaN()};
  double scan_period_sec{0.1};
  // Accepted scans since the last initial pose.
  int accepts_since_initial_pose{0};
};

struct MapDistanceFilterDecision
{
  bool apply{false};
  double distance_m{std::numeric_limits<double>::quiet_NaN()};
  MapDistanceFilterStatus status{MapDistanceFilterStatus::kDisabled};
};

inline MapDistanceFilterDecision decideMapDistanceFilter(
  const MapDistanceFilterParams & params, const MapDistanceFilterInput & input)
{
  MapDistanceFilterDecision decision;
  if (!(params.distance_m > 0.0)) {
    return decision;
  }
  if (!input.have_last_accepted_pose || !std::isfinite(input.since_accept_sec)) {
    decision.status = MapDistanceFilterStatus::kNoAcceptedPose;
    return decision;
  }
  if (input.accepts_since_initial_pose < params.warmup_accepts) {
    decision.status = MapDistanceFilterStatus::kWarmingUp;
    return decision;
  }
  // Before the first accepted scan the time is from the initial pose, which is set,
  // not tracked (and stamped 0 when given before the clock runs): only a zero warmup
  // gets here, and it trusts that pose.
  const double late_sec = input.accepts_since_initial_pose == 0 ? 0.0 :
    std::max(0.0, input.since_accept_sec - std::max(0.0, input.scan_period_sec));
  decision.distance_m = params.distance_m + std::max(0.0, params.growth_m_per_s) * late_sec;
  if (decision.distance_m > params.max_distance_m) {
    decision.status = MapDistanceFilterStatus::kGapTooLong;
    return decision;
  }
  decision.apply = true;
  decision.status = MapDistanceFilterStatus::kApplied;
  return decision;
}

// The SOURCE points that, moved by GUESS, lie within MAX_DISTANCE_M of a point of the
// TREE's cloud. One nearest-neighbour search per point, spread over THREADS; points
// keep their order.
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterByMapDistance(
  const pcl::PointCloud<PointT> & source,
  const pcl::search::Search<PointT> & tree,
  const Eigen::Matrix4f & guess,
  double max_distance_m,
  int threads)
{
  pcl::PointCloud<PointT> moved;
  pcl::transformPointCloud(source, moved, guess);
  const int count = static_cast<int>(moved.size());
  const float limit = static_cast<float>(max_distance_m * max_distance_m);
  std::vector<char> keep(moved.size(), 0);
#pragma omp parallel for num_threads(std::max(1, threads)) schedule(guided, 32)
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(moved[i].x) || !std::isfinite(moved[i].y) ||
      !std::isfinite(moved[i].z))
    {
      continue;
    }
    pcl::Indices index(1);
    std::vector<float> distance(1);
    if (tree.nearestKSearch(moved[i], 1, index, distance) > 0 && distance[0] <= limit) {
      keep[i] = 1;
    }
  }
  typename pcl::PointCloud<PointT>::Ptr kept(new pcl::PointCloud<PointT>());
  kept->reserve(moved.size());
  for (std::size_t i = 0; i < source.size(); ++i) {
    if (keep[i]) {
      kept->push_back(source[i]);
    }
  }
  kept->header = source.header;
  return kept;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MAP_DISTANCE_FILTER_HPP_
