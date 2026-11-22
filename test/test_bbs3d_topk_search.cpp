// Tests of the top-k 3D branch-and-bound search (bbs3d_topk_search.hpp) used by the
// optional G2 engine bbs3d: the scan filters, recovery of a known pose on a map with
// two floors, the separation of the k answers, and the queue cap.

#include "lidar_localization/bbs3d_topk_search.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace bbs3d = lidar_localization::bbs3d;

namespace
{

int failures = 0;

void check(bool condition, const char * what)
{
  if (!condition) {
    std::fprintf(stderr, "FAILED: %s\n", what);
    ++failures;
  }
}

// A wall from (x0, y0) to (x1, y1), from z0 up 2.5 m, sampled every 0.1 m.
void add_wall(bbs3d::Points & points, double x0, double y0, double x1, double y1, double z0)
{
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int steps = static_cast<int>(length / 0.1);
  for (int i = 0; i <= steps; ++i) {
    const double t = steps == 0 ? 0.0 : static_cast<double>(i) / steps;
    for (int j = 0; j <= 25; ++j) {
      points.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0), z0 + 0.1 * j);
    }
  }
}

void add_floor(bbs3d::Points & points, double x0, double y0, double x1, double y1, double z)
{
  const int nx = static_cast<int>(std::floor((x1 - x0) / 0.2 + 1e-9));
  const int ny = static_cast<int>(std::floor((y1 - y0) / 0.2 + 1e-9));
  for (int i = 0; i <= nx; ++i) {
    for (int j = 0; j <= ny; ++j) {
      points.emplace_back(x0 + 0.2 * i, y0 + 0.2 * j, z);
    }
  }
}

// Ground floor: an L-shaped room with a pillar; upper floor, 5 m above, a rectangular
// room with an inner wall, over the same footprint.
bbs3d::Points two_floor_map()
{
  bbs3d::Points points;
  add_floor(points, 0.0, 0.0, 20.0, 12.0, 0.0);
  add_wall(points, 0.0, 0.0, 20.0, 0.0, 0.0);
  add_wall(points, 20.0, 0.0, 20.0, 6.0, 0.0);
  add_wall(points, 20.0, 6.0, 10.0, 6.0, 0.0);
  add_wall(points, 10.0, 6.0, 10.0, 12.0, 0.0);
  add_wall(points, 10.0, 12.0, 0.0, 12.0, 0.0);
  add_wall(points, 0.0, 12.0, 0.0, 0.0, 0.0);
  add_wall(points, 4.0, 3.0, 4.6, 3.0, 0.0);
  add_wall(points, 4.6, 3.0, 4.6, 3.6, 0.0);

  add_floor(points, 0.0, 0.0, 20.0, 12.0, 5.0);
  add_wall(points, 0.0, 0.0, 20.0, 0.0, 5.0);
  add_wall(points, 20.0, 0.0, 20.0, 12.0, 5.0);
  add_wall(points, 20.0, 12.0, 0.0, 12.0, 5.0);
  add_wall(points, 0.0, 12.0, 0.0, 0.0, 5.0);
  add_wall(points, 13.0, 0.0, 13.0, 7.0, 5.0);
  add_wall(points, 6.0, 12.0, 6.0, 8.0, 5.0);
  return points;
}

// The map points on the given floor within range of the sensor, in a sensor frame at
// (x, y, z) turned by yaw.
bbs3d::Points scan_from(
  const bbs3d::Points & map, double x, double y, double z, double yaw, double floor_z)
{
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d position(x, y, z);
  bbs3d::Points scan;
  for (const auto & point : map) {
    if (point.z() < floor_z - 0.05 || point.z() > floor_z + 2.6) {
      continue;
    }
    scan.push_back(rotation.transpose() * (point - position));
  }
  return scan;
}

double yaw_of(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

void test_filters()
{
  const bbs3d::Points points{
    {0.1, 0.1, 0.1}, {0.3, 0.3, 0.3}, {-0.2, 0.0, 0.0}, {1.2, 0.0, 0.0},
    {std::nan(""), 0.0, 0.0}};
  const auto filtered = bbs3d::voxel_centroid_filter(points, 0.5);
  check(filtered.size() == 3, "voxel filter keeps one point per voxel");
  // Ordered by voxel index: (-1, 0, 0), (0, 0, 0), (2, 0, 0).
  check(filtered[0].isApprox(Eigen::Vector3d(-0.2, 0.0, 0.0)), "voxel filter order 0");
  check(filtered[1].isApprox(Eigen::Vector3d(0.2, 0.2, 0.2)), "voxel filter centroid");
  check(filtered[2].isApprox(Eigen::Vector3d(1.2, 0.0, 0.0)), "voxel filter order 2");
  check(bbs3d::voxel_centroid_filter(points, 0.0).size() == points.size(), "leaf 0 keeps all");

  const auto ranged = bbs3d::range_filter(points, 0.25, 1.0);
  check(ranged.size() == 1, "range filter keeps 0.25-1.0 m");
  check(bbs3d::range_filter(points, 0.0, 0.0).size() == 4, "max 0 keeps all finite");
}

void test_recovers_pose_on_the_upper_floor()
{
  const auto map = bbs3d::voxel_centroid_filter(two_floor_map(), 0.1);
  const bbs3d::TopkSearch search(map, 0.5, 4);
  check(search.bucket_count() > 0, "voxel maps built");
  check(search.memory_bytes() == search.bucket_count() * 16, "memory is 16 bytes per bucket");

  const double x = 15.3;
  const double y = 9.1;
  const double z = 6.0;
  const double yaw = 0.7;
  const auto scan = bbs3d::voxel_centroid_filter(
    bbs3d::range_filter(scan_from(map, x, y, z, yaw, 5.0), 1.0, 30.0), 0.5);
  bbs3d::SearchParams params;
  params.k = 4;
  params.min_separation_m = 3.0;
  params.timeout_ms = 60000;
  const auto result = search.search(scan, params);
  check(!result.timed_out && !result.queue_capped, "search completes");
  check(!result.candidates.empty(), "search answers");
  if (result.candidates.empty()) {
    return;
  }
  const auto & top = result.candidates.front();
  const Eigen::Vector3d position = top.pose.block<3, 1>(0, 3);
  std::printf(
    "top: %.2f %.2f %.2f yaw %.3f score %.3f, %zu answers, %zu expanded, %.1f ms\n",
    position.x(), position.y(), position.z(), yaw_of(top.pose), top.score_ratio,
    result.candidates.size(), result.expanded, result.elapsed_ms);
  check(std::hypot(position.x() - x, position.y() - y) < 0.75, "top answer within 0.75 m");
  check(std::abs(position.z() - z) < 0.75, "top answer on the upper floor");
  check(
    std::abs(std::remainder(yaw_of(top.pose) - yaw, 2.0 * M_PI)) < 0.1,
    "top answer heading within 0.1 rad");
  check(top.score_ratio > 0.9, "top answer covers the scan");
  for (std::size_t i = 0; i < result.candidates.size(); ++i) {
    check(
      i == 0 || result.candidates[i].score <= result.candidates[i - 1].score,
      "answers in score order");
    check(
      result.candidates[i].score >= std::ceil(0.9 * top.score), "answers above the floor");
    for (std::size_t j = 0; j < i; ++j) {
      const double distance = (result.candidates[i].pose.block<3, 1>(0, 3) -
        result.candidates[j].pose.block<3, 1>(0, 3)).norm();
      check(distance >= 3.0, "answers at least min_separation_m apart");
    }
  }

  // k = 1 gives the same best answer.
  params.k = 1;
  const auto single = search.search(scan, params);
  check(single.candidates.size() == 1, "k = 1 answers once");
  if (!single.candidates.empty()) {
    check(single.candidates[0].pose.isApprox(top.pose), "k = 1 has the same best pose");
  }
}

void test_limits()
{
  const auto map = bbs3d::voxel_centroid_filter(two_floor_map(), 0.1);
  const bbs3d::TopkSearch search(map, 0.5, 4);
  const auto scan = bbs3d::voxel_centroid_filter(
    bbs3d::range_filter(scan_from(map, 5.0, 8.0, 1.0, -1.2, 0.0), 1.0, 30.0), 0.5);

  bbs3d::SearchParams params;
  params.max_queue_nodes = 1;
  const auto capped = search.search(scan, params);
  check(capped.queue_capped, "a queue cap of one node stops the search");
  check(capped.expanded == 0, "nothing expanded past the cap");

  params.max_queue_nodes = 0;
  params.min_score_ratio = 1.01;
  const auto floored = search.search(scan, params);
  check(floored.candidates.empty(), "no answer above an unreachable score floor");

  const auto empty = search.search(bbs3d::Points{}, bbs3d::SearchParams{});
  check(empty.candidates.empty() && empty.scan_points == 0, "an empty scan has no answer");

  bool threw = false;
  try {
    const bbs3d::TopkSearch no_map(bbs3d::Points{}, 0.5, 4);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "an empty map is refused");
}

}  // namespace

int main()
{
  test_filters();
  test_recovers_pose_on_the_upper_floor();
  test_limits();
  if (failures != 0) {
    std::fprintf(stderr, "%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("bbs3d_topk_search: all checks passed\n");
  return EXIT_SUCCESS;
}
