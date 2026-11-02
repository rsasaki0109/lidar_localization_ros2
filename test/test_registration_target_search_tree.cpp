// setNdtTargetWithSearchTree: the target search tree is built alongside
// setInputTarget() and kept, the alignment and the fitness score match a plain
// setInputTarget(), and a later target replaces the tree instead of reusing it.
#include "lidar_localization/registration_target_search_tree.hpp"

#include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <type_traits>

namespace
{

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

// A 60 x 60 m ground grid at 0.25 m, so a 1 m NDT voxel holds more than the 6
// points it needs, with 3 m walls every 10 m along x and along y, moved by `shift_x`.
Cloud::Ptr makeMap(float shift_x)
{
  Cloud::Ptr map(new Cloud());
  Point p;
  p.intensity = 1.0f;
  for (float x = -30.0f; x <= 30.0f; x += 0.25f) {
    for (float y = -30.0f; y <= 30.0f; y += 0.25f) {
      p.x = x + shift_x;
      p.y = y;
      p.z = 0.0f;
      map->push_back(p);
      const bool wall_x = std::fmod(x + 30.0f, 10.0f) == 0.0f;
      const bool wall_y = std::fmod(y + 30.0f, 10.0f) == 0.0f;
      if (wall_x || wall_y) {
        for (p.z = 0.25f; p.z <= 3.0f; p.z += 0.25f) {
          map->push_back(p);
        }
      }
    }
  }
  return map;
}

// Every 29th map point within 15 m, moved by (0.3, -0.2, 0).
Cloud::Ptr makeScan(const Cloud & map)
{
  Cloud::Ptr scan(new Cloud());
  for (std::size_t i = 0; i < map.size(); i += 29) {
    Point p = map.points[i];
    if (std::hypot(p.x, p.y) > 15.0f) {
      continue;
    }
    p.x += 0.3f;
    p.y -= 0.2f;
    scan->push_back(p);
  }
  return scan;
}

struct Result
{
  double fitness;
  Eigen::Matrix4f transform;
};

Result alignOnce(pcl::Registration<Point, Point> & registration, const Cloud::Ptr & scan)
{
  registration.setInputSource(scan);
  Cloud output;
  registration.align(output, Eigen::Matrix4f::Identity());
  return Result{registration.getFitnessScore(), registration.getFinalTransformation()};
}

void assertSame(const Result & a, const Result & b)
{
  assert(std::abs(a.fitness - b.fitness) <= 1e-12 * std::max(1.0, std::abs(a.fitness)));
  assert((a.transform - b.transform).cwiseAbs().maxCoeff() < 1e-6f);
}

template<typename Ndt>
void checkBackend()
{
  const Cloud::Ptr first_map = makeMap(0.0f);
  const Cloud::Ptr second_map = makeMap(1.0f);
  const Cloud::Ptr scan = makeScan(*first_map);

  auto configure = [](Ndt & ndt) {
      ndt.setResolution(1.0);
      ndt.setStepSize(0.1);
      ndt.setTransformationEpsilon(0.01);
      ndt.setMaximumIterations(35);
      if constexpr (std::is_same_v<Ndt, pclomp::NormalDistributionsTransform<Point, Point>>) {
        ndt.setNumThreads(2);
      }
    };

  Ndt plain;
  configure(plain);
  plain.setInputTarget(first_map);
  const Result plain_first = alignOnce(plain, scan);
  plain.setInputTarget(second_map);
  const Result plain_second = alignOnce(plain, scan);
  // The scan starts 0.36 m off the map, so the alignment iterates and moves it.
  assert((plain_first.transform.block<3, 1>(0, 3).norm() > 0.05f));
  assert(plain_first.fitness > 0.0);

  Ndt overlapped;
  configure(overlapped);
  lidar_localization::setNdtTargetWithSearchTree<Point>(overlapped, first_map);
  // The tree is ready before the first align() and indexes this target.
  assert(overlapped.getSearchMethodTarget()->getInputCloud() == first_map);
  assertSame(alignOnce(overlapped, scan), plain_first);

  // A second target gets its own tree: the first one is not kept.
  lidar_localization::setNdtTargetWithSearchTree<Point>(overlapped, second_map);
  assert(overlapped.getSearchMethodTarget()->getInputCloud() == second_map);
  assertSame(alignOnce(overlapped, scan), plain_second);

  // setRegistrationTarget: the same for NDT, a plain setInputTarget otherwise.
  Ndt dispatched;
  configure(dispatched);
  lidar_localization::setRegistrationTarget<Point>(dispatched, first_map, true);
  assert(dispatched.getSearchMethodTarget()->getInputCloud() == first_map);
  assertSame(alignOnce(dispatched, scan), plain_first);

  Ndt untouched;
  configure(untouched);
  const auto default_tree = untouched.getSearchMethodTarget();
  lidar_localization::setRegistrationTarget<Point>(untouched, first_map, false);
  assert(untouched.getSearchMethodTarget() == default_tree);
  assertSame(alignOnce(untouched, scan), plain_first);

  // An empty target leaves the tree to align(), as a plain setInputTarget() does.
  Ndt emptied;
  configure(emptied);
  lidar_localization::setNdtTargetWithSearchTree<Point>(emptied, first_map);
  lidar_localization::setNdtTargetWithSearchTree<Point>(emptied, Cloud::Ptr(new Cloud()));
  assert(emptied.getSearchMethodTarget()->getInputCloud() == nullptr);
}

}  // namespace

int main()
{
  checkBackend<pclomp::NormalDistributionsTransform<Point, Point>>();
  checkBackend<pcl::NormalDistributionsTransform<Point, Point>>();
  return 0;
}
