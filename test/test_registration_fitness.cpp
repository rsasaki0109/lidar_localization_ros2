// parallelFitnessScore returns pcl::Registration::getFitnessScore() bit for bit, on
// several threads, for NDT_OMP and PCL's NDT, and before and after alignment.
#include "lidar_localization/registration_fitness.hpp"

#include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>

#include <cassert>
#include <cmath>

namespace
{

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

// A 40 x 40 m ground grid at 0.25 m with 3 m walls every 10 m.
Cloud::Ptr makeMap()
{
  Cloud::Ptr map(new Cloud());
  Point p;
  p.intensity = 1.0f;
  for (float x = -20.0f; x <= 20.0f; x += 0.25f) {
    for (float y = -20.0f; y <= 20.0f; y += 0.25f) {
      p.x = x;
      p.y = y;
      p.z = 0.0f;
      map->push_back(p);
      if (std::fmod(x + 20.0f, 10.0f) == 0.0f || std::fmod(y + 20.0f, 10.0f) == 0.0f) {
        for (p.z = 0.25f; p.z <= 3.0f; p.z += 0.25f) {
          map->push_back(p);
        }
      }
    }
  }
  return map;
}

// Every 23rd map point within 12 m, moved by (0.4, -0.3, 0.05) and given a small
// offset so that most of them do not fall on a map point.
Cloud::Ptr makeScan(const Cloud & map)
{
  Cloud::Ptr scan(new Cloud());
  for (std::size_t i = 0; i < map.size(); i += 23) {
    Point p = map.points[i];
    if (std::hypot(p.x, p.y) > 12.0f) {
      continue;
    }
    p.x += 0.4f + 0.01f * static_cast<float>(i % 7);
    p.y -= 0.3f;
    p.z += 0.05f;
    scan->push_back(p);
  }
  return scan;
}

template<typename Registration>
void checkMatchesPcl(Registration & registration)
{
  const double pcl_score = registration.getFitnessScore();
  for (const int threads : {1, 2, 4, 8}) {
    const double parallel_score = lidar_localization::parallelFitnessScore(registration, threads);
    assert(parallel_score == pcl_score);
  }
  assert(std::isfinite(pcl_score));
}

void test_ndt_omp_score_is_identical_before_and_after_alignment()
{
  const Cloud::Ptr map = makeMap();
  const Cloud::Ptr scan = makeScan(*map);
  pclomp::NormalDistributionsTransform<Point, Point> ndt;
  ndt.setResolution(1.0);
  ndt.setNumThreads(2);
  ndt.setInputTarget(map);
  ndt.setInputSource(scan);
  Cloud aligned;
  // One iteration leaves the scan misaligned, so the score is far from zero.
  ndt.setMaximumIterations(1);
  ndt.align(aligned, Eigen::Matrix4f::Identity());
  checkMatchesPcl(ndt);
  ndt.setMaximumIterations(30);
  ndt.align(aligned, Eigen::Matrix4f::Identity());
  checkMatchesPcl(ndt);
}

void test_pcl_ndt_score_is_identical()
{
  const Cloud::Ptr map = makeMap();
  const Cloud::Ptr scan = makeScan(*map);
  pcl::NormalDistributionsTransform<Point, Point> ndt;
  ndt.setResolution(1.0);
  ndt.setInputTarget(map);
  ndt.setInputSource(scan);
  Cloud aligned;
  ndt.align(aligned, Eigen::Matrix4f::Identity());
  checkMatchesPcl(ndt);
}

}  // namespace

int main()
{
  test_ndt_omp_score_is_identical_before_and_after_alignment();
  test_pcl_ndt_score_is_identical();
  return 0;
}
