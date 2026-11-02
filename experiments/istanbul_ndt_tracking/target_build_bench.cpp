// Time what the localizer does when it builds a new local-map target: crop the map
// to a 2D radius (cropLocalMapByRadius), hand it to NDT_OMP (setInputTarget) and
// align a ~100-point probe once, which builds the target search tree that
// pcl::Registration sets up in the first align() (warmUpRegistrationTarget). NDT_OMP
// once with a plain setInputTarget() and once with setNdtTargetWithSearchTree(),
// which builds that tree alongside, as the localizer sets NDT targets. The same for
// GICP_OMP on the crop voxel-filtered with LEAF, where the first align() also
// computes the target covariances. Median over REPEATS, in ms.
//
// usage: target_build_bench MAP.{pcd,ply} X Y RADIUS LEAF REPEATS
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

#include "lidar_localization/registration_target_search_tree.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Clock = std::chrono::steady_clock;

static double elapsed_ms(Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

static double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

// sampleWarmUpSource in src/component_alignment.cpp.
static Cloud::Ptr probe_of(const Cloud & target)
{
  Cloud::Ptr probe(new Cloud);
  const std::size_t stride = std::max<std::size_t>(1, target.size() / 100);
  for (std::size_t i = 0; i < target.size(); i += stride) {
    probe->push_back(target.points[i]);
  }
  return probe;
}

int main(int argc, char ** argv)
{
  if (argc != 7) {
    std::cerr << "usage: " << argv[0] << " MAP.{pcd,ply} X Y RADIUS LEAF REPEATS\n";
    return 1;
  }
  const std::string path = argv[1];
  const float x = std::stof(argv[2]);
  const float y = std::stof(argv[3]);
  const float radius = std::stof(argv[4]);
  const float leaf = std::stof(argv[5]);
  const int repeats = std::atoi(argv[6]);

  Cloud::Ptr map(new Cloud);
  const bool ply = path.size() > 4 && path.compare(path.size() - 4, 4, ".ply") == 0;
  if ((ply ? pcl::io::loadPLYFile(path, *map) : pcl::io::loadPCDFile(path, *map)) != 0) {
    std::cerr << "cannot read " << path << "\n";
    return 1;
  }

  std::vector<double> crop_ms, ndt_target_ms, ndt_first_align_ms;
  std::vector<double> alongside_target_ms, alongside_first_align_ms;
  std::vector<double> voxel_ms, gicp_target_ms, gicp_first_align_ms;
  std::size_t crop_points = 0;
  std::size_t filtered_points = 0;
  for (int r = 0; r < repeats; ++r) {
    // cropLocalMapByRadius in include/lidar_localization/local_map_target_policy.hpp.
    auto start = Clock::now();
    Cloud::Ptr crop(new Cloud);
    crop->reserve(map->size() / 4);
    const float radius_squared = radius * radius;
    for (const auto & p : map->points) {
      const float dx = p.x - x;
      const float dy = p.y - y;
      if (dx * dx + dy * dy <= radius_squared) {
        crop->push_back(p);
      }
    }
    crop_ms.push_back(elapsed_ms(start));
    crop_points = crop->size();

    Cloud out;
    // The NDT_OMP settings of param/istanbul_gtsam_smoother.yaml and the Koide profile.
    const auto configure = [](pclomp::NormalDistributionsTransform<Point, Point> & ndt) {
        ndt.setResolution(1.0);
        ndt.setStepSize(0.1);
        ndt.setTransformationEpsilon(0.01);
        ndt.setMaximumIterations(35);
        ndt.setNumThreads(4);
      };
    pclomp::NormalDistributionsTransform<Point, Point> ndt;
    configure(ndt);
    start = Clock::now();
    ndt.setInputTarget(crop);
    ndt_target_ms.push_back(elapsed_ms(start));
    ndt.setInputSource(probe_of(*crop));
    start = Clock::now();
    ndt.align(out, Eigen::Matrix4f::Identity());
    ndt_first_align_ms.push_back(elapsed_ms(start));

    pclomp::NormalDistributionsTransform<Point, Point> alongside;
    configure(alongside);
    start = Clock::now();
    lidar_localization::setNdtTargetWithSearchTree<Point>(alongside, crop);
    alongside_target_ms.push_back(elapsed_ms(start));
    alongside.setInputSource(probe_of(*crop));
    start = Clock::now();
    alongside.align(out, Eigen::Matrix4f::Identity());
    alongside_first_align_ms.push_back(elapsed_ms(start));

    start = Clock::now();
    Cloud::Ptr filtered(new Cloud);
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(crop);
    voxel.filter(*filtered);
    voxel_ms.push_back(elapsed_ms(start));
    filtered_points = filtered->size();
    pclomp::GeneralizedIterativeClosestPoint<Point, Point> gicp;
    gicp.setMaximumIterations(35);
    gicp.setTransformationEpsilon(0.01);
    gicp.setCorrespondenceRandomness(20);
    gicp.setMaxCorrespondenceDistance(2.0);
    start = Clock::now();
    gicp.setInputTarget(filtered);
    gicp_target_ms.push_back(elapsed_ms(start));
    gicp.setInputSource(probe_of(*filtered));
    start = Clock::now();
    gicp.align(out, Eigen::Matrix4f::Identity());
    gicp_first_align_ms.push_back(elapsed_ms(start));
  }

  std::cout << "crop_points=" << crop_points << " crop_ms=" << median(crop_ms)
            << "\nNDT_OMP setInputTarget_ms=" << median(ndt_target_ms)
            << " first_align_ms=" << median(ndt_first_align_ms)
            << "\nNDT_OMP, tree alongside: setNdtTargetWithSearchTree_ms="
            << median(alongside_target_ms)
            << " first_align_ms=" << median(alongside_first_align_ms)
            << "\nvoxel_leaf=" << leaf << " filtered_points=" << filtered_points
            << " voxel_ms=" << median(voxel_ms)
            << "\nGICP_OMP setInputTarget_ms=" << median(gicp_target_ms)
            << " first_align_ms=" << median(gicp_first_align_ms) << "\n";
  return 0;
}
