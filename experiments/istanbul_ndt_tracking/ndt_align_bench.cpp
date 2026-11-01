// Time pclomp::NormalDistributionsTransform<PointXYZI> as the localizer calls it:
// only ndt_omp.h is included, so align() runs libndt_omp's compiled instantiation.
// usage: ndt_align_bench MAP.pcd SCAN.pcd POSE.txt THREADS REPEATS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using Point = pcl::PointXYZI;
using Clock = std::chrono::steady_clock;

static double ms_since(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

int main(int argc, char ** argv) {
  if (argc != 6) {
    std::cerr << "usage: ndt_align_bench MAP SCAN POSE THREADS REPEATS\n";
    return 1;
  }
  const int threads = std::atoi(argv[4]);
  const int repeats = std::atoi(argv[5]);
  pcl::PointCloud<Point>::Ptr map(new pcl::PointCloud<Point>), scan(new pcl::PointCloud<Point>);
  if (pcl::io::loadPCDFile(argv[1], *map) || pcl::io::loadPCDFile(argv[2], *scan)) {return 1;}
  double x, y, z, qx, qy, qz, qw;
  std::ifstream(argv[3]) >> x >> y >> z >> qx >> qy >> qz >> qw;

  // The localizer's 150 m local map crop (2D radius), unfiltered for NDT.
  pcl::PointCloud<Point>::Ptr crop(new pcl::PointCloud<Point>);
  for (const auto & p : map->points) {
    if (std::hypot(p.x - x, p.y - y) <= 150.0) {crop->push_back(p);}
  }
  map.reset();

  pclomp::NormalDistributionsTransform<Point, Point> ndt;
  ndt.setResolution(1.0);
  ndt.setStepSize(0.1);
  ndt.setTransformationEpsilon(0.01);
  ndt.setMaximumIterations(35);
  ndt.setNumThreads(threads);
  auto t = Clock::now();
  ndt.setInputTarget(crop);
  const double target_ms = ms_since(t);
  ndt.setInputSource(scan);

  Eigen::Matrix4f gnss = Eigen::Matrix4f::Identity();
  gnss.block<3, 3>(0, 0) = Eigen::Quaternionf(qw, qx, qy, qz).normalized().toRotationMatrix();
  gnss.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
  pcl::PointCloud<Point> out;
  t = Clock::now();
  ndt.align(out, gnss);  // first align also builds pcl::Registration's target tree
  const double first_ms = ms_since(t);
  const Eigen::Matrix4f converged = ndt.getFinalTransformation();
  std::cout << "crop_points=" << crop->size() << " scan_points=" << scan->size()
            << " threads=" << threads << " setInputTarget_ms=" << target_ms
            << " first_align_ms=" << first_ms << "\n";

  // Seeds around the converged pose: what the twist smoother typically hands NDT.
  struct Seed {const char * name; float dx, dy, yaw_deg;};
  for (const Seed & s : {Seed{"converged", 0, 0, 0}, Seed{"0.3m_0.5deg", 0.25f, 0.15f, 0.5f},
                         Seed{"1.0m_1deg", 0.8f, 0.6f, 1.0f}}) {
    Eigen::Matrix4f seed = converged;
    seed(0, 3) += s.dx;
    seed(1, 3) += s.dy;
    seed.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(s.yaw_deg * M_PI / 180.0, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
      seed.block<3, 3>(0, 0);
    std::vector<double> times;
    int iterations = 0;
    for (int i = 0; i < repeats; ++i) {
      t = Clock::now();
      ndt.align(out, seed);
      times.push_back(ms_since(t));
      iterations = ndt.getFinalNumIteration();
    }
    std::sort(times.begin(), times.end());
    const Eigen::Matrix4f d = converged.inverse() * ndt.getFinalTransformation();
    std::cout << "  seed=" << s.name << " median_ms=" << times[times.size() / 2]
              << " min_ms=" << times.front() << " iterations=" << iterations
              << " residual_m=" << d.block<3, 1>(0, 3).norm() << "\n";
  }
  return 0;
}
