// Align exported Boreas scans (export_scans.py) against the fixture map the way the
// Boreas preset does: 100 m crop rebuilt every 10 m, scan range 1-60 m, voxel LEAF,
// NDT_OMP (resolution, step 0.1, epsilon, 25 iterations), seed = constant velocity
// from the ground truth, optionally moved by PERTURB m and 2*PERTURB deg. Times
// align(), PCL's getFitnessScore() and lidar_localization::parallelFitnessScore()
// (and checks the two scores are identical); error against the ground truth.
// METHOD is pclomp::NeighborSearchMethod: 0 KDTREE, 1 DIRECT26, 2 DIRECT7, 3 DIRECT1.
// usage: boreas_ndt_bench MAP.ply SCANDIR THREADS RESOLUTION LEAF METHOD EPSILON [PERTURB]
#include "lidar_localization/registration_fitness.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Clock = std::chrono::steady_clock;
static double ms(Clock::time_point t) {return std::chrono::duration<double, std::milli>(Clock::now() - t).count();}

static Cloud::Ptr load_ply(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  std::string line; std::size_t n = 0;
  while (std::getline(in, line)) {
    if (line.rfind("element vertex", 0) == 0) {n = std::stoul(line.substr(15));}
    if (line == "end_header") {break;}
  }
  Cloud::Ptr cloud(new Cloud); cloud->resize(n);
  std::vector<float> buffer(4 * n);
  in.read(reinterpret_cast<char *>(buffer.data()), buffer.size() * sizeof(float));
  for (std::size_t i = 0; i < n; ++i) {
    (*cloud)[i].x = buffer[4 * i]; (*cloud)[i].y = buffer[4 * i + 1];
    (*cloud)[i].z = buffer[4 * i + 2]; (*cloud)[i].intensity = buffer[4 * i + 3];
  }
  return cloud;
}

static Eigen::Matrix4f pose(const double * v)
{
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = Eigen::Quaternionf(v[6], v[3], v[4], v[5]).normalized().toRotationMatrix();
  m(0, 3) = v[0]; m(1, 3) = v[1]; m(2, 3) = v[2];
  return m;
}


int main(int argc, char ** argv)
{
  if (argc != 8 && argc != 9) {std::cerr << "usage: MAP SCANDIR THREADS RES LEAF METHOD EPS [PERTURB_M]\n"; return 1;}
  const float perturb = argc == 9 ? std::atof(argv[8]) : 0.0f;
  const std::string dir = argv[2];
  const int threads = std::atoi(argv[3]);
  const float resolution = std::atof(argv[4]);
  const float leaf = std::atof(argv[5]);
  const int method = std::atoi(argv[6]);
  const double epsilon = std::atof(argv[7]);
  Cloud::Ptr map = load_ply(argv[1]);
  std::ifstream index(dir + "/index.txt");
  std::vector<std::vector<double>> rows;
  for (std::string line; std::getline(index, line);) {
    std::istringstream s(line); std::vector<double> v; double x;
    while (s >> x) {v.push_back(x);}
    rows.push_back(v);
  }
  pclomp::NormalDistributionsTransform<Point, Point> ndt;
  ndt.setResolution(resolution); ndt.setStepSize(0.1); ndt.setTransformationEpsilon(epsilon);
  ndt.setMaximumIterations(25); ndt.setNumThreads(threads);
  ndt.setNeighborhoodSearchMethod(static_cast<pclomp::NeighborSearchMethod>(method));
  Eigen::Vector2f crop_center(1e9, 1e9);
  std::vector<double> parallel_ms; int fitness_mismatches = 0;
  std::vector<double> align_ms, fitness_ms, iterations, trans_err, yaw_err, points;
  for (std::size_t k = 0; k < rows.size(); ++k) {
    const auto & r = rows[k];
    const Eigen::Matrix4f gt = pose(&r[1]), prev = pose(&r[9]), prev2 = pose(&r[17]);
    Eigen::Matrix4f seed = prev * (prev2.inverse() * prev);
    if (perturb > 0.0f) {
      // Deterministic per scan: PERTURB m in x/y and 2*PERTURB deg of yaw, signs from k.
      const float sx = (k % 2) ? 1.0f : -1.0f, sy = ((k / 2) % 2) ? 1.0f : -1.0f;
      const float yaw = ((k / 4) % 2 ? 1.0f : -1.0f) * 2.0f * perturb * float(M_PI) / 180.0f;
      Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
      delta.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
      seed = seed * delta;
      seed(0, 3) += sx * perturb; seed(1, 3) += sy * perturb;
    }
    const Eigen::Vector2f center = gt.block<2, 1>(0, 3);
    if ((center - crop_center).norm() > 10.0f) {
      Cloud::Ptr crop(new Cloud);
      for (const auto & p : *map) {
        if (std::hypot(p.x - center.x(), p.y - center.y()) <= 100.0f) {crop->push_back(p);}
      }
      ndt.setInputTarget(crop);
      crop_center = center;
      // The first align() after setInputTarget() builds the target search tree; the node
      // builds it ahead (setNdtTargetWithSearchTree), so warm up outside the timing.
      Cloud::Ptr probe(new Cloud);
      for (std::size_t i = 0; i < crop->size() && probe->size() < 100; i += crop->size() / 100 + 1) {
        probe->push_back((*crop)[i]);
      }
      ndt.setInputSource(probe);
      Cloud warm;
      ndt.align(warm, gt);
    }
    char name[64]; std::snprintf(name, sizeof(name), "/scan_%04zu.bin", k);
    std::ifstream in(dir + name, std::ios::binary | std::ios::ate);
    const std::size_t bytes = in.tellg(); in.seekg(0);
    std::vector<float> buffer(bytes / sizeof(float));
    in.read(reinterpret_cast<char *>(buffer.data()), bytes);
    Cloud::Ptr scan(new Cloud);
    for (std::size_t i = 0; i + 3 < buffer.size(); i += 4) {
      const float range = std::sqrt(buffer[i] * buffer[i] + buffer[i + 1] * buffer[i + 1] + buffer[i + 2] * buffer[i + 2]);
      if (range < 1.0f || range > 60.0f) {continue;}
      Point p; p.x = buffer[i]; p.y = buffer[i + 1]; p.z = buffer[i + 2]; p.intensity = buffer[i + 3];
      scan->push_back(p);
    }
    Cloud::Ptr filtered(new Cloud);
    pcl::VoxelGrid<Point> grid; grid.setLeafSize(leaf, leaf, leaf); grid.setInputCloud(scan); grid.filter(*filtered);
    ndt.setInputSource(filtered);
    Cloud output;
    auto t = Clock::now();
    ndt.align(output, seed);
    align_ms.push_back(ms(t));
    t = Clock::now();
    const double fitness = ndt.getFitnessScore();
    fitness_ms.push_back(ms(t));
    t = Clock::now();
    const double fitness_parallel = lidar_localization::parallelFitnessScore(ndt, threads);
    parallel_ms.push_back(ms(t));
    if (fitness != fitness_parallel) {++fitness_mismatches;}
    iterations.push_back(ndt.getFinalNumIteration());
    points.push_back(filtered->size());
    const Eigen::Matrix4f result = ndt.getFinalTransformation();
    trans_err.push_back((result.block<3, 1>(0, 3) - gt.block<3, 1>(0, 3)).norm());
    const Eigen::Matrix3f dr = gt.block<3, 3>(0, 0).transpose() * result.block<3, 3>(0, 0);
    yaw_err.push_back(std::abs(std::atan2(dr(1, 0), dr(0, 0))) * 180.0 / M_PI);
  }
  auto mean = [](const std::vector<double> & v) {double s = 0; for (double x : v) {s += x;} return s / v.size();};
  auto pct = [](std::vector<double> v, double q) {std::sort(v.begin(), v.end()); return v[std::min(v.size() - 1, std::size_t(q * v.size()))];};
  double rmse = 0; for (double e : trans_err) {rmse += e * e;} rmse = std::sqrt(rmse / trans_err.size());
  std::printf(
    "threads %d res %.2f leaf %.2f method %d eps %.3f | points %.0f | align mean %.1f p90 %.1f ms | "
    "fitness %.1f ms | iterations %.2f | translation RMSE %.3f max %.3f m | yaw max %.2f deg\n",
    threads, resolution, leaf, method, epsilon, mean(points), mean(align_ms), pct(align_ms, 0.9),
    mean(fitness_ms), mean(iterations), rmse, *std::max_element(trans_err.begin(), trans_err.end()),
    *std::max_element(yaw_err.begin(), yaw_err.end()));
  std::printf("parallel fitness %.2f ms (PCL %.2f ms), mismatches %d of %zu\n", mean(parallel_ms), mean(fitness_ms), fitness_mismatches, parallel_ms.size());
  return 0;
}
