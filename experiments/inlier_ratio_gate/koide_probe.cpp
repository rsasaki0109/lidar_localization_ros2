// Koide outdoor_hard_01a recovery probe. NDT_OMP as the 01a localizer-alone replay runs
// it: resolution 1.0, step 0.1, epsilon 0.01, 35 iterations, 4 threads, DIRECT7, scan
// 1-100 m and a 0.5 m voxel, a 150 m crop of the map; fitness = PCL getFitnessScore.
//
//   probe MAP.ply SCANDIR oracle
//     every scan aligned from its ground-truth pose: fitness, final error
//   probe MAP.ply SCANDIR search X Y Z YAW_DEG RADIUS STEP YAW_RANGE YAW_STEP FIRST LAST
//     scans FIRST..LAST aligned from a grid of seeds around (X, Y, Z, YAW) with the
//     ground truth's roll and pitch: the best candidates by fitness, and their errors
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

static Cloud::Ptr loadPly(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  std::string line;
  std::size_t n = 0;
  while (std::getline(in, line)) {
    if (line.rfind("element vertex", 0) == 0) {n = std::stoul(line.substr(15));}
    if (line == "end_header") {break;}
  }
  std::vector<float> buffer(3 * n);
  in.read(reinterpret_cast<char *>(buffer.data()), buffer.size() * sizeof(float));
  Cloud::Ptr cloud(new Cloud);
  cloud->resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    (*cloud)[i].x = buffer[3 * i]; (*cloud)[i].y = buffer[3 * i + 1]; (*cloud)[i].z = buffer[3 * i + 2];
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

static double yawOf(const Eigen::Matrix4f & m) {return std::atan2(m(1, 0), m(0, 0));}

static double yawErrorDeg(const Eigen::Matrix4f & a, const Eigen::Matrix4f & b)
{
  double d = yawOf(a) - yawOf(b);
  while (d > M_PI) {d -= 2 * M_PI;}
  while (d < -M_PI) {d += 2 * M_PI;}
  return std::abs(d) * 180.0 / M_PI;
}

struct Scans
{
  std::vector<double> stamp;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> gt;
  std::vector<Cloud::Ptr> cloud;
};

static Scans loadScans(const std::string & dir)
{
  Scans s;
  std::ifstream index(dir + "/index.txt");
  for (std::string line; std::getline(index, line);) {
    std::istringstream ls(line);
    double v[8];
    for (double & x : v) {ls >> x;}
    s.stamp.push_back(v[0]);
    s.gt.push_back(pose(v + 1));
  }
  for (std::size_t k = 0; k < s.stamp.size(); ++k) {
    char name[64];
    std::snprintf(name, sizeof(name), "/scan_%04zu.bin", k);
    std::ifstream in(dir + name, std::ios::binary | std::ios::ate);
    const std::size_t bytes = in.tellg();
    in.seekg(0);
    std::vector<float> b(bytes / sizeof(float));
    in.read(reinterpret_cast<char *>(b.data()), bytes);
    Cloud::Ptr raw(new Cloud);
    for (std::size_t i = 0; i + 2 < b.size(); i += 3) {
      const float r = std::sqrt(b[i] * b[i] + b[i + 1] * b[i + 1] + b[i + 2] * b[i + 2]);
      if (r < 1.0f || r > 100.0f) {continue;}
      raw->push_back(Point(b[i], b[i + 1], b[i + 2]));
    }
    Cloud::Ptr filtered(new Cloud);
    pcl::VoxelGrid<Point> grid;
    grid.setLeafSize(0.5f, 0.5f, 0.5f);
    grid.setInputCloud(raw);
    grid.filter(*filtered);
    s.cloud.push_back(filtered);
  }
  return s;
}

struct Localizer
{
  Cloud::Ptr map;
  pclomp::NormalDistributionsTransform<Point, Point> ndt;
  Eigen::Vector2f crop_center{1e9f, 1e9f};
  pcl::KdTreeFLANN<Point> tree;

  explicit Localizer(Cloud::Ptr m) : map(std::move(m))
  {
    ndt.setResolution(1.0);
    ndt.setStepSize(0.1);
    ndt.setTransformationEpsilon(0.01);
    ndt.setMaximumIterations(35);
    ndt.setNumThreads(4);
    ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  }

  void cropAround(const Eigen::Vector2f & c)
  {
    if ((c - crop_center).norm() <= 10.0f) {return;}
    Cloud::Ptr crop(new Cloud);
    for (const auto & p : *map) {
      if (std::hypot(p.x - c.x(), p.y - c.y()) <= 150.0f) {crop->push_back(p);}
    }
    ndt.setInputTarget(crop);
    tree.setInputCloud(crop);
    crop_center = c;
  }

  // Aligns from seed; returns the fitness, sets result.
  double align(const Cloud::Ptr & scan, const Eigen::Matrix4f & seed, Eigen::Matrix4f & result)
  {
    cropAround(seed.block<2, 1>(0, 3));
    ndt.setInputSource(scan);
    Cloud out;
    ndt.align(out, seed);
    result = ndt.getFinalTransformation();
    return ndt.getFitnessScore();
  }

  // Share of scan points within 0.5 m of a map point at pose.
  double inlierRatio(const Cloud::Ptr & scan, const Eigen::Matrix4f & at)
  {
    Cloud moved;
    pcl::transformPointCloud(*scan, moved, at);
    std::vector<int> index(1);
    std::vector<float> distance(1);
    std::size_t inliers = 0;
    for (const auto & p : moved) {
      if (tree.nearestKSearch(p, 1, index, distance) > 0 && distance[0] < 0.25f) {++inliers;}
    }
    return moved.empty() ? 0.0 : static_cast<double>(inliers) / moved.size();
  }
};

int main(int argc, char ** argv)
{
  if (argc < 4) {std::cerr << "usage: MAP SCANDIR oracle|search ...\n"; return 1;}
  Localizer loc(loadPly(argv[1]));
  const Scans s = loadScans(argv[2]);
  const std::string mode = argv[3];
  const double start = 1694532829.4995492;
  if (mode == "oracle") {
    for (std::size_t k = 0; k < s.stamp.size(); ++k) {
      Eigen::Matrix4f result;
      const double fitness = loc.align(s.cloud[k], s.gt[k], result);
      const double et = (result.block<3, 1>(0, 3) - s.gt[k].block<3, 1>(0, 3)).norm();
      std::printf("%7.2f pts %5zu fitness %7.2f inliers %.2f error %5.2f m %5.2f deg\n", s.stamp[k] - start,
        s.cloud[k]->size(), fitness, loc.inlierRatio(s.cloud[k], result), et, yawErrorDeg(result, s.gt[k]));
    }
    return 0;
  }
  if (mode == "eval" && argc == 5) {
    // Inlier ratio at the poses of a TUM file (no alignment), for scans with the same stamp.
    std::ifstream tum(argv[4]);
    for (std::string line; std::getline(tum, line);) {
      std::istringstream ls(line);
      double v[8];
      for (double & x : v) {ls >> x;}
      for (std::size_t k = 0; k < s.stamp.size(); ++k) {
        if (std::abs(s.stamp[k] - v[0]) > 0.005) {continue;}
        const Eigen::Matrix4f at = pose(v + 1);
        loc.cropAround(at.block<2, 1>(0, 3));
        const double et = (at.block<3, 1>(0, 3) - s.gt[k].block<3, 1>(0, 3)).norm();
        std::printf("%7.2f inliers %.2f (GT %.2f) error %5.2f m %5.1f deg\n", s.stamp[k] - start,
          loc.inlierRatio(s.cloud[k], at), loc.inlierRatio(s.cloud[k], s.gt[k]), et, yawErrorDeg(at, s.gt[k]));
      }
    }
    return 0;
  }
  if (mode == "track" && argc == 6) {
    // A bare tracker from the first scan's ground truth: seed = the last accepted pose
    // moved by the last accepted motion, scaled to the elapsed time; no IMU. GATE is
    // "fitness:6" or "inliers:0.7".
    const std::string gate = argv[4];
    const std::string kind = gate.substr(0, gate.find(':'));
    const double limit = std::atof(gate.substr(gate.find(':') + 1).c_str());
    (void)argv[5];
    Eigen::Matrix4f last = s.gt[0], before = s.gt[0];
    double last_t = s.stamp[0], before_t = s.stamp[0] - 0.1;
    for (std::size_t k = 1; k < s.stamp.size(); ++k) {
      const double ratio = (s.stamp[k] - last_t) / std::max(1e-3, last_t - before_t);
      // Constant velocity: delta = before^-1 * last, applied ratio times (translation scaled).
      Eigen::Matrix4f delta = before.inverse() * last;
      Eigen::AngleAxisf aa(Eigen::Matrix3f(delta.block<3, 3>(0, 0)));
      Eigen::Matrix4f step = Eigen::Matrix4f::Identity();
      step.block<3, 3>(0, 0) = Eigen::AngleAxisf(aa.angle() * std::min(ratio, 10.0), aa.axis()).toRotationMatrix();
      step.block<3, 1>(0, 3) = delta.block<3, 1>(0, 3) * std::min(ratio, 10.0);
      const Eigen::Matrix4f seed = last * step;
      if (s.cloud[k]->size() < 50) {continue;}
      Eigen::Matrix4f result;
      const double fitness = loc.align(s.cloud[k], seed, result);
      const double inliers = loc.inlierRatio(s.cloud[k], result);
      const bool accept = kind == "fitness" ? fitness < limit : inliers >= limit;
      const Eigen::Matrix4f & now = accept ? result : seed;
      const double et = (now.block<3, 1>(0, 3) - s.gt[k].block<3, 1>(0, 3)).norm();
      std::printf("%7.2f %s fitness %7.2f inliers %.2f error %6.2f m %5.1f deg\n", s.stamp[k] - start,
        accept ? "accept" : "reject", fitness, inliers, et, yawErrorDeg(now, s.gt[k]));
      if (accept) {before = last; before_t = last_t; last = result; last_t = s.stamp[k];}
    }
    return 0;
  }
  if (mode == "search" && argc == 15) {
    const double cx = std::atof(argv[4]), cy = std::atof(argv[5]), cz = std::atof(argv[6]);
    const double cyaw = std::atof(argv[7]) * M_PI / 180.0;
    const double radius = std::atof(argv[8]), step = std::atof(argv[9]);
    const double yaw_range = std::atof(argv[10]), yaw_step = std::atof(argv[11]);
    const int first = std::atoi(argv[12]), last = std::atoi(argv[13]);
    (void)argv[14];
    for (int k = first; k <= last && k < static_cast<int>(s.stamp.size()); ++k) {
      struct Candidate {double fitness, et, ey, inliers; Eigen::Vector3f seed_offset;};
      std::vector<Candidate> candidates;
      const Eigen::Matrix3f gt_rot = s.gt[k].block<3, 3>(0, 0);
      const double gt_yaw = yawOf(s.gt[k]);
      // Roll and pitch from the ground truth: a level IMU would give them.
      const Eigen::Matrix3f tilt = Eigen::AngleAxisf(-static_cast<float>(gt_yaw), Eigen::Vector3f::UnitZ()) * gt_rot;
      for (double dx = -radius; dx <= radius + 1e-6; dx += step) {
        for (double dy = -radius; dy <= radius + 1e-6; dy += step) {
          if (std::hypot(dx, dy) > radius + 1e-6) {continue;}
          for (double dyaw = -yaw_range; dyaw <= yaw_range + 1e-6; dyaw += yaw_step) {
            Eigen::Matrix4f seed = Eigen::Matrix4f::Identity();
            seed.block<3, 3>(0, 0) =
              Eigen::AngleAxisf(static_cast<float>(cyaw + dyaw * M_PI / 180.0), Eigen::Vector3f::UnitZ()).toRotationMatrix() * tilt;
            seed(0, 3) = cx + dx; seed(1, 3) = cy + dy; seed(2, 3) = cz;
            Eigen::Matrix4f result;
            const double fitness = loc.align(s.cloud[k], seed, result);
            candidates.push_back({fitness,
              (result.block<3, 1>(0, 3) - s.gt[k].block<3, 1>(0, 3)).norm(),
              yawErrorDeg(result, s.gt[k]),
              loc.inlierRatio(s.cloud[k], result),
              Eigen::Vector3f(dx, dy, dyaw)});
          }
        }
      }
      std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {return a.fitness < b.fitness;});
      std::printf("%7.2f %zu seeds, center %.2f m %.1f deg from GT:", s.stamp[k] - start, candidates.size(),
        std::hypot(cx - s.gt[k](0, 3), cy - s.gt[k](1, 3)), std::abs(std::remainder(cyaw - gt_yaw, 2 * M_PI)) * 180 / M_PI);
      for (std::size_t i = 0; i < std::min<std::size_t>(5, candidates.size()); ++i) {
        std::printf(" [f %.2f in %.2f err %.2f m %.1f deg]", candidates[i].fitness, candidates[i].inliers, candidates[i].et, candidates[i].ey);
      }
      // The best candidate that ends within 0.5 m and 3 deg of the ground truth.
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].et < 0.5 && candidates[i].ey < 3.0) {
          std::printf(" | first good at rank %zu f %.2f in %.2f", i + 1, candidates[i].fitness, candidates[i].inliers);
          break;
        }
      }
      std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {return a.inliers > b.inliers;});
      std::printf("\n        by inliers:");
      for (std::size_t i = 0; i < std::min<std::size_t>(4, candidates.size()); ++i) {
        std::printf(" [in %.2f f %.2f err %.2f m %.1f deg]", candidates[i].inliers, candidates[i].fitness, candidates[i].et, candidates[i].ey);
      }
      std::printf("\n");
      std::fflush(stdout);
    }
    return 0;
  }
  std::cerr << "bad arguments\n";
  return 1;
}
