// Align raw and GT-deskewed Koide clouds with the online profile's NDT settings.
// usage: batch_align MAP.ply SCANS_DIR > results.csv
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

static Cloud::Ptr prepare(const std::string & path) {
  Cloud::Ptr raw(new Cloud), ranged(new Cloud), out(new Cloud);
  pcl::io::loadPCDFile(path, *raw);
  for (const auto & p : raw->points) {
    const float r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (r >= 1.0f && r <= 60.0f) {ranged->push_back(p);}
  }
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(0.5f, 0.5f, 0.5f);
  voxel.setInputCloud(ranged);
  voxel.filter(*out);
  return out;
}

int main(int argc, char ** argv) {
  Cloud::Ptr map(new Cloud);
  pcl::io::loadPLYFile(argv[1], *map);
  const std::string dir = argv[2];
  pclomp::NormalDistributionsTransform<Point, Point> ndt;
  ndt.setResolution(1.0);
  ndt.setStepSize(0.1);
  ndt.setTransformationEpsilon(0.01);
  ndt.setMaximumIterations(35);
  ndt.setNumThreads(4);
  Eigen::Vector2f center(1e9f, 1e9f);
  std::ifstream index(dir + "/index.txt");
  std::string line;
  std::cout << "name,window,t,gt_rate_deg_s,cloud,seed,iterations,trans_err_m,yaw_err_deg,fitness,ms\n";
  while (std::getline(index, line)) {
    std::istringstream in(line);
    std::string name, window;
    double t, rate, x, y, z, qx, qy, qz, qw;
    in >> name >> window >> t >> rate >> x >> y >> z >> qx >> qy >> qz >> qw;
    if (std::hypot(x - center.x(), y - center.y()) >= 10.0f) {
      Cloud::Ptr crop(new Cloud);
      for (const auto & p : map->points) {
        if (std::hypot(p.x - x, p.y - y) <= 75.0) {crop->push_back(p);}
      }
      ndt.setInputTarget(crop);
      center = Eigen::Vector2f(x, y);
    }
    Eigen::Matrix4f gt = Eigen::Matrix4f::Identity();
    gt.block<3, 3>(0, 0) = Eigen::Quaternionf(qw, qx, qy, qz).normalized().toRotationMatrix();
    gt.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
    Eigen::Matrix4f perturbed = gt;
    perturbed(0, 3) += 0.15f;
    perturbed.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(0.8f * M_PI / 180.0f, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
      gt.block<3, 3>(0, 0);
    for (const char * kind : {"raw", "desk", "gyro"}) {
      Cloud::Ptr source = prepare(dir + "/" + name + "_" + kind + ".pcd");
      ndt.setInputSource(source);
      for (const auto & [seed_name, seed] : {std::pair<const char *, Eigen::Matrix4f>{"gt", gt},
                                              std::pair<const char *, Eigen::Matrix4f>{"off", perturbed}}) {
        Cloud out;
        const auto start = std::chrono::steady_clock::now();
        ndt.align(out, seed);
        const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        const Eigen::Matrix4f d = gt.inverse() * ndt.getFinalTransformation();
        const double yaw = std::atan2(d(1, 0), d(0, 0)) * 180.0 / M_PI;
        std::cout << name << "," << window << "," << t << "," << rate << "," << kind << "," << seed_name
                  << "," << ndt.getFinalNumIteration() << "," << d.block<3, 1>(0, 3).norm() << ","
                  << std::abs(yaw) << "," << ndt.getFitnessScore() << "," << ms << "\n";
      }
    }
  }
  return 0;
}
