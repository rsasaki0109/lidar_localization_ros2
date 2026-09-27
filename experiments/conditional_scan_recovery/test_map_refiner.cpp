#include "map_refiner.hpp"
#include <pcl/common/transforms.h>
#include <cassert>
#include <random>
#include <iostream>
using Refiner = conditional_scan_recovery::MapRefiner;
int main()
{
  auto scan = Refiner::Cloud::Ptr(new Refiner::Cloud);
  std::mt19937 rng(71);
  std::uniform_real_distribution<float> random(-2.f, 2.f);
  for (int i = 0; i < 1500; ++i) {
    pcl::PointXYZI p{}; p.x = random(rng); p.y = random(rng); p.z = random(rng);
    scan->push_back(p);
  }
  Eigen::Matrix4f truth = Eigen::Matrix4f::Identity();
  truth.block<3, 3>(0, 0) = Eigen::AngleAxisf(.1f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  truth.block<3, 1>(0, 3) = Eigen::Vector3f(.4f, -.2f, .1f);
  auto map = Refiner::Cloud::Ptr(new Refiner::Cloud);
  pcl::transformPointCloud(*scan, *map, truth);
  Refiner refiner(map, .2f);
  assert(refiner.inputMap() == map);
  Eigen::Matrix4f seed = truth; seed(0, 3) += .05f;
  const auto result = refiner.align(scan, seed);
  std::cout << "known transform max coefficient error " << (result.pose-truth).cwiseAbs().maxCoeff() << '\n';
  assert(result.target_ready && result.converged);
  assert((result.pose-truth).cwiseAbs().maxCoeff() < .02f);
  const auto again = refiner.align(scan, seed);
  assert(again.converged && again.pose.isApprox(result.pose, 1e-6f));
  assert(!refiner.align({}, seed).converged);
  auto invalid = Refiner::Cloud::Ptr(new Refiner::Cloud(*scan));
  (*invalid)[0].x = std::numeric_limits<float>::quiet_NaN();
  assert(!refiner.align(invalid, seed).converged);
  seed(0, 0) = std::numeric_limits<float>::quiet_NaN();
  assert(!refiner.align(scan, seed).converged);
  Refiner absent({}, .2f);
  assert(!absent.align(scan, truth).target_ready);
  Refiner bad_map(invalid, .2f);
  assert(!bad_map.align(scan, truth).target_ready);
  Refiner bad_leaf(map, 0.f);
  assert(!bad_leaf.align(scan, truth).target_ready);
  // A replacement map requires a distinct refiner/cache. Holding the old
  // refiner during reset cannot mutate the new target's registration state.
  Eigen::Matrix4f new_truth = truth; new_truth(1, 3) += 1.f;
  auto new_map = Refiner::Cloud::Ptr(new Refiner::Cloud);
  pcl::transformPointCloud(*scan, *new_map, new_truth);
  Refiner replacement(new_map, .2f);
  assert(replacement.inputMap() != refiner.inputMap());
  auto updated = replacement.align(scan, new_truth);
  assert(updated.converged && (updated.pose-new_truth).cwiseAbs().maxCoeff() < .02f);
  const auto old_result = refiner.align(scan, truth);
  assert(old_result.converged && (old_result.pose-truth).cwiseAbs().maxCoeff() < .02f);
}
