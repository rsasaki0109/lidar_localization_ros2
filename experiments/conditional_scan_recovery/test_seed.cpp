#include "accepted_scan_seed.hpp"
#include <cassert>
#include <limits>
#include <random>
using Seed = conditional_scan_recovery::AcceptedScanSeed;
int main()
{
  auto reference = Seed::Cloud::Ptr(new Seed::Cloud);
  reference->header.frame_id = "livox_frame";
  std::mt19937 rng(31);
  std::uniform_real_distribution<float> random(-1.f, 1.f);
  for (int i = 0; i < 300; ++i) {
    pcl::PointXYZI p{}; p.x = random(rng); p.y = random(rng); p.z = random(rng);
    reference->push_back(p);
  }
  auto current = Seed::Cloud::Ptr(new Seed::Cloud(*reference));
  for (auto & p : *current) {p.x -= 0.05f;}
  Eigen::Matrix4f world = Eigen::Matrix4f::Identity(); world(1, 3) = 2.f;
  Seed seed(1.0);
  assert(!seed.estimate(current, 1.1, 7).valid);
  assert(seed.observeAccepted(reference, world, 1.0, 7));
  assert(!seed.estimate(current, 1.0, 7).valid);
  assert(!seed.estimate(current, .9, 7).valid);
  assert(!seed.estimate(current, 2.01, 7).valid);
  assert(!seed.estimate(current, std::numeric_limits<double>::quiet_NaN(), 7).valid);
  assert(!seed.estimate(current, 1.1, 6).valid);
  auto mismatch = Seed::Cloud::Ptr(new Seed::Cloud(*current));
  mismatch->header.frame_id = "different";
  assert(!seed.estimate(mismatch, 1.1, 7).valid);
  auto bad = Seed::Cloud::Ptr(new Seed::Cloud(*current));
  (*bad)[0].x = std::numeric_limits<float>::quiet_NaN();
  assert(!seed.estimate(bad, 1.1, 7).valid);
  const auto first = seed.estimate(current, 1.1, 7);
  assert(first.valid);
  Eigen::Matrix4f expected = world; expected(0, 3) = .05f;
  assert((first.seed - expected).cwiseAbs().maxCoeff() < .003f);
  // Estimating an unaccepted scan must not move the accepted anchor or extend
  // its time horizon. A second current scan is still relative to reference.
  auto later = Seed::Cloud::Ptr(new Seed::Cloud(*reference));
  for (auto & p : *later) {p.x -= .1f;}
  const auto second = seed.estimate(later, 1.2, 7);
  assert(second.valid && std::abs(second.seed(0, 3) - .1f) < .003f);
  assert(!seed.estimate(later, 2.01, 7).valid);
  seed.reset();
  assert(!seed.estimate(current, 1.1, 7).valid);
  assert(seed.observeAccepted(reference, world, 4., 8));
  assert(!seed.estimate(current, 4.1, 7).valid);
  assert(seed.estimate(current, 4.1, 8).valid);
  assert(!seed.observeAccepted({}, world, 5., 8));
  assert(!seed.estimate(current, 5.1, 8).valid);
  world.block<3, 3>(0, 0) = Eigen::AngleAxisf(-.3f, Eigen::Vector3f::UnitY()).toRotationMatrix();
  assert(seed.observeAccepted(reference, world, 6., 9));
  const Eigen::Matrix3f relative_rotation =
    Eigen::AngleAxisf(.45f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  auto rotated = Seed::Cloud::Ptr(new Seed::Cloud(*reference));
  for (auto & p : *rotated) {
    const Eigen::Vector3f v = relative_rotation.transpose() *
      (p.getVector3fMap() - Eigen::Vector3f(.04f, -.02f, .01f));
    p.x = v.x(); p.y = v.y(); p.z = v.z();
  }
  Eigen::Matrix3f hint = world.block<3, 3>(0, 0) * relative_rotation;
  Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
  delta.block<3, 3>(0, 0) = relative_rotation;
  delta.block<3, 1>(0, 3) = Eigen::Vector3f(.04f, -.02f, .01f);
  const auto hinted = seed.estimate(rotated, 6.1, 9, &hint);
  assert(hinted.valid && (hinted.seed - world * delta).cwiseAbs().maxCoeff() < .003f);
  hint(0, 0) = std::numeric_limits<float>::quiet_NaN();
  assert(!seed.estimate(rotated, 6.1, 9, &hint).valid);
  hint = 2.f * Eigen::Matrix3f::Identity();
  assert(!seed.estimate(rotated, 6.1, 9, &hint).valid);
  hint = Eigen::Matrix3f::Identity(); hint(0, 0) = -1.f;
  assert(!seed.estimate(rotated, 6.1, 9, &hint).valid);

}
