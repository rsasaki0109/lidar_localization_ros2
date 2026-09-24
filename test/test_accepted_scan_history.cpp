#include "lidar_localization/accepted_scan_history.hpp"
#include <cassert>

int main()
{
  using History = lidar_localization::AcceptedScanHistory;
  History history;
  History::Cloud::Ptr cloud(new History::Cloud);
  pcl::PointXYZI point;
  point.x = 1; point.y = 2; point.z = 3; point.intensity = 4;
  cloud->push_back(point);
  const Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
  assert(history.combine(cloud, identity) == cloud);
  history.beginScan(1.0);
  history.accept(1.0, cloud, identity);
  history.accept(1.0, cloud, identity);
  assert(history.size() == 1);
  history.beginScan(1.1);
  Eigen::Matrix4f seed = identity;
  seed(0, 3) = 0.2f;
  const auto combined = history.combine(cloud, seed);
  assert(combined->size() == 2 && cloud->size() == 1);
  assert(std::abs(combined->at(1).x - .8f) < 1e-6);
  assert(combined->at(1).intensity == 4);
  // Rejection adds nothing; accepted clouds are the original single scan.
  history.beginScan(1.2);
  assert(history.size() == 1);
  history.accept(1.2, cloud, seed);
  history.beginScan(1.3);
  history.accept(1.3, cloud, seed);
  history.beginScan(1.4);
  history.accept(1.4, cloud, seed);
  assert(history.size() == 3);
  assert(history.combine(cloud, seed)->size() == 4);
  history.beginScan(1.8);
  assert(history.size() == 0);
  history.accept(1.8, cloud, seed);
  history.beginScan(1.7);  // Clock rollback.
  assert(history.size() == 0);
  history.accept(1.7, cloud, seed);
  history.beginScan(1.7);  // Duplicate scan must not use its own history.
  assert(history.size() == 0);
  history.accept(1.7, cloud, seed);
  history.clear();  // Map/initialpose/cleanup reset.
  assert(history.combine(cloud, seed) == cloud);
  history.accept(1.7, cloud, seed);  // A pre-reset in-flight accept is ignored.
  assert(history.size() == 0);
  history.beginScan(2.0);
  Eigen::Matrix4f bad = identity;
  bad(0, 0) = std::numeric_limits<float>::quiet_NaN();
  history.accept(2.0, cloud, bad);
  assert(history.size() == 0);
  history.accept(2.0, cloud, identity);
  history.beginScan(std::numeric_limits<double>::quiet_NaN());
  assert(history.size() == 0);
  // Repeated successful history retries cannot extend the primary anchor lifetime.
  history.beginScan(3.0);
  history.accept(3.0, cloud, identity);
  for (int i = 1; i <= 4; ++i) {
    const double stamp = 3.0 + 0.1 * i;
    history.beginScan(stamp);
    history.accept(stamp, cloud, seed, true);
    assert(history.size() == (i <= 3 ? 1u : 0u));
  }
  assert(history.combine(cloud, identity) == cloud);
  // A later primary match can establish fresh support again.
  history.beginScan(3.5);
  history.accept(3.5, cloud, identity, false);
  assert(history.size() == 1);
  return 0;
}
