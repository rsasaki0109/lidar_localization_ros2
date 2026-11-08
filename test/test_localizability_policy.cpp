#include "lidar_localization/localizability_policy.hpp"

#include <cassert>
#include <cmath>

namespace ll = lidar_localization;

void test_balanced_cloud_has_a_high_ratio()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      cloud.push_back(point);
    }
  }
  const auto metric = ll::evaluateHorizontalLocalizability(cloud);
  assert(metric.valid);
  assert(metric.eigenvalue_ratio > 0.99);
}

void test_corridor_cloud_has_a_low_ratio()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int x = -50; x <= 50; ++x) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(x);
    point.y = 0.01f * static_cast<float>(x % 3);
    cloud.push_back(point);
  }
  const auto metric = ll::evaluateHorizontalLocalizability(cloud);
  assert(metric.valid);
  assert(metric.eigenvalue_ratio < 0.001);
}

int main()
{
  test_balanced_cloud_has_a_high_ratio();
  test_corridor_cloud_has_a_low_ratio();
  return 0;
}
