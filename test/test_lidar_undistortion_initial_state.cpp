#include "lidar_localization/lidar_undistortion.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <new>

int main()
{
  // A new instance must not depend on the previous contents of its storage.
  for (const unsigned char pattern : {0xff, 0x00, 0x7f}) {
    alignas(LidarUndistortion) std::array<unsigned char, sizeof(LidarUndistortion)> storage;
    volatile unsigned char * bytes = storage.data();
    for (std::size_t i = 0; i < storage.size(); ++i) {bytes[i] = pattern;}
    auto * deskew = new (storage.data()) LidarUndistortion();
    for (int i = 0; i < 25; ++i) {
      deskew->getImu(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
        Eigen::Quaternionf::Identity(), 100.0 + i * 0.005);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < 64; ++i) {
      pcl::PointXYZI point;
      const float angle = -6.2f * i / 63;
      point.x = std::cos(angle); point.y = std::sin(angle); point.z = 1.0f;
      point.intensity = static_cast<float>(i);
      cloud->push_back(point);
    }
    const auto original = *cloud;
    deskew->adjustDistortion(cloud, 100.01);
    deskew->~LidarUndistortion();
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      const auto & a = (*cloud)[i]; const auto & b = original[i];
      if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        a.x != b.x || a.y != b.y || a.z != b.z || a.intensity != b.intensity)
      {
        std::cerr << "Stationary deskew depends on initial storage: pattern="
                  << static_cast<int>(pattern) << " point=" << i << '\n';
        return 1;
      }
    }
  }
  return 0;
}
