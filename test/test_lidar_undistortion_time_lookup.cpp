#include "lidar_localization/lidar_undistortion.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

int main()
{
  float max_error = 0.0f;
  for (const float rate : {-1.0f, 0.5f, 1.0f}) {
    for (const float start_yaw : {0.0f, 0.3f}) {
      for (const double offset : {0.0, 0.0025}) {
        LidarUndistortion deskew;
        for (int i = 0; i < 50; ++i) {
          const float yaw = start_yaw + rate * static_cast<float>(i * 0.005);
          deskew.getImu(Eigen::Vector3f(0, 0, rate), Eigen::Vector3f::Zero(),
            Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())),
            100.0 + i * 0.005);
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
        deskew.adjustDistortion(cloud, 100.01 + offset);
        for (int i = 0; i < 64; ++i) {
          const auto & point = (*cloud)[i];
          const Eigen::Vector3f expected =
            Eigen::AngleAxisf(rate * 0.1f * i / 63, Eigen::Vector3f::UnitZ()) *
            original[i].getVector3fMap();
          const float error = (point.getVector3fMap() - expected).norm();
          if (!std::isfinite(error) || point.intensity != original[i].intensity) {return 1;}
          max_error = std::max(max_error, error);
        }
      }
    }
  }
  std::cout << "12 rotating-scan cases, maximum point error: " << max_error << '\n';
  return max_error <= 1e-5f ? 0 : 1;
}
