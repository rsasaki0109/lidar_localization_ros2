#pragma once
// Experiment only: bounded pair state, protected by the component state lock.
#include <cstdint>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace lidar_localization {
class ExperimentalScanMotion {
 public:
  using Message = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using Cloud = pcl::PointCloud<pcl::PointNormal>;
  void reset();
  void anchor(const Message& msg, const Eigen::Matrix4f& world, std::uint64_t generation);
  bool predict(const Message& msg, std::uint64_t generation, Eigen::Matrix4f& world);
 private:
  static Cloud::Ptr prepare(const sensor_msgs::msg::PointCloud2& msg);
  Message previous_;
  Cloud::Ptr normals_;
  Eigen::Matrix4f world_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f relative_{Eigen::Matrix4f::Identity()};
  std::uint64_t generation_{0};
};
}
