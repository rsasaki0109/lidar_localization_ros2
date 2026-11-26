#ifndef LIDAR_LOCALIZATION_INITIAL_MAP_MESSAGE_HPP_
#define LIDAR_LOCALIZATION_INITIAL_MAP_MESSAGE_HPP_

// The map as the node publishes it once loaded (the ROS side of
// map_initialization_policy.hpp, which stays free of ROS for the core).

#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace lidar_localization
{

inline sensor_msgs::msg::PointCloud2 makeInitialMapMessage(
  pcl::PCLPointCloud2 & raw_map_cloud,
  const std::string & frame_id,
  bool downsample,
  double voxel_leaf_size)
{
  sensor_msgs::msg::PointCloud2 map_msg;
  if (downsample) {
    pcl::PCLPointCloud2 filtered_map_cloud;
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    // The filter only reads the map: hand it over without a copy.
    voxel_filter.setInputCloud(
      pcl::PCLPointCloud2::ConstPtr(&raw_map_cloud, [](const pcl::PCLPointCloud2 *) {}));
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(filtered_map_cloud);
    pcl_conversions::moveFromPCL(filtered_map_cloud, map_msg);
  } else {
    pcl_conversions::moveFromPCL(raw_map_cloud, map_msg);
  }
  map_msg.header.frame_id = frame_id;
  return map_msg;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_INITIAL_MAP_MESSAGE_HPP_
