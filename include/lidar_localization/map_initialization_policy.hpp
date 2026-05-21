#ifndef LIDAR_LOCALIZATION_MAP_INITIALIZATION_POLICY_HPP_
#define LIDAR_LOCALIZATION_MAP_INITIALIZATION_POLICY_HPP_

#include <memory>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "lidar_localization/registration_backend_policy.hpp"

namespace lidar_localization
{

enum class MapFileFormat
{
  kUnsupported = 0,
  kPcd,
  kPly,
};

struct MapBounds3d
{
  bool valid{false};
  pcl::PointXYZI min_point{};
  pcl::PointXYZI max_point{};
};

struct InitialMapTargetSetupPlan
{
  bool use_local_map_crop{false};
  bool create_ndt_initializer{false};
  bool set_full_map_as_registration_target{true};
};

inline MapFileFormat detectMapFileFormat(const std::string & map_path)
{
  if (map_path.find(".pcd") != std::string::npos) {
    return MapFileFormat::kPcd;
  }
  if (map_path.find(".ply") != std::string::npos) {
    return MapFileFormat::kPly;
  }
  return MapFileFormat::kUnsupported;
}

inline MapBounds3d computeMapBounds(const pcl::PointCloud<pcl::PointXYZI> & map_cloud)
{
  MapBounds3d bounds;
  if (map_cloud.empty()) {
    return bounds;
  }
  pcl::getMinMax3D(map_cloud, bounds.min_point, bounds.max_point);
  bounds.valid = true;
  return bounds;
}

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
    voxel_filter.setInputCloud(std::make_shared<pcl::PCLPointCloud2>(raw_map_cloud));
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(filtered_map_cloud);
    pcl_conversions::moveFromPCL(filtered_map_cloud, map_msg);
  } else {
    pcl_conversions::moveFromPCL(raw_map_cloud, map_msg);
  }
  map_msg.header.frame_id = frame_id;
  return map_msg;
}

inline InitialMapTargetSetupPlan planInitialMapTargetSetup(
  bool enable_local_map_crop,
  const std::string & registration_method)
{
  const RegistrationBackend backend = parseRegistrationBackend(registration_method);
  InitialMapTargetSetupPlan plan;
  plan.use_local_map_crop = enable_local_map_crop || usesFilteredTarget(backend);
  plan.create_ndt_initializer = plan.use_local_map_crop && supportsNdtInitializer(backend);
  plan.set_full_map_as_registration_target = !plan.use_local_map_crop;
  return plan;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MAP_INITIALIZATION_POLICY_HPP_
