#ifndef LIDAR_LOCALIZATION_MAP_INITIALIZATION_POLICY_HPP_
#define LIDAR_LOCALIZATION_MAP_INITIALIZATION_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
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
  // An NDT voxel grid (the target or the NDT initializer) is built over the full map.
  bool builds_full_map_ndt_grid{true};
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

// Map points and poses are float32. From 2^17 m (131 km) on, neighbouring
// float32 values are 1.6 cm or more apart; MGRS grid coordinates (< 100 km)
// stay below that, UTM northings (millions of m) do not.
constexpr double kLargeMapCoordinateWarningM = 131072.0;

// Distance between neighbouring float32 values at this coordinate.
inline double float32CoordinateSpacingM(double coordinate_m)
{
  const float magnitude = static_cast<float>(std::abs(coordinate_m));
  return static_cast<double>(
    std::nextafter(magnitude, std::numeric_limits<float>::infinity()) - magnitude);
}

inline std::optional<std::string> largeMapCoordinateWarning(const MapBounds3d & bounds)
{
  if (!bounds.valid) {
    return std::nullopt;
  }
  const double max_abs_coordinate = std::max({
    std::abs(static_cast<double>(bounds.min_point.x)),
    std::abs(static_cast<double>(bounds.max_point.x)),
    std::abs(static_cast<double>(bounds.min_point.y)),
    std::abs(static_cast<double>(bounds.max_point.y)),
    std::abs(static_cast<double>(bounds.min_point.z)),
    std::abs(static_cast<double>(bounds.max_point.z))});
  if (max_abs_coordinate < kLargeMapCoordinateWarningM) {
    return std::nullopt;
  }
  std::ostringstream message;
  message << std::fixed << std::setprecision(0) << "Map coordinates reach "
          << max_abs_coordinate << " m from the origin, where float32 map points and poses "
          << "are " << std::setprecision(2) << float32CoordinateSpacingM(max_abs_coordinate)
          << " m apart; localization loses accuracy. Shift the map near the origin "
          << "(docs/troubleshooting.md, maps far from the origin).";
  return message.str();
}

// NDT indexes its voxel grid with int32. Past that many cells, pclomp (and PCL)
// leave the target empty and NDT returns its initial guess: it does not
// register. A city-scale map gets there at 1 m (Istanbul: 15 x 13 km, 5.5e10).
inline double ndtVoxelGridCellCount(const MapBounds3d & bounds, double resolution_m)
{
  // The float arithmetic of pclomp::VoxelGridCovariance::applyFilter.
  const float inverse_resolution = 1.0f / static_cast<float>(resolution_m);
  const auto cells = [inverse_resolution](float min_value, float max_value) {
      return static_cast<double>(
        static_cast<std::int64_t>((max_value - min_value) * inverse_resolution) + 1);
    };
  return cells(bounds.min_point.x, bounds.max_point.x) *
         cells(bounds.min_point.y, bounds.max_point.y) *
         cells(bounds.min_point.z, bounds.max_point.z);
}

inline std::optional<std::string> ndtVoxelGridOverflowError(
  const MapBounds3d & bounds, double resolution_m)
{
  if (!bounds.valid || !(resolution_m > 0.0)) {
    return std::nullopt;
  }
  const double cells = ndtVoxelGridCellCount(bounds, resolution_m);
  if (cells <= static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    return std::nullopt;
  }
  std::ostringstream message;
  message << std::fixed << std::setprecision(0) << "The map spans "
          << bounds.max_point.x - bounds.min_point.x << " x "
          << bounds.max_point.y - bounds.min_point.y << " x "
          << bounds.max_point.z - bounds.min_point.z << " m: an NDT voxel grid over it at "
          << std::setprecision(2) << "ndt_resolution " << resolution_m << " m needs "
          << std::scientific << std::setprecision(1) << cells
          << " cells, more than int32 indexes, so NDT gets an empty target and does not "
          << "register. Set enable_local_map_crop (docs/troubleshooting.md, NDT does not "
          << "correct the pose on a large map).";
  return message.str();
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
  plan.builds_full_map_ndt_grid = plan.create_ndt_initializer ||
    (plan.set_full_map_as_registration_target && usesNdtVoxelGrid(backend));
  return plan;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MAP_INITIALIZATION_POLICY_HPP_
