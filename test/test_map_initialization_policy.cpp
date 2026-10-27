#include "lidar_localization/map_initialization_policy.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <string>

#include <pcl/conversions.h>

namespace ll = lidar_localization;

pcl::PointXYZI make_point(float x, float y, float z = 0.0f)
{
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1.0f;
  return point;
}

void test_map_file_format_detection_matches_existing_path_matching()
{
  assert(ll::detectMapFileFormat("/maps/site.pcd") == ll::MapFileFormat::kPcd);
  assert(ll::detectMapFileFormat("/maps/site.ply") == ll::MapFileFormat::kPly);
  assert(ll::detectMapFileFormat("/maps/site.pcd.backup") == ll::MapFileFormat::kPcd);
  assert(ll::detectMapFileFormat("/maps/site.bin") == ll::MapFileFormat::kUnsupported);
}

void test_compute_map_bounds()
{
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  assert(!ll::computeMapBounds(empty_cloud).valid);

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(make_point(2.0f, -1.0f, 3.0f));
  cloud.push_back(make_point(-4.0f, 5.0f, -2.0f));

  const auto bounds = ll::computeMapBounds(cloud);
  assert(bounds.valid);
  assert(bounds.min_point.x == -4.0f);
  assert(bounds.max_point.y == 5.0f);
  assert(bounds.min_point.z == -2.0f);
  assert(bounds.max_point.z == 3.0f);
}

void test_make_initial_map_message_sets_frame_and_preserves_points_without_downsample()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(make_point(0.0f, 0.0f));
  cloud.push_back(make_point(1.0f, 0.0f));

  pcl::PCLPointCloud2 raw_cloud;
  pcl::toPCLPointCloud2(cloud, raw_cloud);

  const auto msg = ll::makeInitialMapMessage(raw_cloud, "map", false, 0.5);
  assert(msg.header.frame_id == "map");
  assert(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height) == 2);
}

void test_make_initial_map_message_can_downsample()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(make_point(0.0f, 0.0f));
  cloud.push_back(make_point(0.1f, 0.1f));
  cloud.push_back(make_point(2.0f, 0.0f));

  pcl::PCLPointCloud2 raw_cloud;
  pcl::toPCLPointCloud2(cloud, raw_cloud);

  const auto msg = ll::makeInitialMapMessage(raw_cloud, "map", true, 1.0);
  assert(msg.header.frame_id == "map");
  assert(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height) > 0);
  assert(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height) <= 3);
}

void test_initial_map_target_setup_plan()
{
  auto plan = ll::planInitialMapTargetSetup(false, "NDT");
  assert(!plan.use_local_map_crop);
  assert(!plan.create_ndt_initializer);
  assert(plan.set_full_map_as_registration_target);
  assert(plan.builds_full_map_ndt_grid);

  plan = ll::planInitialMapTargetSetup(false, "NDT_OMP");
  assert(plan.set_full_map_as_registration_target);
  assert(plan.builds_full_map_ndt_grid);

  // The GICP backends crop, but their NDT initializer takes the full map.
  plan = ll::planInitialMapTargetSetup(false, "GICP");
  assert(plan.use_local_map_crop);
  assert(plan.create_ndt_initializer);
  assert(!plan.set_full_map_as_registration_target);
  assert(plan.builds_full_map_ndt_grid);

  plan = ll::planInitialMapTargetSetup(true, "NDT");
  assert(plan.use_local_map_crop);
  assert(!plan.create_ndt_initializer);
  assert(!plan.set_full_map_as_registration_target);
  assert(!plan.builds_full_map_ndt_grid);
}

void test_float32_coordinate_spacing()
{
  assert(ll::float32CoordinateSpacingM(1.0) == std::ldexp(1.0, -23));
  // MGRS grid coordinates stay below 100 km: 7.8 mm spacing.
  assert(ll::float32CoordinateSpacingM(99950.0) == std::ldexp(1.0, -7));
  assert(ll::float32CoordinateSpacingM(-99950.0) == std::ldexp(1.0, -7));
  // A UTM northing: 0.25 m spacing.
  assert(ll::float32CoordinateSpacingM(3.9e6) == 0.25);
}

void test_large_map_coordinate_warning()
{
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  assert(!ll::largeMapCoordinateWarning(ll::computeMapBounds(empty_cloud)));

  pcl::PointCloud<pcl::PointXYZI> mgrs_cloud;
  mgrs_cloud.push_back(make_point(99900.0f, 99900.0f, 40.0f));
  mgrs_cloud.push_back(make_point(99999.0f, 99999.0f, 60.0f));
  assert(!ll::largeMapCoordinateWarning(ll::computeMapBounds(mgrs_cloud)));

  pcl::PointCloud<pcl::PointXYZI> utm_cloud;
  utm_cloud.push_back(make_point(350000.0f, 3900000.0f));
  utm_cloud.push_back(make_point(350100.0f, 3900100.0f));
  const auto warning = ll::largeMapCoordinateWarning(ll::computeMapBounds(utm_cloud));
  assert(warning);
  assert(warning->find("3900100 m") != std::string::npos);
  assert(warning->find("0.25 m apart") != std::string::npos);

  // Far on the negative side counts too.
  pcl::PointCloud<pcl::PointXYZI> negative_cloud;
  negative_cloud.push_back(make_point(-200000.0f, 0.0f));
  assert(ll::largeMapCoordinateWarning(ll::computeMapBounds(negative_cloud)));
}

void test_ndt_voxel_grid_overflow_error()
{
  ll::MapBounds3d bounds;
  assert(!ll::ndtVoxelGridOverflowError(bounds, 1.0));

  // A 300 m local crop around the Istanbul route fits easily.
  bounds.valid = true;
  bounds.min_point = make_point(66300.0f, 43470.0f, -10.0f);
  bounds.max_point = make_point(66600.0f, 43770.0f, 90.0f);
  assert(ll::ndtVoxelGridCellCount(bounds, 1.0) == 301.0 * 301.0 * 101.0);
  assert(!ll::ndtVoxelGridOverflowError(bounds, 1.0));

  // The full Istanbul map overflows at 1 m and at 2 m, and fits from 3 m.
  bounds.min_point = make_point(63368.21f, 38581.27f, -66.12f);
  bounds.max_point = make_point(78591.57f, 51305.61f, 217.69f);
  const auto error = ll::ndtVoxelGridOverflowError(bounds, 1.0);
  assert(error);
  assert(error->find("15223 x 12724 x 284 m") != std::string::npos);
  assert(error->find("ndt_resolution 1.00 m needs 5.5e+10 cells") != std::string::npos);
  assert(error->find("enable_local_map_crop") != std::string::npos);
  assert(ll::ndtVoxelGridOverflowError(bounds, 2.0));
  assert(!ll::ndtVoxelGridOverflowError(bounds, 3.0));
  assert(!ll::ndtVoxelGridOverflowError(bounds, 0.0));
}

int main()
{
  test_map_file_format_detection_matches_existing_path_matching();
  test_compute_map_bounds();
  test_make_initial_map_message_sets_frame_and_preserves_points_without_downsample();
  test_make_initial_map_message_can_downsample();
  test_initial_map_target_setup_plan();
  test_float32_coordinate_spacing();
  test_large_map_coordinate_warning();
  test_ndt_voxel_grid_overflow_error();
  return 0;
}
