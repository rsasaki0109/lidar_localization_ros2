#include "lidar_localization/map_initialization_policy.hpp"

#include <cassert>
#include <cstddef>

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

  plan = ll::planInitialMapTargetSetup(false, "GICP");
  assert(plan.use_local_map_crop);
  assert(plan.create_ndt_initializer);
  assert(!plan.set_full_map_as_registration_target);

  plan = ll::planInitialMapTargetSetup(true, "NDT");
  assert(plan.use_local_map_crop);
  assert(!plan.create_ndt_initializer);
  assert(!plan.set_full_map_as_registration_target);
}

int main()
{
  test_map_file_format_detection_matches_existing_path_matching();
  test_compute_map_bounds();
  test_make_initial_map_message_sets_frame_and_preserves_points_without_downsample();
  test_make_initial_map_message_can_downsample();
  test_initial_map_target_setup_plan();
  return 0;
}
