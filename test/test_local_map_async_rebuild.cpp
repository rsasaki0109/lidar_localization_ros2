#include <lidar_localization/lidar_localization_component.hpp>
#include <pcl/filters/voxel_grid.h>
#include <cassert>
#include <cmath>
#include <string>

namespace ll = lidar_localization;

namespace
{

constexpr float kMapHalfSize = 60.0f;
constexpr double kRadius = 30.0;

// A 120 x 120 m ground grid with a 3 m wall every 10 m along x.
pcl::PointCloud<pcl::PointXYZI>::Ptr makeMap()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointXYZI p;
  p.intensity = 1.0f;
  for (float x = -kMapHalfSize; x <= kMapHalfSize; x += 0.5f) {
    for (float y = -kMapHalfSize; y <= kMapHalfSize; y += 0.5f) {
      p.x = x;
      p.y = y;
      p.z = 0.0f;
      map->push_back(p);
      if (std::fmod(x + kMapHalfSize, 10.0f) == 0.0f) {
        for (p.z = 0.5f; p.z <= 3.0f; p.z += 0.5f) {
          map->push_back(p);
        }
      }
    }
  }
  return map;
}

Eigen::Matrix4f poseAt(float x, float y)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose(0, 3) = x;
  pose(1, 3) = y;
  return pose;
}

// The target setInputTargetForPose builds around (x, y) in the scan callback.
std::size_t expectedTargetSize(
  const pcl::PointCloud<pcl::PointXYZI> & map, const std::string & method, float x, float y)
{
  const auto local_map = ll::cropLocalMapByRadius(map, x, y, kRadius);
  if (!ll::usesFilteredTarget(method)) {
    return local_map->size();
  }
  pcl::PointCloud<pcl::PointXYZI> filtered;
  pcl::VoxelGrid<pcl::PointXYZI> grid;
  grid.setLeafSize(1.0f, 1.0f, 1.0f);
  grid.setInputCloud(local_map);
  grid.filter(filtered);
  return filtered.size();
}

bool isCropAround(
  const pcl::PointCloud<pcl::PointXYZI> & target, float x, float y, float beyond_x)
{
  bool reaches_beyond = false;
  for (const auto & p : target.points) {
    if (std::hypot(p.x - x, p.y - y) > kRadius + 0.5) {
      return false;
    }
    reaches_beyond = reaches_beyond || p.x > beyond_x;
  }
  return reaches_beyond;
}

void checkBackend(const std::string & method)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("registration_method", method),
    rclcpp::Parameter("enable_local_map_crop", true),
    rclcpp::Parameter("local_map_radius", kRadius),
    rclcpp::Parameter("local_map_update_distance", 5.0),
    rclcpp::Parameter("local_map_async_rebuild", true),
    rclcpp::Parameter("scan_max_range", 10.0),
    rclcpp::Parameter("voxel_leaf_size", 1.0),
    rclcpp::Parameter("ndt_num_threads", 2)});
  PCLLocalization node(options);
  node.initializeParameters();
  node.initializeRegistration();
  const auto map = makeMap();
  node.use_local_map_crop_ = true;
  node.full_map_cloud_ptr_ = map;
  node.map_bounds_valid_ = true;
  node.map_min_pt_.x = -kMapHalfSize;
  node.map_min_pt_.y = -kMapHalfSize;
  node.map_max_pt_.x = kMapHalfSize;
  node.map_max_pt_.y = kMapHalfSize;

  // Nothing cached: the first target is built in the scan callback.
  assert(node.setInputTargetForPose(poseAt(0.0f, 0.0f)));
  auto * const first = node.registration_;
  assert(!node.local_map_rebuild_.valid());
  assert(first->getInputTarget()->size() == expectedTargetSize(*map, method, 0.0f, 0.0f));

  // Past the update distance and within radius - scan_max_range: keep the
  // cached target and build the next one on the twin in the background.
  assert(node.setInputTargetForPose(poseAt(6.0f, 0.0f)));
  assert(node.registration_ == first);
  assert(node.local_map_rebuild_.valid());
  node.local_map_rebuild_.wait();

  // The next scan swaps the twin in, its target cropped (and filtered like the
  // scan callback's) around (6, 0) and already aligned once.
  assert(node.setInputTargetForPose(poseAt(7.0f, 0.0f)));
  auto * const second = node.registration_;
  assert(second != first);
  assert(second == node.registration_objects_.get());
  assert(node.local_map_target_center_x_ == 6.0f);
  const auto target = second->getInputTarget();
  assert(isCropAround(*target, 6.0f, 0.0f, kRadius + 0.5));
  assert(target->size() == expectedTargetSize(*map, method, 6.0f, 0.0f));
  const auto warm_up_source = second->getInputSource();
  assert(warm_up_source && !warm_up_source->empty());
  assert(warm_up_source->size() < target->size() / 10);
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  if (node.registration_objects_.small_gicp) {
    assert(node.registration_objects_.small_gicp->getTargetCovariances().size() == target->size());
  }
#endif

  // A background target around a center the pose has since left is dropped,
  // and the target is built in the scan callback instead.
  assert(node.setInputTargetForPose(poseAt(12.0f, 0.0f)));
  assert(node.local_map_rebuild_.valid());
  node.local_map_rebuild_.wait();
  assert(node.setInputTargetForPose(poseAt(40.0f, 0.0f)));
  assert(node.registration_ == second);
  assert(!node.local_map_rebuild_.valid());
  assert(node.local_map_target_center_x_ == 40.0f);
  assert(isCropAround(*node.registration_->getInputTarget(), 40.0f, 0.0f, 50.0f));
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  for (const std::string method : {"NDT_OMP", "GICP_OMP", "GICP"}) {
    checkBackend(method);
  }
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  for (const std::string method : {"SMALL_GICP", "SMALL_VGICP"}) {
    checkBackend(method);
  }
#endif
  rclcpp::shutdown();
}
