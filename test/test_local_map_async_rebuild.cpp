// The registration engine's local targets with background rebuilds (no ROS).
#include "lidar_localization/registration_engine.hpp"
#include "lidar_localization/local_map_target_policy.hpp"
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

// The target setTargetForPose builds around (x, y) in the scan callback.
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
  ll::RegistrationEngineConfig config;
  config.registration_method = method;
  config.enable_local_map_crop = true;
  config.local_map_radius = kRadius;
  config.local_map_update_distance = 5.0;
  config.local_map_async_rebuild = true;
  config.scan_max_range = 10.0;
  config.voxel_leaf_size = 1.0;
  config.ndt_num_threads = 2;
  ll::RegistrationEngine engine;
  assert(engine.configure(config));
  const auto map = makeMap();
  engine.setMap(map, ll::computeMapBounds(*map));
  assert(engine.usesLocalMapCrop());

  // Nothing cached: the first target is built in the scan callback.
  assert(engine.setTargetForPose(poseAt(0.0f, 0.0f)));
  auto * const first = engine.registration();
  assert(!engine.localMapRebuildPending());
  assert(first->getInputTarget()->size() == expectedTargetSize(*map, method, 0.0f, 0.0f));

  // Past the update distance and within radius - scan_max_range: keep the
  // cached target and build the next one on the twin in the background.
  assert(engine.setTargetForPose(poseAt(6.0f, 0.0f)));
  assert(engine.registration() == first);
  assert(engine.localMapRebuildPending());
  engine.waitForLocalMapRebuild();

  // The next scan swaps the twin in, its target cropped (and filtered like the
  // scan callback's) around (6, 0) and already aligned once.
  assert(engine.setTargetForPose(poseAt(7.0f, 0.0f)));
  auto * const second = engine.registration();
  assert(second != first);
  assert(second == engine.registrationObjects().get());
  assert(engine.localMapTargetCenter()->x() == 6.0f);
  const auto target = second->getInputTarget();
  assert(isCropAround(*target, 6.0f, 0.0f, kRadius + 0.5));
  assert(target->size() == expectedTargetSize(*map, method, 6.0f, 0.0f));
  const auto warm_up_source = second->getInputSource();
  assert(warm_up_source && !warm_up_source->empty());
  assert(warm_up_source->size() < target->size() / 10);
  // The first align() on a target builds its search tree.
  assert(second->getSearchMethodTarget()->getInputCloud() == target);
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  if (engine.registrationObjects().small_gicp) {
    assert(
      engine.registrationObjects().small_gicp->getTargetCovariances().size() == target->size());
  }
#endif

  // A background target around a center the pose has since left is dropped,
  // and the target is built in the scan callback instead.
  assert(engine.setTargetForPose(poseAt(12.0f, 0.0f)));
  assert(engine.localMapRebuildPending());
  engine.waitForLocalMapRebuild();
  assert(engine.setTargetForPose(poseAt(40.0f, 0.0f)));
  assert(engine.registration() == second);
  assert(!engine.localMapRebuildPending());
  assert(engine.localMapTargetCenter()->x() == 40.0f);
  assert(isCropAround(*engine.registration()->getInputTarget(), 40.0f, 0.0f, 50.0f));
}

}  // namespace

int main()
{
  for (const std::string method : {"NDT_OMP", "GICP_OMP", "GICP"}) {
    checkBackend(method);
  }
#ifdef LIDAR_LOCALIZATION_HAVE_SMALL_GICP
  for (const std::string method : {"SMALL_GICP", "SMALL_VGICP"}) {
    checkBackend(method);
  }
#endif
}
