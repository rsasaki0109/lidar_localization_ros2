// The map-distance prefilter (map_distance_filter.hpp) and the registration engine's
// setSourceNearTarget(), without ROS: when it runs, which points it keeps, and that a
// scan half covered by an obstacle the map lacks then scores as the map alone.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include <pcl/search/kdtree.h>

#include "lidar_localization/map_distance_filter.hpp"
#include "lidar_localization/registration_engine.hpp"

namespace ll = lidar_localization;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

namespace
{

void add(Cloud & cloud, float x, float y, float z)
{
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 1.0f;
  cloud.push_back(p);
}

// A 20 x 14 m room: floor, 3 m walls and a 1 m pillar, every 0.2 m.
Cloud::Ptr room()
{
  Cloud::Ptr map(new Cloud());
  const float step = 0.2f;
  for (float x = -10.0f; x <= 10.0f; x += step) {
    for (float y = -7.0f; y <= 7.0f; y += step) {
      add(*map, x, y, 0.0f);
    }
  }
  for (float z = step; z <= 3.0f; z += step) {
    for (float x = -10.0f; x <= 10.0f; x += step) {
      add(*map, x, -7.0f, z);
      add(*map, x, 7.0f, z);
    }
    for (float y = -7.0f; y <= 7.0f; y += step) {
      add(*map, -10.0f, y, z);
      add(*map, 10.0f, y, z);
    }
    for (float s = 0.0f; s <= 1.0f; s += step) {
      add(*map, 3.0f + s, 2.0f, z);
      add(*map, 3.0f + s, 3.0f, z);
      add(*map, 3.0f, 2.0f + s, z);
      add(*map, 4.0f, 2.0f + s, z);
    }
  }
  return map;
}

Eigen::Matrix4f pose(float x, float y, float yaw)
{
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m(0, 0) = std::cos(yaw);
  m(0, 1) = -std::sin(yaw);
  m(1, 0) = std::sin(yaw);
  m(1, 1) = std::cos(yaw);
  m(0, 3) = x;
  m(1, 3) = y;
  return m;
}

void checkDecision()
{
  ll::MapDistanceFilterParams params;
  ll::MapDistanceFilterInput input{true, 0.1, 0.1, 10};
  // Off by default.
  auto decision = ll::decideMapDistanceFilter(params, input);
  assert(!decision.apply && decision.status == ll::MapDistanceFilterStatus::kDisabled);

  params.distance_m = 1.0;
  decision = ll::decideMapDistanceFilter(params, input);
  assert(decision.apply && decision.status == ll::MapDistanceFilterStatus::kApplied);
  assert(std::abs(decision.distance_m - 1.0) < 1e-9);

  // Grows with the time since the last accepted pose beyond one scan period.
  input.since_accept_sec = 1.1;
  decision = ll::decideMapDistanceFilter(params, input);
  assert(decision.apply && std::abs(decision.distance_m - 2.0) < 1e-9);

  // Past the maximum the scan is aligned whole.
  input.since_accept_sec = 2.2;
  decision = ll::decideMapDistanceFilter(params, input);
  assert(!decision.apply && decision.status == ll::MapDistanceFilterStatus::kGapTooLong);

  // Not before the first accepted pose, nor while the warmup after an initial pose runs.
  input = ll::MapDistanceFilterInput{false, 0.1, 0.1, 10};
  decision = ll::decideMapDistanceFilter(params, input);
  assert(!decision.apply && decision.status == ll::MapDistanceFilterStatus::kNoAcceptedPose);
  input = ll::MapDistanceFilterInput{true, std::numeric_limits<double>::quiet_NaN(), 0.1, 10};
  decision = ll::decideMapDistanceFilter(params, input);
  assert(!decision.apply && decision.status == ll::MapDistanceFilterStatus::kNoAcceptedPose);
  input = ll::MapDistanceFilterInput{true, 0.1, 0.1, 4};
  decision = ll::decideMapDistanceFilter(params, input);
  assert(!decision.apply && decision.status == ll::MapDistanceFilterStatus::kWarmingUp);
  params.warmup_accepts = 4;
  assert(ll::decideMapDistanceFilter(params, input).apply);

  // With no warmup, the first scan after an initial pose trusts it whatever its stamp.
  params.warmup_accepts = 0;
  input = ll::MapDistanceFilterInput{true, 1.0e9, 0.1, 0};
  decision = ll::decideMapDistanceFilter(params, input);
  assert(decision.apply && std::abs(decision.distance_m - 1.0) < 1e-9);
  input.accepts_since_initial_pose = 1;
  assert(!ll::decideMapDistanceFilter(params, input).apply);
  input.since_accept_sec = 0.1;

  // No growth: the distance stays.
  params.growth_m_per_s = 0.0;
  input.since_accept_sec = 100.0;
  decision = ll::decideMapDistanceFilter(params, input);
  assert(decision.apply && std::abs(decision.distance_m - 1.0) < 1e-9);
  assert(std::string(ll::mapDistanceFilterStatusName(decision.status)) == "applied");
}

void checkFilter()
{
  // A floor at z = 0; source points 0.1, 0.9 and 1.5 m above it once moved up 2 m.
  Cloud::Ptr floor(new Cloud());
  for (float x = -5.0f; x <= 5.0f; x += 0.1f) {
    for (float y = -5.0f; y <= 5.0f; y += 0.1f) {
      add(*floor, x, y, 0.0f);
    }
  }
  pcl::search::KdTree<pcl::PointXYZI> tree;
  tree.setInputCloud(floor);
  Cloud source;
  add(source, 0.0f, 0.0f, -1.9f);
  add(source, 1.0f, 0.0f, -1.1f);
  add(source, 2.0f, 0.0f, -0.5f);
  add(source, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
  Eigen::Matrix4f up = Eigen::Matrix4f::Identity();
  up(2, 3) = 2.0f;
  const auto kept = ll::filterByMapDistance(source, tree, up, 1.0, 2);
  assert(kept->size() == 2);
  // The kept points are the source's, unmoved, in order.
  assert(std::abs((*kept)[0].z + 1.9f) < 1e-6f && std::abs((*kept)[1].z + 1.1f) < 1e-6f);
  assert(ll::filterByMapDistance(source, tree, up, 2.0, 1)->size() == 3);
}

// The room seen from TRUTH, plus a 4 x 2 m slab of obstacle points 2 m in front of
// the sensor, 1.2 m above the floor and clear of the pillar, that the map lacks.
Cloud::Ptr scanWithObstacle(const Cloud & map, const Eigen::Matrix4f & truth, std::size_t * obstacle)
{
  Cloud::Ptr scan(new Cloud());
  const Eigen::Matrix4f inverse = truth.inverse();
  for (const auto & p : map) {
    const Eigen::Vector4f q = inverse * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
    add(*scan, q.x(), q.y(), q.z());
  }
  const std::size_t before = scan->size();
  for (float y = -3.0f; y <= 1.0f; y += 0.03f) {
    for (float z = 1.2f; z <= 3.2f; z += 0.03f) {
      add(*scan, 2.0f, y, z);
    }
  }
  *obstacle = scan->size() - before;
  return scan;
}

void checkEngine()
{
  const Cloud::Ptr map = room();
  const Eigen::Matrix4f truth = pose(1.0f, -0.5f, 0.1f);
  std::size_t obstacle = 0;
  const Cloud::Ptr scan = scanWithObstacle(*map, truth, &obstacle);
  std::printf("scan %zu points, %zu on the obstacle\n", scan->size(), obstacle);
  std::fflush(stdout);

  ll::RegistrationEngineConfig config;
  config.registration_method = "NDT_OMP";
  config.ndt_num_threads = 2;
  config.transform_epsilon = 1e-4;
  ll::RegistrationEngine engine;
  assert(engine.configure(config));
  engine.setMap(map, ll::computeMapBounds(*map));
  assert(engine.setTargetForPose(truth));

  // The whole scan: the obstacle's points are outliers.
  engine.setSource(scan);
  auto aligned = engine.align(truth);
  assert(aligned.has_converged);
  const double whole_ratio = engine.fitness(0.5, 10).inlier_ratio;

  // Near the map at a guess 0.3 m off: the obstacle is left out.
  const Eigen::Matrix4f guess = pose(1.25f, -0.35f, 0.1f);
  const auto filtered = engine.setSourceNearTarget(scan, guess, 1.0, 0.3);
  assert(filtered.status == ll::MapDistanceFilterStatus::kApplied);
  assert(filtered.input_points == scan->size());
  // The room's points, about: the obstacle's are all over 1 m from the map.
  const std::size_t room_points = scan->size() - obstacle;
  assert(filtered.kept_points >= room_points * 99 / 100);
  assert(filtered.kept_points <= room_points + obstacle / 50);
  aligned = engine.align(guess);
  assert(aligned.has_converged);
  const double filtered_ratio = engine.fitness(0.5, 10).inlier_ratio;
  const Eigen::Matrix4f error = truth.inverse() * aligned.final_transformation;
  const float off_m = std::hypot(error(0, 3), error(1, 3));
  std::printf(
    "inlier ratio whole %.3f, filtered %.3f; filtered alignment %.3f m off\n",
    whole_ratio, filtered_ratio, off_m);
  std::fflush(stdout);
  assert(whole_ratio < 0.6);
  assert(filtered_ratio > 0.95);

  // NDT's 1 m cells average the room's regular samples: a few centimetres off.
  assert(off_m < 0.15f);

  // A guess far off keeps too few points: the whole scan is set.
  Eigen::Matrix4f lifted = truth;
  lifted(2, 3) = 30.0f;
  const auto far = engine.setSourceNearTarget(scan, lifted, 1.0, 0.3);
  assert(far.status == ll::MapDistanceFilterStatus::kTooFewKept);
  assert(far.kept_points == 0);
  assert(engine.registration()->getInputSource()->size() == scan->size());

  // Without a target search tree (no map yet) the scan is set whole.
  ll::RegistrationEngine empty;
  assert(empty.configure(config));
  const auto untreed = empty.setSourceNearTarget(scan, truth, 1.0, 0.3);
  assert(untreed.status == ll::MapDistanceFilterStatus::kNoTargetTree);
  assert(empty.registration()->getInputSource()->size() == scan->size());
}

}  // namespace

int main()
{
  checkDecision();
  checkFilter();
  checkEngine();
  std::printf("ok\n");
  std::fflush(stdout);
  return 0;
}
