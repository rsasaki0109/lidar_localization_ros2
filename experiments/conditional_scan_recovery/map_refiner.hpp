#ifndef CONDITIONAL_SCAN_RECOVERY_MAP_REFINER_HPP_
#define CONDITIONAL_SCAN_RECOVERY_MAP_REFINER_HPP_

#include <chrono>
#include <cmath>
#include <limits>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/gicp_omp.h>

namespace conditional_scan_recovery
{
// Experimental map measurement, never a pose-publication path. The caller must
// apply the normal gates/backend and discard results after reset/map changes.
// Caller serializes align(); map and source clouds are immutable while retained.
class MapRefiner
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  struct Result
  {
    bool target_ready{false};
    bool converged{false};
    Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    double fitness{std::numeric_limits<double>::infinity()};
    double seconds{0.0};
  };

  explicit MapRefiner(const Cloud::ConstPtr & map, float voxel_leaf) : input_map_(map)
  {
    if (!map || map->size() < 20 || !std::isfinite(voxel_leaf) || voxel_leaf <= 0) {return;}
    for (const auto & p : *map) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {return;}
    }
    Cloud::Ptr target(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
    voxel.setInputCloud(map);
    voxel.filter(*target);
    if (target->size() < 20) {return;}
    registration_.setCorrespondenceRandomness(20);
    registration_.setMaxCorrespondenceDistance(2.0);
    registration_.setTransformationEpsilon(.01);
    registration_.setMaximumIterations(30);
    registration_.setInputTarget(target);
    target_ready_ = true;
  }

  const Cloud::ConstPtr & inputMap() const {return input_map_;}

  Result align(const Cloud::ConstPtr & source, const Eigen::Matrix4f & seed)
  {
    Result result;
    result.target_ready = target_ready_;
    if (!target_ready_ || !source || source->size() < 20 || !seed.allFinite()) {return result;}
    for (const auto & p : *source) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {return result;}
    }
    const auto start = std::chrono::steady_clock::now();
    registration_.setInputSource(source);
    Cloud aligned;
    registration_.align(aligned, seed);
    result.pose = registration_.getFinalTransformation();
    result.converged = registration_.hasConverged() && result.pose.allFinite();
    result.fitness = registration_.getFitnessScore();
    result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return result;
  }

private:
  Cloud::ConstPtr input_map_;
  bool target_ready_{false};
  pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> registration_;
};
}  // namespace conditional_scan_recovery
#endif
