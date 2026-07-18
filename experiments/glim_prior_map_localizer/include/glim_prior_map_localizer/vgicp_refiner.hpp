#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gtsam_points
{
class GaussianVoxelMapCPU;
struct PointCloud;
}

namespace glim_prior_map_localizer
{

struct VgicpRefinementResult
{
  bool valid{false};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  double inlier_fraction{0.0};
  int num_inliers{0};
  double normalized_error{0.0};
  std::shared_ptr<const gtsam_points::GaussianVoxelMapCPU> target_voxels;
};

VgicpRefinementResult refineWithVgicp(
  const std::vector<Eigen::Vector3f> & target_points,
  const std::shared_ptr<const gtsam_points::PointCloud> & source,
  const Eigen::Isometry3d & initial_pose,
  double voxel_resolution_m,
  int max_iterations);

}  // namespace glim_prior_map_localizer
