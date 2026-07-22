#include "glim_prior_map_localizer/vgicp_refiner.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim_prior_map_localizer
{

std::shared_ptr<const gtsam_points::GaussianVoxelMapCPU> makeVgicpTarget(
  const std::shared_ptr<const gtsam_points::PointCloud> & target,
  double voxel_resolution_m)
{
  if (!target || target->size() == 0 || !(voxel_resolution_m > 0.0)) {
    return nullptr;
  }
  auto target_voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(
    voxel_resolution_m);
  target_voxels->insert(*target);
  return target_voxels;
}

VgicpRefinementResult refineWithVgicp(
  const std::shared_ptr<const gtsam_points::GaussianVoxelMapCPU> & target_voxels,
  const std::shared_ptr<const gtsam_points::PointCloud> & source,
  const Eigen::Isometry3d & initial_pose,
  int max_iterations)
{
  VgicpRefinementResult result;
  result.pose = initial_pose;
  result.target_voxels = target_voxels;
  if (!target_voxels || !source || source->size() == 0 ||
    !source->has_covs() || !initial_pose.matrix().allFinite() ||
    max_iterations < 1)
  {
    return result;
  }

  constexpr gtsam::Key source_key = 0;
  auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
    gtsam::Pose3::Identity(), source_key, target_voxels, source);
  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  gtsam::Values values;
  values.insert(source_key, gtsam::Pose3(initial_pose.matrix()));

  gtsam_points::LevenbergMarquardtExtParams params;
  params.setMaxIterations(max_iterations);
  const auto optimized =
    gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, params).optimize();
  const gtsam::Pose3 refined = optimized.at<gtsam::Pose3>(source_key);
  const double error = factor->error(optimized);
  result.pose = Eigen::Isometry3d(refined.matrix());
  result.num_inliers = factor->num_inliers();
  result.inlier_fraction = factor->inlier_fraction();
  result.normalized_error = result.num_inliers > 0 ?
    error / static_cast<double>(result.num_inliers) :
    std::numeric_limits<double>::infinity();
  result.valid = result.pose.matrix().allFinite() &&
    std::isfinite(result.inlier_fraction) &&
    std::isfinite(result.normalized_error) && result.num_inliers > 0;
  return result;
}

VgicpRefinementResult refineWithVgicp(
  const std::vector<Eigen::Vector3f> & target_points,
  const std::shared_ptr<const gtsam_points::PointCloud> & source,
  const Eigen::Isometry3d & initial_pose,
  double voxel_resolution_m,
  int max_iterations)
{
  if (target_points.empty() || !source || source->size() == 0 ||
    !source->has_covs() || !initial_pose.matrix().allFinite() ||
    !(voxel_resolution_m > 0.0) || max_iterations < 1)
  {
    VgicpRefinementResult result;
    result.pose = initial_pose;
    return result;
  }

  std::vector<Eigen::Vector4d> target_points_homogeneous;
  target_points_homogeneous.reserve(target_points.size());
  for (const auto & point : target_points) {
    target_points_homogeneous.emplace_back(
      point.x(), point.y(), point.z(), 1.0);
  }
  auto target = std::make_shared<gtsam_points::PointCloudCPU>(
    target_points_homogeneous);
  // GaussianVoxelMapCPU consumes each input point covariance while building
  // voxel distributions. A raw prior-map PLY has no covariance channel, so
  // give each sample a small isotropic measurement covariance; the voxel's
  // spatial distribution still supplies the dominant surface geometry.
  std::vector<Eigen::Matrix4d> target_covariances(
    target_points_homogeneous.size(), Eigen::Matrix4d::Zero());
  for (auto & covariance : target_covariances) {
    covariance.diagonal().head<3>().setConstant(1.0e-2);
  }
  target->add_covs(target_covariances);
  auto target_voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(
    voxel_resolution_m);
  target_voxels->insert(*target);
  return refineWithVgicp(target_voxels, source, initial_pose, max_iterations);
}

}  // namespace glim_prior_map_localizer
