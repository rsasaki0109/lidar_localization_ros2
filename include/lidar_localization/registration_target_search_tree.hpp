#ifndef LIDAR_LOCALIZATION_REGISTRATION_TARGET_SEARCH_TREE_HPP_
#define LIDAR_LOCALIZATION_REGISTRATION_TARGET_SEARCH_TREE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <pcl/search/kdtree.h>

#include <future>

namespace lidar_localization
{

// Set an NDT target and build, on another thread while setInputTarget() builds the
// NDT voxel grid, the search tree over the target points that pcl::Registration
// otherwise builds in the first align(). NDT aligns with its voxel grid and uses
// that tree only for getFitnessScore(), so the two builds overlap: an 80 m crop of
// the dense Koide outdoor map (753k points) took 0.25 s instead of 0.52 s, with the
// same fitness score (experiments/istanbul_ndt_tracking).
//
// The tree is handed over with force_no_recompute, which pcl::Registration keeps
// for later targets as well. Once a registration got a target from this function,
// give it every later target through this function too.
template<typename PointT>
void setNdtTargetWithSearchTree(
  pcl::Registration<PointT, PointT> & registration,
  const typename pcl::PointCloud<PointT>::ConstPtr & target)
{
  // A fresh tree without force_no_recompute: align() builds it as it would.
  const auto build_tree_in_align = [&registration]() {
      registration.setSearchMethodTarget(
        typename pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>()), false);
    };
  if (!target || target->empty()) {
    build_tree_in_align();
    registration.setInputTarget(target);
    return;
  }
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  std::future<void> tree_built = std::async(
    std::launch::async, [&tree, &target]() {tree->setInputCloud(target);});
  try {
    registration.setInputTarget(target);
  } catch (...) {
    tree_built.wait();
    build_tree_in_align();
    throw;
  }
  try {
    tree_built.get();
  } catch (...) {
    build_tree_in_align();
    throw;
  }
  registration.setSearchMethodTarget(tree, true);
}

// setNdtTargetWithSearchTree() for the NDT backends (usesNdtVoxelGrid), a plain
// setInputTarget() for the others: their setInputTarget() does little, and GICP
// needs the tree in the first align() anyway.
template<typename PointT>
void setRegistrationTarget(
  pcl::Registration<PointT, PointT> & registration,
  const typename pcl::PointCloud<PointT>::ConstPtr & target,
  bool ndt_backend)
{
  if (ndt_backend) {
    setNdtTargetWithSearchTree(registration, target);
  } else {
    registration.setInputTarget(target);
  }
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_REGISTRATION_TARGET_SEARCH_TREE_HPP_
