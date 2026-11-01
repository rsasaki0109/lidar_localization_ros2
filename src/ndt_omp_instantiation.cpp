// The NDT_OMP and GICP_OMP code the localizer runs, compiled with this
// package's optimization flags.
//
// ndt_omp_ros2 compiles the same instantiations into libndt_omp but sets no
// CMAKE_BUILD_TYPE, so a plain `colcon build` compiles them without
// optimization: NDT_OMP then aligns a 1500-point Istanbul scan 12-15 times
// slower (250 ms instead of 19 ms with 4 threads). These definitions are
// linked before libndt_omp, so the component uses them whatever build type
// ndt_omp_ros2 was built with.
#include <pcl/point_types.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;
