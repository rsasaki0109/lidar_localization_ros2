#include "lidar_localization/experimental_scan_motion.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
namespace lidar_localization {
using Point=pcl::PointNormal;
using Cloud=ExperimentalScanMotion::Cloud;
void ExperimentalScanMotion::reset() {
  previous_.reset();normals_.reset();relative_.setIdentity();world_.setIdentity();
}
void ExperimentalScanMotion::anchor(const Message& msg,const Eigen::Matrix4f& world,std::uint64_t generation) {
  if (!world.allFinite()) {reset();return;}
  if (previous_ != msg) {normals_.reset();relative_.setIdentity();}
  previous_=msg;world_=world;generation_=generation;
}
Cloud::Ptr ExperimentalScanMotion::prepare(const sensor_msgs::msg::PointCloud2& msg) {
  pcl::PointCloud<pcl::PointXYZ> xyz;pcl::fromROSMsg(msg,xyz);
  Cloud::Ptr input(new Cloud),output(new Cloud);
  for(const auto& v:xyz) {
    Point p{};p.x=v.x;p.y=v.y;p.z=v.z;
    if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))input->push_back(p);
  }
  pcl::VoxelGrid<Point> voxel;voxel.setLeafSize(.1f,.1f,.1f);voxel.setInputCloud(input);voxel.filter(*output);
  pcl::search::KdTree<Point> tree;tree.setInputCloud(output);
  std::vector<int> ids;std::vector<float> distances;
  for(auto& p:*output) {
    Eigen::Vector4f plane;float curvature=0;
    if(tree.radiusSearch(p,.3,ids,distances,30)>=3 && pcl::computePointNormal(*output,ids,plane,curvature) && plane.allFinite()) {
      p.normal_x=plane[0];p.normal_y=plane[1];p.normal_z=plane[2];
    } else {p.normal_x=0;p.normal_y=0;p.normal_z=1;}
    p.curvature=curvature;
  }
  return output;
}
bool ExperimentalScanMotion::predict(const Message& msg,std::uint64_t generation,Eigen::Matrix4f& world) {
  if(!previous_ || generation!=generation_ || msg->header.frame_id!=previous_->header.frame_id) {reset();return false;}
  const auto ns=[](const auto& stamp){return std::int64_t(stamp.sec)*1000000000LL+stamp.nanosec;};
  const auto dt=ns(msg->header.stamp)-ns(previous_->header.stamp);
  if(dt<=0 || dt>500000000LL){reset();return false;}
  try {
    if(!normals_)normals_=prepare(*previous_);
    auto current=prepare(*msg);
    if(normals_->size()<3 || current->size()<3){reset();return false;}
    pcl::IterativeClosestPointWithNormals<Point,Point> icp;
    using Est=pcl::registration::TransformationEstimationPointToPlaneLLS<Point,Point,float>;
    icp.setTransformationEstimation(pcl::make_shared<Est>());
    icp.setMaxCorrespondenceDistance(.5);icp.setMaximumIterations(30);
    icp.setTransformationEpsilon(1e-6);icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setInputSource(current);icp.setInputTarget(normals_);Cloud aligned;icp.align(aligned,relative_);
    auto relative=icp.getFinalTransformation();
    if(!icp.hasConverged() || !relative.allFinite()){reset();return false;}
    world=world_*relative;
    if(!world.allFinite()){reset();return false;}
    previous_=msg;normals_=current;relative_=relative;world_=world;return true;
  } catch(const std::exception&){reset();return false;}
}
}
