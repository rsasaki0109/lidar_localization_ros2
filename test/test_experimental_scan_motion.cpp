#include "lidar_localization/experimental_scan_motion.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <cassert>
#include <limits>
using Motion=lidar_localization::ExperimentalScanMotion;
Motion::Message message(double stamp,const Eigen::Matrix4f& transform=Eigen::Matrix4f::Identity()) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for(int axis=0;axis<3;++axis)for(int i=-15;i<=15;++i)for(int j=-15;j<=15;++j){
    Eigen::Vector4f p(0,0,0,1);p[(axis+1)%3]=i*.12f;p[(axis+2)%3]=j*.12f;
    p=transform.inverse()*p;cloud.emplace_back(p.x(),p.y(),p.z());
  }
  auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);
  msg->header.frame_id="livox_frame";msg->header.stamp.sec=static_cast<int>(stamp);
  msg->header.stamp.nanosec=static_cast<unsigned>((stamp-msg->header.stamp.sec)*1e9);
  return msg;
}
int main(){
  Motion motion;Eigen::Matrix4f out;const auto identity=Eigen::Matrix4f::Identity().eval();
  auto first=message(1.);assert(!motion.predict(first,1,out));
  Eigen::Matrix4f delta=identity;delta(0,3)=.04f;delta(1,3)=-.03f;
  motion.anchor(first,identity,1);auto second=message(1.1,delta);assert(motion.predict(second,1,out));assert((out-delta).norm()<.01f);
  // Reanchor accepted output without losing the current scan reference.
  Eigen::Matrix4f accepted=out;accepted(0,3)+=10.;motion.anchor(second,accepted,1);
  assert(motion.predict(message(1.2,delta),1,out));assert((out-accepted).norm()<.01f);
  motion.reset();assert(!motion.predict(message(1.3),1,out));
  for(int kind=0;kind<4;++kind){
    motion.anchor(first,identity,1);auto bad=message(kind==0?1.:kind==1?.9:kind==2?1.500001:1.1);
    assert(!motion.predict(bad,kind==3?2:1,out));assert(!motion.predict(message(1.2),1,out));
  }
  motion.anchor(first,identity,1);auto wrong=std::make_shared<sensor_msgs::msg::PointCloud2>(*message(1.1));wrong->header.frame_id="other";
  assert(!motion.predict(wrong,1,out));assert(!motion.predict(message(1.2),1,out));
  motion.anchor(first,identity,1);Eigen::Matrix4f invalid=identity;invalid(0,0)=std::numeric_limits<float>::quiet_NaN();motion.anchor(first,invalid,1);assert(!motion.predict(second,1,out));
  motion.anchor(first,identity,1);auto empty=std::make_shared<sensor_msgs::msg::PointCloud2>(*second);empty->width=0;empty->row_step=0;empty->data.clear();assert(!motion.predict(empty,1,out));
}
