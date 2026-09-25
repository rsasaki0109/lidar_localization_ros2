#pragma once
// Standalone experiment. Caller owns locking; integration uses a value snapshot.
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>
namespace twist_interval_experiment {
struct Sample {
  double stamp;
  Eigen::Vector3f linear, angular;
};
struct Interval {double duration; Sample sample;};
using Plan = std::vector<Interval>;
class History {
  std::size_t capacity_;
  std::deque<Sample> samples_;
public:
  explicit History(std::size_t capacity=1024): capacity_(capacity) {
    if (!capacity) {throw std::invalid_argument("zero capacity");}
  }
  bool insert(const Sample & x) {
    if (!std::isfinite(x.stamp) || x.stamp<0 || !x.linear.allFinite() || !x.angular.allFinite()) {return false;}
    auto it=std::lower_bound(samples_.begin(),samples_.end(),x.stamp,
      [](const Sample & a,double t){return a.stamp<t;});
    if (it!=samples_.end() && it->stamp==x.stamp) {*it=x;} else {samples_.insert(it,x);}
    while(samples_.size()>capacity_) {samples_.pop_front();}
    return true;
  }
  void clear() {samples_.clear();}
  std::size_t size() const {return samples_.size();}
  std::optional<Plan> plan(double start,double end,double max_age) const {
    if (!std::isfinite(start)||!std::isfinite(end)||end<start||
        !std::isfinite(max_age)||max_age<=0) {return std::nullopt;}
    Plan out;
    if (end==start) {return out;}
    auto it=std::upper_bound(samples_.begin(),samples_.end(),start,
      [](double t,const Sample & a){return t<a.stamp;});
    if(it==samples_.begin()) {return std::nullopt;}
    Sample held=*std::prev(it);double cursor=start;
    for (;it!=samples_.end() && it->stamp<end;++it) {
      if(it->stamp-held.stamp>max_age) {return std::nullopt;}
      out.push_back({it->stamp-cursor,held});cursor=it->stamp;held=*it;
    }
    if(end-held.stamp>max_age) {return std::nullopt;}
    out.push_back({end-cursor,held});return out;
  }
};
inline Eigen::Matrix4f integrate(Eigen::Matrix4f pose,const Plan & plan,bool angular=true) {
  for (const auto & interval:plan) {
    const float dt=static_cast<float>(interval.duration);
    const auto & v=interval.sample;
    pose.block<3,1>(0,3)+=pose.block<3,3>(0,0)*(v.linear*dt);
    if (angular) {
      const Eigen::Vector3f a=v.angular*dt;
      const Eigen::Matrix3f delta=(Eigen::AngleAxisf(a.x(),Eigen::Vector3f::UnitX())*
        Eigen::AngleAxisf(a.y(),Eigen::Vector3f::UnitY())*
        Eigen::AngleAxisf(a.z(),Eigen::Vector3f::UnitZ())).toRotationMatrix();
      pose.block<3,3>(0,0)=pose.block<3,3>(0,0)*delta;
    }
  }
  return pose;
}
}
