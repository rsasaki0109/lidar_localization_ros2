#include "lidar_localization/experimental_twist_interval.hpp"
#include <cassert>
using namespace twist_interval_experiment;
Sample sample(double t,float vx,float wz=0) {return {t,{vx,0,0},{0,0,wz}};}
int main() {
 const Eigen::Matrix4f identity=Eigen::Matrix4f::Identity();
 History h;assert(!h.plan(0,1,2));assert(h.plan(0,0,2)->empty());
 h.insert(sample(1,3));assert(!h.plan(0,1,2));h.insert(sample(0,1));
 auto p=h.plan(0,2,2);assert(p && p->size()==2);
 auto result=integrate(identity,*p);assert(std::abs(result(0,3)-4)<1e-6);
 h.insert(sample(2,999));h.insert(sample(5,-999));
 assert((integrate(identity,*h.plan(0,2,2))-result).norm()==0);
 // Immutable captured plan survives later duplicate replacement and reset.
 h.insert(sample(1,4));assert(integrate(identity,*h.plan(0,2,2))(0,3)==5);
 h.clear();assert(!h.plan(0,2,2));assert(integrate(identity,*p)(0,3)==4);
 assert(!h.insert(sample(NAN,1)));assert(!h.insert(sample(-1,1)));
 auto bad=sample(0,1);bad.angular.x()=INFINITY;assert(!h.insert(bad));
 h.insert(sample(0,1));assert(!h.plan(.5,.6,.2));assert(!h.plan(1,0,1));
 assert(!h.plan(0,1,NAN));assert(!h.plan(0,1,0));assert(!h.plan(0,NAN,1));
 History bounded(2);bounded.insert(sample(0,1));bounded.insert(sample(1,1));bounded.insert(sample(2,1));
 assert(bounded.size()==2 && !bounded.plan(0,1,2));
 // Rotation in first interval rotates translation of the second interval.
 History rotating;rotating.insert(sample(0,0,static_cast<float>(M_PI/2)));rotating.insert(sample(1,1));
 auto rotated=integrate(identity,*rotating.plan(0,2,2));
 assert(std::abs(rotated(0,3))<1e-6 && std::abs(rotated(1,3)-1)<1e-6);
 auto noangular=integrate(identity,*rotating.plan(0,2,2),false);assert(noangular(0,3)==1 && noangular(1,3)==0);
 bool threw=false;try {History invalid(0);}catch(const std::invalid_argument &){threw=true;}assert(threw);
}
