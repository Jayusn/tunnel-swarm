#include "object_motion_local.h"
#include "object_motion_evidence.h"
#include <cstdlib>
#include <iostream>
#include <limits>
void check(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
int main() {
  using namespace mot_mapping;
  const std::vector<Eigen::Vector3d> shape{{0,0,0}, {0,.15,0}, {0,0,.15}, {0,.15,.15}};
  const Eigen::Vector3d move(.12,0,0);
  const auto translated = matchLocalMotion(shape, shape, move);
  check(translated.supports(1.,1.), "translation in both directions, inclusive thresholds");
  check(!matchLocalMotion(shape,shape,Eigen::Vector3d::Zero()).supports(.6,.2), "stationary object");
  std::vector<Eigen::Vector3d> jitter=shape;
  for (auto& p : jitter) p.x() -= .01;
  check(!matchLocalMotion(shape,jitter,Eigen::Vector3d(.01,0,0)).supports(.6,.2), "static sampling and center jitter");
  const std::vector<Eigen::Vector3d> partial{shape[0]};
  const auto occluded = matchLocalMotion(shape, partial, move);
  check(occluded.forward_moving==1. && occluded.reverse_moving==.25, "directional denominators include missing support");
  check(!occluded.supports(.6,.2), "one-way overlap is insufficient");
  check(occluded.supports(.25,.25), "exact match and advantage boundary");
  check(!occluded.supports(.250001,.25), "immediately above boundary");
  check(!matchLocalMotion({},shape,move).valid, "empty prior observation");
  auto invalid=shape;invalid[0].x()=std::numeric_limits<double>::quiet_NaN();
  check(!matchLocalMotion(shape,invalid,move).valid, "invalid measured points");
  check(matchLocalMotion({Eigen::Vector3d::Zero()},{Eigen::Vector3d(.05,0,0)},move).forward_moving==1., "five centimeter inclusive radius");
  check(matchLocalMotion({Eigen::Vector3d::Zero()},{Eigen::Vector3d(.050001,0,0)},move).forward_moving==0., "outside matching radius");
  ObjectMotionEvidence evidence;
  auto update=[&](uint64_t t,uint64_t p,bool local=true,double depth=.4) {
    return evidence.update(t,p,true,local,.8,.1,depth,.45,.3,.4);
  };
  check(!update(1,0) && !update(2,1) && update(3,2), "three actual observations");
  check(!update(4,3,false) && !update(5,4) && update(6,5), "local rejection breaks consecutive transitions");
  check(!update(8,7) && !update(9,8) && update(10,9), "missing frame cannot bridge confirmation");
  check(!update(11,10,true,0.), "unknown or occluded depth cannot be replaced by local match");
  std::cout << "PASS local translation, static jitter, asymmetric overlap, boundaries, invalid input, gaps, unknown depth\n";
}
