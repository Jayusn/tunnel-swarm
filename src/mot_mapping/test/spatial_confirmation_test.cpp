#include "spatial_confirmation.h"
#include <cstdlib>
#include <iostream>
#include <limits>
void check(bool value, const char* message) {
  if (!value) { std::cerr << message << '\n'; std::exit(1); }
}
int main() {
  mot_mapping::SpatialConfirmation old;
  const Eigen::Vector3d center(3.1, 0.0, 0.65), size(0.12, 0.20, 0.20);
  check(!old.supports(10.1,center,size,1.0,.45), "unconfirmed region cannot seed detection");
  old.stamp=10.0; old.position=old.confirmed_position=center; old.size=size;
  check(old.supports(10.3,center+Eigen::Vector3d(0,.20,0),size,1.0,.45), "spatial match independent of tracker ID");
  check(old.supports(10.9,center,Eigen::Vector3d(.05,.06,.02),1.0,.45), "partial real return can continue known body");
  check(!old.supports(11.1,center,size,1.0,.45), "unobserved region expires");
  check(!old.supports(10.0,center,size,1.0,.45), "no same-frame chained confirmation");
  check(!old.supports(9.9,center,size,1.0,.45), "no future history");
  check(!old.supports(10.1,center+Eigen::Vector3d(.6,0,0),size,1.0,.45), "unrelated region is not inherited");
  check(!old.supports(10.1,center,Eigen::Vector3d(.1,.2,.8),1.0,.45), "attached rope or wall cannot inherit body confirmation");
  check(!old.supports(10.1,center,Eigen::Vector3d(-.1,.2,.2),1.0,.45), "invalid extent");
  check(!old.supports(10.1,Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),size,1.0,.45), "invalid position");
  old.position = center + Eigen::Vector3d(.4,0,0);
  check(!old.supports(10.1,center+Eigen::Vector3d(.7,0,0),size,1.0,.45), "weak updates cannot walk from confirmed body into another object");
  std::cout << "PASS measured spatial continuation, gaps, geometry, time and unrelated regions\n";
}
