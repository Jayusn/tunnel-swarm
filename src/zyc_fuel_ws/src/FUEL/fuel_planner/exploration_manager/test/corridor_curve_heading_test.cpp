#include <exploration_manager/corridor_curve_heading.h>
#include <iostream>
#include <cstdlib>

using fast_planner::corridor_curve::Result;
using fast_planner::corridor_curve::infer;
void check(bool ok, const char* message) {
  if (!ok) { std::cerr << message << std::endl; std::exit(1); }
}
int main() {
  std::vector<Eigen::Vector2d> left, right;
  for (int i=0;i<=10;++i) {
    const double x=-0.30+0.15*i;
    left.emplace_back(x,0.65+0.20*x*x);
    right.emplace_back(x,-0.65+0.20*x*x);
  }
  Result curve;
  check(infer(left,right,0.06,curve) && curve.curved, "smooth arc not recognized");
  check(curve.yaw(0.8)>curve.yaw(0.2), "arc yaw does not advance with position");
  check(std::fabs(curve.yaw(0.5)-std::atan(0.2))<1e-6,"wrong wall tangent");
  auto noisy_left=left;
  noisy_left[4].y()=0.15; noisy_left[5].y()=0.15;
  Result detour;
  check(infer(noisy_left,right,0.06,detour) && detour.curved,
        "compact obstacle destroyed paired-wall arc");
  check(std::fabs(detour.yaw(0.5)-curve.yaw(0.5))<1e-6,
        "obstacle changed channel yaw");
  for (auto& point:left) point.y()=0.65;
  for (auto& point:right) point.y()=-0.65;
  Result straight;
  check(infer(left,right,0.06,straight) && !straight.curved,
        "straight corridor became a curve");
  left[4].y()=0.15; left[5].y()=0.15;
  check(infer(left,right,0.06,straight) && !straight.curved,
        "straight bypass became a curve");
  left.resize(5);
  check(!infer(left,right,0.06,straight),"corner wall ending became a curve");
  left.clear(); right.clear();
  for (int i=0;i<=10;++i) {
    const double x=-0.30+0.15*i;
    left.emplace_back(x,0.65+std::max(0.0,x-0.30)*2.0);
    right.emplace_back(x,-0.65);
  }
  check(!infer(left,right,0.06,straight),"right-angle branch replaced stop-and-turn");
  for (int i=0;i<=10;++i) {
    const double x=left[i].x();
    left[i].y()=0.65-0.20*x*x; right[i].y()=-0.65-0.20*x*x;
  }
  check(infer(left,right,0.06,curve) && curve.curved && curve.yaw(0.5)<0,
        "right-hand arc not recognized");
  check(!infer({},right,0.06,curve),"one wall was enough to trigger curve follow");
  // 真实内外弯的半径不同，同一个x切面的远端切线并不平行。
  for (int i=0;i<=10;++i) {
    const double x=left[i].x();
    left[i].y()=0.65+1.4-std::sqrt(1.4*1.4-x*x);
    right[i].y()=-0.65+2.7-std::sqrt(2.7*2.7-x*x);
  }
  check(infer(left,right,0.06,curve) && curve.curved,
        "different inner/outer arc radii prevented curve recognition");
  std::cout << "corridor curve geometry: passed" << std::endl;
}
