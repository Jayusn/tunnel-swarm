#include "object_motion_evidence.h"
#include <cstdlib>
#include <iostream>
#include <limits>
void check(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
int main() {
  using mot_mapping::ObjectMotionEvidence;
  ObjectMotionEvidence track, other;
  auto update = [&](uint64_t t, uint64_t p, double m=.8, double s=.1, double d=.4,
                    bool valid=true, bool transition=true) {
    return track.update(t,p,valid,transition,m,s,d,.6,.2,.3);
  };
  check(!update(100,0),"first observation");
  check(!update(200,100),"one transition");
  check(update(300,200),"two transitions must confirm without reversal");
  check(!other.update(300,200,true,true,.8,.1,.4,.6,.2,.3),"independent tracks");
  check(!update(400,300,.8,.8),"static sampling jitter");
  check(!update(500,400),"jitter breaks evidence");
  check(update(600,500),"new two transitions");
  check(!update(700,600,.8,.1,0),"unobserved/occluded depth is not motion");
  check(!update(800,700),"occlusion breaks evidence");
  check(update(900,800),"reconfirmation");
  check(!update(1100,1000),"missing observation breaks evidence");
  check(!update(1200,1100),"reacquisition first transition");
  check(update(1300,1200),"reacquisition second transition");
  check(!update(1400,1300,.8,.1,.4,false),"background/shape rejection");
  check(!update(1500,1400),"invalid previous observation");
  check(!update(1600,1500),"second valid observation");
  check(update(1700,1600),"third valid observation");
  check(!update(1800,1700,std::numeric_limits<double>::quiet_NaN()),"NaN evidence");
  check(!update(1800,1700),"duplicate time");
  track.clear();
  check(!update(2000,1900),"reset");
  std::cout << "PASS adjacency, translation, static jitter, unknown depth, gaps, invalid shape, independent tracks\n";
}
